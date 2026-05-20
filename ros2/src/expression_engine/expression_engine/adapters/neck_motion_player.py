"""Per-emotion 3-level neck motion player — Story 7.1b.

Loads ``config/neck_motion.yaml``; on ``set_expression(emotion,
intensity)`` snaps to the new ``(L1 pose, L2 drift, L3 gesture)``
and a background ~30 Hz thread calls ``driver.move_pose(...)`` with
the time-evolving target. Driver is injectable so unit tests can
pass a stub.

Intensity → level (firmware-strict, parallel to Story 7.1a):

    intensity == 1     → L1   (static pose only)
    intensity == 2     → L2   (+ idle drift)
    intensity 3..5     → L3   (+ idle drift + peak gesture at onset)

Targets are clamped to the YAML's ``safety`` envelope before being
applied. Unknown emotions fall back to ``neutral``; unknown gesture
tokens are silently skipped (L3 collapses to L2).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

from expression_engine.adapters import neck_gestures
from expression_engine.logging_setup import log_event

DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[2] / "config" / "neck_motion.yaml"
)


@dataclass
class _ActiveGesture:
    token: str
    amp_deg: float
    dur_s: float
    start: float


class NeckMotionPlayer:
    """Compose per-emotion L1 + L2 drift + L3 gesture; tick a driver."""

    def __init__(
        self,
        driver,
        config_path: str | Path = DEFAULT_CONFIG,
        tick_hz: float = 30.0,
    ) -> None:
        self._driver = driver
        self._tick_dt = 1.0 / float(tick_hz)
        data = yaml.safe_load(Path(config_path).read_text())
        self._emotions: dict = data["emotions"]
        safety = data.get("safety", {})
        self._lim_pan  = float(safety.get("pan_deg", 80))
        self._lim_tilt = float(safety.get("tilt_deg", 20))
        self._lim_roll = float(safety.get("roll_deg", 15))

        self._lock = threading.Lock()
        self._emotion = "neutral"
        self._level = 1
        self._gesture: Optional[_ActiveGesture] = None
        self._start_t = time.monotonic()

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # ── public ─────────────────────────────────────────────────

    @staticmethod
    def intensity_to_level(intensity: int) -> int:
        if intensity <= 1:
            return 1
        if intensity == 2:
            return 2
        return 3

    def set_expression(self, emotion: str, intensity: int) -> None:
        """Switch the active per-emotion plan. Re-fires L3 gesture."""
        level = self.intensity_to_level(int(intensity))
        name = (emotion or "neutral").lower()
        if name not in self._emotions:
            name = "neutral"
        with self._lock:
            self._emotion = name
            self._level = level
            self._gesture = None
            if level >= 3:
                g = self._emotions[name].get("l3_gesture")
                if g and g.get("token") in neck_gestures.GESTURES:
                    self._gesture = _ActiveGesture(
                        token=str(g["token"]),
                        amp_deg=float(g["amp_deg"]),
                        dur_s=float(g["dur_s"]),
                        start=time.monotonic(),
                    )
        log_event(
            logging.DEBUG,
            "neck_player_set",
            emotion=name, intensity_level=level,
            gesture=(self._gesture.token if self._gesture else None),
        )

    def compute_target(self, t: float | None = None) -> tuple[float, float, float]:
        """Pure: emotion + level + gesture → (pan, tilt, roll) deg."""
        t = time.monotonic() if t is None else t
        with self._lock:
            spec = self._emotions[self._emotion]
            level = self._level
            gesture = self._gesture
        l1 = spec["l1"]
        pan, tilt, roll = float(l1["pan"]), float(l1["tilt"]), float(l1["roll"])

        # L2 idle drift — three independent sinusoids on a single
        # phase (deterministic from player start_t).
        if level >= 2:
            amp = float(spec.get("l2_drift_deg", 0.3))
            per = float(spec.get("l2_period_s", 5.0))
            phase = (t - self._start_t) * (2.0 * math.pi / per)
            pan  += amp        * math.sin(phase)
            tilt += amp * 0.7  * math.cos(phase * 1.13)
            roll += amp * 0.5  * math.sin(phase * 0.81)

        # L3 peak gesture — additive, finite, decays then clears.
        if gesture is not None:
            dt = t - gesture.start
            if dt >= gesture.dur_s:
                with self._lock:
                    if self._gesture is gesture:
                        self._gesture = None
            else:
                u = dt / gesture.dur_s
                fn = neck_gestures.GESTURES.get(gesture.token)
                if fn is not None:
                    p_off, t_off, r_off = fn(u, gesture.amp_deg)
                    pan += p_off; tilt += t_off; roll += r_off

        return (
            max(-self._lim_pan,  min(self._lim_pan,  pan)),
            max(-self._lim_tilt, min(self._lim_tilt, tilt)),
            max(-self._lim_roll, min(self._lim_roll, roll)),
        )

    # ── threaded driver tick ──────────────────────────────────

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, name="NeckMotionPlayer", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._thread = None

    def _loop(self) -> None:
        while not self._stop.is_set():
            pan, tilt, roll = self.compute_target()
            try:
                self._driver.move_pose(pan=pan, tilt=tilt, roll=roll)
            except Exception as exc:                       # noqa: BLE001
                log_event(
                    logging.WARNING, "neck_player_apply_failed",
                    error=repr(exc))
            self._stop.wait(self._tick_dt)
