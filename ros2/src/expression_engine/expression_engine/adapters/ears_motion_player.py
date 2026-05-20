"""Per-emotion 3-level ears motion player — Story 7.1c.

Loads ``config/ears_motion.yaml``; on ``set_expression(emotion,
intensity)`` snaps to the new (L1 4-joint pose, L2 drift, L3
gesture) and a background ~30 Hz thread calls the driver's per-
joint move methods with the time-evolving target.

Intensity → level (firmware-strict, parallel to 7.1a / 7.1b):

    intensity == 1     → L1   (static pose only)
    intensity == 2     → L2   (+ idle drift)
    intensity 3..5     → L3   (+ idle drift + peak gesture at onset)

Per-joint targets are clamped to the YAML's ``safety`` envelope
before being applied. Unknown emotions fall back to ``neutral``;
unknown gesture tokens are silently skipped (L3 collapses to L2).
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

from expression_engine.adapters import ears_gestures
from expression_engine.logging_setup import log_event

DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[2] / "config" / "ears_motion.yaml"
)

JOINTS = ("left_pan", "left_tilt", "right_pan", "right_tilt")


@dataclass
class _ActiveGesture:
    token: str
    amp_deg: float
    dur_s: float
    start: float


class EarsMotionPlayer:
    """Compose per-emotion L1 + L2 drift + L3 gesture; tick the driver."""

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
        self._limits: dict[str, tuple[float, float]] = {
            j: (
                float(safety.get(j, {}).get("lo", -90.0)),
                float(safety.get(j, {}).get("hi",  90.0)),
            )
            for j in JOINTS
        }

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
                if g and g.get("token") in ears_gestures.GESTURES:
                    self._gesture = _ActiveGesture(
                        token=str(g["token"]),
                        amp_deg=float(g["amp_deg"]),
                        dur_s=float(g["dur_s"]),
                        start=time.monotonic(),
                    )
        log_event(
            logging.DEBUG,
            "ears_player_set",
            emotion=name, intensity_level=level,
            gesture=(self._gesture.token if self._gesture else None),
        )

    def compute_target(
        self, t: float | None = None,
    ) -> tuple[float, float, float, float]:
        """Pure: emotion + level + gesture → (LP, LT, RP, RT) degrees."""
        t = time.monotonic() if t is None else t
        with self._lock:
            spec = self._emotions[self._emotion]
            level = self._level
            gesture = self._gesture
        l1 = spec["l1"]
        lp = float(l1["left_pan"])
        lt = float(l1["left_tilt"])
        rp = float(l1["right_pan"])
        rt = float(l1["right_tilt"])

        # L2 drift — independent phases per joint so the two ears
        # don't perfectly mirror.
        if level >= 2:
            amp = float(spec.get("l2_drift_deg", 0.3))
            per = float(spec.get("l2_period_s", 5.0))
            ph  = (t - self._start_t) * (2.0 * math.pi / per)
            lp += amp        * math.sin(ph)
            lt += amp * 0.7  * math.cos(ph * 1.13)
            rp += amp        * math.sin(ph * 0.91 + 1.5)
            rt += amp * 0.7  * math.cos(ph * 1.07 + 2.1)

        # L3 peak gesture — additive, finite, decays then clears.
        if gesture is not None:
            dt = t - gesture.start
            if dt >= gesture.dur_s:
                with self._lock:
                    if self._gesture is gesture:
                        self._gesture = None
            else:
                u = dt / gesture.dur_s
                fn = ears_gestures.GESTURES.get(gesture.token)
                if fn is not None:
                    o_lp, o_lt, o_rp, o_rt = fn(u, gesture.amp_deg)
                    lp += o_lp; lt += o_lt; rp += o_rp; rt += o_rt

        return (
            self._clamp("left_pan",   lp),
            self._clamp("left_tilt",  lt),
            self._clamp("right_pan",  rp),
            self._clamp("right_tilt", rt),
        )

    def _clamp(self, joint: str, v: float) -> float:
        lo, hi = self._limits[joint]
        return max(lo, min(hi, v))

    # ── threaded driver tick ──────────────────────────────────

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, name="EarsMotionPlayer", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._thread = None

    def _loop(self) -> None:
        while not self._stop.is_set():
            lp, lt, rp, rt = self.compute_target()
            try:
                self._driver.move_left_pan(lp)
                self._driver.move_left_tilt(lt)
                self._driver.move_right_pan(rp)
                self._driver.move_right_tilt(rt)
            except Exception as exc:                       # noqa: BLE001
                log_event(
                    logging.WARNING, "ears_player_apply_failed",
                    error=repr(exc))
            self._stop.wait(self._tick_dt)
