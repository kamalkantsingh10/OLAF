"""Real neck ContinuousAdapter — Story 6.4 Task 2 (wraps NeckServoDriver).

Implements the frozen `ContinuousAdapter` Protocol (§4). The render
loop (Story 6.3) has already eased the per-tick absolute target; this
adapter just issues it — it does NOT ease (AR1).

Driver mapping (verbatim, §4 / story Dev Notes):
  apply({"pan","tilt","roll"}) → NeckServoDriver.move_pose(pan,tilt,roll)
  neutral()                    → {pan:0, tilt:0, roll:0}
  connect/close                → driver ctor / .close()

The driver's serial port opens in its ctor, so `connect()` constructs
it (lazily imported — importing this module needs no hardware/serial).
A `driver_factory` is injectable for hardware-free unit tests.
"""

from __future__ import annotations

import logging
from typing import Callable, Optional

from expression_engine.logging_setup import log_event

_JOINTS = ("pan", "tilt", "roll")

# Defensive safety envelope (deg). Code review 2026-05-17: the render
# loop sums activity + mood + speech + stacked gestures (nod ±14 /
# shake ±16) with no upper bound; the ears adapter clamps, the neck
# did not. The STS3215 linkage tilt/roll are mechanically limited to
# ±20° (config/servo-ids.yaml); pan to ±90 (kept ≤80 conservative).
# The driver also clamps the linkage internally — this is
# defence-in-depth + an operator-visible warning, mirroring ears.
_LIMITS = {
    "pan": (-80.0, 80.0),
    # tilt raised to ±28 (Story 7.3, Kamal-confirmed the linkage reaches
    # 28° without binding) so the sleep pose can droop deeper. Matches
    # config/servo-ids.yaml linkage min/max ±28.
    "tilt": (-28.0, 28.0),
    "roll": (-15.0, 15.0),
}


# Per-joint "currently clamping" flags so the clamp warning logs ONCE
# per out-of-range episode instead of every ~100Hz tick. Story 6.5: the
# idle micro-drift nudges the neck past the tilt limit while it rests at
# the full sleep droop (-28), which otherwise floods journald.
_clamp_warned: dict[str, bool] = {}


def _clamp(joint: str, deg: float) -> float:
    lo, hi = _LIMITS[joint]
    c = max(lo, min(hi, deg))
    if c != deg:
        if not _clamp_warned.get(joint):
            log_event(
                logging.WARNING,
                "neck_target_clamped",
                joint=joint,
                requested=round(deg, 2),
                clamped=c,
                note="further clamps for this joint suppressed until in-range",
            )
            _clamp_warned[joint] = True
    else:
        _clamp_warned[joint] = False
    return c


def _default_driver_factory():
    # Lazy import — only when actually connecting to hardware.
    from neck_driver.neck_servo_driver import NeckServoDriver

    return NeckServoDriver()


class NeckAdapter:
    """`ContinuousAdapter` over `NeckServoDriver` (neck pan/tilt/roll)."""

    def __init__(
        self, driver_factory: Optional[Callable[[], object]] = None
    ) -> None:
        self._factory = driver_factory or _default_driver_factory
        self._driver = None

    def connect(self) -> None:
        # Fatal on failure (NFR7) — startup must not continue if the
        # neck cannot be driven.
        self._driver = self._factory()
        log_event(logging.INFO, "neck_adapter_connected")

    def close(self) -> None:
        if self._driver is not None:
            self._driver.close()
            self._driver = None

    def apply(self, targets: dict[str, float], speed_pct: float = 0.0) -> None:
        if self._driver is None:
            return
        # speed_pct is unused for the neck: the loop already eased the
        # per-tick delta; the driver uses its configured default speed.
        self._driver.move_pose(
            pan=_clamp("pan", float(targets.get("pan", 0.0))),
            tilt=_clamp("tilt", float(targets.get("tilt", 0.0))),
            roll=_clamp("roll", float(targets.get("roll", 0.0))),
        )

    def neutral(self) -> dict[str, float]:
        return {j: 0.0 for j in _JOINTS}
