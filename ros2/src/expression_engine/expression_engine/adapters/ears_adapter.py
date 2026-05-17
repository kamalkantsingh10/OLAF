"""Real ears ContinuousAdapter — Story 6.4 Task 2 (wraps EarsServoDriver).

Implements the frozen `ContinuousAdapter` Protocol (§4). Per-tick
absolute targets, no easing here (AR1).

Driver mapping (verbatim, §4):
  apply({"left_pan","left_tilt","right_pan","right_tilt"}) →
      move_left_pan/left_tilt/right_pan/right_tilt(deg, speed_pct)
  neutral() → all-zero (driver center)
  connect/close → driver ctor / .close()

⚠️ Safety clamp: `right_pan` binds ≥65° on the SCS0009 (project memory
/ servo-ids.yaml). Story Dev Notes mandate keeping it ≤50°. The
adapter clamps every joint to a safe envelope and logs when it does —
a binding servo is a hardware hazard, so safety wins over fidelity.
"""

from __future__ import annotations

import logging
from typing import Callable, Optional

from expression_engine.logging_setup import log_event

_JOINTS = ("left_pan", "left_tilt", "right_pan", "right_tilt")

# Safe per-joint envelopes (deg from centre). pan ≤50 (right_pan binds
# ≥65 — project memory); tilt range -90..+110 per driver docstrings,
# kept conservative for the reference run.
_LIMITS = {
    "left_pan": (-50.0, 50.0),
    "right_pan": (-50.0, 50.0),
    "left_tilt": (-60.0, 90.0),
    "right_tilt": (-60.0, 90.0),
}


def _default_driver_factory():
    from head_ears_driver.ears_servo_driver import EarsServoDriver

    return EarsServoDriver()


def _clamp(joint: str, deg: float) -> float:
    lo, hi = _LIMITS[joint]
    c = max(lo, min(hi, deg))
    if c != deg:
        log_event(
            logging.WARNING,
            "ears_target_clamped",
            joint=joint,
            requested=deg,
            clamped=c,
        )
    return c


class EarsAdapter:
    """`ContinuousAdapter` over `EarsServoDriver` (4 ear joints)."""

    def __init__(
        self, driver_factory: Optional[Callable[[], object]] = None
    ) -> None:
        self._factory = driver_factory or _default_driver_factory
        self._driver = None

    def connect(self) -> None:
        self._driver = self._factory()
        log_event(logging.INFO, "ears_adapter_connected")

    def close(self) -> None:
        if self._driver is not None:
            self._driver.close()
            self._driver = None

    def apply(self, targets: dict[str, float], speed_pct: float = 0.0) -> None:
        if self._driver is None:
            return
        d = self._driver
        d.move_left_pan(_clamp("left_pan", float(targets.get("left_pan", 0.0))), speed_pct)
        d.move_left_tilt(_clamp("left_tilt", float(targets.get("left_tilt", 0.0))), speed_pct)
        d.move_right_pan(_clamp("right_pan", float(targets.get("right_pan", 0.0))), speed_pct)
        d.move_right_tilt(_clamp("right_tilt", float(targets.get("right_tilt", 0.0))), speed_pct)

    def neutral(self) -> dict[str, float]:
        return {j: 0.0 for j in _JOINTS}
