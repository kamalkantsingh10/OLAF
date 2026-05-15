#!/usr/bin/env python3
"""Story 5.3 — Ears driver hardware verification.

Standalone (non-ROS) script proving EarsServoDriver moves real hardware
in-process, validating the Phase 2 in-process-adapter assumption (FR9).

Run on Pi:
    cd ~/olaf
    PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver:ros2/src/olaf_drivers/neck_driver:libs \
    ~/.local/bin/poetry run python scripts/verify_ears_driver.py

Exit code: 0 = sequence completed (operator confirms observed motion);
non-zero = driver/hardware fault.

SAFETY: right_pan binds at >=65 deg (center ~241, direction -1). The
sweep below caps pan at 45 deg, well inside the documented safe range.
The driver also clamps to config/servo-ids.yaml min/max per servo.
"""

import sys
import time

from head_ears_driver.ears_servo_driver import EarsServoDriver

PAN = 45.0   # outward; capped below the ~55 deg right_pan binding limit
TILT = 15.0  # forward; modest, driver clamps to per-ear limits
SETTLE = 1.2


def _step(msg: str) -> None:
    print(f"[EARS] {msg}", flush=True)


def main() -> int:
    _step("Connecting EarsServoDriver (opens /dev/waveshare_ears)...")
    ears = EarsServoDriver()
    _step("Connected.")
    try:
        _step("Center all.")
        ears.center_all()
        time.sleep(SETTLE)

        _step(f"Left pan +{PAN} (left ear OUT)")
        ears.move_left_pan(PAN)
        time.sleep(SETTLE)
        _step(f"Right pan +{PAN} (right ear OUT)")
        ears.move_right_pan(PAN)
        time.sleep(SETTLE)

        _step(f"Left tilt +{TILT} (left ear FORWARD)")
        ears.move_left_tilt(TILT)
        time.sleep(SETTLE)
        _step(f"Right tilt +{TILT} (right ear FORWARD)")
        ears.move_right_tilt(TILT)
        time.sleep(SETTLE)

        _step("Asymmetric: left out / right in (perk test)")
        ears.move_left_pan(PAN)
        ears.move_right_pan(0.0)
        time.sleep(SETTLE)

        _step("Center all.")
        ears.center_all()
        time.sleep(SETTLE)
        _step("Sequence complete. Confirm motion matched the printed steps "
              "and that NEITHER ear bound/stalled.")
        return 0
    except Exception as exc:  # noqa: BLE001 - report any driver/hw fault
        _step(f"FAILED: {type(exc).__name__}: {exc}")
        return 1
    finally:
        _step("Closing serial port.")
        ears.close()


if __name__ == "__main__":
    sys.exit(main())
