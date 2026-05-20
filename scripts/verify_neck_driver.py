#!/usr/bin/env python3
"""Story 5.3 — Neck driver hardware verification.

Standalone (non-ROS) script proving NeckServoDriver moves real hardware
in-process, validating the Phase 2 in-process-adapter assumption (FR9).

Run on Pi:
    cd ~/olaf
    PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver:ros2/src/olaf_drivers/neck_driver:libs \
    ~/.local/bin/poetry run python scripts/verify_neck_driver.py

Exit code: 0 = sequence completed (operator confirms observed motion);
non-zero = driver/hardware fault.

Expected observable motion:
    pan +  -> OLAF looks LEFT      pan -  -> looks RIGHT
    tilt + -> chin UP              tilt - -> chin DOWN
    roll + -> head tilts to OLAF's RIGHT
The driver clamps to config/servo-ids.yaml limits; angles here are
deliberately conservative.
"""

import sys
import time

from neck_driver.neck_servo_driver import NeckServoDriver

# Conservative sweep (deg). Driver clamps to servo-ids.yaml limits regardless.
PAN = 12.0
TILT = 10.0
ROLL = 8.0
SETTLE = 1.2  # seconds between moves, so motion is human-observable


def _step(msg: str) -> None:
    print(f"[NECK] {msg}", flush=True)


def main() -> int:
    _step("Connecting NeckServoDriver (opens /dev/waveshare_neck)...")
    neck = NeckServoDriver()
    _step("Connected.")
    try:
        _step("Center.")
        neck.center_all()
        time.sleep(SETTLE)

        _step(f"Pan +{PAN} (look LEFT)")
        neck.move_pose(pan=PAN)
        time.sleep(SETTLE)
        _step(f"Pan -{PAN} (look RIGHT)")
        neck.move_pose(pan=-PAN)
        time.sleep(SETTLE)

        _step(f"Tilt +{TILT} (chin UP)")
        neck.move_pose(tilt=TILT)
        time.sleep(SETTLE)
        _step(f"Tilt -{TILT} (chin DOWN)")
        neck.move_pose(tilt=-TILT)
        time.sleep(SETTLE)

        _step(f"Roll +{ROLL} (tilt head to OLAF's RIGHT)")
        neck.move_pose(roll=ROLL)
        time.sleep(SETTLE)
        _step(f"Roll -{ROLL} (tilt head to OLAF's LEFT)")
        neck.move_pose(roll=-ROLL)
        time.sleep(SETTLE)

        _step("Combined pose: pan+ tilt+ roll+")
        neck.move_pose(pan=PAN, tilt=TILT, roll=ROLL)
        time.sleep(SETTLE)

        _step("Return to center.")
        neck.center_all()
        time.sleep(SETTLE)
        _step("Sequence complete. Confirm motion matched the printed steps.")
        return 0
    except Exception as exc:  # noqa: BLE001 - report any driver/hw fault
        _step(f"FAILED: {type(exc).__name__}: {exc}")
        return 1
    finally:
        _step("Closing serial port.")
        neck.close()


if __name__ == "__main__":
    sys.exit(main())
