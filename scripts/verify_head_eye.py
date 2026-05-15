#!/usr/bin/env python3
"""Story 5.3 — Head/eye path hardware verification.

Standalone (non-ROS) script proving HeadI2CClient drives the Head ESP32
(eyes/display) in-process over I2C, validating the Phase 2
delegating-adapter assumption (FR9, AR1).

Run on Pi:
    cd ~/olaf
    PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver:ros2/src/olaf_drivers/neck_driver:libs \
    ~/.local/bin/poetry run python scripts/verify_head_eye.py

Exit code: 0 = sequence completed (operator confirms observed eye/display
output); non-zero = client/I2C fault.

The Head ESP32 owns the 60 FPS eye animation; this script only sends the
seven supported semantic expression strings + blink + look direction.
"""

import sys
import time

from head_ears_driver.head_i2c_client import HeadI2CClient

# Only these 7 expression strings are accepted by the Head firmware.
EXPRESSIONS = ["neutral", "happy", "sad", "surprised", "angry", "sleepy", "wink"]
HOLD = 1.5


def _step(msg: str) -> None:
    print(f"[HEAD] {msg}", flush=True)


def main() -> int:
    _step("Connecting HeadI2CClient (I2C bus 1, addr 0x08)...")
    eyes = HeadI2CClient()
    eyes.open()
    _step("Connected.")
    try:
        for name in EXPRESSIONS:
            _step(f"set_expression('{name}', intensity=4)")
            eyes.set_expression(name, 4)
            time.sleep(HOLD)

        _step("Back to neutral.")
        eyes.set_expression("neutral", 3)
        time.sleep(HOLD)

        _step("trigger_blink()")
        eyes.trigger_blink()
        time.sleep(HOLD)

        _step("look LEFT  (-80, 0)")
        eyes.set_look_direction(-80, 0)
        time.sleep(HOLD)
        _step("look RIGHT (+80, 0)")
        eyes.set_look_direction(80, 0)
        time.sleep(HOLD)
        _step("look UP    (0, +80)")
        eyes.set_look_direction(0, 80)
        time.sleep(HOLD)
        _step("look DOWN  (0, -80)")
        eyes.set_look_direction(0, -80)
        time.sleep(HOLD)
        _step("look CENTER (0, 0)")
        eyes.set_look_direction(0, 0)
        time.sleep(HOLD)

        _step("Sequence complete. Confirm each expression, the blink, and "
              "all four look directions were visible on the display.")
        return 0
    except Exception as exc:  # noqa: BLE001 - report any client/I2C fault
        _step(f"FAILED: {type(exc).__name__}: {exc}")
        return 1
    finally:
        _step("Closing I2C client.")
        eyes.close()


if __name__ == "__main__":
    sys.exit(main())
