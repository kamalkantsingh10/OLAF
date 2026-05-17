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

The Head ESP32 owns the 60 FPS eye animation; this script sends a
'woke_up' system status (the firmware boots asleep and suppresses all
expression/blink until awake), the seven supported semantic expression
strings + blink + look direction, then 'going_idle'.
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
        # Head firmware boots ASLEEP (animation_engine.cpp:87 "Engine ready
        # — asleep", wake_level_=0). Expressions are suppressed until
        # wake_level_ > 0.8 (:175) and blink until > 0.5 (:186), driven by
        # set_system_status('woke_up'). Without this every expression/blink/
        # look is received over I2C but deliberately ignored and the eyes
        # render shut. This wake step is required for AC #3 to be observable.
        _step("set_system_status('woke_up') — open the eyes")
        eyes.set_system_status("woke_up")
        time.sleep(1.5)  # WAKE_MS transition → wake_level_ climbs past 0.8

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

        _step("set_system_status('going_idle') — eyes close gracefully")
        eyes.set_system_status("going_idle")
        time.sleep(1.5)

        _step("Sequence complete. Confirm: eyes OPENED on wake, each "
              "expression + the blink + all four look directions were "
              "visible, then eyes CLOSED on going_idle.")
        return 0
    except Exception as exc:  # noqa: BLE001 - report any client/I2C fault
        _step(f"FAILED: {type(exc).__name__}: {exc}")
        return 1
    finally:
        _step("Closing I2C client.")
        eyes.close()


if __name__ == "__main__":
    sys.exit(main())
