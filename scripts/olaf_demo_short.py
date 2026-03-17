#!/usr/bin/env python3
"""OLAF short demo — 75-second highlight reel.

Wake up → curious scan → happy interaction → sleepy wind-down.

Run on Pi:
    cd ~/olaf
    PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver:ros2/src/olaf_drivers/neck_driver:libs \
    ~/.local/bin/poetry run python scripts/olaf_demo_short.py
"""

import sys
import time

from head_ears_driver.head_i2c_client import HeadI2CClient
from head_ears_driver.ears_servo_driver import EarsServoDriver
from head_ears_driver.expressions import get_preset_by_name
from neck_driver.neck_servo_driver import NeckServoDriver


class OlafDemo:
    """Orchestrates eyes, ears, LEDs, and neck for coordinated demo."""

    def __init__(self):
        print("[INIT] Connecting subsystems...")
        self.eyes = HeadI2CClient()
        self.eyes.open()
        print("  Eyes (I2C 0x08): OK")

        self.ears = EarsServoDriver()
        print("  Ears (serial): OK")

        self.neck = NeckServoDriver()
        print("  Neck (serial): OK")

        print("[INIT] All systems ready.\n")

    def close(self):
        self.eyes.close()
        self.ears.close()
        self.neck.close()

    # -- Convenience wrappers --

    def expression(self, name: str, intensity: int = 3):
        self.eyes.set_expression(name, intensity)

    def status(self, name: str):
        self.eyes.set_system_status(name)

    def blink(self):
        self.eyes.trigger_blink()

    def look(self, x: int, y: int):
        self.eyes.set_look_direction(x, y)

    def ear_preset(self, name: str, intensity: float = 1.0, speed_pct: float = 0.0):
        angles = get_preset_by_name(name, intensity)
        if angles:
            for servo, deg in angles.items():
                self.ears.move(servo, deg, speed_pct)

    def ear_angles(self, left_pan=0, left_tilt=0, right_pan=0, right_tilt=0,
                   speed_pct=0.0):
        self.ears.move("left_pan", left_pan, speed_pct)
        self.ears.move("left_tilt", left_tilt, speed_pct)
        self.ears.move("right_pan", right_pan, speed_pct)
        self.ears.move("right_tilt", right_tilt, speed_pct)

    def ear_center(self):
        self.ears.center_all()

    def neck_pose(self, pan=0.0, tilt=0.0, roll=0.0, speed=None):
        self.neck.move_pose(pan, tilt, roll, speed)

    def neck_center(self, speed=None):
        self.neck.center_all(speed)


# ── Demo Scenes ─────────────────────────────────────────────────

SCENES = [
    # ── WAKE UP (~14 s) ──
    {
        "title": "Sleeping",
        "desc": "Deep sleep. Head down, ears drooped.",
        "actions": lambda o: [
            o.status("idle"),
            o.expression("sleepy", 5),
            o.ear_preset("sleepy", speed_pct=-0.3),
            o.neck_pose(pan=0, tilt=-15, speed=200),
            o.look(0, -80),
        ],
        "hold": 5.0,
    },
    {
        "title": "Wake Up!",
        "desc": "Eyes snap open, ears bolt up, head lifts.",
        "actions": lambda o: [
            o.status("woke_up"),
            o.expression("surprised", 4),
            o.ear_center(),
            o.neck_pose(pan=0, tilt=0, speed=3000),
            o.look(0, 0),
        ],
        "hold": 3.0,
    },
    {
        "title": "Blink Awake",
        "desc": "Sleepy blinks, settling in.",
        "actions": lambda o: [
            o.blink(),
            time.sleep(0.7),
            o.blink(),
            time.sleep(0.5),
            o.expression("neutral"),
            o.neck_pose(pan=5, tilt=3, roll=4, speed=700),
            time.sleep(0.8),
            o.blink(),
        ],
        "hold": 2.5,
    },

    # ── CURIOSITY (~14 s) ──
    {
        "title": "What's That?",
        "desc": "Something to the left — quick snap.",
        "actions": lambda o: [
            o.ear_preset("curious"),
            o.neck_pose(pan=50, tilt=4, roll=-5, speed=2500),
            o.look(-60, 10),
        ],
        "hold": 3.5,
    },
    {
        "title": "Over There!",
        "desc": "Now to the right — even faster.",
        "actions": lambda o: [
            o.expression("surprised", 3),
            o.ear_preset("surprised"),
            o.neck_pose(pan=-50, tilt=4, roll=5, speed=3000),
            o.look(60, 10),
            time.sleep(0.4),
            o.blink(),
        ],
        "hold": 3.0,
    },
    {
        "title": "Look Up",
        "desc": "Something above? Big chin-up tilt.",
        "actions": lambda o: [
            o.ear_preset("curious"),
            o.neck_pose(pan=0, tilt=15, speed=1500),
            o.look(0, 60),
        ],
        "hold": 2.5,
    },
    {
        "title": "Confused Head Tilt",
        "desc": "Classic dog head tilt. What is it?",
        "actions": lambda o: [
            o.expression("neutral"),
            o.ear_preset("confused"),
            o.neck_pose(pan=10, tilt=2, roll=-10, speed=600),
            o.look(-15, 15),
        ],
        "hold": 3.5,
    },

    # ── INTERACTION (~30 s) ──
    {
        "title": "Saying No",
        "desc": "Nope! Firm head shake side to side.",
        "actions": lambda o: [
            o.expression("angry", 3),
            o.ear_preset("angry", intensity=0.5),
            o.neck_pose(pan=35, tilt=-3, roll=-4, speed=2500),
            o.look(-20, 0),
            time.sleep(0.3),
            o.neck_pose(pan=-35, tilt=-3, roll=4, speed=2500),
            o.look(20, 0),
            time.sleep(0.3),
            o.neck_pose(pan=30, tilt=-2, roll=-3, speed=2500),
            o.look(-15, 0),
            time.sleep(0.3),
            o.neck_pose(pan=-30, tilt=-2, roll=3, speed=2500),
            o.look(15, 0),
            time.sleep(0.3),
            o.neck_pose(pan=0, tilt=0, speed=1500),
            o.look(0, 0),
        ],
        "hold": 3.0,
    },
    {
        "title": "Listening",
        "desc": "Someone's talking. OLAF pays attention.",
        "actions": lambda o: [
            o.status("listening"),
            o.expression("neutral"),
            o.ear_preset("thinking"),
            o.neck_pose(pan=0, tilt=3, roll=5, speed=500),
            o.look(0, 0),
        ],
        "hold": 4.0,
    },
    {
        "title": "Saying Yes!",
        "desc": "Enthusiastic nodding — understood!",
        "actions": lambda o: [
            o.expression("happy", 4),
            o.ear_preset("happy"),
            o.neck_pose(pan=0, tilt=10, speed=2000),
            o.look(0, 10),
            time.sleep(0.3),
            o.neck_pose(pan=0, tilt=-5, speed=2000),
            o.look(0, -5),
            time.sleep(0.3),
            o.neck_pose(pan=0, tilt=10, speed=2000),
            o.look(0, 10),
            time.sleep(0.3),
            o.neck_pose(pan=0, tilt=-5, speed=2000),
            time.sleep(0.3),
            o.neck_pose(pan=0, tilt=8, speed=1500),
        ],
        "hold": 2.0,
    },
    {
        "title": "Happy!",
        "desc": "Big smile, chin up, LEDs light up!",
        "actions": lambda o: [
            o.status("speaking"),
            o.expression("happy", 5),
            o.ear_preset("happy"),
            o.neck_pose(pan=0, tilt=12, speed=1200),
            o.look(0, 10),
            time.sleep(0.6),
            o.neck_pose(pan=0, tilt=5, speed=1800),
            time.sleep(0.25),
            o.neck_pose(pan=0, tilt=12, speed=1800),
        ],
        "hold": 3.5,
    },
    {
        "title": "Happy Look Around",
        "desc": "Sharing the joy — looking at everyone.",
        "actions": lambda o: [
            o.expression("happy", 4),
            o.neck_pose(pan=35, tilt=8, roll=-4, speed=1000),
            o.look(-25, 10),
            time.sleep(1.5),
            o.neck_pose(pan=-35, tilt=8, roll=4, speed=1000),
            o.look(25, 10),
            time.sleep(1.2),
            o.blink(),
        ],
        "hold": 2.5,
    },
    {
        "title": "Cheeky Wink",
        "desc": "A sly wink and head tilt.",
        "actions": lambda o: [
            o.status("woke_up"),
            o.expression("wink", 4),
            o.neck_pose(pan=-10, tilt=8, roll=10, speed=800),
            o.ear_angles(left_pan=20, left_tilt=-10, right_pan=0, right_tilt=0),
        ],
        "hold": 3.5,
    },

    {
        "title": "Excited Bounce",
        "desc": "Can't contain it! Quick side-to-side snaps.",
        "actions": lambda o: [
            o.expression("happy", 4),
            o.ear_preset("excited"),
            o.neck_pose(pan=40, tilt=6, roll=-4, speed=3000),
            o.look(-25, 5),
            time.sleep(0.35),
            o.neck_pose(pan=-40, tilt=6, roll=4, speed=3000),
            o.look(25, 5),
            time.sleep(0.35),
            o.neck_pose(pan=20, tilt=10, roll=-2, speed=2500),
            o.look(-10, 10),
            time.sleep(0.3),
            o.neck_pose(pan=-20, tilt=10, roll=2, speed=2500),
            o.look(10, 10),
            time.sleep(0.3),
            o.neck_pose(pan=0, tilt=12, speed=2000),
            o.look(0, 15),
        ],
        "hold": 2.5,
    },

    # ── WIND DOWN (~15 s) ──
    {
        "title": "Calm",
        "desc": "Settling down. Gentle return to center.",
        "actions": lambda o: [
            o.expression("neutral"),
            o.ear_center(),
            o.neck_pose(pan=0, tilt=0, roll=0, speed=400),
            o.look(0, 0),
            time.sleep(1.0),
            o.blink(),
            time.sleep(0.8),
            o.blink(),
        ],
        "hold": 3.0,
    },
    {
        "title": "Sleepy",
        "desc": "Eyes getting heavy... slow droop.",
        "actions": lambda o: [
            o.status("going_idle"),
            o.expression("sleepy", 4),
            o.ear_preset("sleepy", intensity=0.7, speed_pct=-0.3),
            o.neck_pose(pan=0, tilt=-10, roll=2, speed=200),
            o.look(0, -40),
            time.sleep(1.2),
            o.blink(),
        ],
        "hold": 3.5,
    },
    {
        "title": "Asleep",
        "desc": "Goodnight, OLAF.",
        "actions": lambda o: [
            o.status("idle"),
            o.expression("sleepy", 5),
            o.ear_preset("sleepy", speed_pct=-0.3),
            o.neck_pose(pan=0, tilt=-15, speed=150),
            o.look(0, -80),
        ],
        "hold": 5.0,
    },
]


def run_demo():
    print("=" * 60)
    print("  OLAF Short Demo (75s)")
    print("  Eyes + Ears + LEDs + Neck")
    print("=" * 60)
    print()

    olaf = OlafDemo()

    total = len(SCENES)
    try:
        for i, scene in enumerate(SCENES, 1):
            print(f"[{i:2d}/{total}] {scene['title']}")
            print(f"       {scene['desc']}")

            scene["actions"](olaf)
            time.sleep(scene["hold"])
            print()

        # Final reset — keep ears in sleepy droop
        print("[--] Done.")
        olaf.status("idle")
        olaf.neck_center(speed=200)
        time.sleep(1)

    except KeyboardInterrupt:
        print("\n[--] Interrupted. Resetting...")
        olaf.status("idle")
        olaf.neck_center(speed=200)
        time.sleep(1)

    finally:
        olaf.close()

    print()
    print("=" * 60)
    print("  Demo complete.")
    print("=" * 60)


if __name__ == "__main__":
    try:
        run_demo()
    except ConnectionError as e:
        print(f"[ERROR] {e}")
        print("Check: power on? All adapters connected?")
        sys.exit(1)
    except OSError as e:
        print(f"[ERROR] I2C/serial failure: {e}")
        sys.exit(1)
