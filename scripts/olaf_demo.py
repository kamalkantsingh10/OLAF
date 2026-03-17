#!/usr/bin/env python3
"""OLAF full-body demo — eyes, ears, LEDs, and neck working together.

Coordinates all subsystems to perform a narrative sequence showing
OLAF waking up, reacting to the world, and going back to sleep.

Run on Pi:
    cd ~/olaf
    PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver:ros2/src/olaf_drivers/neck_driver:libs \
    ~/.local/bin/poetry run python scripts/olaf_demo.py
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


# ── Demo Sequences ──────────────────────────────────────────────


SCENES = [
    # ── ACT 1: WAKE UP ──
    {
        "title": "Sleeping",
        "desc": "OLAF is in deep sleep. Everything still.",
        "actions": lambda o: [
            o.status("idle"),
            o.expression("sleepy"),
            o.ear_preset("sleepy", speed_pct=-0.3),
            o.neck_pose(pan=0, tilt=-7, speed=200),
            o.look(0, -50),
        ],
        "hold": 3.0,
    },
    {
        "title": "Wake Up!",
        "desc": "Something stirred. Eyes snap open, ears bolt up, head lifts.",
        "actions": lambda o: [
            o.status("woke_up"),
            o.expression("surprised", 4),
            o.ear_center(),
            o.neck_pose(pan=0, tilt=0, speed=3000),
            o.look(0, 0),
        ],
        "hold": 2.5,
    },
    {
        "title": "Blink Awake",
        "desc": "A few sleepy blinks as OLAF adjusts.",
        "actions": lambda o: [
            o.blink(),
            time.sleep(0.6),
            o.blink(),
            time.sleep(0.4),
            o.expression("neutral"),
            o.neck_pose(pan=5, tilt=2, roll=3, speed=700),
        ],
        "hold": 1.5,
    },

    # ── ACT 2: CURIOSITY ──
    {
        "title": "Hear Something Left",
        "desc": "A sound to the left — ears perk, head snaps, eyes track.",
        "actions": lambda o: [
            o.expression("neutral"),
            o.ear_preset("curious"),
            o.neck_pose(pan=50, tilt=3, roll=-4, speed=2500),
            o.look(-60, 10),
        ],
        "hold": 2.5,
    },
    {
        "title": "Investigate Right",
        "desc": "Now something on the right! Even faster snap.",
        "actions": lambda o: [
            o.expression("surprised", 3),
            o.ear_preset("surprised"),
            o.neck_pose(pan=-50, tilt=3, roll=4, speed=3000),
            o.look(60, 10),
            o.blink(),
        ],
        "hold": 2.5,
    },
    {
        "title": "Look Up",
        "desc": "Something overhead? Chin tilts up, eyes follow.",
        "actions": lambda o: [
            o.expression("surprised", 3),
            o.ear_preset("curious"),
            o.neck_pose(pan=0, tilt=7, speed=1000),
            o.look(0, 60),
        ],
        "hold": 2.0,
    },

    # ── ACT 3: THINKING ──
    {
        "title": "Listening",
        "desc": "OLAF listens carefully. Curious head tilt.",
        "actions": lambda o: [
            o.status("listening"),
            o.expression("neutral"),
            o.ear_preset("thinking"),
            o.neck_pose(pan=8, tilt=0, roll=6, speed=600),
            o.look(0, 0),
        ],
        "hold": 3.0,
    },
    {
        "title": "Processing",
        "desc": "Thinking hard... head drifts, eyes wander.",
        "actions": lambda o: [
            o.status("processing"),
            o.expression("neutral"),
            o.ear_preset("thinking"),
            o.neck_pose(pan=-15, tilt=2, roll=-5, speed=400),
            o.look(30, -20),
        ],
        "hold": 2.5,
    },

    # ── ACT 4: EMOTIONS ──
    {
        "title": "Happy!",
        "desc": "OLAF figured it out! Chin up, quick nod.",
        "actions": lambda o: [
            o.status("speaking"),
            o.expression("happy", 5),
            o.ear_preset("happy"),
            o.neck_pose(pan=0, tilt=6, speed=1200),
            o.look(0, 10),
            time.sleep(0.5),
            o.neck_pose(pan=0, tilt=3, speed=1800),
            time.sleep(0.2),
            o.neck_pose(pan=0, tilt=6, speed=1800),
        ],
        "hold": 2.0,
    },
    {
        "title": "Excited!",
        "desc": "Bouncing with energy. Rapid head snaps.",
        "actions": lambda o: [
            o.status("woke_up"),
            o.expression("happy", 4),
            o.ear_preset("excited"),
            o.neck_pose(pan=45, tilt=4, roll=-3, speed=3000),
            o.look(-30, 0),
            time.sleep(0.35),
            o.neck_pose(pan=-45, tilt=4, roll=3, speed=3000),
            o.look(30, 0),
            time.sleep(0.35),
            o.neck_pose(pan=25, tilt=6, roll=-2, speed=2500),
            o.look(-15, 15),
            time.sleep(0.3),
            o.neck_pose(pan=-25, tilt=6, roll=2, speed=2500),
            o.look(15, 15),
            time.sleep(0.3),
            o.neck_pose(pan=0, tilt=7, speed=2000),
        ],
        "hold": 1.5,
    },
    {
        "title": "Wink",
        "desc": "A cheeky wink. OLAF knows something you don't.",
        "actions": lambda o: [
            o.expression("wink", 4),
            o.neck_pose(pan=-10, tilt=5, roll=7, speed=800),
            o.ear_angles(left_pan=20, left_tilt=5, right_pan=0, right_tilt=0),
        ],
        "hold": 2.0,
    },
    {
        "title": "Confused",
        "desc": "Wait, what? Slow head tilt, asymmetric ears.",
        "actions": lambda o: [
            o.expression("surprised", 3),
            o.ear_preset("confused"),
            o.neck_pose(pan=15, tilt=0, roll=-7, speed=500),
            o.look(-20, 20),
        ],
        "hold": 2.5,
    },
    {
        "title": "Sad",
        "desc": "Something went wrong. Slow heavy head droop.",
        "actions": lambda o: [
            o.expression("sad", 4),
            o.ear_preset("sad"),
            o.neck_pose(pan=0, tilt=-7, speed=200),
            o.look(0, -40),
        ],
        "hold": 3.0,
    },
    {
        "title": "Angry!",
        "desc": "Don't mess with OLAF. Slow menacing chin-down.",
        "actions": lambda o: [
            o.expression("angry", 5),
            o.ear_preset("angry"),
            o.neck_pose(pan=0, tilt=-6, speed=300),
            o.look(0, -10),
        ],
        "hold": 2.5,
    },

    # ── ACT 5: WIND DOWN ──
    {
        "title": "Calm Down",
        "desc": "Deep breath. Slow return to neutral.",
        "actions": lambda o: [
            o.expression("neutral"),
            o.ear_center(),
            o.neck_pose(pan=0, tilt=0, roll=0, speed=400),
            o.look(0, 0),
            time.sleep(0.5),
            o.blink(),
            time.sleep(0.8),
            o.blink(),
        ],
        "hold": 2.0,
    },
    {
        "title": "Getting Sleepy",
        "desc": "Eyes getting heavy... slow droop begins.",
        "actions": lambda o: [
            o.status("going_idle"),
            o.expression("sleepy", 4),
            o.ear_preset("sleepy", intensity=0.7, speed_pct=-0.3),
            o.neck_pose(pan=0, tilt=-5, speed=250),
            o.look(0, -30),
            time.sleep(1),
            o.blink(),
        ],
        "hold": 3.0,
    },
    {
        "title": "Asleep",
        "desc": "Goodnight, OLAF. Full droop, eyes closed, everything still.",
        "actions": lambda o: [
            o.status("idle"),
            o.expression("sleepy", 5),
            o.ear_preset("sleepy", speed_pct=-0.3),
            o.neck_pose(pan=0, tilt=-7, speed=200),
            o.look(0, -80),
        ],
        "hold": 3.0,
    },
]


def run_demo():
    print("=" * 60)
    print("  OLAF Full-Body Demo")
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

        # Final reset
        print("[--] Returning to neutral...")
        olaf.status("idle")
        olaf.expression("neutral")
        olaf.ear_center()
        olaf.neck_center(speed=200)
        olaf.look(0, 0)
        time.sleep(1)

    except KeyboardInterrupt:
        print("\n[--] Interrupted. Resetting...")
        olaf.status("idle")
        olaf.ear_center()
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
