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
    # ══════════════════════════════════════════════════════════════
    # ACT 1: DEEP SLEEP → WAKE  (~18 s)
    # ══════════════════════════════════════════════════════════════
    {
        "title": "Deep Sleep",
        "desc": "OLAF is powered down. Everything still and dark.",
        "actions": lambda o: [
            o.status("idle"),
            o.expression("sleepy", 5),
            o.ear_preset("sleepy", speed_pct=-0.3),
            o.neck_pose(pan=0, tilt=-7, speed=200),
            o.look(0, -80),
        ],
        "hold": 5.0,
    },
    {
        "title": "Stir",
        "desc": "A faint sound. Ears twitch, head shifts slightly.",
        "actions": lambda o: [
            o.ear_angles(left_pan=40, left_tilt=-30, right_pan=50, right_tilt=-7,
                         speed_pct=-0.2),
            o.neck_pose(pan=3, tilt=-6, speed=150),
            time.sleep(1.5),
            o.ear_preset("sleepy", speed_pct=-0.3),
            o.neck_pose(pan=-2, tilt=-7, speed=150),
        ],
        "hold": 3.0,
    },
    {
        "title": "Wake Up!",
        "desc": "Eyes snap open! Ears bolt upright, head lifts fast.",
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
        "desc": "Sleepy blinks. Eyes adjusting to the light.",
        "actions": lambda o: [
            o.blink(),
            time.sleep(0.7),
            o.blink(),
            time.sleep(0.5),
            o.expression("neutral"),
            o.neck_pose(pan=5, tilt=2, roll=3, speed=700),
            time.sleep(0.8),
            o.blink(),
        ],
        "hold": 2.5,
    },

    # ══════════════════════════════════════════════════════════════
    # ACT 2: CURIOSITY & SCANNING  (~28 s)
    # ══════════════════════════════════════════════════════════════
    {
        "title": "Hear Something Left",
        "desc": "A sound to the left — ears perk, head snaps, eyes track.",
        "actions": lambda o: [
            o.expression("neutral"),
            o.ear_preset("curious"),
            o.neck_pose(pan=50, tilt=3, roll=-4, speed=2500),
            o.look(-60, 10),
        ],
        "hold": 3.5,
    },
    {
        "title": "Slow Scan Right",
        "desc": "Tracking the sound... slow deliberate head pan.",
        "actions": lambda o: [
            o.ear_preset("thinking"),
            o.neck_pose(pan=-30, tilt=2, roll=2, speed=400),
            o.look(-30, 5),
            time.sleep(1.0),
            o.look(0, 5),
            time.sleep(1.0),
            o.look(30, 5),
        ],
        "hold": 2.5,
    },
    {
        "title": "Snap Right!",
        "desc": "Something on the right! Fast snap with startled ears.",
        "actions": lambda o: [
            o.expression("surprised", 4),
            o.ear_preset("surprised"),
            o.neck_pose(pan=-50, tilt=3, roll=4, speed=3000),
            o.look(60, 10),
            time.sleep(0.4),
            o.blink(),
        ],
        "hold": 3.0,
    },
    {
        "title": "Look Up",
        "desc": "Something overhead? Chin tilts up, eyes follow.",
        "actions": lambda o: [
            o.expression("surprised", 3),
            o.ear_preset("curious"),
            o.neck_pose(pan=0, tilt=7, speed=1200),
            o.look(0, 60),
        ],
        "hold": 3.0,
    },
    {
        "title": "Double Take Left",
        "desc": "Wait — was that the same thing? Quick glance back left.",
        "actions": lambda o: [
            o.expression("neutral"),
            o.ear_preset("confused"),
            o.neck_pose(pan=40, tilt=2, roll=-3, speed=2800),
            o.look(-50, 0),
            time.sleep(0.6),
            o.blink(),
            time.sleep(0.5),
            o.neck_pose(pan=25, tilt=3, roll=-5, speed=600),
            o.look(-30, 10),
        ],
        "hold": 3.0,
    },
    {
        "title": "Back to Center",
        "desc": "Nothing there. Slow return, confused head tilt.",
        "actions": lambda o: [
            o.expression("neutral"),
            o.ear_center(),
            o.neck_pose(pan=0, tilt=0, roll=0, speed=500),
            o.look(0, 0),
            time.sleep(0.8),
            o.blink(),
        ],
        "hold": 2.5,
    },

    # ══════════════════════════════════════════════════════════════
    # ACT 3: LISTENING & THINKING  (~20 s)
    # ══════════════════════════════════════════════════════════════
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
        "hold": 4.0,
    },
    {
        "title": "Processing",
        "desc": "Thinking hard... head drifts left, eyes wander.",
        "actions": lambda o: [
            o.status("processing"),
            o.expression("neutral"),
            o.ear_preset("thinking"),
            o.neck_pose(pan=-15, tilt=2, roll=-5, speed=400),
            o.look(30, -20),
        ],
        "hold": 3.5,
    },
    {
        "title": "Still Thinking...",
        "desc": "Head drifts the other way. Deep in thought.",
        "actions": lambda o: [
            o.neck_pose(pan=12, tilt=1, roll=4, speed=350),
            o.look(-20, -10),
            time.sleep(1.5),
            o.look(-30, 10),
            o.neck_pose(pan=18, tilt=2, roll=6, speed=300),
        ],
        "hold": 3.5,
    },
    {
        "title": "Eureka!",
        "desc": "Got it! Quick upward snap — idea moment.",
        "actions": lambda o: [
            o.expression("surprised", 3),
            o.ear_center(),
            o.neck_pose(pan=0, tilt=7, speed=2500),
            o.look(0, 20),
            time.sleep(0.5),
            o.blink(),
        ],
        "hold": 2.5,
    },

    # ══════════════════════════════════════════════════════════════
    # ACT 4: EMOTIONS — HAPPY ARC  (~22 s)
    # ══════════════════════════════════════════════════════════════
    {
        "title": "Happy!",
        "desc": "OLAF figured it out! Chin up, joyful nod.",
        "actions": lambda o: [
            o.status("speaking"),
            o.expression("happy", 5),
            o.ear_preset("happy"),
            o.neck_pose(pan=0, tilt=6, speed=1200),
            o.look(0, 10),
            time.sleep(0.6),
            o.neck_pose(pan=0, tilt=3, speed=1800),
            time.sleep(0.25),
            o.neck_pose(pan=0, tilt=6, speed=1800),
        ],
        "hold": 3.0,
    },
    {
        "title": "Happy Look Around",
        "desc": "Sharing the joy — looking left and right with a grin.",
        "actions": lambda o: [
            o.expression("happy", 4),
            o.ear_preset("happy"),
            o.neck_pose(pan=30, tilt=5, roll=-3, speed=1000),
            o.look(-20, 10),
            time.sleep(1.2),
            o.neck_pose(pan=-30, tilt=5, roll=3, speed=1000),
            o.look(20, 10),
            time.sleep(1.0),
            o.blink(),
        ],
        "hold": 2.5,
    },
    {
        "title": "Excited Bounce!",
        "desc": "Can't contain it! Rapid head snaps side to side.",
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
        "hold": 2.0,
    },
    {
        "title": "Cheeky Wink",
        "desc": "A sly wink. OLAF knows something you don't.",
        "actions": lambda o: [
            o.expression("wink", 4),
            o.neck_pose(pan=-10, tilt=5, roll=7, speed=800),
            o.ear_angles(left_pan=20, left_tilt=5, right_pan=0, right_tilt=0),
        ],
        "hold": 3.0,
    },

    # ══════════════════════════════════════════════════════════════
    # ACT 5: EMOTIONS — NEGATIVE ARC  (~25 s)
    # ══════════════════════════════════════════════════════════════
    {
        "title": "Confused",
        "desc": "Wait, what? Slow head tilt, asymmetric ears.",
        "actions": lambda o: [
            o.expression("surprised", 3),
            o.ear_preset("confused"),
            o.neck_pose(pan=15, tilt=0, roll=-7, speed=500),
            o.look(-20, 20),
        ],
        "hold": 3.5,
    },
    {
        "title": "More Confused",
        "desc": "Tilts the other way. Still doesn't get it.",
        "actions": lambda o: [
            o.ear_angles(left_pan=55, left_tilt=5, right_pan=0, right_tilt=20),
            o.neck_pose(pan=-12, tilt=0, roll=7, speed=500),
            o.look(20, 15),
            time.sleep(1.0),
            o.blink(),
        ],
        "hold": 3.0,
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
        "hold": 4.0,
    },
    {
        "title": "Deeper Sadness",
        "desc": "Head sinks further. A slow blink.",
        "actions": lambda o: [
            o.expression("sad", 5),
            o.ear_preset("sad", intensity=0.8),
            o.neck_pose(pan=-5, tilt=-7, roll=-2, speed=150),
            o.look(-10, -50),
            time.sleep(1.5),
            o.blink(),
        ],
        "hold": 3.0,
    },
    {
        "title": "Angry!",
        "desc": "Enough! Slow menacing chin-down, ears pinned back.",
        "actions": lambda o: [
            o.expression("angry", 5),
            o.ear_preset("angry"),
            o.neck_pose(pan=0, tilt=-6, speed=300),
            o.look(0, -10),
        ],
        "hold": 3.5,
    },
    {
        "title": "Angry Glare",
        "desc": "Locked on. Slow tracking glare left to right.",
        "actions": lambda o: [
            o.expression("angry", 4),
            o.ear_preset("angry", intensity=0.8),
            o.neck_pose(pan=20, tilt=-5, roll=-2, speed=300),
            o.look(-30, -10),
            time.sleep(1.5),
            o.neck_pose(pan=-20, tilt=-5, roll=2, speed=300),
            o.look(30, -10),
        ],
        "hold": 3.0,
    },

    # ══════════════════════════════════════════════════════════════
    # ACT 6: RECOVERY & WIND DOWN  (~25 s)
    # ══════════════════════════════════════════════════════════════
    {
        "title": "Deep Breath",
        "desc": "Anger fades. Slow exhale, head rises to center.",
        "actions": lambda o: [
            o.expression("neutral"),
            o.ear_center(),
            o.neck_pose(pan=0, tilt=-3, roll=0, speed=300),
            o.look(0, -10),
            time.sleep(1.5),
            o.neck_pose(pan=0, tilt=0, speed=300),
            o.look(0, 0),
            time.sleep(0.8),
            o.blink(),
        ],
        "hold": 2.5,
    },
    {
        "title": "Gentle Curiosity",
        "desc": "Life goes on. A soft glance around, ears relaxed.",
        "actions": lambda o: [
            o.expression("neutral"),
            o.ear_preset("curious", intensity=0.5),
            o.neck_pose(pan=20, tilt=2, roll=-2, speed=500),
            o.look(-15, 5),
            time.sleep(1.2),
            o.neck_pose(pan=-15, tilt=2, roll=2, speed=500),
            o.look(10, 5),
            time.sleep(1.0),
            o.blink(),
        ],
        "hold": 2.0,
    },
    {
        "title": "Soft Smile",
        "desc": "A quiet happy moment. Content.",
        "actions": lambda o: [
            o.expression("happy", 3),
            o.ear_preset("happy", intensity=0.6),
            o.neck_pose(pan=0, tilt=3, roll=0, speed=500),
            o.look(0, 5),
        ],
        "hold": 3.5,
    },
    {
        "title": "Yawn",
        "desc": "Getting tired... big slow blink and head dip.",
        "actions": lambda o: [
            o.status("going_idle"),
            o.expression("sleepy", 3),
            o.ear_preset("neutral"),
            o.neck_pose(pan=0, tilt=1, speed=400),
            o.look(0, 0),
            time.sleep(1.0),
            o.blink(),
            time.sleep(0.3),
            o.expression("sleepy", 4),
            o.neck_pose(pan=0, tilt=-3, speed=300),
            o.look(0, -20),
        ],
        "hold": 3.0,
    },
    {
        "title": "Getting Sleepy",
        "desc": "Eyes getting heavy... ears droop, slow drift.",
        "actions": lambda o: [
            o.expression("sleepy", 4),
            o.ear_preset("sleepy", intensity=0.7, speed_pct=-0.3),
            o.neck_pose(pan=3, tilt=-5, roll=2, speed=200),
            o.look(0, -30),
            time.sleep(1.5),
            o.blink(),
            time.sleep(1.0),
            o.neck_pose(pan=-2, tilt=-6, roll=-1, speed=150),
            o.look(0, -50),
        ],
        "hold": 3.0,
    },
    {
        "title": "Last Blink",
        "desc": "One final heavy blink...",
        "actions": lambda o: [
            o.expression("sleepy", 5),
            o.ear_preset("sleepy", speed_pct=-0.3),
            o.neck_pose(pan=0, tilt=-6, speed=150),
            o.look(0, -60),
            time.sleep(1.0),
            o.blink(),
        ],
        "hold": 3.0,
    },
    {
        "title": "Asleep",
        "desc": "Goodnight, OLAF. Everything still.",
        "actions": lambda o: [
            o.status("idle"),
            o.expression("sleepy", 5),
            o.ear_preset("sleepy", speed_pct=-0.3),
            o.neck_pose(pan=0, tilt=-7, speed=100),
            o.look(0, -80),
        ],
        "hold": 5.0,
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
