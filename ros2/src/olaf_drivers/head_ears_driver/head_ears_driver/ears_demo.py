#!/usr/bin/env python3
"""Ears expression demo — cycles through 12 emotional states.

Run from project root on Pi:
    PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver \
    poetry run python ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/ears_demo.py

Each expression holds for a few seconds with a description printed to the console.
"""

import time
import sys

from head_ears_driver.ears_servo_driver import EarsServoDriver

# Demo sequence: (name, description, angles, hold_seconds). Angles are
# explicit per step. (Story 7.1 retired head_ears_driver/expressions.py
# — the expression_engine map is the single source of expression data;
# this manual ears-only demo carries its own baked-in angle dicts,
# kept identical to the former presets×intensity for behavior parity.)
DEMO_SEQUENCE = [
    {
        "name": "Neutral",
        "desc": "Ears erect, straight up. The Doberman at attention — vigilant, poised.",
        "angles": {"left_pan": 0, "left_tilt": 0, "right_pan": 0, "right_tilt": 0},
        "hold": 3.0,
    },
    {
        "name": "Alert",
        "desc": "Slight forward lean, locked on target. Something caught OLAF's attention.",
        "angles": {"left_pan": 0, "left_tilt": 15, "right_pan": 0, "right_tilt": 15},
        "hold": 3.0,
    },
    {
        "name": "Curious",
        "desc": "Asymmetric forward tilt — one ear cocks wider. What was that sound?",
        "angles": {"left_pan": 5, "left_tilt": 20, "right_pan": 55, "right_tilt": 12},
        "hold": 3.0,
    },
    {
        "name": "Mildly Curious",
        "desc": "Same posture, half intensity. Just a passing interest.",
        "angles": {"left_pan": 2.5, "left_tilt": 10, "right_pan": 27.5, "right_tilt": 6.0},
        "hold": 2.5,
    },
    {
        "name": "Thinking",
        "desc": "Subtle asymmetric scan. Processing, contemplating.",
        "angles": {"left_pan": 8, "left_tilt": 18, "right_pan": 12, "right_tilt": 12},
        "hold": 3.0,
    },
    {
        "name": "Happy",
        "desc": "The Doberman greeting pin — ears wide and pulled back. Pure joy.",
        "angles": {"left_pan": 20, "left_tilt": 18, "right_pan": 20, "right_tilt": 18},
        "hold": 3.0,
    },
    {
        "name": "Excited",
        "desc": "Perked forward with moderate splay. Ready to play!",
        "angles": {"left_pan": 15, "left_tilt": 20, "right_pan": 15, "right_tilt": 20},
        "hold": 3.0,
    },
    {
        "name": "Relaxed",
        "desc": "Gentle airplane ears. Calm, content, at ease.",
        "angles": {"left_pan": 40, "left_tilt": 0, "right_pan": 40, "right_tilt": 0},
        "hold": 3.0,
    },
    {
        "name": "Confused",
        "desc": "Strong asymmetry — one ear up, one way out. What is happening?",
        "angles": {"left_pan": 0, "left_tilt": 20, "right_pan": 55, "right_tilt": 5},
        "hold": 3.0,
    },
    {
        "name": "Sad",
        "desc": "Full airplane droop. Ears splay wide and fold back. Dejected.",
        "angles": {"left_pan": 80, "left_tilt": 20, "right_pan": 55, "right_tilt": 30},
        "hold": 3.0,
    },
    {
        "name": "Sleepy",
        "desc": "Slow drift to full airplane mode. Eyes heavy, ears heavy.",
        "angles": {"left_pan": 60, "left_tilt": -15, "right_pan": 55, "right_tilt": -5},
        "speed_pct": -0.3,
        "hold": 3.5,
    },
    {
        "name": "Startled",
        "desc": "Snap to attention! Ears bolt upright from droop. What was THAT?",
        "angles": {"left_pan": 0, "left_tilt": 20, "right_pan": 0, "right_tilt": 20},
        "speed_pct": 0.3,
        "hold": 2.5,
    },
]


def run_demo() -> None:
    print("=" * 60)
    print("  OLAF Ears Expression Demo — 12 Emotional States")
    print("  Doberman-inspired ear movements")
    print("=" * 60)
    print()

    driver = EarsServoDriver()
    driver.center_all()
    time.sleep(1.0)

    total = len(DEMO_SEQUENCE)
    for i, step in enumerate(DEMO_SEQUENCE, 1):
        name = step["name"]
        desc = step["desc"]
        hold = step["hold"]
        speed_pct = step.get("speed_pct", 0.0)

        print(f"[{i:2d}/{total}] {name}")
        print(f"       {desc}")

        angles = step["angles"]

        for servo_name, degrees in angles.items():
            driver.move(servo_name, degrees, speed_pct)

        time.sleep(hold)
        print()

    # Return to neutral
    print("[--] Returning to neutral...")
    driver.center_all()
    time.sleep(0.5)
    driver.close()

    print()
    print("=" * 60)
    print("  Demo complete.")
    print("=" * 60)


if __name__ == "__main__":
    try:
        run_demo()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(0)
    except ConnectionError as e:
        print(f"[ERROR] {e}")
        print("Check: power on? /dev/waveshare_ears connected?")
        sys.exit(1)
