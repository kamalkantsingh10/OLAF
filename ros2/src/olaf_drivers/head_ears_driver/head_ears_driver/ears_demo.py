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
from head_ears_driver.expressions import get_preset_by_name

# Demo sequence: (name, description, angles_override, intensity, hold_seconds)
# If angles_override is None, uses the preset. Otherwise, direct angles.
DEMO_SEQUENCE = [
    {
        "name": "Neutral",
        "desc": "Ears erect, straight up. The Doberman at attention — vigilant, poised.",
        "preset": "neutral",
        "intensity": 1.0,
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
        "preset": "curious",
        "intensity": 1.0,
        "hold": 3.0,
    },
    {
        "name": "Mildly Curious",
        "desc": "Same posture, half intensity. Just a passing interest.",
        "preset": "curious",
        "intensity": 0.5,
        "hold": 2.5,
    },
    {
        "name": "Thinking",
        "desc": "Subtle asymmetric scan. Processing, contemplating.",
        "preset": "thinking",
        "intensity": 1.0,
        "hold": 3.0,
    },
    {
        "name": "Happy",
        "desc": "The Doberman greeting pin — ears wide and pulled back. Pure joy.",
        "preset": "happy",
        "intensity": 1.0,
        "hold": 3.0,
    },
    {
        "name": "Excited",
        "desc": "Perked forward with moderate splay. Ready to play!",
        "preset": "excited",
        "intensity": 1.0,
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
        "preset": "confused",
        "intensity": 1.0,
        "hold": 3.0,
    },
    {
        "name": "Sad",
        "desc": "Full airplane droop. Ears splay wide and fold back. Dejected.",
        "preset": "sad",
        "intensity": 1.0,
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

        if "preset" in step:
            angles = get_preset_by_name(step["preset"], step.get("intensity", 1.0))
        else:
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
