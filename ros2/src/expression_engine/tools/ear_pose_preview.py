#!/usr/bin/env python3
"""Ear-pose preview / hardware-tuning tool.

Drives OLAF's ears to each speech-emotion's ear pose from
expression_map.yaml so you can eyeball and tune one at a time.

Run on the Pi from ~/olaf:

  PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver \\
    ~/.local/bin/poetry run python \\
    ros2/src/expression_engine/tools/ear_pose_preview.py [emotion ...]

  (no args)              step through all 12, Enter to advance
  sad melancholic angry  only those, in that order

Press Enter to advance, Ctrl-C to stop. Always rests at neutral on exit.
"""
import os
import sys

import yaml

from head_ears_driver.ears_servo_driver import EarsServoDriver

_CANDIDATES = [
    os.path.expanduser("~/olaf/ros2/src/expression_engine/config/expression_map.yaml"),
    os.path.join(os.path.dirname(__file__), "..", "config", "expression_map.yaml"),
]
MAP = next((p for p in _CANDIDATES if os.path.isfile(p)), _CANDIDATES[0])

ORDER = [
    "neutral", "happy", "excited", "surprised", "curious", "content",
    "sympathetic", "sad", "melancholic", "scared", "angry", "frustrated",
]


def _drive(d, ears):
    d.move_left_pan(ears["left_pan"])
    d.move_left_tilt(ears["left_tilt"])
    d.move_right_pan(ears["right_pan"])
    d.move_right_tilt(ears["right_tilt"])


def main():
    emo = yaml.safe_load(open(MAP))["speech_emotion"]
    names = sys.argv[1:] or ORDER
    d = EarsServoDriver()
    try:
        for name in names:
            if name not in emo:
                print(f"  (skip unknown emotion: {name})")
                continue
            e = emo[name]["pose"]["ears"]
            print(f"\n=== {name.upper():12s} ===  "
                  f"LP={e['left_pan']:>3}  LT={e['left_tilt']:>3}  "
                  f"RP={e['right_pan']:>3}  RT={e['right_tilt']:>3}")
            _drive(d, e)
            try:
                input("  [Enter] next  |  Ctrl-C stop  ")
            except EOFError:
                break
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        _drive(d, emo["neutral"]["pose"]["ears"])
        d.close()
        print("rest -> neutral")


if __name__ == "__main__":
    main()
