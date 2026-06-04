#!/usr/bin/env python3
"""Ear-GESTURE preview / tuning tool.

Fires each vocalization's ear gesture (laughter, gasp, sigh, emphasis, …)
on hardware so you can watch the transient and tune amp/dur. The gesture
trajectory is animated over its real duration on top of the NEUTRAL base
pose — exactly the ``GESTURES[token](u, amp)`` the live engine uses
(loaded straight from ears_gestures.py, no ROS needed).

Run on the Pi from ~/olaf:

  PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver \\
    ~/.local/bin/poetry run python \\
    ros2/src/expression_engine/tools/ear_gesture_preview.py [vocalization ...]

  (no args)            every vocalization that has an ears_gesture
  laughter gasp sigh   only those, in order

Enter fires the next gesture; Ctrl-C stops. Amp/dur are sampled from the
map's [min,max] ranges each fire (like the real engine).
"""
import importlib.util
import os
import random
import sys
import time

import yaml

from head_ears_driver.ears_servo_driver import EarsServoDriver

ROOT = os.path.expanduser("~/olaf")
MAP = os.path.join(ROOT, "ros2/src/expression_engine/config/expression_map.yaml")
EG = os.path.join(
    ROOT, "ros2/src/expression_engine/expression_engine/adapters/ears_gestures.py"
)
JOINTS = ("left_pan", "left_tilt", "right_pan", "right_tilt")
HZ = 50.0


def load_gestures():
    """Load ears_gestures.py directly (no package __init__ / ROS imports)."""
    spec = importlib.util.spec_from_file_location("ears_gestures", EG)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.GESTURES


def pick(v):
    if isinstance(v, (list, tuple)):
        return random.uniform(float(v[0]), float(v[1]))
    return float(v)


def main():
    GEST = load_gestures()
    m = yaml.safe_load(open(MAP))
    voc = m["vocalization"]
    base = m["speech_emotion"]["neutral"]["pose"]["ears"]

    def has_ears(v):
        return isinstance(v, dict) and v.get("layered_action", {}).get("ears_gesture")

    names = sys.argv[1:] or [k for k, v in voc.items() if has_ears(v)]
    d = EarsServoDriver()

    def drive(off):
        d.move_left_pan(base["left_pan"] + off[0])
        d.move_left_tilt(base["left_tilt"] + off[1])
        d.move_right_pan(base["right_pan"] + off[2])
        d.move_right_tilt(base["right_tilt"] + off[3])

    try:
        for name in names:
            v = voc.get(name)
            eg = has_ears(v) and v["layered_action"]["ears_gesture"]
            if not eg:
                print(f"\n=== {name.upper()} ===  (no ears_gesture — skipped)")
                continue
            token = eg["token"]
            fn = GEST.get(token)
            amp = pick(eg.get("amp_deg", 8))
            dur = pick(eg.get("dur_s", 0.7))
            print(f"\n=== {name.upper():12s} ===  token={token}  "
                  f"amp={amp:.1f}  dur={dur:.2f}s")
            if fn is None:
                print(f"  !! unknown token {token} (add it to ears_gestures.py)")
                continue
            try:
                input("  [Enter] FIRE  |  Ctrl-C stop  ")
            except EOFError:
                break
            steps = max(2, int(dur * HZ))
            for i in range(steps + 1):
                drive(fn(i / steps, amp))
                time.sleep(1.0 / HZ)
            drive((0.0, 0.0, 0.0, 0.0))  # settle back to neutral base
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        drive((0.0, 0.0, 0.0, 0.0))
        d.close()
        print("done -> neutral")


if __name__ == "__main__":
    main()
