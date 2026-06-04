#!/usr/bin/env python3
"""Live ear-MOTION preview — the dynamic 'alive' layer (hare).

Watch OLAF's ears be alive on hardware so we can tune the feel BEFORE
porting the final values into render_loop.py. Three things at once:

  1. Ambient twitch  — a faint continuous tremor + occasional single-ear
     flick, ALWAYS while awake (the anti-rigidity signature).
  2. Talk-motion     — while 'speaking', re-sampled tilt+pan moves with
     occasional single-ear accents, amplitude/pace scaled by the emotion.
  3. Secondary motion— each joint is driven by an UNDER-damped spring, so
     moves overshoot and the (long) ear tips wobble before settling.

Run on the Pi from ~/olaf:

  PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver \\
    ~/.local/bin/poetry run python \\
    ros2/src/expression_engine/tools/ear_motion_preview.py [mode] [emotion] [secs]

  idle               ambient twitch only (default), base = neutral
  speaking excited   talk-motion + twitch, base = that emotion's pose
  speaking sad 25    ... for 25 seconds

Ctrl-C stops early; always rests at neutral. All tuning knobs are the
CONSTANTS block below — change, re-run, repeat.
"""
import math
import os
import random
import sys
import time

import yaml

from head_ears_driver.ears_servo_driver import EarsServoDriver

# ── TUNING KNOBS ───────────────────────────────────────────────
TICK_HZ = 60.0

# Under-damped spring per joint (the secondary-motion / tip-wobble).
# zeta = C / (2*sqrt(K)); <1 = overshoot. K=120,C=11 -> zeta~0.50 (~16%).
SPRING_K = 120.0
SPRING_C = 11.0

# Ambient twitch (always while awake).
TREMOR_DEG = 1.4          # continuous tiny tilt tremor amplitude
TREMOR_HZ = (3.5, 6.0)    # per-ear tremor frequency band
FLICK_GAP_S = (4.0, 11.0) # seconds between random single-ear flicks
FLICK_DEG = (5.0, 12.0)   # flick size (tilt)
FLICK_DUR_S = 0.18        # flick half-sine duration

# Talk-motion (only while speaking), scaled by emotion energy.
SPK_TILT_DEG = (3.0, 9.0)
SPK_PAN_DEG = (2.0, 5.0)
SPK_PERIOD_S = (0.6, 1.6)
SPK_ACCENT_P = 0.30       # P(single-ear asymmetric accent)

ENERGY = {
    "excited": 1.6, "surprised": 1.5, "scared": 1.4, "angry": 1.4,
    "playful": 1.3, "frustrated": 1.3, "happy": 1.2, "curious": 1.1,
    "neutral": 1.0, "content": 0.8, "sympathetic": 0.8, "sad": 0.6,
    "melancholic": 0.5,
}
JOINTS = ("left_pan", "left_tilt", "right_pan", "right_tilt")
MAP = os.path.expanduser(
    "~/olaf/ros2/src/expression_engine/config/expression_map.yaml")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "idle"
    emotion = sys.argv[2] if len(sys.argv) > 2 else "neutral"
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
    speaking = (mode == "speaking")
    energy = ENERGY.get(emotion, 1.0)

    emap = yaml.safe_load(open(MAP))["speech_emotion"]
    base = dict(emap.get(emotion, emap["neutral"])["pose"]["ears"])
    print(f"mode={mode} emotion={emotion} energy={energy} base={base}")

    rng = random.Random()
    tphase = {j: rng.uniform(0, 6.28) for j in JOINTS}
    tfreq = {j: rng.uniform(*TREMOR_HZ) for j in JOINTS}
    pos = dict(base)
    vel = {j: 0.0 for j in JOINTS}

    spk = {j: 0.0 for j in JOINTS}
    spk_next = 0.0
    flick = {"end": 0.0, "joint": "left_tilt", "amp": 0.0}
    flick_next = rng.uniform(*FLICK_GAP_S)

    d = EarsServoDriver()
    dt = 1.0 / TICK_HZ
    t = 0.0
    try:
        while t < secs:
            # talk-motion re-sample
            if speaking and t >= spk_next:
                tl = rng.uniform(*SPK_TILT_DEG) * energy * rng.choice((-1, 1))
                pn = rng.uniform(*SPK_PAN_DEG) * energy * rng.choice((-1, 1))
                spk = {"left_tilt": tl, "right_tilt": tl,
                       "left_pan": pn, "right_pan": pn}
                if rng.random() < SPK_ACCENT_P:           # single-ear accent
                    hold = "left" if rng.random() < 0.5 else "right"
                    spk[f"{hold}_tilt"] = 0.0
                    spk[f"{hold}_pan"] = 0.0
                spk_next = t + rng.uniform(*SPK_PERIOD_S) / max(0.5, energy)
            # ambient flick schedule
            if t >= flick_next:
                flick = {"end": t + FLICK_DUR_S,
                         "joint": rng.choice(("left_tilt", "right_tilt")),
                         "amp": rng.uniform(*FLICK_DEG) * rng.choice((-1, 1))}
                flick_next = t + rng.uniform(*FLICK_GAP_S)

            # compose per-joint target = base + talk + tremor + flick
            for j in JOINTS:
                tgt = base[j] + spk[j]
                if j.endswith("tilt"):
                    tgt += TREMOR_DEG * math.sin(2 * math.pi * tfreq[j] * t + tphase[j])
                if flick["joint"] == j and t < flick["end"]:
                    u = 1.0 - (flick["end"] - t) / FLICK_DUR_S
                    tgt += flick["amp"] * math.sin(math.pi * u)
                # under-damped spring integrate
                acc = SPRING_K * (tgt - pos[j]) - SPRING_C * vel[j]
                vel[j] += acc * dt
                pos[j] += vel[j] * dt

            d.move_left_pan(pos["left_pan"]); d.move_left_tilt(pos["left_tilt"])
            d.move_right_pan(pos["right_pan"]); d.move_right_tilt(pos["right_tilt"])
            time.sleep(dt)
            t += dt
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        n = emap["neutral"]["pose"]["ears"]
        d.move_left_pan(n["left_pan"]); d.move_left_tilt(n["left_tilt"])
        d.move_right_pan(n["right_pan"]); d.move_right_tilt(n["right_tilt"])
        time.sleep(0.4)
        d.close()
        print("done -> neutral")


if __name__ == "__main__":
    main()
