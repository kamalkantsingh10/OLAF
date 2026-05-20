#!/usr/bin/env python3
"""Manual vocalization stepper — Story 7.2 hardware review.

Same pattern as `eye_stepper.py` (per-emotion review). Each "shot"
fires ONE of the 6 vocalization tags (laughter / sigh / gasp /
clears_throat / nod / shake) as a layered transient over a held
baseline (a chosen speech_emotion + mood). Lets you observe:

  - the per-fire RANDOM SAMPLES (amplitude / duration / window /
    eye intensity) — two fires of the same tag never look identical
  - that the body returns cleanly to the baseline pose after settle
    (AR12 additivity invariant — neck/ears + eye)
  - for nod/shake, the eye accent is taken from the CURRENT MOOD
    via `mood.<mood>.eye.expression`

    cd ~/olaf
    ~/.local/bin/poetry run python modules/head/firmware/tools/vocalization_stepper.py

(Plain `python3` won't work: `expression_engine.schema` imports
`pydantic` which is only in the project's Poetry venv. The script's
own sys.path bootstrap still resolves the OLAF package roots, so no
PYTHONPATH is needed beyond invoking via Poetry.)

Drives directly:
  • head ESP32 over I2C (eye expression)
  • neck servos over serial via NeckServoDriver
  • ear servos over serial via EarsServoDriver

No ROS, no expression_engine node — uses ONLY the same
`_VocalizationAction` class the engine uses, so what you see on the
robot is exactly what the engine will render at runtime.

Controls:
  ENTER     fire CURRENT tag                n / b   next / prev tag
  m         cycle baseline mood             s       cycle baseline speech_emotion
  q         quit
"""

from __future__ import annotations

import logging
import os
import random
import sys
import time
from pathlib import Path

# ── bootstrap sys.path (mirrors eye_stepper.py) ────────────────────
_HERE = Path(__file__).resolve()
for cand in _HERE.parents:
    if (cand / ".git").exists() and (cand / "ros2").exists():
        _REPO = cand
        break
else:                                              # fall back to ~/olaf
    _REPO = Path(os.environ.get("HOME", "~")).expanduser() / "olaf"

for p in (
    _REPO / "ros2" / "src" / "expression_engine",
    _REPO / "ros2" / "src" / "olaf_drivers" / "neck_driver",
    _REPO / "ros2" / "src" / "olaf_drivers" / "head_ears_driver",
    _REPO / "libs",
):
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

logging.basicConfig(level=logging.WARNING)

from expression_engine.map_loader import load_expression_map         # noqa: E402
from expression_engine.render_loop import _VocalizationAction        # noqa: E402
from expression_engine.schema import VOCALIZATION_TAGS, Mood         # noqa: E402
from head_ears_driver.head_i2c_client import HeadI2CClient           # noqa: E402

from typing import get_args                                          # noqa: E402

TICK_HZ = 30.0
TICK_DT = 1.0 / TICK_HZ
MAP_PATH = _REPO / "ros2" / "src" / "expression_engine" / "config" / "expression_map.yaml"

# Baseline rotation menu — short representative subset across the
# valence range; press `s` to cycle.
SPEECH_BASELINES = ("neutral", "happy", "content", "sad", "excited")
MOOD_BASELINES = list(get_args(Mood))


def _open_neck():
    try:
        from neck_driver.neck_servo_driver import NeckServoDriver
        return NeckServoDriver()
    except Exception as exc:                                # noqa: BLE001
        print(f"[neck] open skipped ({exc!s}) — neck-less mode.")
        return None


def _open_ears():
    try:
        from head_ears_driver.ears_servo_driver import EarsServoDriver
        return EarsServoDriver()
    except Exception as exc:                                # noqa: BLE001
        print(f"[ears] open skipped ({exc!s}) — ears-less mode.")
        return None


def _eye_translate(canonical: str) -> str:
    """Map canonical → ESP32 device name (same table as eye_adapter)."""
    try:
        from expression_engine.adapters.eye_adapter import EyeAdapter
        return EyeAdapter.translate(canonical)
    except Exception:
        return canonical


# Mirror eye_stepper.py's verify-with-retry I2C protocol. The head ESP32
# has a write-swap quirk: writing EXPRESSION_TYPE (0x10) causes
# EXPRESSION_INTENSITY (0x11) to read back the type value. eye_stepper
# uses this as a verify probe — write INTENSITY first, then TYPE, then
# read 0x11; if it equals the type value, the write landed. We mimic
# that exact pattern here instead of relying on `HeadI2CClient.set_
# expression` (which writes TYPE-then-INTENSITY without verify).
def _set_eye_robust(bus, canonical_name: str, intensity: int) -> bool:
    from head_ears_driver.head_i2c_client import (
        EXPRESSION_MAP, I2C_HEAD_ADDRESS,
        REG_EXPRESSION_TYPE, REG_EXPRESSION_INTENSITY,
    )
    esp = _eye_translate(canonical_name)
    expr_val = EXPRESSION_MAP.get(esp.lower())
    if expr_val is None:
        return False
    level = max(1, min(5, int(intensity)))
    for _ in range(5):
        try:
            bus.write_byte_data(I2C_HEAD_ADDRESS, REG_EXPRESSION_INTENSITY, level)
            bus.write_byte_data(I2C_HEAD_ADDRESS, REG_EXPRESSION_TYPE, expr_val)
        except OSError:
            time.sleep(0.2)
            continue
        try:
            if bus.read_byte_data(I2C_HEAD_ADDRESS, REG_EXPRESSION_INTENSITY) == expr_val:
                return True
        except OSError:
            pass
        time.sleep(0.2)
    return False


def _baseline_neck(emap, emotion: str) -> dict:
    pose = emap.speech_emotion.get(emotion, {}).get("pose", {}).get("neck", {})
    return {
        "pan":  float(pose.get("pan", 0.0)),
        "tilt": float(pose.get("tilt", 0.0)),
        "roll": float(pose.get("roll", 0.0)),
    }


def _baseline_ears(emap, emotion: str) -> dict:
    pose = emap.speech_emotion.get(emotion, {}).get("pose", {}).get("ears", {})
    return {
        "left_pan":   float(pose.get("left_pan", 0.0)),
        "left_tilt":  float(pose.get("left_tilt", 0.0)),
        "right_pan":  float(pose.get("right_pan", 0.0)),
        "right_tilt": float(pose.get("right_tilt", 0.0)),
    }


def _baseline_eye(emap, emotion: str) -> tuple[str, int]:
    eye = emap.speech_emotion.get(emotion, {}).get("eye", {})
    return (str(eye.get("expression", "neutral")), int(eye.get("intensity", 3)))


def _drive_neck(neck, target: dict) -> None:
    if neck is None:
        return
    try:
        neck.move_pose(pan=target["pan"], tilt=target["tilt"], roll=target["roll"])
    except Exception as exc:                                # noqa: BLE001
        print(f"  [!] neck.move_pose failed: {exc}")


def _drive_ears(ears, target: dict) -> None:
    if ears is None:
        return
    try:
        for j, v in target.items():
            ears.move(j, v, 0.0)
    except Exception as exc:                                # noqa: BLE001
        print(f"  [!] ears.move failed: {exc}")


def _drive_eye(eyes: HeadI2CClient, last: tuple[str, int] | None,
               target: tuple[str, int]) -> tuple[str, int]:
    """Fire-on-change eye write using eye_stepper's verify-with-retry
    protocol (head ESP32 write-swap quirk). Naive ``set_expression``
    writes type→intensity without verify and lost about 1-in-2 sigh
    fires during the 2026-05-20 review run; the protocol below writes
    intensity→type and reads back REG_EXPRESSION_INTENSITY to confirm.
    The fire-on-change guard keeps this off the per-tick path — only
    runs on transitions.
    """
    if last is None or target != last:
        _set_eye_robust(eyes._bus, target[0], target[1])
    return target


def _hold_baseline(eyes, neck, ears, base_neck, base_ears, base_eye, seconds: float):
    """Drive to baseline + hold for `seconds` so settle is visible."""
    _drive_neck(neck, base_neck)
    _drive_ears(ears, base_ears)
    _drive_eye(eyes, None, base_eye)
    time.sleep(seconds)


def _fire(
    emap,
    eyes: HeadI2CClient,
    neck,
    ears,
    tag: str,
    base_neck: dict,
    base_ears: dict,
    base_eye: tuple[str, int],
    mood_name: str,
) -> None:
    """One vocalization shot — sample, tick to window_s, return to base."""
    entry = emap.vocalization.get(tag, {})
    spec = entry.get("layered_action") if isinstance(entry, dict) else None
    if spec is None:
        print(f"  [!] no layered_action authored for {tag!r} — skipping")
        return

    # nod / shake derive eye.expression from the current mood.
    mood_eye_expr = None
    mood_entry = emap.mood.get(mood_name) if mood_name else None
    if isinstance(mood_entry, dict):
        me = mood_entry.get("eye")
        if isinstance(me, dict):
            mood_eye_expr = me.get("expression")

    rng = random.Random()                    # unseeded → real variance
    start = time.monotonic()
    action = _VocalizationAction(
        tag=tag, spec=spec, start=start, rng=rng,
        mood_eye_expression=mood_eye_expr,
    )

    # Programmatic evidence (what's about to be rendered).
    print(f"  → sampled: window={action.window_s:.3f}s", end="")
    if action.neck_token:
        print(f"  neck[{action.neck_token} amp={action.neck_amp:+5.2f}° "
              f"dur={action.neck_dur:.3f}s]", end="")
    if action.ears_token:
        print(f"  ears[{action.ears_token} amp={action.ears_amp:+5.2f}° "
              f"dur={action.ears_dur:.3f}s]", end="")
    print(f"  eye[{action.eye_expression!r} i={action.eye_intensity}]")

    last_eye = base_eye
    deadline = start + action.window_s
    while time.monotonic() < deadline:
        now = time.monotonic()
        n_off = action.neck_offset(now)
        e_off = action.ears_offset(now)
        n_tgt = {j: base_neck[j] + n_off.get(j, 0.0) for j in base_neck}
        e_tgt = {j: base_ears[j] + e_off.get(j, 0.0) for j in base_ears}
        eye_t = action.eye_target(now) or base_eye
        _drive_neck(neck, n_tgt)
        _drive_ears(ears, e_tgt)
        last_eye = _drive_eye(eyes, last_eye, eye_t)
        time.sleep(TICK_DT)

    # AR12 — return to baseline cleanly.
    _drive_neck(neck, base_neck)
    _drive_ears(ears, base_ears)
    _drive_eye(eyes, last_eye, base_eye)


def main() -> int:
    print("Loading expression_map ...")
    emap = load_expression_map(MAP_PATH)
    eyes = HeadI2CClient()
    eyes.open()
    print(f"Eyes (I2C 0x{eyes._address:02X}): OK")
    print("Waking head ESP32 ...")
    eyes.set_system_status("woke_up")
    time.sleep(2.0)
    eyes.set_system_status("speaking")           # expressions render here
    time.sleep(0.5)
    neck = _open_neck()
    if neck is not None:
        print("Neck servos: OK")
    ears = _open_ears()
    if ears is not None:
        print("Ears servos: OK")

    tags = list(VOCALIZATION_TAGS)
    i = 0
    s_idx = SPEECH_BASELINES.index("neutral")
    m_idx = MOOD_BASELINES.index("happy")

    try:
        while True:
            speech = SPEECH_BASELINES[s_idx]
            mood = MOOD_BASELINES[m_idx]
            base_neck = _baseline_neck(emap, speech)
            base_ears = _baseline_ears(emap, speech)
            base_eye = _baseline_eye(emap, speech)

            print()
            print(f"[{i + 1}/{len(tags)}] tag={tags[i]:<14s} "
                  f"baseline speech_emotion={speech!r}  mood={mood!r}")
            print(f"   base neck={base_neck}  ears={base_ears}  eye={base_eye}")

            # Hold baseline briefly so the AR12 'return' after settle
            # is visually obvious.
            _hold_baseline(eyes, neck, ears, base_neck, base_ears, base_eye, 0.8)

            cmd = input(
                "  ENTER=fire  n=next  b=prev  m=cycle mood  "
                "s=cycle speech  q=quit > "
            ).strip().lower()
            if cmd == "q":
                break
            if cmd == "n":
                i = (i + 1) % len(tags)
                continue
            if cmd == "b":
                i = (i - 1) % len(tags)
                continue
            if cmd == "m":
                m_idx = (m_idx + 1) % len(MOOD_BASELINES)
                continue
            if cmd == "s":
                s_idx = (s_idx + 1) % len(SPEECH_BASELINES)
                continue

            # ENTER (or anything else) → fire current tag.
            _fire(emap, eyes, neck, ears, tags[i],
                  base_neck, base_ears, base_eye, mood)

        # Park gracefully on neutral.
        print("\nParking on neutral ...")
        eyes.set_expression("neutral", 2)
        if neck is not None:
            _drive_neck(neck, {"pan": 0.0, "tilt": 0.0, "roll": 0.0})
        if ears is not None:
            _drive_ears(
                ears,
                {"left_pan": 0.0, "left_tilt": 0.0,
                 "right_pan": 0.0, "right_tilt": 0.0},
            )
        time.sleep(0.5)
    finally:
        if neck is not None:
            try: neck.close()
            except Exception: pass
        if ears is not None:
            try: ears.close()
            except Exception: pass
        eyes.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
