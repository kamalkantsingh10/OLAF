#!/usr/bin/env python3
"""Manual expression + intensity-level stepper (Stories 7.1a / b / c).

Runs ON THE PI. Drives:
  • the head ESP32 over I2C (0x08) for the eye expression renderer
  • the neck servos over serial via NeckMotionPlayer
    (per-emotion 3-level: L1 pose / L2 idle drift / L3 peak gesture)
  • the ear servos over serial via EarsMotionPlayer
    (same 3-level model, 4 joints: LP/LT/RP/RT)

Each "shot" snaps the eyes AND restarts the neck + ears plans so
what you see on the LCDs + the head pose + ear pose are coherent.

    python3 modules/head/firmware/tools/eye_stepper.py

Levels (firmware-strict 3-level model, parallel to the eye renderer):
    L1  base shape / pose, monochrome / static
    L2  + colour tint / static mark   /   + small idle drift
    L3  + animated anime FX           /   + peak gesture at onset

Controls:
    ENTER   next expression          b   previous expression
    1/2/3   set level L1 / L2 / L3    a   auto-sweep all × L1..L3
    q       quit

The eyes are woken + put in SPEAKING state first (expressions only
render there). Device EXPR values must match
modules/head/firmware/include/i2c_slave.h and
head_i2c_client.EXPRESSION_MAP.

Add sibling repo roots to sys.path so this script can be run by
absolute path without manual PYTHONPATH setup:
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

# ── bootstrap sys.path: find the OLAF repo root, then sibling roots
_HERE = Path(__file__).resolve()
for cand in _HERE.parents:
    if (cand / ".git").exists() and (cand / "ros2").exists():
        _REPO = cand
        break
else:                                          # fall back to ~/olaf
    _REPO = Path(os.environ.get("HOME", "~")).expanduser() / "olaf"

for p in (
    _REPO / "ros2" / "src" / "expression_engine",
    _REPO / "ros2" / "src" / "olaf_drivers" / "neck_driver",
    _REPO / "ros2" / "src" / "olaf_drivers" / "head_ears_driver",
    _REPO / "libs",
):
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

from smbus2 import SMBus                                            # noqa: E402

ADDR          = 0x08
BUS           = 1
REG_EXPR      = 0x10
REG_INTENSITY = 0x11
REG_STATUS    = 0x30

# (name, EXPR value) — 12 canonical + sleepy/wink + flirty.
# Names MUST match the keys in neck_motion.yaml `emotions`.
EXPRESSIONS = [
    ("neutral",      0),
    ("happy",        1),
    ("content",      7),
    ("excited",      8),
    ("sad",          2),
    ("melancholic", 13),
    ("sympathetic", 11),
    ("angry",        4),
    ("frustrated",  12),
    ("scared",       9),
    ("curious",     10),
    ("surprised",    3),
    ("sleepy",       5),
    ("wink",         6),
    ("flirty",      14),
]

LEVELS = {
    1: "L1 base shape/pose                 |  static",
    2: "L2 + colour tint + mark / drift    |  drift",
    3: "L3 + animated FX / peak gesture    |  gesture",
}


# ── eye I2C ────────────────────────────────────────────────────────
def _write(bus: SMBus, reg: int, val: int, tries: int = 5) -> bool:
    for _ in range(tries):
        try:
            bus.write_byte_data(ADDR, reg, val)
            return True
        except OSError:
            time.sleep(0.5)
    return False


def _set_eye(bus: SMBus, value: int, level: int) -> bool:
    # 0x11 reads back the expression_type (known swap-quirk) — use
    # it to confirm the write landed.
    for _ in range(5):
        _write(bus, REG_INTENSITY, level)
        _write(bus, REG_EXPR, value)
        try:
            if bus.read_byte_data(ADDR, REG_INTENSITY) == value:
                return True
        except OSError:
            pass
        time.sleep(0.4)
    return False


# ── optional neck wiring ─────────────────────────────────────────
def _open_neck():
    """Return (player, driver) or (None, None) if hardware/dependencies
    aren't available (e.g. dev PC). Stepper degrades to eyes-only."""
    try:
        from neck_driver.neck_servo_driver import NeckServoDriver
        from expression_engine.adapters.neck_motion_player import (
            NeckMotionPlayer,
        )
    except Exception as exc:                                # noqa: BLE001
        print(f"[neck] import skipped ({exc!s}) — eyes-only mode.")
        return None, None
    try:
        driver = NeckServoDriver()
        player = NeckMotionPlayer(driver)
        player.start()
        return player, driver
    except Exception as exc:                                # noqa: BLE001
        print(f"[neck] open skipped ({exc!s}) — eyes-only mode.")
        return None, None


def _close_neck(player, driver) -> None:
    if player is not None:
        try:
            player.stop()
        except Exception:                                   # noqa: BLE001
            pass
    if driver is not None:
        try:
            driver.close()
        except Exception:                                   # noqa: BLE001
            pass


def _open_ears():
    """Return (player, driver) or (None, None) if hardware/deps
    aren't available. Stepper degrades gracefully."""
    try:
        from head_ears_driver.ears_servo_driver import EarsServoDriver
        from expression_engine.adapters.ears_motion_player import (
            EarsMotionPlayer,
        )
    except Exception as exc:                                # noqa: BLE001
        print(f"[ears] import skipped ({exc!s}) — ears-less mode.")
        return None, None
    try:
        driver = EarsServoDriver()
        player = EarsMotionPlayer(driver)
        player.start()
        return player, driver
    except Exception as exc:                                # noqa: BLE001
        print(f"[ears] open skipped ({exc!s}) — ears-less mode.")
        return None, None


def _close_ears(player, driver) -> None:
    if player is not None:
        try:
            player.stop()
        except Exception:                                   # noqa: BLE001
            pass
    if driver is not None:
        try:
            driver.close()
        except Exception:                                   # noqa: BLE001
            pass


# ── orchestrator ────────────────────────────────────────────────
def _show(bus: SMBus, neck, ears, i: int, level: int) -> None:
    label, val = EXPRESSIONS[i]
    eye_ok = _set_eye(bus, val, level)
    if neck is not None:
        neck.set_expression(label, level)
    if ears is not None:
        ears.set_expression(label, level)
    print(f"[{i + 1:>2}/{len(EXPRESSIONS)}] {label:12s} EXPR={val:<2d}  "
          f"{LEVELS[level]}"
          f"{'' if eye_ok else '   <- eye I2C unconfirmed'}")


def main() -> int:
    with SMBus(BUS) as bus:
        print("Waking head ESP32 ...")
        _write(bus, REG_STATUS, 1)
        time.sleep(3.0)
        _write(bus, REG_STATUS, 4)              # SPEAKING (renders exprs)
        time.sleep(1.0)

        neck_player, neck_driver = _open_neck()
        if neck_player is not None:
            print("Neck player: ready (background thread ticking).")
        ears_player, ears_driver = _open_ears()
        if ears_player is not None:
            print("Ears player: ready (background thread ticking).")

        try:
            i = 0
            level = 2
            n = len(EXPRESSIONS)
            while True:
                _show(bus, neck_player, ears_player, i, level)
                cmd = input(
                    "  ENTER=next  b=back  1/2/3=level  "
                    "a=auto  q=quit > "
                ).strip().lower()
                if cmd == "q":
                    break
                if cmd == "b":
                    i = (i - 1) % n
                elif cmd in ("1", "2", "3"):
                    level = int(cmd)
                elif cmd == "a":
                    print("Auto-sweep: every expression × L1..L3 "
                          "(2.5 s each, Ctrl-C to stop) ...")
                    try:
                        for k in range(n):
                            for lv in (1, 2, 3):
                                _show(bus, neck_player, ears_player,
                                      k, lv)
                                time.sleep(2.5)
                    except KeyboardInterrupt:
                        print("\n  (auto-sweep interrupted)")
                    i = 0
                else:
                    i = (i + 1) % n

            # Park gracefully on neutral.
            _set_eye(bus, 0, 2)
            if neck_player is not None:
                neck_player.set_expression("neutral", 1)
            if ears_player is not None:
                ears_player.set_expression("neutral", 1)
            time.sleep(0.5)
            print("Parked on neutral.")
        finally:
            _close_neck(neck_player, neck_driver)
            _close_ears(ears_player, ears_driver)
    return 0


if __name__ == "__main__":
    sys.exit(main())
