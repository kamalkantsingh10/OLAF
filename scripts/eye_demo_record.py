#!/usr/bin/env python3
"""Record the OLAF eye-expression demo.

Drives each (expression × intensity-level) on the head ESP32 for a
fixed `SHOT_S` while capturing the dev-PC webcam to an AVI; writes a
timing JSON the composer reads next.

Run on the DEV PC (webcam attached here):

    poetry run python scripts/eye_demo_record.py

Outputs (in `docs/captures/`, git-ignored):
    eye_demo_raw_<TS>.avi      — native MJPEG webcam capture
    eye_demo_timing_<TS>.json  — per-shot {expression, level, start, end}

Story 7.1a · 2026-05-20.
"""

from __future__ import annotations

import json
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
OUT  = REPO / "docs" / "captures"
OUT.mkdir(parents=True, exist_ok=True)
TS   = time.strftime("%Y%m%d-%H%M%S")
RAW  = OUT / f"eye_demo_raw_{TS}.avi"
LOG  = OUT / f"eye_demo_timing_{TS}.json"

# Same order as eye_stepper.py — 12 canonical + sleepy + wink + flirty.
EXPRS = [
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
LEVELS    = (1, 2, 3)
SHOT_S    = 2.0
WARMUP_S  = 1.5      # let gst settle before the first shot
TAIL_S    = 1.0      # capture the last shot fully
FRAMERATE = 15

# ── SSH multiplex (one open connection, instant sub-calls) ─────────
SOCK = "/tmp/eye_demo_ssh.sock"
SSH_BASE = [
    "ssh",
    "-o", "ConnectTimeout=8",
    "-o", "BatchMode=yes",
    "-o", "ControlMaster=auto",
    "-o", "ControlPersist=60s",
    "-S", SOCK,
    "olaf.local",
]


def pi(remote_cmd: str, timeout: float = 8.0) -> None:
    """Run `remote_cmd` on the Pi via the multiplexed SSH socket.

    `remote_cmd` is the FULL command line; ssh forwards it to the Pi
    bash literally as one string (NOT as separate argv — passing a
    list to subprocess would let ssh space-join and remote-bash
    re-split, which mangles `python3 -c \"…;…\"`)."""
    subprocess.run(SSH_BASE + [remote_cmd],
                   check=True, timeout=timeout,
                   stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)


def pi_close() -> None:
    subprocess.run(
        ["ssh", "-S", SOCK, "-O", "exit", "olaf.local"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def pi_set_status(status: int) -> None:
    pi('python3 -c "from smbus2 import SMBus;b=SMBus(1);'
       f'b.write_byte_data(0x08,0x30,{status});b.close()"')


def pi_set_shot(expr_value: int, level: int) -> None:
    pi('python3 -c "from smbus2 import SMBus;b=SMBus(1);'
       f'b.write_byte_data(0x08,0x11,{level});'
       f'b.write_byte_data(0x08,0x10,{expr_value});b.close()"')


# ── Webcam (PipeWire MJPEG → AVI native, per reference-webcam-capture) ─
def pipewire_camid() -> str:
    pw = subprocess.run(["pw-dump"], capture_output=True, text=True)
    try:
        nodes = json.loads(pw.stdout)
    except json.JSONDecodeError:
        return ""
    for n in nodes:
        if not n.get("type", "").endswith("Node"):
            continue
        try:
            desc = str(n["info"]["props"]["node.description"])
        except (KeyError, TypeError):
            continue
        if "Camera" in desc:
            return str(n["id"])
    return ""


def main() -> int:
    total_s = WARMUP_S + len(EXPRS) * len(LEVELS) * SHOT_S + TAIL_S
    num_buf = int(total_s * FRAMERATE) + 30
    camid   = pipewire_camid()

    gst = ["gst-launch-1.0", "-e", "pipewiresrc"]
    if camid:
        gst += [f"path={camid}"]
    gst += [
        f"num-buffers={num_buf}",
        "!", f"image/jpeg,width=1280,height=720,framerate={FRAMERATE}/1",
        "!", "avimux",
        "!", "filesink", f"location={RAW}",
    ]

    print(f"Recording {total_s:.1f}s → {RAW.name}", flush=True)
    print(f"           shots: {len(EXPRS) * len(LEVELS)} "
          f"({len(EXPRS)} expressions × {len(LEVELS)} levels)", flush=True)
    if not camid:
        print("           (pipewire camera id NOT resolved — gst will "
              "still try the default)", flush=True)

    # Open SSH multiplex up front so the first set is fast.
    print("Opening SSH multiplex to Pi …", flush=True)
    try:
        pi("true")
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        print(f"ERROR: cannot reach Pi over SSH: {exc}", file=sys.stderr)
        return 2

    print("Starting gst …", flush=True)
    gst_proc = subprocess.Popen(gst,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
    # Let gst negotiate caps & buffer.
    time.sleep(WARMUP_S)
    if gst_proc.poll() is not None:
        print("ERROR: gst-launch exited early — webcam busy?",
              file=sys.stderr)
        pi_close()
        return 1

    try:
        print("Waking head + SPEAKING …", flush=True)
        pi_set_status(1); time.sleep(2.5)    # wake
        pi_set_status(4); time.sleep(0.5)    # SPEAKING

        t0 = time.monotonic()
        shots: list[dict] = []
        total = len(EXPRS) * len(LEVELS)
        for i, (name, exv) in enumerate(EXPRS):
            for j, lvl in enumerate(LEVELS):
                k = i * len(LEVELS) + j
                target = WARMUP_S + k * SHOT_S
                # Account for gst's WARMUP_S having already elapsed.
                t_target = target - WARMUP_S
                while time.monotonic() - t0 < t_target:
                    time.sleep(0.005)
                t_send = time.monotonic() - t0 + WARMUP_S
                pi_set_shot(exv, lvl)
                t_done = time.monotonic() - t0 + WARMUP_S
                shots.append({
                    "expression": name,
                    "expr_value": exv,
                    "level":      lvl,
                    "start":      round(target, 3),
                    "end":        round(target + SHOT_S, 3),
                    "set_at":     round(t_done, 3),
                })
                print(f"  [{k + 1:>2}/{total}] {name:12s} L{lvl}  "
                      f"target={target:6.2f}s  set@{t_done:6.2f}s "
                      f"(Δ{t_done - target:+.2f})", flush=True)

        # Tail to capture the last full shot.
        time.sleep(TAIL_S)
    finally:
        pi_close()
        try:
            gst_proc.send_signal(signal.SIGINT)
            gst_proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            gst_proc.terminate()
            try:
                gst_proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                gst_proc.kill()

    meta = {
        "raw_video":   str(RAW.relative_to(REPO)),
        "framerate":   FRAMERATE,
        "warmup_s":    WARMUP_S,
        "shot_s":      SHOT_S,
        "tail_s":      TAIL_S,
        "total_s":     round(total_s, 3),
        "shot_count":  len(shots),
        "shots":       shots,
    }
    LOG.write_text(json.dumps(meta, indent=2))
    sz_kb = RAW.stat().st_size // 1024 if RAW.exists() else 0
    print(f"\nRaw video:  {RAW}  ({sz_kb} KB)", flush=True)
    print(f"Timing:     {LOG}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
