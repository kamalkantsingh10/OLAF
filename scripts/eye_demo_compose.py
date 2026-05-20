#!/usr/bin/env python3
"""Compose the eye-expression demo mp4 from a recorded raw clip
and its timing JSON.

Adds an intro card (ref image + Pinterest credit), a transition
card, and per-segment text overlays on the right side of the
webcam frame. Output is a YouTube-ready mp4.

Run on the DEV PC:

    poetry run python scripts/eye_demo_compose.py            # uses most-recent
    poetry run python scripts/eye_demo_compose.py <timing.json>

Story 7.1a · 2026-05-20.
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
OUT  = REPO / "docs" / "captures"
REF  = REPO / ".ai" / "eye_ref_sheet.jpg"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
W, H = 1280, 720

INTRO_S = 6.0
TRANS_S = 3.0

# Per-level brief descriptions shown under the title on the right.
DESC: dict[str, dict[str, str]] = {
    "neutral":     {"L1": "round eye, pupils nudged nasal",
                    "L2": "—",
                    "L3": "—"},
    "happy":       {"L1": "curved + slanted bottom crop\nsmall low pupil",
                    "L2": "+ tint #fdffdd",
                    "L3": "+ right-edge yellow sparkle"},
    "content":     {"L1": "Pleased crop\npupils close together",
                    "L2": "+ tint #edf2d8",
                    "L3": "+ soft outer glow-pulse"},
    "excited":     {"L1": "big pupil\nwhite sparkle inside pupil",
                    "L2": "+ hint #fcffcd",
                    "L3": "+ deep-yellow background sparkles"},
    "sad":         {"L1": "both eyes curved-top crop\npupils to eyeline",
                    "L2": "+ pale-blue + 3 purple slashes (R)",
                    "L3": "+ tear stream (L) + gloom lines"},
    "melancholic": {"L1": "Cold: top-crop, R smaller\npupils to screen-right",
                    "L2": "+ cold-blue tint",
                    "L3": "+ drifting gloom lines"},
    "sympathetic": {"L1": "Serious half-lid\nR eye smaller",
                    "L2": "+ soft-warm tint",
                    "L3": "+ teary glisten pulse"},
    "angry":       {"L1": "diagonal crops L/R\nno marks",
                    "L2": "+ subtle red #fab0a0",
                    "L3": "+ noticeable red #ff6040"},
    "frustrated":  {"L1": "Silly: flat top crop\npupils up-outward",
                    "L2": "+ hinted orange #ffd0a0",
                    "L3": "+ defined orange #ffa060"},
    "scared":      {"L1": "big tense eye, tiny pupil",
                    "L2": "+ pale-blue tint",
                    "L3": "+ tremble + shock lines"},
    "curious":     {"L1": "asymmetric, pupils up",
                    "L2": "+ light cyan + ?",
                    "L3": "+ bobbing ?"},
    "surprised":   {"L1": "wide raised eye, tiny pupil",
                    "L2": "+ ! + colour motion lines (L)",
                    "L3": "+ impact flash burst"},
    "sleepy":      {"L1": "heavy droopy lid",
                    "L2": "+ dim + ZZZ",
                    "L3": "+ drifting Z's"},
    "wink":        {"L1": "Devious: curved+slanted top\nR smaller, glance out",
                    "L2": "+ sparkle glint",
                    "L3": "+ animated twinkle"},
    "flirty":      {"L1": "small coy eye, bottom-crop\nglance out-up",
                    "L2": "+ pink + motion lines + hearts",
                    "L3": "+ floating pink hearts in BG"},
}


def latest_timing() -> Path:
    files = sorted(OUT.glob("eye_demo_timing_*.json"))
    if not files:
        sys.exit("No eye_demo_timing_*.json under docs/captures/")
    return files[-1]


def main() -> int:
    timing_json = Path(sys.argv[1]) if len(sys.argv) > 1 else latest_timing()
    meta = json.loads(timing_json.read_text())
    raw  = REPO / meta["raw_video"]
    if not raw.exists():
        sys.exit(f"Raw video missing: {raw}")

    warmup    = meta["warmup_s"]
    shot_s    = meta["shot_s"]
    n_shots   = len(meta["shots"])
    webcam_s  = n_shots * shot_s

    ts = time.strftime("%Y%m%d-%H%M%S")
    out_mp4 = OUT / f"eye_expression_demo_{ts}.mp4"

    # Text files (avoids drawtext escaping hell).
    tmpdir = Path(tempfile.mkdtemp(prefix="eye_demo_txt_"))
    intro_top = tmpdir / "intro_top.txt"
    intro_bot = tmpdir / "intro_bot.txt"
    trans_t   = tmpdir / "trans.txt"
    intro_top.write_text("This is the reference image I used")
    intro_bot.write_text("from Pinterest — thanks 3dDesignSchool")
    trans_t.write_text("Then we adapted it — one eye expression at a time")

    seg_files: list[tuple[Path, Path]] = []
    for k, s in enumerate(meta["shots"]):
        title = f"{s['expression']}  ·  L{s['level']}"
        body  = DESC.get(s["expression"], {}).get(f"L{s['level']}", "")
        tt = tmpdir / f"t{k}.txt"; tt.write_text(title)
        tb = tmpdir / f"b{k}.txt"; tb.write_text(body)
        seg_files.append((tt, tb))

    # Build the filter graph.
    parts: list[str] = []

    # [0] = ref image (looped 6s). Pad to 1280x720, then drawtext two lines.
    parts.append(
        f"[0:v]scale={W}:{H}:force_original_aspect_ratio=decrease,"
        f"pad={W}:{H}:(ow-iw)/2:(oh-ih)/2:black,setsar=1,fps=30,"
        f"drawtext=fontfile={FONT}:textfile={intro_top}:"
        f"x=(w-text_w)/2:y=40:fontcolor=white:fontsize=44:"
        f"box=1:boxcolor=black@0.7:boxborderw=18:"
        f"enable='between(t,0.4,{INTRO_S - 0.4})',"
        f"drawtext=fontfile={FONT}:textfile={intro_bot}:"
        f"x=(w-text_w)/2:y=h-90:fontcolor=white:fontsize=30:"
        f"box=1:boxcolor=black@0.7:boxborderw=14:"
        f"enable='between(t,0.4,{INTRO_S - 0.4})',"
        f"format=yuv420p[intro]"
    )

    # [1] = lavfi black 3s. Centred line.
    parts.append(
        f"[1:v]drawtext=fontfile={FONT}:textfile={trans_t}:"
        f"x=(w-text_w)/2:y=(h-text_h)/2:fontcolor=white:fontsize=40:"
        f"box=1:boxcolor=black@0.0:boxborderw=0:"
        f"enable='between(t,0.3,{TRANS_S - 0.3})',"
        f"format=yuv420p[trans]"
    )

    # [2] = raw webcam. Trim warmup, scale to 1280x720, fps 30,
    # then chain per-segment title+body drawtexts (enabled per range).
    parts.append(
        f"[2:v]trim=start={warmup}:duration={webcam_s},"
        f"setpts=PTS-STARTPTS,scale={W}:{H},fps=30,setsar=1[w0]"
    )
    cur = "w0"
    for k, (tt, tb) in enumerate(seg_files):
        t0 = round(k * shot_s, 3)
        t1 = round(t0 + shot_s, 3)
        nxt = f"w{k + 1}"
        # Title (large, top-right).
        parts.append(
            f"[{cur}]drawtext=fontfile={FONT}:textfile={tt}:"
            f"x={W // 2 + 40}:y=200:fontcolor=white:fontsize=48:"
            f"box=1:boxcolor=black@0.55:boxborderw=16:"
            f"enable='between(t,{t0},{t1})'[{nxt}t]"
        )
        cur_after_title = f"{nxt}t"
        # Body (smaller, below title). Skip if empty / "—".
        body = tb.read_text().strip()
        if body and body != "—":
            parts.append(
                f"[{cur_after_title}]drawtext=fontfile={FONT}:textfile={tb}:"
                f"x={W // 2 + 40}:y=290:fontcolor=white:fontsize=28:"
                f"line_spacing=10:"
                f"box=1:boxcolor=black@0.45:boxborderw=12:"
                f"enable='between(t,{t0},{t1})'[{nxt}]"
            )
            cur = nxt
        else:
            cur = cur_after_title

    parts.append(f"[{cur}]format=yuv420p[webcam]")
    parts.append("[intro][trans][webcam]concat=n=3:v=1:a=0[v]")

    filter_complex = ";".join(parts)
    # Write filter to a file too (ffmpeg has CLI length limits).
    fc_file = tmpdir / "filter.txt"
    fc_file.write_text(filter_complex)

    cmd = [
        "ffmpeg", "-y",
        "-loop", "1", "-t", str(INTRO_S), "-i", str(REF),
        "-f", "lavfi", "-t", str(TRANS_S),
            "-i", f"color=black:size={W}x{H}:rate=30",
        "-i", str(raw),
        "-filter_complex_script", str(fc_file),
        "-map", "[v]",
        "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(out_mp4),
    ]
    print(f"Composing → {out_mp4.name}")
    print(f"  intro {INTRO_S:.0f}s + trans {TRANS_S:.0f}s + webcam {webcam_s:.0f}s "
          f"= {INTRO_S + TRANS_S + webcam_s:.0f}s")
    rc = subprocess.call(cmd)
    if rc != 0:
        sys.exit(f"ffmpeg failed (rc={rc})")
    sz_mb = out_mp4.stat().st_size / (1024 * 1024)
    print(f"Done: {out_mp4}  ({sz_mb:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
