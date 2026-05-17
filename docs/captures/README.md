# Captures

Webcam shots of OLAF for Epic 7 expression verification (dev-PC cam → bot).
Capture method: see memory `reference_webcam_capture.md` (native MJPEG via
gst pipewiresrc; do NOT decode→re-encode — green-band artifact).

- **Stills** (`YYYYMMDD-HHMMSS-<emotion>.jpg`): tracked — authoring evidence.
- **Video** (`*.avi` etc.): git-ignored — kept locally; analysed by
  extracting frames (`ffmpeg -i clip.avi -vf "select=eq(n\,N)" -frames:v 1`)
  at t=0 / mid-ease / settled to verify easing & gesture transitions.
- `bot_*.jpg` raw multi-frame grabs are scratch (ignored); keep only the
  meaningfully-named final shot.
