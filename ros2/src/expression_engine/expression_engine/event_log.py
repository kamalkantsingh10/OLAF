"""File logs for what the body RECEIVES (Story 7.6 debug aid).

Writes two append-only files under ``$OLAF_LOG_DIR`` — set by
``start-body.sh`` (default ``<repo>/logs``); set ``OLAF_LOG_DIR=off`` to
disable. **Opt-in by env**, so the host test suite (no env) never writes
files.

- ``received.log``     — every ROS message the body accepts, all four
  topics, verbatim payload. The authoritative answer to "did the body
  actually get this event?".
- ``conversation.log`` — a readable narrative of the interaction
  (activity transitions, mood, speech emotion, vocalizations).

Append + best-effort: a logging failure must NEVER break the engine.
"""

from __future__ import annotations

import logging
import os
import threading
from datetime import datetime
from pathlib import Path

from expression_engine.logging_setup import publish_rosout
from expression_engine.schema import EventEnvelope

_lock = threading.Lock()
_dir: Path | None = None
_resolved = False


def _logdir() -> Path | None:
    global _dir, _resolved
    if _resolved:
        return _dir
    _resolved = True
    env = os.environ.get("OLAF_LOG_DIR")
    if not env or env.lower() in ("off", "none", "0", "disabled"):
        _dir = None
        return None
    path = Path(env).expanduser()
    try:
        path.mkdir(parents=True, exist_ok=True)
        _dir = path
    except OSError:
        _dir = None
    return _dir


def _append(filename: str, line: str) -> None:
    d = _logdir()
    if d is None:
        return
    try:
        with _lock, (d / filename).open("a") as fh:
            fh.write(line + "\n")
    except OSError:
        pass  # best-effort — never let logging break the engine


def log_received(topic_key: str, event: EventEnvelope) -> None:
    """Record an accepted ROS message → received.log + /rosout."""
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    payload = event.payload.model_dump(exclude_none=True)
    line = f"{topic_key:<15} {payload}  src={event.source}"
    _append("received.log", f"{ts}  {line}")
    publish_rosout(logging.INFO, f"rx  {line}")


def log_conversation(topic_key: str, event: EventEnvelope) -> None:
    """Record a readable narrative line → conversation.log + /rosout."""
    ts = datetime.now().strftime("%H:%M:%S")
    p = event.payload
    line: str | None = None
    if topic_key == "activity":
        sub = f"/{p.working_submode}" if getattr(p, "working_submode", None) else ""
        reason = f"  ({p.transition_reason})" if getattr(p, "transition_reason", None) else ""
        line = f"{ts}  →  {p.state}{sub}{reason}"
    elif topic_key == "mood":
        line = f"{ts}  mood: {p.mood}" + (f"  ({p.reason})" if p.reason else "")
    elif topic_key == "speech_emotion":
        line = f"{ts}  speaks  [{p.emotion}]"
    elif topic_key == "vocalization":
        line = f"{ts}  *{p.tag}*"
    if line is not None:
        _append("conversation.log", line)
        publish_rosout(logging.INFO, line)
