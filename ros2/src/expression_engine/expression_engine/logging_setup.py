"""Structured journald logging — Story 6.1 (NFR8, minimal).

System-wide requirement: structured JSON log lines to journald.
systemd captures a service's stderr into the journal, so a JSON
formatter on a stderr handler is the minimal correct setup. Full
logging hardening (rotation, levels, correlation propagation) is
Story 6.7 — keep this small and self-contained.

Small SRP leaf module supporting node.py's startup; the architecture
§3 core modules are unchanged.
"""

from __future__ import annotations

import json
import logging
import sys

_LOGGER_NAME = "expression_engine"
_configured = False


class _JsonFormatter(logging.Formatter):
    """One JSON object per line: ts, level, logger, event, + extras."""

    def format(self, record: logging.LogRecord) -> str:
        entry = {
            "ts": self.formatTime(record, "%Y-%m-%dT%H:%M:%S%z"),
            "level": record.levelname,
            "logger": record.name,
            "event": record.getMessage(),
        }
        extra = getattr(record, "fields", None)
        if isinstance(extra, dict):
            entry.update(extra)
        if record.exc_info:
            entry["exc"] = self.formatException(record.exc_info)
        return json.dumps(entry, default=str)


def setup_logging() -> logging.Logger:
    """Idempotently configure and return the engine logger."""
    global _configured
    logger = logging.getLogger(_LOGGER_NAME)
    if not _configured:
        handler = logging.StreamHandler(sys.stderr)
        handler.setFormatter(_JsonFormatter())
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = False
        _configured = True
    return logger


def log_event(level: int, event: str, **fields) -> None:
    """Emit a structured journald line: ``event`` + arbitrary fields."""
    setup_logging().log(level, event, extra={"fields": fields})
