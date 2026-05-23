"""Engine logging — Story 6.1 (NFR8) + readable console (Story 7.6).

Two formats, auto-selected so each context gets the right one:

* **Interactive terminal** (stderr is a TTY — e.g. `just body-up`):
  human-readable, colour-coded lines so a crash JUMPS out. Errors and
  the fatal startup paths print the full traceback.
* **Piped / journald** (non-TTY — systemd): structured JSON, one object
  per line (NFR8), so the journal stays machine-parseable.

Overrides: ``OLAF_LOG_JSON=1`` forces JSON anywhere; ``OLAF_LOG_HUMAN=1``
forces the readable format anywhere. Full hardening (rotation, levels,
correlation propagation) is still Story 6.7.
"""

from __future__ import annotations

import json
import logging
import os
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


class _HumanFormatter(logging.Formatter):
    """Readable ``HH:MM:SS LEVEL  event  k=v ...`` (+ colour + traceback)."""

    _COLOR = {
        "DEBUG": "\033[2m",        # dim
        "INFO": "\033[36m",        # cyan
        "WARNING": "\033[33m",     # yellow
        "ERROR": "\033[31m",       # red
        "CRITICAL": "\033[1;37;41m",  # bold white on red — crashes JUMP out
    }
    _RESET = "\033[0m"

    def __init__(self, color: bool) -> None:
        super().__init__()
        self._color = color

    def format(self, record: logging.LogRecord) -> str:
        ts = self.formatTime(record, "%H:%M:%S")
        level = record.levelname
        event = record.getMessage()
        extra = getattr(record, "fields", None)
        kv = ""
        if isinstance(extra, dict) and extra:
            kv = "  " + " ".join(f"{k}={v}" for k, v in extra.items())
        line = f"{ts} {level:<8} {event}{kv}"
        if self._color and level in self._COLOR:
            line = f"{self._COLOR[level]}{line}{self._RESET}"
        if record.exc_info:
            line += "\n" + self.formatException(record.exc_info)
        return line


def _choose_formatter() -> logging.Formatter:
    if os.environ.get("OLAF_LOG_JSON"):
        return _JsonFormatter()
    is_tty = sys.stderr.isatty()
    if os.environ.get("OLAF_LOG_HUMAN") or is_tty:
        return _HumanFormatter(color=is_tty)
    return _JsonFormatter()  # non-TTY (journald/pipe) → JSON (NFR8)


#: The node's logger, registered by node.main once the rclpy node
#: exists. A NODE logger publishes to /rosout; a bare
#: rclpy.logging.get_logger() does NOT (it only writes the console).
_ros_logger = None


def set_ros_logger(logger) -> None:
    """Register ``node.get_logger()`` so logs also publish to /rosout.

    Call once, right after the node is created. Until then,
    :func:`publish_rosout` is a no-op (so host tests stay quiet).
    """
    global _ros_logger
    _ros_logger = logger


def publish_rosout(levelno: int, msg: str) -> None:
    """Publish ``msg`` to the standard ROS ``/rosout`` topic (Story 7.6).

    Lets the body be watched from another machine
    (``ros2 topic echo /rosout`` / rqt_console). Does NOT violate
    subscribe-only (the FR3 test permits ``/rosout``). No-op until the
    node logger is registered via :func:`set_ros_logger`.
    """
    logger = _ros_logger
    if logger is None:
        return
    try:
        from rclpy.logging import LoggingSeverity

        sev = {
            logging.DEBUG: LoggingSeverity.DEBUG,
            logging.INFO: LoggingSeverity.INFO,
            logging.WARNING: LoggingSeverity.WARN,
            logging.ERROR: LoggingSeverity.ERROR,
            logging.CRITICAL: LoggingSeverity.FATAL,
        }.get(levelno, LoggingSeverity.INFO)
        logger.log(msg, sev)
    except Exception:
        pass


class _RosoutHandler(logging.Handler):
    """Forward engine log records to ``/rosout`` (network-visible)."""

    def emit(self, record: logging.LogRecord) -> None:
        extra = getattr(record, "fields", None)
        kv = ""
        if isinstance(extra, dict) and extra:
            kv = "  " + " ".join(f"{k}={v}" for k, v in extra.items())
        publish_rosout(record.levelno, f"{record.getMessage()}{kv}")


def setup_logging() -> logging.Logger:
    """Idempotently configure and return the engine logger."""
    global _configured
    logger = logging.getLogger(_LOGGER_NAME)
    if not _configured:
        handler = logging.StreamHandler(sys.stderr)
        handler.setFormatter(_choose_formatter())
        logger.addHandler(handler)
        logger.addHandler(_RosoutHandler())  # also publish to /rosout
        logger.setLevel(logging.INFO)
        logger.propagate = False
        _configured = True
    return logger


def log_event(level: int, event: str, exc_info: bool = False, **fields) -> None:
    """Emit a log line: ``event`` + arbitrary fields.

    Pass ``exc_info=True`` from inside an ``except`` block to attach the
    traceback (rendered inline in human mode, as ``exc`` in JSON).
    """
    setup_logging().log(level, event, exc_info=exc_info, extra={"fields": fields})
