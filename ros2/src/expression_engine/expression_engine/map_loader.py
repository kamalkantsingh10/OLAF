"""expression_map.yaml loader + vocabulary completeness — Story 6.2.

Two distinct failure modes (Dev Notes — do NOT conflate):

- **Startup (FR6/NFR7, AR8):** the map missing a canonical name the
  pinned companion release defines → **fatal**. `MapValidationError`
  (a `ValueError` subclass, so node.main's existing fail-fast handler
  catches it → structured journald + exit 1, same posture as 6.1).
- **Runtime (FR13):** an event arrives with a canonical name the map
  lacks (engine map lags companion) → **graceful**: WARN
  `expression.unmapped_<topic>` + the `defaults` render, never raise.

Strict-at-startup, graceful-at-runtime — that asymmetry is the design
(NFR5). The required canonical set is derived from `schema.py`
(single source of truth, re-derived @ PINNED_COMPANION_TAG); never
re-listed by hand. Extra map entries beyond the pinned set are
ACCEPTED — vocabulary growth is a pure YAML edit (NFR5).

Disambiguation is BY TOPIC: `mood.happy` and `speech_emotion.happy`
are distinct keys (architecture §5.1/§5.2, brief #4).

Scope: pure/testable map data + the FR13 resolver. No rendering, no
hardware, no render loop (Story 6.3).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, get_args

import yaml

from expression_engine import schema
from expression_engine.logging_setup import log_event

_TOPICS = ("mood", "activity", "speech_emotion", "vocalization")
_REQUIRED_TOP_LEVEL = (
    "schema_version",
    "pinned_companion_tag",
    "defaults",
    *_TOPICS,
)


class MapValidationError(ValueError):
    """Startup-fatal expression_map.yaml defect (FR6/FR7/NFR7, AR8).

    Subclasses ``ValueError`` so node.main's existing
    ``except (FileNotFoundError, ValueError)`` fail-fast path handles
    it identically to a bad config (consistency with Story 6.1).
    """


@dataclass(frozen=True)
class ExpressionMap:
    """Validated, queryable map data + the FR13 fallback resolver."""

    schema_version: int
    pinned_companion_tag: str
    defaults: dict[str, Any]
    mood: dict[str, Any]
    activity: dict[str, Any]
    speech_emotion: dict[str, Any]
    vocalization: dict[str, Any]

    def _topic(self, topic: str) -> dict[str, Any]:
        if topic not in _TOPICS:
            raise KeyError(f"unknown expression topic: {topic!r}")
        return getattr(self, topic)

    def resolve(self, topic: str, name: str) -> dict[str, Any]:
        """Return the map entry for ``name`` on ``topic``.

        FR13: an unmapped *name* is graceful — log
        ``expression.unmapped_<topic>`` at WARN and return the
        ``defaults`` render. Never raises, never freezes. An unknown
        *topic* is a programming error and raises ``KeyError``.
        """
        entry = self._topic(topic).get(name)
        if entry is None:
            log_event(
                logging.WARNING,
                f"expression.unmapped_{topic}",
                topic=topic,
                name=name,
            )
            return self.defaults
        return entry


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise MapValidationError(msg)


def _assert_complete(
    block: dict[str, Any], required: tuple[str, ...], topic: str
) -> None:
    missing = sorted(set(required) - set(block))
    _require(
        not missing,
        f"expression_map.yaml: '{topic}' is missing canonical "
        f"entries for the pinned companion set "
        f"({schema.PINNED_COMPANION_TAG}): {missing}. "
        f"Add them to the map — startup is fatal until complete "
        f"(FR6/NFR7).",
    )


def load_expression_map(path: str | Path) -> ExpressionMap:
    """Load + fully validate the map; fail fast on any startup defect.

    Raises:
        FileNotFoundError: the map file is absent.
        MapValidationError: malformed YAML, a missing top-level key, a
            pinned-tag mismatch, an incomplete canonical vocabulary
            (FR6), or nod/shake without ``visible_only: true`` (FR7).
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"expression_map.yaml not found: {path}")

    try:
        with path.open() as fh:
            raw = yaml.safe_load(fh)
    except yaml.YAMLError as exc:
        raise MapValidationError(f"invalid YAML in {path}: {exc}") from exc

    _require(
        isinstance(raw, dict),
        f"{path}: expression_map.yaml must be a YAML mapping",
    )
    for key in _REQUIRED_TOP_LEVEL:
        _require(key in raw, f"{path}: missing required top-level key '{key}'")
    for key in (*_TOPICS, "defaults"):
        _require(
            isinstance(raw[key], dict),
            f"{path}: top-level '{key}' must be a mapping",
        )

    # Lockstep: the map's pinned tag must equal the tag schema.py was
    # re-derived from (the NFR5 enforcement promised in Story 6.1).
    _require(
        raw["pinned_companion_tag"] == schema.PINNED_COMPANION_TAG,
        f"{path}: pinned_companion_tag "
        f"{raw['pinned_companion_tag']!r} != schema.py "
        f"{schema.PINNED_COMPANION_TAG!r}. The map and the re-derived "
        f"schema must move in lockstep (NFR5, §12.4).",
    )

    activity = raw["activity"]
    # Completeness vs the pinned canonical set (single source: schema).
    _assert_complete(raw["mood"], get_args(schema.Mood), "mood")
    _assert_complete(activity, get_args(schema.ActivityState), "activity")
    _require(
        isinstance(activity.get("working"), dict),
        f"{path}: 'activity.working' must be a mapping nested by "
        f"working_submode (thinking | delegating)",
    )
    _assert_complete(
        activity["working"], get_args(schema.WorkingSubmode), "activity.working"
    )
    _assert_complete(
        raw["speech_emotion"], schema.SPEECH_EMOTION_CANONICAL, "speech_emotion"
    )
    _assert_complete(
        raw["vocalization"], schema.VOCALIZATION_TAGS, "vocalization"
    )

    # FR7 / AR8 step 4 — gesture cues are silent: visible_only MUST be
    # present AND exactly True.
    for cue in schema.VOCALIZATION_GESTURE_CUES:
        entry = raw["vocalization"][cue]
        _require(
            isinstance(entry, dict) and entry.get("visible_only") is True,
            f"{path}: vocalization '{cue}' must set "
            f"visible_only: true (FR7 — silent gesture, no audio). "
            f"Missing or false is fatal.",
        )

    return ExpressionMap(
        schema_version=raw["schema_version"],
        pinned_companion_tag=raw["pinned_companion_tag"],
        defaults=raw["defaults"],
        mood=raw["mood"],
        activity=activity,
        speech_emotion=raw["speech_emotion"],
        vocalization=raw["vocalization"],
    )
