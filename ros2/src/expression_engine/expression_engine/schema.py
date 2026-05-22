"""Schema-3 wire contract — Story 6.1 Task 2 (AC #2, #3; FR2, FR4, AR13).

RE-DERIVED, NOT IMPORTED. There is intentionally zero cross-repo build
coupling (AR13). These models mirror the companion's wire contract:

  olaf_companion @ tag v3.0.0  (commit 321d9f8)
    src/voice_agent_pipeline/schemas/envelope.py
    src/voice_agent_pipeline/schemas/mood_event.py
    src/voice_agent_pipeline/schemas/activity_event.py
    src/voice_agent_pipeline/schemas/speech_emotion_event.py
    src/voice_agent_pipeline/schemas/vocalization_event.py
  build_documents/planning-artifacts/olaf-embodiment-brief.md §Appendix A

Hand-mirrored models drift; Story 6.2 adds a cross-check against
:data:`INTERFACE_VERSION` (the map carries ``interface_version`` and
the loader asserts it matches). Any contract edit here MUST cite the
companion source module + Appendix A section it mirrors.

Fail-fast posture (FR4, NFR7, brief §A.3 "Subscriber rule"): a wire
``schema_version`` other than 3 is fatal — :func:`assert_schema_version`
raises :class:`SchemaVersionError`; the subscriber boundary logs a
structured journald line and ``sys.exit(1)`` (symmetry with the
pipeline's own ``SchemaVersionError`` → exit 1). No coercion, no
truncation, no best-effort. This is distinct from FR13's *runtime*
unknown-name graceful fallback (Story 6.2 — do not conflate).
"""

from __future__ import annotations

import json
from datetime import datetime
from typing import Literal
from uuid import UUID

from pydantic import BaseModel, ConfigDict, Field, model_validator

#: Body-owned interface contract version (semver). MIRRORS
#: ``contract/VERSION`` — cross-checked by ``test/test_contract.py`` so
#: the two can never drift. Story 7.6 made the expression interface a
#: body-owned, versioned standard: the body no longer pins the
#: companion's *release* (the old ``PINNED_COMPANION_TAG = "v3.0.0"``);
#: it declares the *interface* version it speaks. This is DISTINCT from
#: the wire :data:`SUPPORTED_SCHEMA_VERSION` below (the envelope-format
#: major gate) — bump INTERFACE_VERSION for any vocabulary / envelope
#: governance change (major = breaking, minor = additive/forward-compat).
INTERFACE_VERSION = "1.0.0"

#: The only wire envelope-format version this engine accepts (brief
#: §A.3/§A.8) — the major-fatal gate (:func:`assert_schema_version`).
#: A breaking *wire* change bumps this AND INTERFACE_VERSION's major.
SUPPORTED_SCHEMA_VERSION = 3

#: Envelope config — STRICT: a stray/typo envelope field fails loudly.
_FROZEN = ConfigDict(frozen=True, extra="forbid")

#: Payload config — frozen but FORWARD-COMPATIBLE (Story 7.6): unknown
#: payload fields are IGNORED, not fatal. A producer on a newer
#: interface MINOR may add an additive payload field and an older body
#: tolerates it. Only payloads relax; the envelope stays strict.
_PAYLOAD_CONFIG = ConfigDict(frozen=True, extra="ignore")


class SchemaVersionError(Exception):
    """Raised when a wire event's ``schema_version`` is not 3 (FR4).

    Carries structured context (not an f-string) so the subscriber can
    emit a structured JSON journald line (NFR8), mirroring the
    pipeline's ``config/version.py`` ``SchemaVersionError(found,
    supported, source)``.
    """

    def __init__(self, found: object, supported: int, source: object) -> None:
        self.found = found
        self.supported = supported
        self.source = source
        super().__init__(
            f"unsupported schema_version {found!r} "
            f"(this engine supports {supported}); source={source!r}"
        )


# ── Payloads — mirror companion schemas/*_event.py @ v3.0.0 ──────────

#: schemas/mood_event.py:Mood — code-level Literal (Appendix A.4).
Mood = Literal[
    "calm", "happy", "playful", "curious",
    "thoughtful", "sleepy", "grumpy", "excited",
]

#: schemas/activity_event.py:ActivityState (Appendix A.5).
ActivityState = Literal[
    "starting", "sleeping", "waking", "listening",
    "working", "speaking", "going_to_sleep",
]
#: schemas/activity_event.py:WorkingSubmode (Appendix A.5).
WorkingSubmode = Literal["thinking", "delegating"]

# ── Pinned canonical vocabulary @ v3.0.0 (Story 6.2 completeness src) ─
#
# `SpeechEmotionPayload.emotion` and `VocalizationPayload.tag` are
# `str` on the wire (forward-compat — subscribers ignore unknowns,
# FR13). But the *required map-coverage* set for a given pinned
# companion release IS fixed. These tuples are that pinned set,
# re-derived from brief §A.6/§A.7 @ INTERFACE_VERSION. They are the
# single source of truth for the Story 6.2 map-loader completeness
# check — do NOT re-list canonical names anywhere else.

#: 12 first-class speech-emotion names (brief §A.6).
SPEECH_EMOTION_CANONICAL: tuple[str, ...] = (
    # Primary (6)
    "neutral", "content", "excited", "sad", "angry", "scared",
    # Secondary (6)
    "happy", "curious", "sympathetic", "surprised",
    "frustrated", "melancholic",
)

#: 6 v1 vocalization tags (brief §A.7).
VOCALIZATION_TAGS: tuple[str, ...] = (
    # audio bursts
    "laughter", "sigh", "gasp", "clears_throat",
    # gesture cues — MUST be visible_only:true in the map (FR7)
    "nod", "shake",
)

#: Gesture-cue vocalizations that MUST carry ``visible_only: true``
#: (brief §A.7 policy split — silent gestures, no audio).
VOCALIZATION_GESTURE_CUES: tuple[str, ...] = ("nod", "shake")


class MoodPayload(BaseModel):
    """Mirror of companion ``schemas/mood_event.py:MoodPayload`` (A.4)."""

    model_config = _PAYLOAD_CONFIG

    mood: Mood
    reason: str | None = None


class ActivityPayload(BaseModel):
    """Mirror of companion ``schemas/activity_event.py:ActivityPayload``.

    Invariants re-derived verbatim (Appendix A.5):
      - ``working_submode`` non-null iff ``state == "working"``.
      - ``from_state`` is ``None`` iff this is the initial
        ``state == "starting"`` event.
    """

    model_config = _PAYLOAD_CONFIG

    state: ActivityState
    working_submode: WorkingSubmode | None = None
    transition_reason: str | None = None
    from_state: ActivityState | None = None

    @model_validator(mode="after")
    def _check_working_submode(self) -> "ActivityPayload":
        if self.state == "working" and self.working_submode is None:
            raise ValueError("working_submode required when state='working'")
        if self.state != "working" and self.working_submode is not None:
            raise ValueError("working_submode allowed only when state='working'")
        return self

    @model_validator(mode="after")
    def _check_from_state(self) -> "ActivityPayload":
        if self.state == "starting" and self.from_state is not None:
            raise ValueError("from_state must be None when state='starting'")
        if self.state != "starting" and self.from_state is None:
            raise ValueError("from_state required when state != 'starting'")
        return self


class SpeechEmotionPayload(BaseModel):
    """Mirror of ``schemas/speech_emotion_event.py`` (A.6).

    Schema-3: the pre-3 ``expression_data`` field is GONE (A.8). The
    renderer mapping is the engine's own concern, keyed on ``emotion``
    (12 canonical names — enforced by the Story 6.2 map loader, NOT
    here). ``audio_frame_id`` is ``str | None`` (not int).
    """

    model_config = _PAYLOAD_CONFIG

    emotion: str
    source_tag: str
    audio_frame_id: str | None = None
    raw_tag: str
    resolved_fallback: str | None


class VocalizationPayload(BaseModel):
    """Mirror of ``schemas/vocalization_event.py`` (A.7)."""

    model_config = _PAYLOAD_CONFIG

    tag: str
    audio_frame_id: str | None = None
    tts_supported: bool


# ── Envelope + typed events — mirror companion envelope.py @ v3.0.0 ──


class EventEnvelope(BaseModel):
    """Mirror of companion ``schemas/envelope.py:EventEnvelope`` (A.3).

    ``frozen=True`` → events are safe to hand between the rclpy
    executor thread and the (later) render thread without defensive
    copies. ``extra="forbid"`` → a wire typo fails loudly. Field
    defaults exist for standalone construction; published events
    always carry all five (brief §A.3).
    """

    model_config = _FROZEN

    schema_version: int = SUPPORTED_SCHEMA_VERSION
    timestamp: datetime
    #: Producer identity. Story 7.6 relaxed this from the
    #: ``"voice_agent_pipeline"`` literal to ANY non-empty id — a
    #: body-owned interface must not name one producer (the companion
    #: is just one publisher; a test rig / joystick are others).
    source: str = Field(min_length=1)
    correlation_id: UUID
    payload: BaseModel


class MoodEvent(EventEnvelope):
    """``mood`` topic — transient_local/latched depth 1 (A.2/A.4)."""

    payload: MoodPayload  # type: ignore[assignment]


class ActivityEvent(EventEnvelope):
    """``activity`` topic — transient_local/latched depth 1 (A.2/A.5)."""

    payload: ActivityPayload  # type: ignore[assignment]


class SpeechEmotionEvent(EventEnvelope):
    """``speech_emotion`` topic — volatile depth 8 (A.2/A.6)."""

    payload: SpeechEmotionPayload  # type: ignore[assignment]


class VocalizationEvent(EventEnvelope):
    """``vocalization`` topic — volatile depth 8 (A.2/A.7)."""

    payload: VocalizationPayload  # type: ignore[assignment]


#: Canonical topic key -> concrete typed event model. Keys match
#: config.REQUIRED_TOPICS and the companion's [publisher.topics].
EVENT_MODELS: dict[str, type[EventEnvelope]] = {
    "mood": MoodEvent,
    "activity": ActivityEvent,
    "speech_emotion": SpeechEmotionEvent,
    "vocalization": VocalizationEvent,
}


_MISSING = object()


def assert_schema_version(data: dict) -> None:
    """Fail fast unless ``data['schema_version']`` is exactly int ``3``.

    Checked on the raw dict *before* typed validation so a version
    problem surfaces as :class:`SchemaVersionError` (the FR4 fatal
    path) rather than a generic pydantic error.

    Hardened (code review 2026-05-17):
      - **Absent** ``schema_version`` is fatal. The companion always
        sends all five envelope fields (brief §A.3); a missing version
        is a contract breach, not a v3 default (the model default
        exists only for standalone construction). Previously a
        version-less publisher was silently accepted as v3 — exactly
        the FR4 silent-drift case.
      - A **non-int** version (e.g. the string ``"3"``) is fatal, not
        a crash elsewhere; ``3.0`` is accepted (JSON number that
        equals 3).

    Raises:
        SchemaVersionError: ``schema_version`` absent, wrong type, or
            not equal to 3.
    """
    version = data.get("schema_version", _MISSING)
    if version is _MISSING:
        raise SchemaVersionError(
            found=None,
            supported=SUPPORTED_SCHEMA_VERSION,
            source=data.get("source"),
        )
    # bool is an int subclass — reject it explicitly. Accept int or a
    # float that is integer-valued (JSON has no int/float distinction).
    ok = (
        isinstance(version, int) and not isinstance(version, bool)
    ) or (
        isinstance(version, float) and version.is_integer()
    )
    if not ok or int(version) != SUPPORTED_SCHEMA_VERSION:
        raise SchemaVersionError(
            found=version,
            supported=SUPPORTED_SCHEMA_VERSION,
            source=data.get("source"),
        )


def parse_event(topic_key: str, raw: str | bytes) -> EventEnvelope:
    """Deserialize a wire ``std_msgs/String`` body into a typed event.

    Args:
        topic_key: One of :data:`EVENT_MODELS` (canonical topic key).
        raw: The JSON envelope string from ``std_msgs/String.data``.

    Returns:
        The concrete typed event (``MoodEvent`` / ``ActivityEvent`` /
        ``SpeechEmotionEvent`` / ``VocalizationEvent``).

    Raises:
        KeyError: ``topic_key`` is not a known canonical topic.
        ValueError: ``raw`` is not valid JSON.
        SchemaVersionError: ``schema_version`` present and != 3 (FR4).
        pydantic.ValidationError: envelope/payload shape is invalid
            (malformed, short, extra field, bad enum, invariant
            violation) — Task 5.
    """
    model = EVENT_MODELS[topic_key]
    try:
        obj = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"{topic_key}: payload is not valid JSON: {exc}") from exc
    if not isinstance(obj, dict):
        raise ValueError(f"{topic_key}: envelope must be a JSON object")
    assert_schema_version(obj)
    return model.model_validate(obj)
