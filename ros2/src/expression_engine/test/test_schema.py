"""Story 6.1 Task 2/5 — schema-3 model tests (AC #2, #3).

Re-derived from companion `olaf_companion` @ v3.0.0:
  src/voice_agent_pipeline/schemas/*_event.py  +  brief Appendix A.

AC #2 — a valid schema-3 envelope on any of the four topics
        deserializes into typed objects.
AC #3 — schema_version != 3 raises SchemaVersionError (no coercion).
Task 5 — malformed / short envelopes are rejected.

Process-exit (AC #3 "exits non-zero") is asserted in
test_subscribe_only.py at the subscriber boundary.
"""

import json
import uuid
from datetime import datetime, timezone

import pytest
from pydantic import ValidationError

from expression_engine.schema import (
    PINNED_COMPANION_TAG,
    SUPPORTED_SCHEMA_VERSION,
    ActivityEvent,
    MoodEvent,
    SchemaVersionError,
    SpeechEmotionEvent,
    VocalizationEvent,
    assert_schema_version,
    parse_event,
)

TS = "2026-05-10T13:42:18.123456+00:00"
CID = "12345678-1234-5678-1234-567812345678"


def envelope(payload: dict, version: int = 3) -> str:
    return json.dumps(
        {
            "schema_version": version,
            "timestamp": TS,
            "source": "voice_agent_pipeline",
            "correlation_id": CID,
            "payload": payload,
        }
    )


class TestPinnedTag:
    def test_pinned_companion_tag_is_v300(self):
        # Source-of-truth string; cross-checked in Story 6.2.
        assert PINNED_COMPANION_TAG == "v3.0.0"

    def test_supported_version_is_3(self):
        assert SUPPORTED_SCHEMA_VERSION == 3


class TestValidSchema3:  # AC #2
    def test_mood_parses_typed(self):
        ev = parse_event("mood", envelope({"mood": "happy", "reason": "set_mood tool"}))
        assert isinstance(ev, MoodEvent)
        assert ev.payload.mood == "happy"
        assert ev.payload.reason == "set_mood tool"
        assert ev.schema_version == 3
        assert ev.source == "voice_agent_pipeline"
        assert isinstance(ev.timestamp, datetime)
        assert ev.timestamp == datetime(2026, 5, 10, 13, 42, 18, 123456, tzinfo=timezone.utc)
        assert ev.correlation_id == uuid.UUID(CID)

    def test_mood_reason_optional(self):
        ev = parse_event("mood", envelope({"mood": "calm"}))
        assert ev.payload.reason is None

    def test_activity_starting_no_from_state(self):
        ev = parse_event("activity", envelope({"state": "starting"}))
        assert isinstance(ev, ActivityEvent)
        assert ev.payload.state == "starting"
        assert ev.payload.from_state is None
        assert ev.payload.working_submode is None

    def test_activity_working_requires_submode(self):
        ev = parse_event(
            "activity",
            envelope({"state": "working", "working_submode": "delegating", "from_state": "listening"}),
        )
        assert ev.payload.working_submode == "delegating"

    def test_speech_emotion_parses_typed(self):
        ev = parse_event(
            "speech_emotion",
            envelope(
                {
                    "emotion": "excited",
                    "source_tag": "enthusiastic",
                    "audio_frame_id": "frame-42",
                    "raw_tag": "enthusiastic",
                    "resolved_fallback": "high_energy_positive",
                }
            ),
        )
        assert isinstance(ev, SpeechEmotionEvent)
        assert ev.payload.emotion == "excited"
        assert ev.payload.audio_frame_id == "frame-42"
        assert ev.payload.resolved_fallback == "high_energy_positive"

    def test_speech_emotion_nullable_fields(self):
        ev = parse_event(
            "speech_emotion",
            envelope({"emotion": "neutral", "source_tag": "neutral", "raw_tag": "neutral", "resolved_fallback": None}),
        )
        assert ev.payload.audio_frame_id is None
        assert ev.payload.resolved_fallback is None

    def test_vocalization_parses_typed(self):
        ev = parse_event(
            "vocalization",
            envelope({"tag": "laughter", "audio_frame_id": "frame-7", "tts_supported": True}),
        )
        assert isinstance(ev, VocalizationEvent)
        assert ev.payload.tag == "laughter"
        assert ev.payload.tts_supported is True


class TestSchemaVersionFailFast:  # AC #3
    @pytest.mark.parametrize("bad", [2, 4, 1, 0, 99])
    def test_wrong_version_raises_schema_version_error(self, bad):
        with pytest.raises(SchemaVersionError) as exc:
            parse_event("mood", envelope({"mood": "calm"}, version=bad))
        assert exc.value.found == bad
        assert exc.value.supported == 3
        assert exc.value.source == "voice_agent_pipeline"

    def test_assert_schema_version_helper(self):
        assert_schema_version({"schema_version": 3, "source": "voice_agent_pipeline"})
        with pytest.raises(SchemaVersionError):
            assert_schema_version({"schema_version": 2, "source": "voice_agent_pipeline"})

    def test_no_coercion_no_truncation(self):
        # A version-2 envelope that is otherwise a valid mood payload
        # must NOT be silently accepted/coerced to 3.
        with pytest.raises(SchemaVersionError):
            parse_event("mood", envelope({"mood": "happy"}, version=2))


class TestMalformedRejected:  # Task 5
    def test_not_json(self):
        with pytest.raises(ValueError):
            parse_event("mood", "{not json")

    def test_short_envelope_missing_payload(self):
        raw = json.dumps({"schema_version": 3, "source": "voice_agent_pipeline"})
        with pytest.raises(ValidationError):
            parse_event("mood", raw)

    def test_extra_forbidden_field(self):
        with pytest.raises(ValidationError):
            parse_event("mood", envelope({"mood": "calm", "bogus": 1}))

    def test_wrong_enum_value(self):
        with pytest.raises(ValidationError):
            parse_event("mood", envelope({"mood": "ecstatic"}))

    def test_activity_invariant_violation(self):
        # working state without working_submode violates the model_validator.
        with pytest.raises(ValidationError):
            parse_event("activity", envelope({"state": "working", "from_state": "listening"}))

    def test_activity_non_starting_requires_from_state(self):
        with pytest.raises(ValidationError):
            parse_event("activity", envelope({"state": "listening"}))

    def test_unknown_topic_key(self):
        with pytest.raises(KeyError):
            parse_event("not_a_topic", envelope({"mood": "calm"}))

    def test_frozen_event_is_immutable(self):
        ev = parse_event("mood", envelope({"mood": "calm"}))
        with pytest.raises(ValidationError):
            ev.payload.mood = "happy"
