"""Story 6.4 Task 1 — FR14 mock companion publisher (envelope builder).

The builder is pure + testable without ROS: every emitted body must
deserialize through the frozen schema-3 models (Story 6.1). The
reference sequence must include speech_emotion=happy with a valid
audio_frame_id (drives the AC#1 end-to-end run).
"""

import json

from expression_engine import schema
from mock_publisher import HAPPY_REFERENCE_SEQUENCE, build_envelope


class TestEnvelopeBuilder:
    def test_mood_envelope_parses(self):
        raw = build_envelope({"mood": "happy"})
        ev = schema.parse_event("mood", raw)
        assert ev.payload.mood == "happy"
        assert ev.schema_version == 3
        assert ev.source == "voice_agent_pipeline"

    def test_speech_emotion_with_audio_frame_id_parses(self):
        raw = build_envelope(
            {
                "emotion": "happy",
                "source_tag": "joyful",
                "raw_tag": "joyful",
                "resolved_fallback": None,
                "audio_frame_id": "frame-42",
            }
        )
        ev = schema.parse_event("speech_emotion", raw)
        assert ev.payload.emotion == "happy"
        assert ev.payload.audio_frame_id == "frame-42"

    def test_activity_envelope_parses(self):
        raw = build_envelope({"state": "listening", "from_state": "waking"})
        ev = schema.parse_event("activity", raw)
        assert ev.payload.state == "listening"

    def test_each_envelope_is_single_line_json(self):
        raw = build_envelope({"mood": "calm"})
        assert "\n" not in raw
        json.loads(raw)  # valid JSON


class TestHappyReferenceSequence:
    def test_sequence_topics_are_canonical(self):
        topics = {topic for topic, _ in HAPPY_REFERENCE_SEQUENCE}
        assert topics <= {"mood", "activity", "speech_emotion", "vocalization"}

    def test_sequence_emits_happy_speech_emotion_with_anchor(self):
        speech = [
            p for t, p in HAPPY_REFERENCE_SEQUENCE if t == "speech_emotion"
        ]
        assert speech, "reference sequence must emit speech_emotion"
        happy = [p for p in speech if p["emotion"] == "happy"]
        assert happy, "reference sequence must emit speech_emotion=happy"
        assert all(p.get("audio_frame_id") for p in happy), (
            "happy must carry an audio_frame_id (AC#1 anticipatory)"
        )

    def test_every_sequence_payload_validates(self):
        for topic, payload in HAPPY_REFERENCE_SEQUENCE:
            ev = schema.parse_event(topic, build_envelope(payload))
            assert ev.schema_version == 3
