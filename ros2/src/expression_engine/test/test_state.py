"""Story 6.1 Task 3 — EngineState write-side handoff tests.

The render-side is later; here we only verify last-write-wins per
topic and snapshot isolation (AR3 write-side).
"""

import json

from expression_engine.schema import parse_event
from expression_engine.state import EngineState

TS = "2026-05-10T13:42:18.123456+00:00"
CID = "12345678-1234-5678-1234-567812345678"


def _mood_event(mood: str):
    raw = json.dumps(
        {
            "schema_version": 3,
            "timestamp": TS,
            "source": "voice_agent_pipeline",
            "correlation_id": CID,
            "payload": {"mood": mood},
        }
    )
    return parse_event("mood", raw)


class TestEngineState:
    def test_absent_until_first_event(self):
        st = EngineState()
        assert st.get("mood") is None
        assert st.snapshot() == {}

    def test_apply_then_get(self):
        st = EngineState()
        ev = _mood_event("happy")
        st.apply("mood", ev)
        assert st.get("mood") is ev
        assert st.get("mood").payload.mood == "happy"

    def test_last_write_wins_per_topic(self):
        st = EngineState()
        st.apply("mood", _mood_event("calm"))
        st.apply("mood", _mood_event("excited"))
        assert st.get("mood").payload.mood == "excited"

    def test_snapshot_is_isolated_copy(self):
        st = EngineState()
        st.apply("mood", _mood_event("calm"))
        snap = st.snapshot()
        st.apply("mood", _mood_event("grumpy"))
        # Old snapshot must not see the later write.
        assert snap["mood"].payload.mood == "calm"
        assert st.get("mood").payload.mood == "grumpy"
