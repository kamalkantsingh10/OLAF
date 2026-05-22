"""Story 7.3 — activity → system_status + mood → LED overlay wiring.

The engine's missing wire: it subscribes to `activity` and `mood` but
never drove the Head ESP32's `system_status` (which controls the wake
level + WS2812 strip pattern) or the LED colour overlay. This pins:

- `EyeAdapter.status_for_activity` device-translation table (AR10-style).
- `EyeAdapter.set_system_status` / `set_led_overlay` passthrough.
- The render loop sends both fire-on-change on activity / mood change.
- Each mood carries a known `led_overlay` tint token in the map.

Mood tints (Kamal, 2026-05-22): happy/playful→warm, excited→bright,
calm/curious/thoughtful/sleepy→cool, grumpy→hot.
"""

import json
import uuid
from pathlib import Path
from typing import get_args

import pytest

from expression_engine import schema
from expression_engine.adapters._testing import (
    RecordingDelegatingAdapter,
    ears_test_adapter,
    neck_test_adapter,
)
from expression_engine.adapters.eye_adapter import EyeAdapter
from expression_engine.config import AnimationConfig
from expression_engine.map_loader import load_expression_map
from expression_engine.node import _default_map_path
from expression_engine.render_loop import RenderLoop
from expression_engine.state import EngineState

EMAP = load_expression_map(_default_map_path())
ANIM = AnimationConfig()
PACKAGED_MAP = Path(_default_map_path())
_VALID_TINTS = {"none", "warm", "cool", "hot", "bright"}
# Mirror of head_i2c_client.STATUS_MAP keys (the driver pkg is not on
# the host test PYTHONPATH — eye_adapter imports it lazily on hardware).
_VALID_STATUSES = {"idle", "woke_up", "listening", "processing", "speaking", "going_idle"}


class SimClock:
    def __init__(self, t0: float = 1000.0) -> None:
        self.t = t0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def _env(payload: dict) -> str:
    return json.dumps({
        "schema_version": 3,
        "timestamp": "2026-05-22T10:00:00.000000+00:00",
        "source": "voice_agent_pipeline",
        "correlation_id": str(uuid.uuid4()),
        "payload": payload,
    })


def _push(state: EngineState, topic: str, payload: dict) -> None:
    state.apply(topic, schema.parse_event(topic, _env(payload)))


def _loop(state, clock):
    return RenderLoop(
        state, EMAP, ANIM,
        neck=neck_test_adapter(clock),
        ears=ears_test_adapter(clock),
        eye=RecordingDelegatingAdapter(clock),
        clock=clock,
    )


# ── EyeAdapter.status_for_activity — device translation table ────────


def test_status_for_activity_explicit_mapping():
    assert EyeAdapter.status_for_activity("starting") == "idle"
    assert EyeAdapter.status_for_activity("sleeping") == "idle"
    assert EyeAdapter.status_for_activity("waking") == "woke_up"
    assert EyeAdapter.status_for_activity("listening") == "listening"
    assert EyeAdapter.status_for_activity("working") == "processing"
    assert EyeAdapter.status_for_activity("speaking") == "speaking"
    assert EyeAdapter.status_for_activity("going_to_sleep") == "going_idle"


def test_status_for_activity_unknown_is_none():
    assert EyeAdapter.status_for_activity("not_a_state") is None


def test_every_canonical_activity_state_maps():
    for state in get_args(schema.ActivityState):
        status = EyeAdapter.status_for_activity(state)
        assert status is not None, f"{state} has no system_status mapping"
        # must be a real device status name (head_i2c_client.STATUS_MAP)
        assert status in _VALID_STATUSES, f"{state}→{status} not a device status"


# ── EyeAdapter passthrough → HeadI2CClient ──────────────────────────


class _FakeHeadClient:
    def __init__(self):
        self.opened = False
        self.statuses: list[str] = []
        self.overlays: list[str] = []

    def open(self):
        self.opened = True

    def close(self):
        self.opened = False

    def set_system_status(self, status):
        self.statuses.append(status)
        return True

    def set_led_overlay(self, overlay):
        self.overlays.append(overlay)
        return True

    def set_expression(self, expr, intensity):
        return True


def test_eye_adapter_status_and_overlay_passthrough():
    client = _FakeHeadClient()
    ad = EyeAdapter(client_factory=lambda: client)
    ad.connect()  # connect() itself wakes the head ("woke_up")
    ad.set_system_status("listening")
    ad.set_led_overlay("warm")
    assert client.statuses == ["woke_up", "listening"]
    assert client.overlays == ["warm"]


def test_eye_adapter_status_overlay_noop_before_connect():
    # No client yet → calls are safe no-ops (don't raise).
    ad = EyeAdapter(client_factory=lambda: _FakeHeadClient())
    ad.set_system_status("listening")
    ad.set_led_overlay("warm")  # no exception


# ── render loop: activity → system_status (fire-on-change) ───────────


_ACTIVITY_CASES = [
    ({"state": "starting", "from_state": None}, "idle"),
    ({"state": "sleeping", "from_state": "starting"}, "idle"),
    ({"state": "waking", "from_state": "sleeping"}, "woke_up"),
    ({"state": "listening", "from_state": "waking"}, "listening"),
    ({"state": "working", "working_submode": "thinking", "from_state": "listening"}, "processing"),
    ({"state": "working", "working_submode": "delegating", "from_state": "working"}, "processing"),
    ({"state": "speaking", "from_state": "working"}, "speaking"),
    ({"state": "going_to_sleep", "from_state": "speaking"}, "going_idle"),
]


@pytest.mark.parametrize("payload,expected", _ACTIVITY_CASES)
def test_activity_change_drives_system_status(payload, expected):
    st = EngineState()
    ck = SimClock()
    lp = _loop(st, ck)
    _push(st, "activity", payload)
    lp.tick(ck())
    assert lp._eye.statuses, "no system_status sent on activity change"
    assert lp._eye.statuses[-1][1] == expected


def test_system_status_fire_on_change_skips_redundant():
    st = EngineState()
    ck = SimClock()
    lp = _loop(st, ck)
    # starting → idle (write #1)
    _push(st, "activity", {"state": "starting", "from_state": None})
    lp.tick(ck()); ck.advance(0.01)
    # sleeping → idle (SAME status → no new write)
    _push(st, "activity", {"state": "sleeping", "from_state": "starting"})
    lp.tick(ck()); ck.advance(0.01)
    assert [s for _, s in lp._eye.statuses] == ["idle"]
    # listening → listening (different → write #2)
    _push(st, "activity", {"state": "listening", "from_state": "sleeping"})
    lp.tick(ck())
    assert [s for _, s in lp._eye.statuses] == ["idle", "listening"]


def test_working_submode_change_keeps_single_processing():
    st = EngineState()
    ck = SimClock()
    lp = _loop(st, ck)
    _push(st, "activity", {"state": "working", "working_submode": "thinking", "from_state": "listening"})
    lp.tick(ck()); ck.advance(0.01)
    _push(st, "activity", {"state": "working", "working_submode": "delegating", "from_state": "working"})
    lp.tick(ck())
    # both submodes → processing; fire-on-change → only one write
    assert [s for _, s in lp._eye.statuses] == ["processing"]


# ── render loop: mood → LED overlay (fire-on-change) ─────────────────


_MOOD_CASES = [
    ("happy", "warm"), ("playful", "warm"), ("excited", "bright"),
    ("calm", "cool"), ("curious", "cool"), ("thoughtful", "cool"),
    ("sleepy", "cool"), ("grumpy", "hot"),
]


@pytest.mark.parametrize("mood,expected", _MOOD_CASES)
def test_mood_change_drives_led_overlay(mood, expected):
    st = EngineState()
    ck = SimClock()
    lp = _loop(st, ck)
    _push(st, "mood", {"mood": mood, "reason": "t"})
    lp.tick(ck())
    assert lp._eye.overlays, "no led_overlay sent on mood change"
    assert lp._eye.overlays[-1][1] == expected


def test_led_overlay_fire_on_change_skips_same_tint():
    st = EngineState()
    ck = SimClock()
    lp = _loop(st, ck)
    # happy → warm (write #1)
    _push(st, "mood", {"mood": "happy"})
    lp.tick(ck()); ck.advance(0.01)
    # playful → warm (same tint → no new write)
    _push(st, "mood", {"mood": "playful"})
    lp.tick(ck()); ck.advance(0.01)
    assert [o for _, o in lp._eye.overlays] == ["warm"]
    # grumpy → hot (different → write #2)
    _push(st, "mood", {"mood": "grumpy"})
    lp.tick(ck())
    assert [o for _, o in lp._eye.overlays] == ["warm", "hot"]


# ── map: every mood carries a known tint token ──────────────────────


def test_every_mood_has_known_led_overlay_token():
    m = load_expression_map(PACKAGED_MAP)
    for mood in get_args(schema.Mood):
        ov = m.mood[mood].get("led_overlay")
        assert ov in _VALID_TINTS, f"{mood} led_overlay {ov!r} not a valid tint"
