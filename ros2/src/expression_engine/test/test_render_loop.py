"""Story 6.3 — animation render loop (AC #1–#4; §2/§6, AR1/3/6/7).

Deterministic: a simulated clock drives `tick()` directly — no real
threads/sleep — so easing, the anticipatory window, and gesture
landing are exact. One thread-liveness test covers the real threaded
driver (AR3) with generous tolerance.
"""

import json
import threading
import time
import uuid
from pathlib import Path

import pytest

from expression_engine import schema
from expression_engine.adapters.base import (
    ContinuousAdapter,
    DelegatingAdapter,
)
from expression_engine.adapters._testing import (
    RecordingDelegatingAdapter,
    neck_test_adapter,
    ears_test_adapter,
)
from expression_engine.config import AnimationConfig
from expression_engine.map_loader import load_expression_map
from expression_engine.node import _default_map_path
from expression_engine.render_loop import RenderLoop, smooth_damp
from expression_engine.state import EngineState

EMAP = load_expression_map(_default_map_path())
ANIM = AnimationConfig()


class SimClock:
    def __init__(self, t0: float = 1000.0) -> None:
        self.t = t0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def _env(payload: dict, fid: str | None = None) -> str:
    return json.dumps(
        {
            "schema_version": 3,
            "timestamp": "2026-05-10T13:42:18.123456+00:00",
            "source": "voice_agent_pipeline",
            "correlation_id": str(uuid.uuid4()),
            "payload": payload,
        }
    )


def _push(state: EngineState, topic: str, payload: dict) -> None:
    state.apply(topic, schema.parse_event(topic, _env(payload)))


def _loop(state, clock, anchor=None):
    return RenderLoop(
        state, EMAP, ANIM,
        neck=neck_test_adapter(clock),
        ears=ears_test_adapter(clock),
        eye=RecordingDelegatingAdapter(clock),
        clock=clock,
        audio_anchor_resolver=anchor,
    )


def _run(loop, clock, seconds, hz=100.0):
    dt = 1.0 / hz
    n = int(seconds * hz)
    for _ in range(n):
        loop.tick(clock())
        clock.advance(dt)


# ── smooth_damp unit ────────────────────────────────────────────────


class TestSmoothDamp:
    def test_no_overshoot_monotonic(self):
        v, vel = 0.0, 0.0
        prev = -1.0
        for _ in range(2000):
            v, vel = smooth_damp(v, 10.0, vel, 0.5, 0.01)
            assert v <= 10.0 + 1e-6  # never overshoots
            assert v >= prev - 1e-9  # monotonic toward target
            prev = v
        assert abs(v - 10.0) < 0.05  # converges

    def test_reaches_target_within_3x_smoothtime(self):
        # Critically-damped: ~98% reached by ≈2.9× smooth_time.
        v, vel = 0.0, 0.0
        for _ in range(900):  # 9.0s at 100Hz, smooth_time=3.0 (3×)
            v, vel = smooth_damp(v, 1.0, vel, 3.0, 0.01)
        assert v > 0.95


# ── AC#1 — fixed-tick loop, snapshot per tick ───────────────────────


class TestTickLoop:
    def test_protocols_satisfied_by_test_doubles(self):
        assert isinstance(neck_test_adapter(), ContinuousAdapter)
        assert isinstance(RecordingDelegatingAdapter(), DelegatingAdapter)

    def test_applies_continuous_every_tick(self):
        st = EngineState()
        _push(st, "activity", {"state": "listening", "from_state": "waking"})
        ck = SimClock()
        lp = _loop(st, ck)
        _run(lp, ck, 0.5)
        assert lp.tick_count == 50
        assert len(lp._neck.applied) == 50  # one absolute write per tick
        assert len(lp._ears.applied) == 50

    def test_snapshot_per_tick_last_write_wins(self):
        # Many state writes between ticks → the loop sees only the
        # latest (jitter-free, AR3); result is cadence-independent.
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck)
        for m in ("calm", "happy", "playful", "excited"):
            _push(st, "mood", {"mood": m})  # 4 writes, 0 ticks
        _run(lp, ck, 12.0)  # converge
        # Mood layer settled on 'excited' (last write), not calm/happy.
        assert lp._m.value["tilt"] == pytest.approx(
            float(EMAP.mood["excited"]["lean_bias"]), abs=0.4
        )

    def test_threaded_driver_is_separate_thread(self):
        st = EngineState()
        _push(st, "activity", {"state": "listening", "from_state": "waking"})
        lp = RenderLoop(
            st, EMAP, AnimationConfig(servo_tick_hz=200.0),
            neck=neck_test_adapter(), ears=ears_test_adapter(),
            eye=RecordingDelegatingAdapter(),
        )
        lp.start()
        try:
            time.sleep(0.15)
        finally:
            lp.stop()
        assert lp.tick_count > 5  # generous lower bound
        assert lp._thread is None  # joined cleanly
        assert threading.current_thread() is threading.main_thread()


# ── AC#2 — easing: mood never snaps, continuous interpolated ────────


class TestEasing:
    def test_mood_eases_over_seconds_never_steps(self):
        st = EngineState()
        _push(st, "mood", {"mood": "excited"})  # lean_bias 7
        ck = SimClock()
        lp = _loop(st, ck)
        target = float(EMAP.resolve("mood", "excited")["lean_bias"])
        prev = 0.0
        max_step = 0.0
        reached_t = None
        dt = 0.01
        for i in range(800):  # up to 8s
            lp.tick(ck())
            cur = lp._m.value["tilt"]
            max_step = max(max_step, abs(cur - prev))
            prev = cur
            if reached_t is None and abs(cur - target) < 0.05 * target:
                reached_t = i * dt
            ck.advance(dt)
        # never snaps: no single 100Hz tick covers a big fraction
        assert max_step < target * 0.05
        # eases over ≥2s (NFR3: mood 2–4s)
        assert reached_t is not None and reached_t >= 2.0

    def test_continuous_does_not_jump_on_first_tick(self):
        st = EngineState()
        _push(st, "speech_emotion", {
            "emotion": "surprised", "source_tag": "x",
            "raw_tag": "x", "resolved_fallback": None})
        ck = SimClock()
        lp = _loop(st, ck)
        lp.tick(ck()); ck.advance(0.01); lp.tick(ck())
        first = lp._neck.applied[0][1]
        # surprised has neck.tilt 10 — must not be reached in 1–2 ticks
        assert abs(first["tilt"]) < 2.0


# ── AC#3 — anticipatory window + early delegating eye ───────────────


class TestAnticipatory:
    def test_continuous_reaches_target_30_80ms_before_anchor(self):
        st = EngineState()
        ck = SimClock()
        anchor_t = ck() + 1.0  # audio anchor 1s out

        lp = _loop(st, ck, anchor=lambda fid: anchor_t)
        _push(st, "speech_emotion", {
            "emotion": "happy", "source_tag": "h", "raw_tag": "h",
            "resolved_fallback": None, "audio_frame_id": "frame-1"})
        tgt = float(EMAP.resolve("speech_emotion", "happy")["pose"]["neck"]["tilt"])

        reach_t = None
        dt = 0.005
        while ck() < anchor_t + 0.2:
            lp.tick(ck())
            if reach_t is None and abs(lp._s.value["tilt"] - tgt) < 0.02 * abs(tgt):
                reach_t = ck()
            ck.advance(dt)
        assert reach_t is not None, "speech layer never reached target"
        lead_ms = (anchor_t - reach_t) * 1000.0
        assert 30.0 <= lead_ms <= 80.0, f"lead {lead_ms:.1f}ms not in [30,80]"

    def test_eye_set_expression_sent_early_and_on_change_only(self):
        st = EngineState()
        ck = SimClock()
        anchor_t = ck() + 1.0
        lp = _loop(st, ck, anchor=lambda fid: anchor_t)
        _push(st, "speech_emotion", {
            "emotion": "happy", "source_tag": "h", "raw_tag": "h",
            "resolved_fallback": None, "audio_frame_id": "frame-1"})
        _run(lp, ck, 0.5)  # 50 ticks, all same emotion
        eye = lp._eye
        assert len(eye.expressions) == 1, "eye fired per tick, not on change"
        sent_t, name, _ = eye.expressions[0]
        assert name == "happy"
        assert sent_t < anchor_t - 0.080  # sent well before the window


# ── AC#4 — parametric gestures land ≤150ms ──────────────────────────


class TestGestures:
    @pytest.mark.parametrize("tag,joint", [("nod", "tilt"), ("shake", "pan")])
    def test_gesture_lands_within_150ms_then_releases(self, tag, joint):
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck)
        # tts_supported is a required schema-3 wire field; the engine
        # never produces audio (TTS is the companion's job) — the loop
        # ignores its value. Supplied only so the envelope deserializes.
        _push(st, "vocalization", {"tag": tag, "tts_supported": False})
        peak = abs(_pk(tag))
        max_amp = 0.0
        dt = 0.005
        for _ in range(int(0.15 / dt)):  # first 150ms
            lp.tick(ck())
            max_amp = max(max_amp, abs(lp._neck.applied[-1][1][joint]))
            ck.advance(dt)
        assert max_amp >= 0.5 * peak, "gesture didn't land within 150ms"
        # after attack+settle the gesture has released
        _run(lp, ck, 0.4)
        assert abs(lp._neck.applied[-1][1][joint]) < 0.05 * peak


def _pk(tag):
    from expression_engine.render_loop import _Gesture
    return _Gesture.SHAPES[tag][1]


# ── NFR1 — wake short-circuit (sleeping→waking < 100ms) ─────────────


class TestWakeShortCircuit:
    def test_wake_begins_motion_within_100ms(self):
        # Precondition: engine settled in 'sleeping' (neck tilt ≈ -25).
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck)
        _push(st, "activity", {"state": "sleeping", "from_state": "going_to_sleep"})
        _run(lp, ck, 6.0)  # converge to sleeping
        sleeping_tilt = lp._a.value["tilt"]
        waking_tilt = float(EMAP.activity["waking"]["pose"]["neck"]["tilt"])
        gap = abs(waking_tilt - sleeping_tilt)
        assert gap > 15  # sanity: there IS a real move to make

        # Transition sleeping→waking, measure motion over next 100ms.
        _push(st, "activity", {"state": "waking", "from_state": "sleeping"})
        v0 = lp._a.value["tilt"]
        for _ in range(20):  # 100ms at 200Hz steps
            lp.tick(ck())
            ck.advance(0.005)
        moved = abs(lp._a.value["tilt"] - v0)
        # NFR1: a substantial fraction of the move begins inside 100ms
        # (short-circuit). Medium-tau easing (0.5s) would barely move.
        assert moved >= 0.25 * gap, (
            f"wake not short-circuited: moved {moved:.2f} of {gap:.2f} "
            f"in 100ms"
        )


# ── compose: layering + topic namespacing + FR13 fallback ───────────


class TestCompose:
    def test_layers_compose_activity_plus_speech(self):
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck)
        _push(st, "activity", {"state": "listening", "from_state": "waking"})
        _push(st, "speech_emotion", {
            "emotion": "happy", "source_tag": "h", "raw_tag": "h",
            "resolved_fallback": None})
        _run(lp, ck, 6.0)  # converge all layers
        out = lp._neck.applied[-1][1]
        base = EMAP.activity["listening"]["pose"]["neck"]["tilt"]
        overlay = EMAP.speech_emotion["happy"]["pose"]["neck"]["tilt"]
        assert out["tilt"] == pytest.approx(base + overlay, abs=0.5)

    def test_topic_namespacing_mood_vs_speech_happy(self):
        # mood.happy → lean bias only; speech.happy → ear pose overlay.
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck)
        _push(st, "mood", {"mood": "happy"})
        _run(lp, ck, 12.0)  # mood smooth_time 3.0 → ~4× to settle
        assert lp._m.value["tilt"] == pytest.approx(
            float(EMAP.mood["happy"]["lean_bias"]), abs=0.4)
        assert lp._neck.applied[-1][1]["pan"] == pytest.approx(0.0, abs=0.3)

    def test_unknown_speech_name_falls_back_no_crash(self):
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck)
        _push(st, "speech_emotion", {
            "emotion": "no_such", "source_tag": "x", "raw_tag": "x",
            "resolved_fallback": "unknown"})
        _run(lp, ck, 0.3)  # FR13: defaults, loop stays alive
        assert lp.tick_count == 30
