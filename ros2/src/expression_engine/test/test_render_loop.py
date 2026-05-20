"""Story 6.3 — animation render loop (AC #1–#4; §2/§6, AR1/3/6/7).

Deterministic: a simulated clock drives `tick()` directly — no real
threads/sleep — so easing, the anticipatory window, and gesture
landing are exact. One thread-liveness test covers the real threaded
driver (AR3) with generous tolerance.
"""

import json
import random
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


def _loop(state, clock, anchor=None, rng=None):
    return RenderLoop(
        state, EMAP, ANIM,
        neck=neck_test_adapter(clock),
        ears=ears_test_adapter(clock),
        eye=RecordingDelegatingAdapter(clock),
        clock=clock,
        audio_anchor_resolver=anchor,
        rng=rng,
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


# ── Story 7.2 — vocalization layered_action dispatch ────────────────


def _voc(state, tag):
    """Push a vocalization snapshot — `tts_supported` is a required
    schema-3 wire field; engine ignores its value (TTS is the
    companion's job), supplied only so the envelope deserializes."""
    _push(state, "vocalization", {"tag": tag, "tts_supported": False})


class TestVocalizationDispatch:
    """Story 7.2 AC#3/#4 — map-driven, randomized, layered transients."""

    @pytest.mark.parametrize(
        "tag,neck_token,ears_token",
        [
            # Review iter (2026-05-20): clears_throat dropped ears
            # (literal-still per author intent) + uses small `shake`
            # on neck instead of `dip` (minor sideways jerks).
            ("laughter",      "nod",   "flick"),
            ("sigh",          "dip",   "droop"),
            ("gasp",          "peek",  "perk_up"),
            ("clears_throat", "shake", None),
            ("nod",           "nod",   None),
            ("shake",         "shake", None),
        ],
    )
    def test_each_vocalization_fires_authored_components(
        self, tag, neck_token, ears_token,
    ):
        spec = EMAP.vocalization[tag]["layered_action"]
        assert spec["neck_gesture"]["token"] == neck_token
        if ears_token is None:
            assert "ears_gesture" not in spec
        else:
            assert spec["ears_gesture"]["token"] == ears_token

    def test_neck_and_ears_both_displaced_during_window(self):
        # laughter authors both neck + ears components → both adapters
        # must show non-zero gesture contribution during the window.
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck, rng=random.Random(42))
        _voc(st, "laughter")
        max_neck = 0.0
        max_ears = 0.0
        dt = 0.01
        for _ in range(120):  # 1.2s — covers laughter window
            lp.tick(ck())
            n = lp._neck.applied[-1][1]
            e = lp._ears.applied[-1][1]
            max_neck = max(
                max_neck,
                abs(n["tilt"]) + abs(n["pan"]) + abs(n["roll"]),
            )
            max_ears = max(
                max_ears,
                sum(abs(v) for v in e.values()),
            )
            ck.advance(dt)
        assert max_neck > 1.0, "laughter neck never moved"
        assert max_ears > 1.0, "laughter ears never moved"

    def test_releases_after_window(self):
        # gasp window ≤ ~1.5s; after 2s every joint contribution from
        # the action must be zero (action expired + popped).
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck, rng=random.Random(0))
        _voc(st, "gasp")
        _run(lp, ck, 2.0)
        assert lp._vocalizations == [], "expired action not popped"
        n = lp._neck.applied[-1][1]
        e = lp._ears.applied[-1][1]
        # No state-layer is driving any joint either (no speech /
        # mood / activity) → all joints back to defaults (~0).
        for j in ("pan", "tilt", "roll"):
            assert abs(n[j]) < 0.1, f"neck.{j} did not release: {n[j]}"
        for j in e:
            assert abs(e[j]) < 0.1, f"ears.{j} did not release: {e[j]}"

    def test_two_fires_produce_different_samples(self):
        # AC#4: never identical twice in a row. With the SAME RNG
        # advancing across two fires, the second fire must sample
        # different amp/dur than the first.
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck, rng=random.Random(123))
        _voc(st, "laughter")
        lp.tick(ck())
        action_a = lp._vocalizations[-1]
        ck.advance(3.0)              # let it expire
        lp.tick(ck())
        assert lp._vocalizations == []
        _voc(st, "laughter")
        lp.tick(ck())
        action_b = lp._vocalizations[-1]
        diffs = (
            action_a.neck_amp != action_b.neck_amp,
            action_a.neck_dur != action_b.neck_dur,
            action_a.ears_amp != action_b.ears_amp,
            action_a.window_s != action_b.window_s,
        )
        assert any(diffs), (
            "two fires sampled identical values — RNG not advancing"
        )


class TestVocalizationEye:
    """Story 7.2 AC#5 — eye override during window + fire-on-change release."""

    def test_audible_vocalization_overrides_eye(self):
        # Laughter eye = happy (from layered_action), should fire on
        # vocalization onset over whatever was previously displayed.
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck, rng=random.Random(7))
        _push(st, "speech_emotion", {
            "emotion": "neutral", "source_tag": "x", "raw_tag": "x",
            "resolved_fallback": None})
        _run(lp, ck, 0.05)            # let speech_emotion eye fire ("neutral")
        eye = lp._eye
        assert eye.expressions[-1][1] == "neutral"
        _voc(st, "laughter")
        lp.tick(ck())                 # vocalization onset → eye overrides
        assert eye.expressions[-1][1] == "happy"

    def test_eye_returns_to_composed_after_window(self):
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck, rng=random.Random(11))
        _push(st, "speech_emotion", {
            "emotion": "neutral", "source_tag": "x", "raw_tag": "x",
            "resolved_fallback": None})
        _run(lp, ck, 0.05)
        _voc(st, "gasp")
        # Run well past the gasp window (max ~1.5s) so the action
        # expires and the next tick's eye = composed (neutral).
        _run(lp, ck, 2.0)
        eye = lp._eye
        # The sequence we expect: [..., neutral, sad, neutral, ...]
        # (each `set_expression` is fire-on-change, never per-tick.
        # Review iter #2 2026-05-20: gasp eye = sad at L1 intensity 1.)
        names = [n for _, n, _ in eye.expressions]
        assert "sad" in names, "gasp eye never fired"
        # The LAST entry must be the composed (post-window) eye.
        assert names[-1] == "neutral"
        # Fire-on-change: each distinct eye fires exactly once.
        assert sum(1 for n in names if n == "sad") == 1
        assert sum(1 for n in names if n == "neutral") == 2

    def test_body_only_vocalizations_never_override_eye(self):
        # Review iter #2: clears_throat / nod / shake have NO eye
        # block — the composed speech_emotion eye must keep showing
        # through the entire gesture window (no extra fire-on-change
        # write at onset or expiry).
        for tag in ("clears_throat", "nod", "shake"):
            st = EngineState(); ck = SimClock()
            lp = _loop(st, ck, rng=random.Random(0))
            _push(st, "speech_emotion", {
                "emotion": "happy", "source_tag": "x", "raw_tag": "x",
                "resolved_fallback": None})
            _push(st, "mood", {"mood": "excited"})
            _run(lp, ck, 0.05)
            baseline_calls = len(lp._eye.expressions)
            _voc(st, tag)
            _run(lp, ck, 1.6)
            names_after = [n for _, n, _ in lp._eye.expressions[baseline_calls:]]
            assert names_after == [], (
                f"{tag}: expected no extra eye fires, got {names_after}"
            )


class TestVocalizationAdditivity:
    """Story 7.2 AC#5 — AR12: composed neck/ears with vs without
    vocalization differ ONLY during the action window."""

    def _capture(self, with_voc: bool, seed: int):
        st = EngineState()
        ck = SimClock()
        lp = _loop(st, ck, rng=random.Random(seed))
        _push(st, "speech_emotion", {
            "emotion": "happy", "source_tag": "x", "raw_tag": "x",
            "resolved_fallback": None})
        _run(lp, ck, 0.6)             # converge speech overlay
        if with_voc:
            _voc(st, "laughter")
        trace = []
        dt = 0.01
        for _ in range(220):          # 2.2s — well past laughter window
            lp.tick(ck())
            trace.append((
                ck(),
                dict(lp._neck.applied[-1][1]),
                dict(lp._ears.applied[-1][1]),
                lp._vocalizations[0].window_s if lp._vocalizations else None,
            ))
            ck.advance(dt)
        return trace

    def test_base_layer_unchanged_outside_window(self):
        seed = 99
        base = self._capture(with_voc=False, seed=seed)
        with_voc = self._capture(with_voc=True, seed=seed)
        # Window length sampled at fire time — find when the action
        # cleared (window_s value latched at index 0).
        window_s = None
        for _, _, _, w in with_voc:
            if w is not None:
                window_s = w
                break
        assert window_s is not None, "no vocalization action ever existed"
        t0 = with_voc[0][0]
        # AR12: AFTER window_s + a small tick-margin both traces must
        # match the base trace to within numerical noise.
        margin = 0.05
        diffs_after = 0
        for (_, n_b, e_b, _), (t, n_w, e_w, _) in zip(base, with_voc):
            if t - t0 < window_s + margin:
                continue
            for j in n_b:
                if abs(n_w[j] - n_b[j]) > 0.05:
                    diffs_after += 1
            for j in e_b:
                if abs(e_w[j] - e_b[j]) > 0.05:
                    diffs_after += 1
        assert diffs_after == 0, (
            f"{diffs_after} joint samples differ after vocalization window"
        )

    def test_difference_present_during_window(self):
        # Sanity: the action MUST actually move joints during the
        # window — otherwise the "matches after" check above would
        # pass trivially.
        seed = 99
        base = self._capture(with_voc=False, seed=seed)
        with_voc = self._capture(with_voc=True, seed=seed)
        max_diff = 0.0
        for (_, n_b, e_b, _), (_, n_w, e_w, _) in zip(base, with_voc):
            for j in n_b:
                max_diff = max(max_diff, abs(n_w[j] - n_b[j]))
            for j in e_b:
                max_diff = max(max_diff, abs(e_w[j] - e_b[j]))
        assert max_diff > 2.0, (
            f"vocalization barely moved any joint (max diff {max_diff:.3f})"
        )


class TestReviewIterExtensions:
    """Story 7.2 review iter (2026-05-20) — cycles kwarg + list eye."""

    def test_nod_cycles_sampled_and_passed_through(self):
        # nod authors cycles ∈ [1, 3] — seeded RNG must produce a
        # cycles value in that range, stored on the action and applied
        # via the neck_gesture fn (kwargs).
        st = EngineState(); ck = SimClock()
        lp = _loop(st, ck, rng=random.Random(5))
        _push(st, "mood", {"mood": "happy"})
        _voc(st, "nod")
        lp.tick(ck())
        action = lp._vocalizations[-1]
        assert "cycles" in action.neck_kwargs
        assert action.neck_kwargs["cycles"] in (1, 2, 3)

    def test_nod_cycles_value_affects_trajectory_frequency(self):
        # With cycles=1 vs cycles=3, the SAME u sees a different
        # oscillation frequency on the neck.tilt offset. Test the
        # underlying neck_gestures.nod fn rather than the loop (we
        # don't control which int seed produces which cycles value).
        from expression_engine.adapters import neck_gestures
        _, t1, _ = neck_gestures.GESTURES["nod"](0.5, 10.0, cycles=1)
        _, t3, _ = neck_gestures.GESTURES["nod"](0.5, 10.0, cycles=3)
        # cycles=1: sin(2π·1·0.5)=sin(π)=0; cycles=3: sin(2π·3·0.5)=
        # sin(3π)=0. Both zero AT u=0.5, but at u=0.125: c=1→sin(π/4)≈
        # 0.707 → -7.07; c=3→sin(3π/4)≈0.707 → also -7.07. Pick a u
        # that actually separates them: u=0.25 → c=1: sin(π/2)=1; c=3:
        # sin(3π/2)=-1.
        _, t1q, _ = neck_gestures.GESTURES["nod"](0.25, 10.0, cycles=1)
        _, t3q, _ = neck_gestures.GESTURES["nod"](0.25, 10.0, cycles=3)
        assert t1q == pytest.approx(-10.0, abs=1e-6)
        assert t3q == pytest.approx(+10.0, abs=1e-6)

    def test_sigh_eye_is_fixed_neutral(self):
        # Review iter #3: sigh eye = `neutral` (oval-steady) — the
        # earlier `content` was visually indistinguishable from
        # `happy` on the device (both are firmware "crescent-up"
        # shapes; only depth differs). neutral reads as a clean
        # baseline beat.
        st = EngineState(); ck = SimClock()
        lp = _loop(st, ck, rng=random.Random(2026))
        seen = set()
        for _ in range(20):
            _voc(st, "sigh")
            lp.tick(ck())
            seen.add(lp._vocalizations[-1].eye_expression)
            ck.advance(2.0)
            lp.tick(ck())
        assert seen == {"neutral"}, seen

    def test_layered_action_accepts_eye_list_when_authored(self):
        # The list-of-strings sampling path is still supported for
        # any future authoring — exercise it via a synthetic spec
        # rather than relying on a packaged map entry.
        from expression_engine.render_loop import _VocalizationAction
        spec = {
            "attack_ms": 80, "settle_ms": 400,
            "eye": {"expression": ["happy", "content"], "intensity": 3},
        }
        seen = set()
        for seed in range(20):
            a = _VocalizationAction(
                tag="synthetic", spec=spec, start=0.0,
                rng=random.Random(seed), mood_eye_expression=None,
            )
            seen.add(a.eye_expression)
        assert seen == {"happy", "content"}

    def test_clears_throat_has_no_ears_component(self):
        # Review iter: clears_throat must NOT touch the ears — the
        # action has no ears_token.
        st = EngineState(); ck = SimClock()
        lp = _loop(st, ck, rng=random.Random(0))
        _voc(st, "clears_throat")
        lp.tick(ck())
        action = lp._vocalizations[-1]
        assert action.ears_token is None
        assert action.ears_offset(ck() + 0.1) == {}


class TestDeterministicSampling:
    """Story 7.2 AC#6 — seeded RNG → exact sampled values."""

    def test_same_seed_same_samples(self):
        st1 = EngineState(); ck1 = SimClock()
        lp1 = _loop(st1, ck1, rng=random.Random(2026))
        _voc(st1, "laughter"); lp1.tick(ck1())
        a = lp1._vocalizations[-1]

        st2 = EngineState(); ck2 = SimClock()
        lp2 = _loop(st2, ck2, rng=random.Random(2026))
        _voc(st2, "laughter"); lp2.tick(ck2())
        b = lp2._vocalizations[-1]

        assert a.neck_amp == b.neck_amp
        assert a.neck_dur == b.neck_dur
        assert a.ears_amp == b.ears_amp
        assert a.window_s == b.window_s
        assert a.eye_intensity == b.eye_intensity


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
