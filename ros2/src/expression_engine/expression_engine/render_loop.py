"""Animation render loop — Story 6.3 (AC #1–#4; §2, §3, §6, AR1/3/6/7).

A fixed-tick loop on its OWN thread (never the rclpy executor — AR3).
Each tick: copy the mutex-protected state snapshot, `compose()` a
layered target, ease current→target per layer, drive the continuous
adapters (neck/ears) with absolute angles, and push the delegating eye
adapter only on change (fire-on-change, sent early for the
anticipatory window). Gestures are parametric, code-side, superimposed
on the composed base then released. Idle/ambient is Story 6.5 — a
clean seam is left (`ambient_target`) but not implemented here.

Layering (§5.1): activity_base (absolute) → mood_bias (subtle
additive) → speech_overlay (additive, decays to base) →
active_vocalization gesture (crisp transient). Each layer eases with
its OWN time-constant (mood long, activity medium, speech short) so
mood never snaps (NFR3).
"""

from __future__ import annotations

import logging
import os
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from expression_engine.adapters import ears_gestures, neck_gestures
from expression_engine.adapters.base import ContinuousAdapter, DelegatingAdapter
from expression_engine.adapters.eye_adapter import EyeAdapter
from expression_engine.config import AnimationConfig, IdleConfig
from expression_engine.idle import IdleController
from expression_engine.logging_setup import log_event
from expression_engine.map_loader import ExpressionMap
from expression_engine.state import EngineState

_NECK = ("pan", "tilt", "roll")
_EARS = ("left_pan", "left_tilt", "right_pan", "right_tilt")

# Per-layer ease time-constants (seconds). Mood comes from config
# (NFR3, 2–4s); activity is medium; speech is short (§6).
_ACTIVITY_EASE_S = 0.5
_SPEECH_EASE_S = 0.15
# Wake short-circuit: first motion must begin within 100ms (NFR1).
_WAKE_FAST_EASE_S = 0.06
_WAKE_WINDOW_S = 0.10
# Circuit breaker: a deterministic per-tick failure (e.g. a bad map
# value that slipped through) must not become an unbounded 100Hz log
# flood + zombie engine. After this many CONSECUTIVE tick failures,
# escalate to fatal (NFR7 — systemd restarts) instead of looping.
_MAX_CONSECUTIVE_TICK_FAILURES = 100
# Empirically, critically-damped smooth_damp reaches ~98% of a step in
# ≈2.9 × smooth_time. To make the pose *arrive* (~98%) at the
# anticipatory deadline we size smooth_time = (deadline-now)/this.
_SMOOTHDAMP_SETTLE_RATIO = 2.9


def smooth_damp(
    current: float,
    target: float,
    velocity: float,
    smooth_time: float,
    dt: float,
) -> tuple[float, float]:
    """Critically-damped approach (Game Programming Gems / Unity).

    No overshoot, monotonic, reaches ~target in ~``smooth_time``.
    Returns ``(new_value, new_velocity)``.
    """
    smooth_time = max(1e-4, smooth_time)
    if dt <= 0:
        return current, velocity
    omega = 2.0 / smooth_time
    x = omega * dt
    exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x)
    change = current - target
    temp = (velocity + omega * change) * dt
    new_vel = (velocity - omega * temp) * exp
    new = target + (change + temp) * exp
    # Clamp overshoot (Unity's guard).
    if (target - current > 0.0) == (new > target):
        new = target
        new_vel = (new - current) / dt
    return new, new_vel


@dataclass
class _EaseChannel:
    """Per-joint critically-damped ease state for one layer."""

    value: dict[str, float] = field(default_factory=dict)
    vel: dict[str, float] = field(default_factory=dict)

    def step(self, target: dict[str, float], smooth_time: float, dt: float) -> None:
        for joint, tgt in target.items():
            cur = self.value.get(joint, 0.0)
            v = self.vel.get(joint, 0.0)
            nv, nvel = smooth_damp(cur, tgt, v, smooth_time, dt)
            self.value[joint] = nv
            self.vel[joint] = nvel

    def snap_progress(self, target: dict[str, float], frac: float) -> None:
        """Jump-start toward target by ``frac`` (wake short-circuit)."""
        for joint, tgt in target.items():
            cur = self.value.get(joint, 0.0)
            self.value[joint] = cur + (tgt - cur) * frac


@dataclass
class _ComposedTarget:
    activity_neck: dict[str, float]
    activity_ears: dict[str, float]
    speech_neck: dict[str, float]
    speech_ears: dict[str, float]
    eye_expression: str
    eye_intensity: int
    speech_active: bool


def _overlay(base: dict[str, float], partial: Optional[dict]) -> dict[str, float]:
    """Absolute overlay: partial joints override base, rest inherit."""
    out = dict(base)
    if isinstance(partial, dict):
        for k, v in partial.items():
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                out[k] = float(v)
    return out


def _offsets(partial: Optional[dict], joints: tuple[str, ...]) -> dict[str, float]:
    """Additive offsets for the named joints (absent → 0)."""
    out = {j: 0.0 for j in joints}
    if isinstance(partial, dict):
        for k, v in partial.items():
            if k in out and isinstance(v, (int, float)) and not isinstance(v, bool):
                out[k] = float(v)
    return out


def _sample_int(value: Any, rng: random.Random) -> int:
    """Sample an int from a 2-list range OR pass through a scalar (Story 7.2)."""
    if isinstance(value, list):
        return rng.randint(int(value[0]), int(value[1]))
    return int(value)


def _sample_float(value: Any, rng: random.Random) -> float:
    """Sample a float from a 2-list range OR pass through a scalar (Story 7.2)."""
    if isinstance(value, list):
        return rng.uniform(float(value[0]), float(value[1]))
    return float(value)


def _sample_param(value: Any, rng: random.Random) -> Any:
    """Range-aware sampler for ARBITRARY gesture kwargs (Story 7.2 review iter).

    Numeric ranges with two int bounds → ``randint`` (e.g. nod ``cycles``);
    other 2-lists → ``uniform`` (float). Scalars pass through unchanged.
    """
    if isinstance(value, list) and len(value) == 2:
        lo, hi = value
        bool_safe = (not isinstance(lo, bool)) and (not isinstance(hi, bool))
        if bool_safe and isinstance(lo, int) and isinstance(hi, int):
            return rng.randint(lo, hi)
        return rng.uniform(float(lo), float(hi))
    return value


def _gesture_extra_kwargs(block: dict, rng: random.Random) -> dict:
    """Sample every key beyond ``token/amp_deg/dur_s`` so authors can pass
    arbitrary parameters (e.g. ``cycles``) into the gesture function as
    kwargs. Each value is range-or-scalar via ``_sample_param``."""
    out: dict = {}
    for k, v in block.items():
        if k in ("token", "amp_deg", "dur_s"):
            continue
        out[k] = _sample_param(v, rng)
    return out


class _VocalizationAction:
    """Map-driven, randomized layered transient — Story 7.2 / AR12.

    Replaces the Story 6.3 hard-coded ``_Gesture.SHAPES`` table. Reads
    ``expression_map.yaml`` ``vocalization.<tag>.layered_action`` and
    samples every range field via an injected ``random.Random`` at
    construction — so two fires of the same vocalization never look
    identical at the joint level.

    Composes three independent layers on the same gesture window:
      - **neck_offset(now)** — additive pan/tilt/roll using a token from
        ``adapters.neck_gestures.GESTURES``.
      - **ears_offset(now)** — additive 4-joint offset using a token from
        ``adapters.ears_gestures.GESTURES``.
      - **eye_target(now)** — (expression, intensity) the delegating eye
        should show *during* the window; nod/shake derive expression
        from the current mood (passed in at construction).

    The action's window is ``attack_ms + settle_ms`` (sampled), padded to
    cover every component's sampled ``dur_s``. ``expired(now)`` is true
    after the window — the render loop drops the action and the eye
    fire-on-change naturally re-fires the composed (non-vocalization)
    eye on the next tick.
    """

    def __init__(
        self,
        tag: str,
        spec: dict,
        start: float,
        rng: random.Random,
        mood_eye_expression: Optional[str],
    ) -> None:
        self.tag = tag
        self.start = start

        attack_s = _sample_int(spec["attack_ms"], rng) / 1000.0
        settle_s = _sample_int(spec["settle_ms"], rng) / 1000.0

        neck = spec.get("neck_gesture")
        if neck is not None:
            self.neck_token: Optional[str] = str(neck["token"])
            self.neck_amp = _sample_float(neck["amp_deg"], rng)
            self.neck_dur = max(_sample_float(neck["dur_s"], rng), 1e-3)
            self.neck_kwargs = _gesture_extra_kwargs(neck, rng)
        else:
            self.neck_token = None
            self.neck_amp = 0.0
            self.neck_dur = 0.0
            self.neck_kwargs = {}

        ears = spec.get("ears_gesture")
        if ears is not None:
            self.ears_token: Optional[str] = str(ears["token"])
            self.ears_amp = _sample_float(ears["amp_deg"], rng)
            self.ears_dur = max(_sample_float(ears["dur_s"], rng), 1e-3)
            self.ears_kwargs = _gesture_extra_kwargs(ears, rng)
        else:
            self.ears_token = None
            self.ears_amp = 0.0
            self.ears_dur = 0.0
            self.ears_kwargs = {}

        # Eye is OPTIONAL (Story 7.2 review iter #2). When omitted, the
        # vocalization is body-only — the composed eye (held
        # speech_emotion / activity) shows through unchanged. Used for
        # clears_throat / nod / shake which are physical, not emotional.
        eye = spec.get("eye")
        if eye is None:
            self.eye_expression: Optional[str] = None
            self.eye_intensity = 0
        else:
            eye_intensity = _sample_int(eye["intensity"], rng)
            eye_intensity = max(1, min(5, int(eye_intensity)))
            eye_expr_field = eye.get("expression")
            if isinstance(eye_expr_field, list):
                # Per-fire choice from a candidate list. Non-empty
                # list is validated at map-load time.
                self.eye_expression = str(rng.choice(eye_expr_field))
            elif isinstance(eye_expr_field, str):
                self.eye_expression = eye_expr_field
            else:
                # ``expression`` absent → mood-derived path (kept for
                # forward-compat; no vocalization currently uses it).
                self.eye_expression = mood_eye_expression
            self.eye_intensity = eye_intensity

        self._window_s = max(attack_s + settle_s, self.neck_dur, self.ears_dur)

    @property
    def window_s(self) -> float:
        return self._window_s

    def expired(self, now: float) -> bool:
        return (now - self.start) > self._window_s

    def neck_offset(self, now: float) -> dict[str, float]:
        if self.neck_token is None:
            return {}
        e = now - self.start
        if e < 0 or e > self.neck_dur:
            return {}
        u = e / self.neck_dur
        fn = neck_gestures.GESTURES.get(self.neck_token)
        if fn is None:
            return {}
        p_off, t_off, r_off = fn(u, self.neck_amp, **self.neck_kwargs)
        return {"pan": p_off, "tilt": t_off, "roll": r_off}

    def ears_offset(self, now: float) -> dict[str, float]:
        if self.ears_token is None:
            return {}
        e = now - self.start
        if e < 0 or e > self.ears_dur:
            return {}
        u = e / self.ears_dur
        fn = ears_gestures.GESTURES.get(self.ears_token)
        if fn is None:
            return {}
        lp, lt, rp, rt = fn(u, self.ears_amp, **self.ears_kwargs)
        return {
            "left_pan": lp, "left_tilt": lt,
            "right_pan": rp, "right_tilt": rt,
        }

    def eye_target(self, now: float) -> Optional[tuple[str, int]]:
        """``(expression, intensity)`` while the window is open; else ``None``."""
        if self.eye_expression is None:
            return None
        e = now - self.start
        if e < 0 or e > self._window_s:
            return None
        return (self.eye_expression, self.eye_intensity)


class RenderLoop:
    """Fixed-tick layered render loop (Story 6.3).

    Deterministic seam: :meth:`tick` is pure given an injected clock —
    tests drive it with a simulated clock. :meth:`start`/:meth:`stop`
    run it on a dedicated thread at ``servo_tick_hz``.
    """

    def __init__(
        self,
        state: EngineState,
        expression_map: ExpressionMap,
        animation: AnimationConfig,
        neck: ContinuousAdapter,
        ears: ContinuousAdapter,
        eye: DelegatingAdapter,
        clock: Callable[[], float] = time.monotonic,
        audio_anchor_resolver: Optional[Callable[[str], float]] = None,
        rng: Optional[random.Random] = None,
        idle: Optional[IdleConfig] = None,
    ) -> None:
        self._state = state
        self._map = expression_map
        self._anim = animation
        self._neck = neck
        self._ears = ears
        self._eye = eye
        self._clock = clock
        # Seam for the real Pipecat frame→walltime mapping (later
        # story). None → no anticipatory biasing (immediate ease).
        self._anchor_resolver = audio_anchor_resolver
        # Story 7.2: vocalization layered_action ranges are sampled
        # via THIS RNG. Tests inject a seeded Random so sampled values
        # are exact; production uses an unseeded default.
        self._rng = rng if rng is not None else random.Random()

        self._a = _EaseChannel()   # activity (absolute)
        self._s = _EaseChannel()   # speech overlay (additive)

        self._last_evt: dict[str, object] = {}
        self._last_eye: tuple[str, int] | None = None
        self._vocalizations: list[_VocalizationAction] = []
        self._speech_smooth_s = _SPEECH_EASE_S
        self._wake_until = 0.0
        self._last_tick: float | None = None
        self.tick_count = 0
        # Story 7.3 — head system_status (activity-driven) + WS2812 LED
        # overlay (mood-driven). Fire-on-change: only push to the head
        # peripheral when the mapped value actually changes.
        self._last_status: str | None = None
        self._last_overlay: str | None = None
        # Story 6.5 — idle drift-to-sleep. Its own RNG so step pacing
        # never perturbs the vocalization RNG. The `sleeping` activity
        # pose is the droop target the whole head decays toward.
        _idle_cfg = idle or IdleConfig()
        self._idle = IdleController(_idle_cfg, rng=random.Random())
        self._idle_ease = _idle_cfg.ease_seconds   # gentle dreamy idle transitions
        _spose = self._map.activity.get("sleeping", {}).get("pose", {})
        self._sleep_neck = _overlay({j: 0.0 for j in _NECK}, _spose.get("neck"))
        self._sleep_ears = _overlay({j: 0.0 for j in _EARS}, _spose.get("ears"))
        # Boot priming: the activity ease channel starts at neutral (0).
        # On the FIRST activity we initialise it AT the pose so the bot
        # appears directly in its boot pose (e.g. sleep) instead of
        # sweeping neutral→pose (Story 7.3 boot=sleep). Later transitions
        # ease normally (NFR3).
        self._activity_primed = False

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    # ── composition ────────────────────────────────────────────────

    def _compose(self, snap: dict) -> _ComposedTarget:
        d = self._map.defaults
        dpose = d.get("pose", {})
        base_neck = _offsets(dpose.get("neck"), _NECK)
        base_ears = _offsets(dpose.get("ears"), _EARS)
        eye_expr = d.get("eye", {}).get("expression", "neutral")
        eye_int = int(d.get("eye", {}).get("intensity", 3))

        act_neck, act_ears = dict(base_neck), dict(base_ears)
        act = snap.get("activity")
        if act is not None:
            p = act.payload
            if p.state == "working":
                working = self._map.activity.get("working", {})
                entry = working.get(p.working_submode)
                if entry is None:
                    # Mirror resolve()'s FR13 observability for the
                    # working-submode path (code review 2026-05-17).
                    log_event(
                        logging.WARNING,
                        "expression.unmapped_activity",
                        topic="activity",
                        name=f"working.{p.working_submode}",
                    )
                    entry = self._map.defaults
            else:
                entry = self._map.resolve("activity", p.state)
            pose = entry.get("pose", {})
            act_neck = _overlay(base_neck, pose.get("neck"))
            act_ears = _overlay(base_ears, pose.get("ears"))
            if "eye" in entry:
                eye_expr = entry["eye"].get("expression", eye_expr)
                eye_int = int(entry["eye"].get("intensity", eye_int))

        # Mood does NOT touch the body/eye (Story 7.4 de-scope, 2026-05-22):
        # the head already has activity + speech_emotion + vocalization
        # competing for it. Mood's emotional display is the HEART (Epic
        # 8.1); its only head presence is the WS2812 colour tint (sent in
        # _handle_events as led_overlay) and, later, idle-drift modulation
        # (Story 6.5). The nod/shake transient still sources its eye from
        # mood.eye via _handle_events.
        speech_neck = {j: 0.0 for j in _NECK}
        speech_ears = {j: 0.0 for j in _EARS}
        speech_active = False
        sp = snap.get("speech_emotion")
        if sp is not None:
            entry = self._map.resolve("speech_emotion", sp.payload.emotion)
            pose = entry.get("pose", {})
            speech_neck = _offsets(pose.get("neck"), _NECK)
            speech_ears = _offsets(pose.get("ears"), _EARS)
            if "eye" in entry:
                eye_expr = entry["eye"].get("expression", eye_expr)
                eye_int = int(entry["eye"].get("intensity", eye_int))
            speech_active = True

        return _ComposedTarget(
            act_neck, act_ears,
            speech_neck, speech_ears, eye_expr, eye_int, speech_active,
        )

    def _mood_energy(self, snap: dict) -> float:
        """Idle micro-drift amplitude scale from the current mood — calmer
        moods (lower `led_bias.intensity`) drift more gently. Default 1.0
        when no mood is set. (Story 6.5 — the only mood→head influence,
        per the 7.4 de-scope.)"""
        mood_evt = snap.get("mood")
        if mood_evt is None:
            return 1.0
        entry = self._map.mood.get(mood_evt.payload.mood, {})
        bias = entry.get("led_bias") if isinstance(entry, dict) else None
        inten = bias.get("intensity") if isinstance(bias, dict) else None
        if not isinstance(inten, (int, float)) or isinstance(inten, bool):
            return 1.0
        # led_bias.intensity ~0.2..0.6 → energy ~0.5..1.5 (clamped).
        return max(0.3, min(1.5, float(inten) / 0.4))

    # ── per-event side effects (fire-on-change) ────────────────────

    def _changed(self, snap: dict, topic: str) -> object | None:
        # Compare by VALUE, not identity (code review 2026-05-17):
        # a latched topic re-delivered on DDS re-match is a new
        # EventEnvelope instance with identical content — identity
        # would spuriously re-fire the eye and re-arm the wake
        # short-circuit. Frozen pydantic models support ==; equal
        # content (same correlation_id/timestamp) is NOT a change,
        # two genuinely distinct events still differ.
        evt = snap.get(topic)
        if evt is not None and evt != self._last_evt.get(topic):
            self._last_evt[topic] = evt
            return evt
        return None

    def _handle_events(self, snap: dict, now: float) -> None:
        # speech_emotion: anticipatory biasing + (the eye is fired by
        # the change-detect below, early — before the anchor).
        sp = self._changed(snap, "speech_emotion")
        if sp is not None:
            self._speech_smooth_s = _SPEECH_EASE_S
            self._idle.notify_event(now)   # speech resets idle (Story 6.5)
            fid = sp.payload.audio_frame_id
            if fid is not None and self._anchor_resolver is not None:
                try:
                    anchor = self._anchor_resolver(fid)
                except Exception:
                    anchor = None
                if anchor is not None:
                    lead = self._anim.emotion_anticipatory_ms / 1000.0
                    deadline = anchor - lead
                    sized = (deadline - now) / _SMOOTHDAMP_SETTLE_RATIO
                    if sized < _SPEECH_EASE_S:
                        # Code review 2026-05-17: a stale/late anchor
                        # (deadline already near/past) would clamp to a
                        # ~1ms smooth_time → instant neck/ears SNAP
                        # (NFR3 "never snap" violation) with no signal.
                        # Floor to the normal short speech ease and log
                        # the missed window instead of snapping.
                        log_event(
                            logging.WARNING,
                            "expression.anticipatory_window_missed",
                            audio_frame_id=fid,
                            deadline_in_s=round(deadline - now, 4),
                        )
                        self._speech_smooth_s = _SPEECH_EASE_S
                    else:
                        # Size the ease so the pose ~arrives (≈98%) AT
                        # the deadline → lands `lead` ms before the
                        # audio anchor (NFR2 anticipatory window).
                        self._speech_smooth_s = sized

        # activity: wake short-circuit (sleeping→waking, NFR1) + reset
        # idle (Story 6.5). The head system_status is driven in tick()
        # (idle-aware — idle forces the strip OFF), not here.
        act = self._changed(snap, "activity")
        if act is not None:
            p = act.payload
            if p.from_state == "sleeping" and p.state == "waking":
                self._wake_until = now + _WAKE_WINDOW_S
            self._idle.notify_event(now)

        # mood: drive the WS2812 colour wash (Story 7.3) — the current
        # mood's nearest tint bucket. Separate event from system_status;
        # only visibly tints the active (listening/working/speaking)
        # strip animation. Fire-on-change.
        mood_evt = self._changed(snap, "mood")
        if mood_evt is not None:
            entry = self._map.mood.get(mood_evt.payload.mood)
            overlay = entry.get("led_overlay") if isinstance(entry, dict) else None
            if overlay is not None and overlay != self._last_overlay:
                self._last_overlay = overlay
                set_overlay = getattr(self._eye, "set_led_overlay", None)
                if callable(set_overlay):
                    set_overlay(overlay)

        # vocalization: map-driven, randomized parametric transient
        # (Story 7.2 — replaces _Gesture.SHAPES).
        voc = self._changed(snap, "vocalization")
        if voc is not None:
            self._idle.notify_event(now)   # vocalization resets idle (Story 6.5)
            tag = voc.payload.tag
            entry = self._map.vocalization.get(tag)
            spec = entry.get("layered_action") if isinstance(entry, dict) else None
            if spec is not None:
                # nod/shake source their eye.expression from current mood.
                mood_evt = snap.get("mood")
                mood_eye_expr: Optional[str] = None
                if mood_evt is not None:
                    mood_entry = self._map.mood.get(mood_evt.payload.mood, {})
                    mood_eye = mood_entry.get("eye") if isinstance(mood_entry, dict) else None
                    if isinstance(mood_eye, dict):
                        mood_eye_expr = mood_eye.get("expression")
                self._vocalizations.append(
                    _VocalizationAction(
                        tag=tag,
                        spec=spec,
                        start=now,
                        rng=self._rng,
                        mood_eye_expression=mood_eye_expr,
                    )
                )
            else:
                # FR13 — unknown tag (or no layered_action): render
                # nothing, but SAY so (mirrors expression.unmapped_activity)
                # so a producer/map tag mismatch is visible, not silently
                # dropped — e.g. the producer sends a tag the map lacks.
                log_event(
                    logging.WARNING,
                    "expression.unmapped_vocalization",
                    tag=tag,
                    hint="not in expression_map.vocalization (or no layered_action)",
                )

    # ── the tick ───────────────────────────────────────────────────

    def tick(self, now: float) -> dict[str, float]:
        """Advance one render tick; return the final flat joint pose.

        Snapshot-per-tick (AR3): jitter-free regardless of event
        cadence — never blocks on DDS.
        """
        snap = self._state.snapshot()
        self._handle_events(snap, now)
        target = self._compose(snap)

        dt = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now

        activity_tau = _ACTIVITY_EASE_S
        if now < self._wake_until:
            activity_tau = _WAKE_FAST_EASE_S  # begin motion <100ms

        # Story 6.5 — idle drift-to-sleep. When decaying, the WHOLE HEAD
        # eases toward the `sleeping` pose by the stage's droop fraction
        # and the eye shows the stage expression; any event reset it in
        # _handle_events.
        act_evt = snap.get("activity")
        act_state = act_evt.payload.state if act_evt is not None else None
        idle_stage = self._idle.tick(
            now, act_state, (target.eye_expression, target.eye_intensity)
        )
        if idle_stage is not None:
            # Idle transitions ease GENTLY (dreamy), not at the brisk
            # active tau — a sudden wake/stir read as jarring (Kamal).
            activity_tau = self._idle_ease
            d = idle_stage.droop
            act_target = {
                **{j: target.activity_neck[j]
                   + (self._sleep_neck[j] - target.activity_neck[j]) * d
                   for j in _NECK},
                **{j: target.activity_ears[j]
                   + (self._sleep_ears[j] - target.activity_ears[j]) * d
                   for j in _EARS},
            }
        else:
            act_target = {**target.activity_neck, **target.activity_ears}

        if not self._activity_primed and snap.get("activity") is not None:
            # First activity since boot — initialise the ease channel AT
            # the pose (no sweep from the neutral zero-init) so the bot
            # boots directly into its pose, e.g. sleep (Story 7.3).
            self._a.value = dict(act_target)
            self._a.vel = {j: 0.0 for j in act_target}
            self._activity_primed = True
        self._a.step(act_target, activity_tau, dt)

        # Head system_status (Story 7.3) — idle-aware (Story 6.5): while
        # decaying we want the strip OFF but the EYES OPEN (so the decay
        # expressions neutral→content→sleepy are visible, drooping, not a
        # flat shut line). `woke_up` is the eyes-open + strip-dark status
        # (only listening/processing/speaking light the strip); `idle`
        # would also force wake_level→0 and shut the eyes. Fire-on-change;
        # nothing before the first activity.
        eff_status = "woke_up" if idle_stage is not None else (
            EyeAdapter.status_for_activity(act_state)
            if act_state is not None else None
        )
        if eff_status is not None and eff_status != self._last_status:
            self._last_status = eff_status
            set_status = getattr(self._eye, "set_system_status", None)
            if callable(set_status):
                set_status(eff_status)

        self._s.step(
            {**target.speech_neck, **target.speech_ears},
            self._speech_smooth_s,
            dt,
        )

        # vocalization transient (additive, finite, not eased) —
        # Story 7.2 / AR12. Drops expired actions; sums neck + ears
        # offsets; first active action with an eye override wins for
        # the delegating eye.
        self._vocalizations = [
            a for a in self._vocalizations if not a.expired(now)
        ]
        gest_neck: dict[str, float] = {}
        gest_ears: dict[str, float] = {}
        voc_eye: Optional[tuple[str, int]] = None
        for action in self._vocalizations:
            for j, v in action.neck_offset(now).items():
                gest_neck[j] = gest_neck.get(j, 0.0) + v
            for j, v in action.ears_offset(now).items():
                gest_ears[j] = gest_ears.get(j, 0.0) + v
            if voc_eye is None:
                voc_eye = action.eye_target(now)

        # Idle micro-drift: sub-degree breath-like neck sway so the head
        # is never statue-still while decaying (Story 6.5). Amplitude
        # scales with the current mood (calmer → gentler).
        drift = {j: 0.0 for j in _NECK}
        if idle_stage is not None:
            dp, dtl = self._idle.drift_offset(now, self._mood_energy(snap))
            drift["pan"] = dp
            drift["tilt"] = dtl
        neck_out = {
            j: self._a.value.get(j, 0.0)
            + self._s.value.get(j, 0.0)
            + gest_neck.get(j, 0.0)
            + drift[j]
            for j in _NECK
        }
        ears_out = {
            j: self._a.value.get(j, 0.0)
            + self._s.value.get(j, 0.0)
            + gest_ears.get(j, 0.0)
            for j in _EARS
        }

        self._neck.apply(neck_out)
        self._ears.apply(ears_out)

        # Delegating eye: fire-on-change ONLY, sent immediately. While
        # a vocalization is active and provides an eye_target, it
        # OVERRIDES the composed eye (Story 7.2 AC#5). When the
        # vocalization window expires `voc_eye` returns to None and the
        # next tick's composed eye differs from `_last_eye` → fire-on-
        # change re-fires the composed eye automatically.
        if voc_eye is not None:
            eye = voc_eye
        elif idle_stage is not None:
            eye = (idle_stage.eye_expr, idle_stage.eye_level)
        else:
            eye = (target.eye_expression, target.eye_intensity)
        if eye != self._last_eye:
            self._last_eye = eye
            self._eye.set_expression(eye[0], eye[1])

        self.tick_count += 1
        return {**neck_out, **ears_out}

    # ── threaded driver ────────────────────────────────────────────

    def start(self) -> None:
        """Run the loop on a dedicated thread (NOT the executor — AR3)."""
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="expression_render", daemon=True
        )
        self._thread.start()
        log_event(
            logging.INFO,
            "render_loop_started",
            servo_tick_hz=self._anim.servo_tick_hz,
        )

    def _run(self) -> None:
        period = 1.0 / self._anim.servo_tick_hz
        next_t = self._clock()
        consecutive_failures = 0
        while not self._stop.is_set():
            now = self._clock()
            try:
                self.tick(now)
                consecutive_failures = 0
            except Exception as exc:  # a tick must never kill the loop
                consecutive_failures += 1
                log_event(
                    logging.ERROR,
                    "render_tick_error",
                    error=type(exc).__name__,
                    detail=str(exc),
                    consecutive=consecutive_failures,
                )
                if consecutive_failures >= _MAX_CONSECUTIVE_TICK_FAILURES:
                    # Deterministic, unrecoverable failure — fail fast
                    # rather than zombie (NFR7). os._exit so systemd
                    # restarts the unit (same posture as node.main's
                    # fatal paths); a daemon-thread raise would just be
                    # swallowed and leave the engine alive-but-frozen.
                    log_event(
                        logging.CRITICAL,
                        "render_loop_fatal",
                        consecutive=consecutive_failures,
                        error=type(exc).__name__,
                        detail=str(exc),
                    )
                    os._exit(1)
            next_t += period
            sleep = next_t - self._clock()
            if sleep > 0:
                self._stop.wait(sleep)
            else:
                next_t = self._clock()  # fell behind — resync, no spiral

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
