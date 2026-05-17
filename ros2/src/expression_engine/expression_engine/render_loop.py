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
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from expression_engine.adapters.base import ContinuousAdapter, DelegatingAdapter
from expression_engine.config import AnimationConfig
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
    mood_neck: dict[str, float]
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


class _Gesture:
    """A parametric, code-side joint transient (nod / shake) — AR12.

    Crisp attack to peak then settle/release; never eased. Lands ≤150ms
    (peak by ~attack_ms). Map content is irrelevant (vocalization is
    deferred — only the tag selects the trajectory).
    """

    SHAPES = {
        "nod": ("tilt", 14.0),     # head nod = neck tilt
        "shake": ("pan", 16.0),    # head shake = neck pan
    }

    def __init__(self, tag: str, start: float, attack_s: float, settle_s: float):
        self.tag = tag
        self.start = start
        self.attack_s = max(1e-3, attack_s)
        self.settle_s = max(1e-3, settle_s)
        self.joint, self.peak = self.SHAPES[tag]

    @property
    def total_s(self) -> float:
        return self.attack_s + self.settle_s

    def offset(self, now: float) -> dict[str, float]:
        e = now - self.start
        if e < 0 or e > self.total_s:
            return {}
        if e <= self.attack_s:
            amp = self.peak * (e / self.attack_s)
        else:
            amp = self.peak * (1.0 - (e - self.attack_s) / self.settle_s)
        # shake oscillates around 0; nod is a single dip-and-return.
        if self.tag == "shake":
            amp *= math.sin(e / self.attack_s * math.pi)
        return {self.joint: amp}

    def expired(self, now: float) -> bool:
        return (now - self.start) > self.total_s


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

        self._a = _EaseChannel()   # activity (absolute)
        self._m = _EaseChannel()   # mood bias (additive)
        self._s = _EaseChannel()   # speech overlay (additive)

        self._last_evt: dict[str, object] = {}
        self._last_eye: tuple[str, int] | None = None
        self._gestures: list[_Gesture] = []
        self._speech_smooth_s = _SPEECH_EASE_S
        self._wake_until = 0.0
        self._last_tick: float | None = None
        self.tick_count = 0

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
                entry = self._map.activity.get("working", {}).get(
                    p.working_submode, self._map.defaults
                )
            else:
                entry = self._map.resolve("activity", p.state)
            pose = entry.get("pose", {})
            act_neck = _overlay(base_neck, pose.get("neck"))
            act_ears = _overlay(base_ears, pose.get("ears"))
            if "eye" in entry:
                eye_expr = entry["eye"].get("expression", eye_expr)
                eye_int = int(entry["eye"].get("intensity", eye_int))

        mood_neck = {j: 0.0 for j in _NECK}
        mood = snap.get("mood")
        if mood is not None:
            entry = self._map.resolve("mood", mood.payload.mood)
            # Mood is a SUBTLE modifier (§5.1): lean_bias → small
            # forward neck tilt only.
            lean = entry.get("lean_bias", 0)
            if isinstance(lean, (int, float)) and not isinstance(lean, bool):
                mood_neck["tilt"] = float(lean)

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
            act_neck, act_ears, mood_neck,
            speech_neck, speech_ears, eye_expr, eye_int, speech_active,
        )

    # ── per-event side effects (fire-on-change) ────────────────────

    def _changed(self, snap: dict, topic: str) -> object | None:
        evt = snap.get(topic)
        if evt is not None and evt is not self._last_evt.get(topic):
            self._last_evt[topic] = evt
            return evt
        return None

    def _handle_events(self, snap: dict, now: float) -> None:
        # speech_emotion: anticipatory biasing + (the eye is fired by
        # the change-detect below, early — before the anchor).
        sp = self._changed(snap, "speech_emotion")
        if sp is not None:
            self._speech_smooth_s = _SPEECH_EASE_S
            fid = sp.payload.audio_frame_id
            if fid is not None and self._anchor_resolver is not None:
                try:
                    anchor = self._anchor_resolver(fid)
                except Exception:
                    anchor = None
                if anchor is not None:
                    lead = self._anim.emotion_anticipatory_ms / 1000.0
                    deadline = anchor - lead
                    # Size the ease so the pose ~arrives (≈98%) AT the
                    # deadline → it lands `lead` ms before the audio
                    # anchor (NFR2 anticipatory window).
                    self._speech_smooth_s = max(
                        1e-3,
                        (deadline - now) / _SMOOTHDAMP_SETTLE_RATIO,
                    )

        # activity: wake short-circuit (sleeping→waking, NFR1).
        act = self._changed(snap, "activity")
        if act is not None:
            p = act.payload
            if p.from_state == "sleeping" and p.state == "waking":
                self._wake_until = now + _WAKE_WINDOW_S

        # vocalization: trigger a parametric gesture (AR12).
        voc = self._changed(snap, "vocalization")
        if voc is not None and voc.payload.tag in _Gesture.SHAPES:
            self._gestures.append(
                _Gesture(
                    voc.payload.tag,
                    now,
                    self._anim.gesture_attack_ms / 1000.0,
                    self._anim.gesture_settle_ms / 1000.0,
                )
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

        self._a.step(
            {**target.activity_neck, **target.activity_ears},
            activity_tau,
            dt,
        )
        self._m.step(target.mood_neck, self._anim.mood_ease_seconds, dt)
        self._s.step(
            {**target.speech_neck, **target.speech_ears},
            self._speech_smooth_s,
            dt,
        )

        # gesture transient (crisp, additive, not eased)
        self._gestures = [g for g in self._gestures if not g.expired(now)]
        gest: dict[str, float] = {}
        for g in self._gestures:
            for j, v in g.offset(now).items():
                gest[j] = gest.get(j, 0.0) + v

        neck_out = {
            j: self._a.value.get(j, 0.0)
            + self._m.value.get(j, 0.0)
            + self._s.value.get(j, 0.0)
            + gest.get(j, 0.0)
            for j in _NECK
        }
        ears_out = {
            j: self._a.value.get(j, 0.0) + self._s.value.get(j, 0.0)
            for j in _EARS
        }

        self._neck.apply(neck_out)
        self._ears.apply(ears_out)

        # Delegating eye: fire-on-change ONLY, sent immediately (early
        # → the ESP32's own ramp lands inside the anticipatory window).
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
        while not self._stop.is_set():
            now = self._clock()
            try:
                self.tick(now)
            except Exception as exc:  # a tick must never kill the loop
                log_event(
                    logging.ERROR,
                    "render_tick_error",
                    error=type(exc).__name__,
                    detail=str(exc),
                )
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
