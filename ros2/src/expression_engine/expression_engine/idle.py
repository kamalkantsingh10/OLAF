"""Idle drift-to-sleep — Story 6.5 (re-scoped 2026-05-22).

Left alone (not speaking / not working), the WHOLE HEAD winds down to
deep sleep along a fixed expression path at a random per-step pace, then
loops at sleep so it's never statue-still:

    current expr (down to L1) → neutral → content → sleepy L1 → L2 → L3
    ── then loop the tail (neutral → … → sleepy L3) forever (the "stir")

Pure + testable: the render loop injects the clock + ``random.Random``
and calls :meth:`tick` each frame. ``idle.py`` owns ONLY the decision
(which stage, how drooped, drift offset) — never hardware, never the
ease/interpolation (that stays in the render loop, AR3). Any qualifying
event (speech_emotion / activity / vocalization) calls
:meth:`notify_event` to reset back to ACTIVE.

Most tuning lives in ``[idle]`` (config.IdleConfig): the quiet delay,
the random step pace, and the micro-movement amplitude/period.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Optional

from expression_engine.config import IdleConfig


@dataclass(frozen=True)
class IdleStage:
    """One step of the decay path."""

    eye_expr: str
    eye_level: int
    droop: float   # 0 = current upright pose, 1 = full sleep pose


# Repeating tail (the "stir" loop): neutral → content → progressively
# sleepy. After sleepy L3 the controller wraps back to the tail start, so
# the head drifts up and re-settles forever — never statue-still.
_TAIL: tuple[IdleStage, ...] = (
    IdleStage("neutral", 1, 0.20),
    IdleStage("content", 1, 0.45),
    IdleStage("sleepy",  1, 0.65),
    IdleStage("sleepy",  2, 0.85),
    IdleStage("sleepy",  3, 1.00),
)

# Activity states that SUPPRESS idle entirely (the bot is busy).
_SUPPRESS = ("speaking", "working")


class IdleController:
    """Drift-to-sleep decay state machine (Story 6.5)."""

    def __init__(self, cfg: IdleConfig, rng: Optional[random.Random] = None) -> None:
        self._cfg = cfg
        self._rng = rng if rng is not None else random.Random()
        self._active = False
        self._stages: tuple[IdleStage, ...] = ()
        self._idx = 0
        self._tail_start = 0
        self._next_step_at = 0.0
        self._last_active_at: Optional[float] = None

    @property
    def active(self) -> bool:
        return self._active

    def notify_event(self, now: float) -> None:
        """A qualifying event (speech_emotion / activity / vocalization)
        resets idle → back to ACTIVE; decay restarts only after another
        ``idle_after_seconds`` of quiet."""
        self._last_active_at = now
        self._active = False
        self._stages = ()
        self._idx = 0

    def tick(
        self,
        now: float,
        activity_state: Optional[str],
        entry_eye: tuple[str, int],
    ) -> Optional[IdleStage]:
        """Return the current idle override stage, or ``None`` to render
        normally.

        Args:
            now: monotonic seconds (injected clock).
            activity_state: current ActivityState (``working``/``speaking``
                suppress decay); ``None`` before any activity.
            entry_eye: the currently-composed ``(expression, level)`` —
                the decay's starting point when it first engages.
        """
        if activity_state in _SUPPRESS:
            # Busy → no decay; keep the timer fresh so decay only restarts
            # after the bot next goes quiet.
            self._last_active_at = now
            self._active = False
            self._stages = ()
            return None
        if self._last_active_at is None:
            self._last_active_at = now
            return None
        if not self._active:
            if now - self._last_active_at < self._cfg.idle_after_seconds:
                return None
            self._begin(now, entry_eye)
        elif now >= self._next_step_at:
            self._advance(now)
        return self._stages[self._idx]

    def drift_offset(self, now: float, mood_energy: float = 1.0) -> tuple[float, float]:
        """Sub-degree breath-like neck (pan, tilt) micro-movement so the
        head is never statue-still. Amplitude scales with ``mood_energy``
        (calmer mood → gentler). Slow periods → never twitchy."""
        amp = self._cfg.drift_amplitude_deg * mood_energy
        period = self._cfg.drift_period_seconds
        pan = amp * math.sin(2.0 * math.pi * now / period)
        tilt = 0.5 * amp * math.sin(2.0 * math.pi * now / (period * 1.7))
        return pan, tilt

    # ── internals ──────────────────────────────────────────────────

    def _begin(self, now: float, entry_eye: tuple[str, int]) -> None:
        expr, level = entry_eye
        # Device eye levels are 1..3; clamp the entry so the wind-down
        # doesn't emit redundant L4/L5 steps.
        level = min(3, max(1, int(level)))
        prefix: list[IdleStage] = []
        wind = list(range(level - 1, 0, -1))   # e.g. L3 → [2, 1]
        for i, lvl in enumerate(wind):
            frac = (i + 1) / (len(wind) + 1)
            prefix.append(IdleStage(expr, lvl, 0.15 * frac))
        self._stages = tuple(prefix) + _TAIL
        self._tail_start = len(prefix)
        self._idx = 0
        self._active = True
        self._schedule(now)

    def _advance(self, now: float) -> None:
        if self._idx < len(self._stages) - 1:
            self._idx += 1
        else:
            self._idx = self._tail_start   # loop the tail (the stir)
        self._schedule(now)

    def _schedule(self, now: float) -> None:
        self._next_step_at = now + self._rng.uniform(
            self._cfg.step_min_seconds, self._cfg.step_max_seconds
        )
