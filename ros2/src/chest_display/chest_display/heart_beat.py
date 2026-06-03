"""Pure (pygame-free) heart-beat model — lub-dub timing, per-beat jitter, wake
fade-in, and the derived scale envelope.

Emotion drives two things (Story 8.4 wires them from the expression map):
  * bpm        — how fast it beats
  * amplitude  — how hard it beats. At amplitude 1.0 (extreme) the scale throbs
                 between ~110% (relaxed) and ~40% (full contraction); calm
                 emotions use a small amplitude for a gentle pulse.

(The per-emotion heart *image* is swapped by the widget, not here.)
Kept import-light so it is unit-testable headlessly.
"""

from __future__ import annotations

import random

RESTING_BPM = 62.0
BPM_JITTER = 0.05          # +/-5% per-beat jitter so it is never metronomic
DEFAULT_AMPLITUDE = 0.35   # calm / neutral / content
AMP_GROW = 0.10            # at amplitude 1.0, relaxed scale = 110%
AMP_SHRINK = 0.60          # at amplitude 1.0, full-contraction scale = 40%
MAX_AMPLITUDE = 1.0
WAKE_SECONDS = 0.8         # one-shot fade-in on first show

# lub-dub bumps within a normalized cycle phase [0,1): (center, rise, fall, amp).
# rise < fall -> fast attack, slow release (reads as a pump, not a throb).
_LUB = (0.06, 0.05, 0.14, 1.00)   # systole: strong, fast
_DUB = (0.30, 0.04, 0.18, 0.55)   # second beat: smaller


def cycle_duration(bpm: float) -> float:
    """Seconds per beat for a given bpm."""
    return 60.0 / max(bpm, 1e-6)


def _ease_in(x: float) -> float:
    return x * x


def _ease_out(x: float) -> float:
    return 1.0 - (1.0 - x) * (1.0 - x)


def _bump(phase: float, center: float, rise: float, fall: float, amp: float) -> float:
    if phase < center - rise or phase > center + fall:
        return 0.0
    if phase <= center:
        return amp * _ease_in((phase - (center - rise)) / rise)
    return amp * _ease_out(1.0 - (phase - center) / fall)


def beat_envelope(phase: float) -> float:
    """Contraction in [0,1] for a normalized cycle phase: fast lub, smaller dub,
    long diastolic rest."""
    c = _bump(phase, *_LUB) + _bump(phase, *_DUB)
    return max(0.0, min(1.0, c))


class HeartModel:
    """Stateful beat clock. `update(dt)` advances it; `scale` drives the render."""

    def __init__(
        self,
        bpm: float = RESTING_BPM,
        amplitude: float = DEFAULT_AMPLITUDE,
        jitter: float = BPM_JITTER,
        rng: random.Random | None = None,
        wake: bool = True,
    ):
        self._bpm = float(bpm)
        self.amplitude = float(amplitude)
        self.jitter = jitter
        self._rng = rng or random.Random()
        self._phase = 0.0
        self._elapsed = 0.0
        self._cycle = self._next_cycle()
        self._wake_t = 0.0 if wake else WAKE_SECONDS

    def _next_cycle(self) -> float:
        base = cycle_duration(self._bpm)
        return base * (1.0 + self._rng.uniform(-self.jitter, self.jitter))

    # -- inputs (Story 8.4 drives these from the engine-composed heart: state) --
    def set_beat_rate(self, bpm: float) -> None:
        self._bpm = float(bpm)

    def set_amplitude(self, amplitude: float) -> None:
        self.amplitude = max(0.0, min(MAX_AMPLITUDE, float(amplitude)))

    # -- update --
    def update(self, dt: float) -> None:
        self._wake_t = min(WAKE_SECONDS, self._wake_t + dt)
        self._elapsed += dt
        while self._elapsed >= self._cycle:
            self._elapsed -= self._cycle
            self._cycle = self._next_cycle()   # re-jitter every beat
        self._phase = self._elapsed / self._cycle

    # -- derived (render reads these) --
    @property
    def phase(self) -> float:
        return self._phase

    @property
    def wake(self) -> float:
        """0..1 ramp over WAKE_SECONDS (the heart fades in on first show)."""
        return self._wake_t / WAKE_SECONDS

    @property
    def contraction(self) -> float:
        return beat_envelope(self._phase)

    @property
    def scale(self) -> float:
        """Render scale. Relaxed ~ (1 + amp*GROW); full contraction ~ (1 - amp*SHRINK).
        At amplitude 1.0 this is the 110% <-> 40% extreme throb."""
        high = 1.0 + self.amplitude * AMP_GROW
        low = 1.0 - self.amplitude * AMP_SHRINK
        return high - self.contraction * (high - low)
