"""Headless tests for the lub-dub heart-beat model (no pygame needed).

Covers AC#2 (lub-dub), AC#3 (always-alive + jitter), AC#4 (wake), AC#6
(bpm/amplitude setters), and the emotion amplitude range (40%<->110% at extreme).
The actual image rendering is verified on the panel.
"""

import random

from chest_display.heart_beat import (
    cycle_duration,
    beat_envelope,
    HeartModel,
    AMP_GROW,
    AMP_SHRINK,
    WAKE_SECONDS,
)


def test_cycle_duration():
    assert cycle_duration(60) == 1.0
    assert cycle_duration(120) == 0.5


def test_envelope_is_bounded():
    for i in range(101):
        assert 0.0 <= beat_envelope(i / 100) <= 1.0


def test_envelope_rests_late_cycle():
    assert beat_envelope(0.7) == 0.0
    assert beat_envelope(0.99) == 0.0


def test_lub_is_stronger_than_dub():
    assert beat_envelope(0.06) > beat_envelope(0.30) > 0.0


def test_model_is_always_alive():
    m = HeartModel(bpm=60, rng=random.Random(1), wake=False)
    vals = []
    for _ in range(120):           # ~2 seconds at 60 fps
        m.update(1 / 60)
        vals.append(m.contraction)
    assert max(vals) > 0.3         # a beat occurred
    assert min(vals) == 0.0        # and it rests between beats


def test_scale_within_amplitude_bounds():
    amp = 0.6
    m = HeartModel(amplitude=amp, rng=random.Random(2), wake=False)
    high = 1.0 + amp * AMP_GROW
    low = 1.0 - amp * AMP_SHRINK
    for _ in range(200):
        m.update(1 / 60)
        assert low - 1e-9 <= m.scale <= high + 1e-9


def test_extreme_amplitude_is_40_to_110_percent():
    # At amplitude 1.0 the throb spans relaxed 110% <-> full-contraction 40%.
    assert abs((1.0 + 1.0 * AMP_GROW) - 1.10) < 1e-9
    assert abs((1.0 - 1.0 * AMP_SHRINK) - 0.40) < 1e-9


def test_amplitude_is_clamped():
    m = HeartModel(wake=False)
    m.set_amplitude(5.0)
    assert m.amplitude == 1.0
    m.set_amplitude(-1.0)
    assert m.amplitude == 0.0


def test_jitter_is_deterministic_with_seed():
    a = HeartModel(bpm=60, jitter=0.06, rng=random.Random(5), wake=False)
    b = HeartModel(bpm=60, jitter=0.06, rng=random.Random(5), wake=False)
    for _ in range(50):
        a.update(0.02)
        b.update(0.02)
    assert a.phase == b.phase       # same seed -> same trajectory


def test_wake_ramps_in_from_zero():
    m = HeartModel(rng=random.Random(3), wake=True)
    assert m.wake == 0.0
    m.update(WAKE_SECONDS)
    assert m.wake == 1.0


def test_setters_for_reactive_wiring():
    m = HeartModel(wake=False)
    m.set_beat_rate(120)
    m.set_amplitude(0.8)
    assert m.amplitude == 0.8
