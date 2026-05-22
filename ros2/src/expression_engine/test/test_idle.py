"""Story 6.5 — idle drift-to-sleep controller (re-scoped 2026-05-22).

Unit tests for `idle.IdleController`: the decay engages only after the
quiet threshold, walks the path (current → wind down → neutral → content
→ sleepy L1/L2/L3) at a random per-step pace, loops at sleepy L3 (never
statue-still), is suppressed while speaking/working, resets on any event,
and the micro-drift is sub-degree and mood-scaled.

Render-loop INTEGRATION (idle engages on quiet, forces LED off, resets
on speech) lives in test_render_loop.py.
"""

import math
import random

from expression_engine.config import IdleConfig
from expression_engine.idle import IdleController, _TAIL

CFG = IdleConfig(
    idle_after_seconds=30.0,
    step_min_seconds=10.0,
    step_max_seconds=30.0,
    drift_amplitude_deg=0.6,
    drift_period_seconds=4.0,
)


def _ctrl(seed: int = 0) -> IdleController:
    return IdleController(CFG, random.Random(seed))


def _walk(c, entry, *, start=0.0, n=40, step=31.0, state="listening"):
    """Drive the controller past the threshold and step it `n` times,
    advancing the clock by `step` (> step_max so each tick advances one
    stage). Returns the (expr, level, droop) sequence of returned stages."""
    c.tick(start, state, entry)            # baseline tick (sets the timer)
    seq = []
    t = start + CFG.idle_after_seconds + 1.0
    for _ in range(n):
        s = c.tick(t, state, entry)
        if s is not None:
            seq.append((s.eye_expr, s.eye_level, s.droop))
        t += step
    return seq


# ── threshold / engagement ──────────────────────────────────────────


def test_no_decay_before_threshold():
    c = _ctrl()
    assert c.tick(0.0, "listening", ("curious", 2)) is None
    assert c.tick(29.0, "listening", ("curious", 2)) is None
    assert not c.active


def test_decay_starts_after_threshold():
    c = _ctrl()
    c.tick(0.0, "listening", ("curious", 2))
    stage = c.tick(31.0, "listening", ("curious", 2))
    assert stage is not None
    assert c.active


# ── suppression while busy ──────────────────────────────────────────


def test_suppressed_in_speaking_and_working():
    for state in ("speaking", "working"):
        c = _ctrl()
        c.tick(0.0, state, ("happy", 3))
        assert c.tick(1000.0, state, ("happy", 3)) is None
        assert not c.active


# ── the decay path ──────────────────────────────────────────────────


def test_winddown_then_calm_then_sleepy_path():
    seq = _walk(_ctrl(), ("happy", 3))
    exprs = [(e, lvl) for (e, lvl, _) in seq]
    # current expr winds down first, then neutral → content → sleepy
    assert exprs[0] == ("happy", 2)
    assert exprs[1] == ("happy", 1)
    assert ("neutral", 1) in exprs
    assert ("content", 1) in exprs
    assert ("sleepy", 1) in exprs and ("sleepy", 2) in exprs and ("sleepy", 3) in exprs
    # neutral comes before content before sleepy3
    assert exprs.index(("neutral", 1)) < exprs.index(("content", 1)) < exprs.index(("sleepy", 3))


def test_droop_increases_to_full_sleep():
    seq = _walk(_ctrl(), ("happy", 3))
    droops = [d for (_, _, d) in seq]
    assert max(droops) == 1.0           # reaches the full sleep pose
    assert droops[0] < droops[-1] or 1.0 in droops
    # sleepy@3 stage carries droop 1.0
    for (e, lvl, d) in seq:
        if (e, lvl) == ("sleepy", 3):
            assert d == 1.0


def test_low_level_entry_skips_winddown():
    # excited L1 → no L3/L2 wind-down; straight into the calm→sleep tail.
    seq = _walk(_ctrl(), ("excited", 1))
    exprs = [(e, lvl) for (e, lvl, _) in seq]
    assert ("excited", 2) not in exprs and ("excited", 3) not in exprs
    assert exprs[0] == ("neutral", 1)


def test_loops_at_sleepy_l3():
    # From a neutral entry the path IS the tail; after sleepy@3 it must
    # wrap back to the tail start (neutral) — never freezes.
    seq = [(e, lvl) for (e, lvl, _) in _walk(_ctrl(), ("neutral", 1), n=20)]
    first_deep = seq.index(("sleepy", 3))
    # the next distinct stage after sleepy@3 is the tail start, neutral
    assert seq[first_deep + 1] == ("neutral", 1)


# ── reset ───────────────────────────────────────────────────────────


def test_notify_event_resets():
    c = _ctrl()
    c.tick(0.0, "listening", ("curious", 2))
    assert c.tick(31.0, "listening", ("curious", 2)) is not None
    c.notify_event(31.0)
    assert not c.active
    # within idle_after_seconds of the reset → no decay
    assert c.tick(40.0, "listening", ("curious", 2)) is None


# ── micro-drift ─────────────────────────────────────────────────────


def test_drift_is_subdegree_and_periodic():
    c = _ctrl()
    for t in (0.0, 0.37, 1.0, 2.5, 3.9, 7.1):
        pan, tilt = c.drift_offset(t, 1.0)
        assert abs(pan) <= CFG.drift_amplitude_deg + 1e-9
        assert abs(tilt) <= 0.5 * CFG.drift_amplitude_deg + 1e-9
    # periodic: one full drift_period_seconds later → same value
    p0, t0 = c.drift_offset(1.0, 1.0)
    p1, t1 = c.drift_offset(1.0 + CFG.drift_period_seconds, 1.0)
    assert math.isclose(p0, p1, abs_tol=1e-9)


def test_drift_scales_with_mood_energy():
    c = _ctrl()
    # pick a phase where sin != 0
    t = CFG.drift_period_seconds * 0.25
    p_low, _ = c.drift_offset(t, 0.3)
    p_high, _ = c.drift_offset(t, 1.5)
    assert abs(p_high) > abs(p_low)


# ── sanity on the tail constant ─────────────────────────────────────


def test_tail_is_neutral_content_sleepy_123():
    assert [(s.eye_expr, s.eye_level) for s in _TAIL] == [
        ("neutral", 1), ("content", 1), ("sleepy", 1), ("sleepy", 2), ("sleepy", 3),
    ]
    assert _TAIL[-1].droop == 1.0


def test_idle_stage_eyes_translate_via_ar10():
    # Regression (2026-05-22): the idle eye `sleepy` was NOT a key in the
    # eye_adapter AR10 table (only a device VALUE), so it fell back to
    # neutral and the eyes never went sleepy. Every idle stage eye MUST
    # be a translatable canonical.
    from expression_engine.adapters.eye_adapter import _CANONICAL_TO_ESP32
    for s in _TAIL:
        assert s.eye_expr in _CANONICAL_TO_ESP32, (
            f"idle eye {s.eye_expr!r} not in eye_adapter._CANONICAL_TO_ESP32 "
            f"— would fall back to neutral"
        )
