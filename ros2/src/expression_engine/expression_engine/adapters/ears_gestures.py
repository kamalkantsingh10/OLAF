"""Ears-gesture token library — Story 7.1c.

Each token maps to a function ``f(u, amp_deg) → (LP, LT, RP, RT)``
where ``u`` is normalised gesture progress in ``[0, 1]`` and the
4-tuple is the per-joint offset added to the per-emotion L1 pose
(plus any L2 drift) inside the player.

Tokens are deliberately small + parametric — emotion-specific
flavour is owned by ``ears_motion.yaml`` (``l3_gesture.amp_deg``
and ``dur_s`` choose magnitude and pace).
"""

from __future__ import annotations

import math
from typing import Callable, Dict, Tuple

OffsetT = Tuple[float, float, float, float]   # (LP, LT, RP, RT)


def _pulse(u: float) -> float:
    """Single half-sine bump: 0 → 1 → 0 across u ∈ [0, 1]."""
    return math.sin(math.pi * u)


# ── Symmetric gestures ─────────────────────────────────────────

def perk_up(u: float, amp: float) -> OffsetT:
    """Both ears pulse UP (tilt up) — alert / surprise."""
    t = amp * _pulse(u)
    return (0.0, t, 0.0, t)


def droop(u: float, amp: float) -> OffsetT:
    """Both ears pulse DOWN (tilt down) — sad / sleepy."""
    t = -amp * _pulse(u)
    return (0.0, t, 0.0, t)


def flatten(u: float, amp: float) -> OffsetT:
    """Both ears pulse BACK-and-DOWN — anger / fear (pull in + drop)."""
    p = -amp * 0.5 * _pulse(u)        # pull pan slightly IN
    t = -amp * _pulse(u)              # tilt back/down
    return (p, t, p, t)


def flick(u: float, amp: float) -> OffsetT:
    """Both ears do a fast two-cycle tilt wiggle — excited / surprised."""
    osc = amp * math.sin(4.0 * math.pi * u)
    return (0.0, osc, 0.0, osc)


def twitch(u: float, amp: float) -> OffsetT:
    """Both ears do a tiny rapid wiggle on the tilt — nervous / content."""
    osc = amp * math.sin(6.0 * math.pi * u)
    return (0.0, osc, 0.0, osc)


def swivel_in(u: float, amp: float) -> OffsetT:
    """Both ears pulse INWARD on pan — listening / attentive."""
    p = -amp * _pulse(u)
    return (p, 0.0, p, 0.0)


def swivel_out(u: float, amp: float) -> OffsetT:
    """Both ears pulse OUTWARD on pan — splaying out."""
    p = amp * _pulse(u)
    return (p, 0.0, p, 0.0)


# ── Asymmetric gestures ────────────────────────────────────────

def one_cock(u: float, amp: float) -> OffsetT:
    """Cock the LEFT ear up; the RIGHT ear droops slightly — curious."""
    return (0.0, +amp * _pulse(u), 0.0, -amp * 0.4 * _pulse(u))


# ── Hare repertoire (2026-06-04) ───────────────────────────────

def flutter(u: float, amp: float) -> OffsetT:
    """Both ears rapid multi-cycle tilt flutter (rise+fall envelope) —
    glee / laughter. Faster + busier than `flick`."""
    osc = amp * math.sin(8.0 * math.pi * u) * _pulse(u)
    return (0.0, osc, 0.0, osc)


def startle_bolt(u: float, amp: float) -> OffsetT:
    """Snap UP fast (first 15%), HOLD with a fine tremble, then release —
    gasp / startle. The hare 'freeze + ears bolt up' reaction. Tremble is
    carved OUT of amp (plateau + tremble peak == amp) so it never
    overshoots the requested amplitude, and returns to 0 at both ends."""
    tr = 0.12
    peak = amp * (1.0 - tr)
    if u < 0.15:
        base = peak * (u / 0.15)
    elif u < 0.70:
        base = peak
    else:
        base = peak * (1.0 - (u - 0.70) / 0.30)
    t = base + amp * tr * math.sin(20.0 * math.pi * u)
    return (0.0, t, 0.0, t)


def drop_and_spring(u: float, amp: float) -> OffsetT:
    """Droop DOWN, then spring back up through zero and settle — a sigh
    that recovers. A decaying half-cycle: 0 at both ends, dips down, then
    overshoots up before returning to rest."""
    t = -amp * math.sin(2.0 * math.pi * u) * (1.0 - u)
    return (0.0, t, 0.0, t)


GESTURES: Dict[str, Callable[[float, float], OffsetT]] = {
    "perk_up":         perk_up,
    "droop":           droop,
    "flatten":         flatten,
    "flick":           flick,
    "twitch":          twitch,
    "swivel_in":       swivel_in,
    "swivel_out":      swivel_out,
    "one_cock":        one_cock,
    "flutter":         flutter,
    "startle_bolt":    startle_bolt,
    "drop_and_spring": drop_and_spring,
}
