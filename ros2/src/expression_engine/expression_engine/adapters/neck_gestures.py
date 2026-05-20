"""Neck-gesture token library — Story 7.1b.

Each token maps to a function ``f(u, amp_deg) → (pan_off, tilt_off,
roll_off)`` where ``u`` is normalised gesture progress in ``[0, 1]``
and ``amp_deg`` is the peak amplitude in degrees. All outputs are
ADDED to the per-emotion L1 pose (plus any L2 drift) in the player.

Tokens here are deliberately small and parametric — emotion-specific
flavour is owned by ``neck_motion.yaml`` (``l3_gesture.amp_deg`` and
``dur_s`` choose magnitude and pace).
"""

from __future__ import annotations

import math
from typing import Callable, Dict, Tuple


def _pulse(u: float) -> float:
    """Single half-sine bump: 0 → 1 → 0 across u ∈ [0, 1]."""
    return math.sin(math.pi * u)


def nod(u: float, amp: float, cycles: int = 1) -> Tuple[float, float, float]:
    """Multi-cycle tilt oscillation — head nodding.

    ``cycles`` (Story 7.2 feedback iteration): integer number of full
    sine cycles across ``u ∈ [0, 1]`` so the gesture starts AND ends
    at 0 (continuous release — ``sin(2π·N) = 0`` for integer N).
    cycles=1 (default) preserves the original 7.1b trajectory used by
    `NeckMotionPlayer`; vocalization fire sites can sample
    ``cycles ∈ [1, 3]`` for more / fewer bobs per fire.
    """
    return (0.0, -amp * math.sin(2.0 * math.pi * cycles * u), 0.0)


def dip(u: float, amp: float) -> Tuple[float, float, float]:
    """Single tilt-down-and-back (slower than nod)."""
    return (0.0, -amp * _pulse(u), 0.0)


def shake(u: float, amp: float) -> Tuple[float, float, float]:
    """Two-cycle pan oscillation — head shake (no)."""
    return (amp * math.sin(4.0 * math.pi * u), 0.0, 0.0)


def tilt_left(u: float, amp: float) -> Tuple[float, float, float]:
    """Pulse roll to the LEFT and back (negative roll)."""
    return (0.0, 0.0, -amp * _pulse(u))


def tilt_right(u: float, amp: float) -> Tuple[float, float, float]:
    """Pulse roll to the RIGHT and back (positive roll)."""
    return (0.0, 0.0, amp * _pulse(u))


def look_away(u: float, amp: float) -> Tuple[float, float, float]:
    """Pulse pan to one side and back (single dwell)."""
    return (amp * _pulse(u), 0.0, 0.0)


# `peek` is morphologically the same as `look_away` — distinct
# entry so YAML emotions can signal intent. Differentiation lives
# in the per-emotion amp_deg / dur_s.
peek = look_away


GESTURES: Dict[str, Callable[[float, float], Tuple[float, float, float]]] = {
    "nod":        nod,
    "dip":        dip,
    "shake":      shake,
    "tilt_left":  tilt_left,
    "tilt_right": tilt_right,
    "look_away":  look_away,
    "peek":       peek,
}
