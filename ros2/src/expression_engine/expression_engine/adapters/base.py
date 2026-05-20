"""Adapter Protocols — architecture §4.

🔒 FROZEN — 2026-05-17, Story 6.4, proven on real hardware @ commit
`0f43dfa` (reference `happy` rendered end-to-end on neck + ears +
eyes). These signatures are LOCKED: Epic 7 authors expression content
only and adds NO Protocol changes. Any change is a documented
architecture amendment (§4) with re-proof on hardware. Proposed in
Story 6.3; concrete adapters (neck/ears/eye) added + proven in 6.4.

The defining distinction (AR1, §2):
  - Continuous (neck, ears): the engine interpolates per tick and
    issues absolute joint angles — the adapter NEVER eases.
  - Delegating (eyes): the engine sends a semantic event on change;
    the smart peripheral owns its own 60 FPS animation.
  - Surface (LED, heart): engine-owned; rendered from a SurfaceFrame.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable


@dataclass(frozen=True)
class SurfaceFrame:
    """One render frame for an engine-owned surface (LED / heart).

    Minimal in Story 6.3 (no real surface adapter yet — 6.6/8.1).
    Frozen alongside the Protocols in Story 6.4.
    """

    led_color: str = "#404040"
    led_intensity: float = 0.2
    led_pattern: str = "solid"
    heart_bpm: int = 60
    heart_intensity: int = 2
    heart_color: str = "#802020"


@runtime_checkable
class ContinuousAdapter(Protocol):
    """Joint-space adapter the render loop ticks. Engine owns easing."""

    def connect(self) -> None: ...  # fatal on failure (NFR7)

    def close(self) -> None: ...

    def apply(self, targets: dict[str, float], speed_pct: float = 0.0) -> None:
        """Issue absolute joint angles (deg). Called every tick with
        the loop's interpolated values — the adapter does NOT ease."""

    def neutral(self) -> dict[str, float]:
        """The joint-space 'centered' pose, for idle/return-to-base."""


@runtime_checkable
class DelegatingAdapter(Protocol):
    """Semantic adapter for a smart peripheral that self-animates."""

    def connect(self) -> None: ...

    def close(self) -> None: ...

    def set_expression(self, canonical_name: str, intensity: int) -> None: ...

    def blink(self) -> None: ...

    def look(self, x: int, y: int) -> None: ...


@runtime_checkable
class SurfaceAdapter(Protocol):
    """Engine-owned surface (LED strip, heart display)."""

    def connect(self) -> None: ...

    def close(self) -> None: ...

    def render(self, frame: SurfaceFrame) -> None: ...
