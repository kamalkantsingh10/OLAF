"""Recording test doubles — Story 6.3 (no hardware).

The render loop is exercised against these in tests; real hardware
adapters land in 6.4/6.6/8.1. Each call is timestamped via an
injectable clock so timing ACs (anticipatory window, gesture landing)
are deterministic.
"""

from __future__ import annotations

import time
from typing import Callable

from expression_engine.adapters.base import SurfaceFrame

_NECK = ("pan", "tilt", "roll")
_EARS = ("left_pan", "left_tilt", "right_pan", "right_tilt")


class RecordingContinuousAdapter:
    """ContinuousAdapter double: records every applied target + time."""

    def __init__(
        self,
        joints: tuple[str, ...],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._joints = joints
        self._clock = clock
        self.connected = False
        self.closed = False
        # list of (t, targets, speed_pct)
        self.applied: list[tuple[float, dict[str, float], float]] = []

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.closed = True

    def apply(self, targets: dict[str, float], speed_pct: float = 0.0) -> None:
        self.applied.append((self._clock(), dict(targets), speed_pct))

    def neutral(self) -> dict[str, float]:
        return {j: 0.0 for j in self._joints}

    # convenience for assertions
    @property
    def last(self) -> dict[str, float] | None:
        return self.applied[-1][1] if self.applied else None


def neck_test_adapter(clock: Callable[[], float] = time.monotonic) -> RecordingContinuousAdapter:
    return RecordingContinuousAdapter(_NECK, clock)


def ears_test_adapter(clock: Callable[[], float] = time.monotonic) -> RecordingContinuousAdapter:
    return RecordingContinuousAdapter(_EARS, clock)


class RecordingDelegatingAdapter:
    """DelegatingAdapter double: records semantic eye calls + time."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self.connected = False
        self.closed = False
        self.expressions: list[tuple[float, str, int]] = []
        self.blinks: list[float] = []
        self.looks: list[tuple[float, int, int]] = []

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.closed = True

    def set_expression(self, canonical_name: str, intensity: int) -> None:
        self.expressions.append((self._clock(), canonical_name, intensity))

    def blink(self) -> None:
        self.blinks.append(self._clock())

    def look(self, x: int, y: int) -> None:
        self.looks.append((self._clock(), x, y))


class NullContinuousAdapter:
    """No-op ContinuousAdapter — Story 6.3 node-runtime PLACEHOLDER.

    The real neck/ears adapters land in Story 6.4. Until then the node
    starts the render loop against these so the loop/threading is
    exercised end-to-end without pretending to move hardware.
    """

    def __init__(self, joints: tuple[str, ...]) -> None:
        self._joints = joints

    def connect(self) -> None: ...

    def close(self) -> None: ...

    def apply(self, targets: dict[str, float], speed_pct: float = 0.0) -> None: ...

    def neutral(self) -> dict[str, float]:
        return {j: 0.0 for j in self._joints}


class NullDelegatingAdapter:
    """No-op DelegatingAdapter — Story 6.3 node-runtime PLACEHOLDER (6.4)."""

    def connect(self) -> None: ...

    def close(self) -> None: ...

    def set_expression(self, canonical_name: str, intensity: int) -> None: ...

    def blink(self) -> None: ...

    def look(self, x: int, y: int) -> None: ...


class RecordingSurfaceAdapter:
    """SurfaceAdapter double: records rendered frames + time."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self.connected = False
        self.closed = False
        self.frames: list[tuple[float, SurfaceFrame]] = []

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.closed = True

    def render(self, frame: SurfaceFrame) -> None:
        self.frames.append((self._clock(), frame))
