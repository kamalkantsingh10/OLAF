"""Engine state — Story 6.1 Task 3 (write-side only; AR3).

The full state model (idle decay, mood-base + emotion-overlay, render
consumption) lands in later Epic 6 stories. Story 6.1 only needs the
*write-side handoff point*: validated events from the rclpy executor
thread land here, last-write-wins per topic, under a mutex.

AR3: one rclpy executor thread writes; a later render thread reads.
The lock + plain-snapshot shape is deliberately render-thread-friendly
(non-blocking snapshot read) without building the render loop here.
"""

from __future__ import annotations

import threading

from expression_engine.schema import EventEnvelope


class EngineState:
    """Mutex-protected, last-write-wins snapshot of the four topics.

    Keys are canonical topic keys (``mood`` / ``activity`` /
    ``speech_emotion`` / ``vocalization``); values are the most recent
    validated :class:`EventEnvelope` for that topic, or absent until
    the first event arrives.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: dict[str, EventEnvelope] = {}

    def apply(self, topic_key: str, event: EventEnvelope) -> None:
        """Record ``event`` as the latest for ``topic_key`` (write-side)."""
        with self._lock:
            self._latest[topic_key] = event

    def get(self, topic_key: str) -> EventEnvelope | None:
        """Return the latest event for ``topic_key``, or ``None``."""
        with self._lock:
            return self._latest.get(topic_key)

    def snapshot(self) -> dict[str, EventEnvelope]:
        """Return a shallow copy of the current per-topic latest events.

        Events are frozen pydantic models, so the shallow copy is safe
        to hand to a (later) render thread without defensive deep-copy.
        """
        with self._lock:
            return dict(self._latest)
