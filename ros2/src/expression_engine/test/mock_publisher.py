"""FR14 mock companion publisher — Story 6.4 Task 1 (architecture §10).

The dev/CI substitute for the live `voice_agent_pipeline`. Publishes
schema-3 `std_msgs/String` envelopes on the four canonical topics with
QoS matching the engine's subscribers (Story 6.1), so the engine
cannot tell it from the real companion. Scriptable sequences drive
end-to-end runs; `HAPPY_REFERENCE_SEQUENCE` is the AC#1 reference.

The envelope builder is pure (no ROS) so it is unit-testable; the
node/CLI is the only ROS traffic in the FR9 in-process design.
"""

from __future__ import annotations

import json
import sys
import time
import uuid
from datetime import datetime, timezone

# ── pure envelope builder (testable without ROS) ────────────────────


def build_envelope(payload: dict) -> str:
    """A single-line schema-3 envelope JSON for ``payload``.

    Mirrors the companion wire shape (brief §A.3) — the engine's
    frozen `schema.py` models must accept it verbatim.
    """
    return json.dumps(
        {
            "schema_version": 3,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "source": "voice_agent_pipeline",
            "correlation_id": str(uuid.uuid4()),
            "payload": payload,
        },
        separators=(",", ":"),
    )


#: AC#1 reference: wake to a listening posture, set a happy mood base,
#: then the anchored happy speech_emotion that must visibly render on
#: neck + ears + eyes within the anticipatory window.
HAPPY_REFERENCE_SEQUENCE: list[tuple[str, dict]] = [
    ("activity", {"state": "starting", "from_state": None}),
    ("activity", {"state": "waking", "from_state": "sleeping"}),
    ("activity", {"state": "listening", "from_state": "waking"}),
    ("mood", {"mood": "happy", "reason": "reference run"}),
    (
        "speech_emotion",
        {
            "emotion": "happy",
            "source_tag": "joyful",
            "audio_frame_id": "ref-happy-0001",
            "raw_tag": "joyful",
            "resolved_fallback": None,
        },
    ),
]


# ── ROS node + CLI (the only ROS traffic; FR9) ──────────────────────


def _qos(topic_key: str):
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    if topic_key in ("mood", "activity"):
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        depth=8,
    )


def run_sequence(
    topics: dict[str, str] | None = None,
    sequence: list[tuple[str, dict]] | None = None,
    gap_s: float = 1.0,
    hold_s: float = 4.0,
) -> None:
    """Publish ``sequence`` once on the configured topics, then hold.

    Used by the AC#1 end-to-end hardware run. ``hold_s`` keeps the
    process (and its latched mood/activity) alive so the engine's
    render loop has time to ease into the target on real servos.
    """
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    topics = topics or {
        "mood": "/olaf/mood",
        "activity": "/olaf/activity",
        "speech_emotion": "/olaf/speech_emotion",
        "vocalization": "/olaf/vocalization",
    }
    sequence = sequence or HAPPY_REFERENCE_SEQUENCE

    rclpy.init()
    node = Node("mock_companion_publisher")
    pubs = {
        key: node.create_publisher(String, name, _qos(key))
        for key, name in topics.items()
    }
    try:
        for topic_key, payload in sequence:
            msg = String()
            msg.data = build_envelope(payload)
            pubs[topic_key].publish(msg)
            node.get_logger().info(
                f"published {topic_key}: {payload}"
            )
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(gap_s)
        # Keep latched topics alive while the engine eases on hardware.
        end = time.monotonic() + hold_s
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    run_sequence()


if __name__ == "__main__":
    main(sys.argv[1:])
