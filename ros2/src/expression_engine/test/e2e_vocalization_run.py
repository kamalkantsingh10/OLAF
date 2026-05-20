"""Story 7.2 Task 6 — vocalization end-to-end hardware harness.

Mock companion publisher → engine (real subscribers + render loop) →
REAL neck / ears / eye adapters. Fires each of the 6 `vocalization`
tags ONE AT A TIME against a held baseline (mood + speech_emotion +
activity), waits for the layered_action window to complete, then
prints the action's sampled parameters as programmatic evidence to
accompany the human observation (Story 7.2 AC#7).

The baseline is left ACTIVE for the entire run so the AR12 invariant
(neck/ears base layer + eye fire-on-change return) is visible: the
body should return to the held speech_emotion pose after each
vocalization settles.

RUN ON THE ROBOT ONLY (drives real servos):

    ssh olaf.local
    cd ~/olaf
    PYTHONPATH=ros2/src/expression_engine:\\
ros2/src/olaf_drivers/neck_driver:\\
ros2/src/olaf_drivers/head_ears_driver:libs \\
      ~/.local/bin/poetry run python \\
      ros2/src/expression_engine/test/e2e_vocalization_run.py

    # Or run a subset:
    #   ... e2e_vocalization_run.py laughter gasp

Safety: adapters are driven to `neutral()` then `close()` in a
`finally` — Ctrl-C is safe. Stay within `config/servo-ids.yaml`
limits (the ears adapter also clamps right_pan ≤50°).

Not a pytest test (no `test_*` prefix). AC#7 visible-distinctness is a
human observation; this harness fires the tags + prints evidence.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from typing import Sequence

import rclpy
from rclpy.executors import SingleThreadedExecutor

from expression_engine.adapters.ears_adapter import EarsAdapter
from expression_engine.adapters.eye_adapter import EyeAdapter
from expression_engine.adapters.neck_adapter import NeckAdapter
from expression_engine.config import load_config
from expression_engine.map_loader import load_expression_map
from expression_engine.node import (
    ExpressionEngineNode,
    _default_config_path,
    _default_map_path,
)
from expression_engine.schema import VOCALIZATION_TAGS

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mock_publisher import build_envelope  # noqa: E402


def _publish_one(node, topic_key: str, payload: dict) -> None:
    """Publish ONE schema-3 envelope on `topic_key` via a transient
    publisher created inside the running node. Used to drive the
    baseline + each vocalization without spinning up a parallel
    publisher process.
    """
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import String

    topic = node.config.topics[topic_key]
    if topic_key in ("mood", "activity"):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
    else:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=8,
        )
    pub = node.create_publisher(String, topic, qos)
    msg = String(); msg.data = build_envelope(payload)
    pub.publish(msg)
    time.sleep(0.05)              # let DDS drain
    node.destroy_publisher(pub)


def _fire_vocalization(node, tag: str) -> None:
    _publish_one(node, "vocalization", {"tag": tag, "tts_supported": False})


def _print_action_evidence(rl, tag: str) -> None:
    if not rl._vocalizations:
        print(f"  [!] no active _VocalizationAction for {tag} — was it dispatched?")
        return
    a = rl._vocalizations[-1]
    print(f"  → action sampled  window_s={a.window_s:.3f}")
    if a.neck_token:
        print(f"     neck  token={a.neck_token:<10s} amp={a.neck_amp:+6.2f}°  dur={a.neck_dur:.3f}s")
    if a.ears_token:
        print(f"     ears  token={a.ears_token:<10s} amp={a.ears_amp:+6.2f}°  dur={a.ears_dur:.3f}s")
    print(f"     eye   expression={a.eye_expression!r:<14s} intensity={a.eye_intensity}")


def run(tags: Sequence[str]) -> None:
    config = load_config(_default_config_path())
    emap = load_expression_map(_default_map_path())

    print("=== Story 7.2 vocalization run ===")
    print(f"  baseline: activity=listening + mood=happy + speech_emotion=content")
    print(f"  tags to fire: {list(tags)}")

    rclpy.init()
    node = ExpressionEngineNode(
        config, emap,
        neck=NeckAdapter(),
        ears=EarsAdapter(),
        eye=EyeAdapter(),
    )
    node.connect_adapters()       # §9 step 5
    node.wire_subscriptions()     # §9 step 6
    node.render_loop.start()      # §9 step 7
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def _spin_for(seconds: float) -> None:
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            executor.spin_once(timeout_sec=0.05)

    try:
        # Baseline so AR12 has something to return TO after each settle.
        _publish_one(node, "activity", {"state": "waking", "from_state": "sleeping"})
        _spin_for(1.0)
        _publish_one(node, "activity", {"state": "listening", "from_state": "waking"})
        _publish_one(node, "mood", {"mood": "happy", "reason": "7.2 hw verify"})
        _publish_one(node, "speech_emotion", {
            "emotion": "content", "source_tag": "baseline",
            "audio_frame_id": None, "raw_tag": "baseline",
            "resolved_fallback": None,
        })
        _spin_for(2.0)            # let baseline converge
        print("  baseline converged; firing vocalizations ↓")

        rl = node.render_loop
        for tag in tags:
            print(f"\n[{tag}]")
            _fire_vocalization(node, tag)
            # Give the engine one tick to register + sample the action.
            _spin_for(0.05)
            _print_action_evidence(rl, tag)
            # Cover the full window + margin so we can observe RTB.
            _spin_for(2.0)

        # Fire each tag a SECOND time to demonstrate that the sampled
        # joint values differ (per-fire randomness; AC#4).
        print("\n=== second pass — same tags, different samples ===")
        for tag in tags:
            print(f"\n[{tag} #2]")
            _fire_vocalization(node, tag)
            _spin_for(0.05)
            _print_action_evidence(rl, tag)
            _spin_for(2.0)

        print("\n=== done — observe: every fire visibly differs from its pair ===")
    finally:
        node.render_loop.stop()
        try:
            node._neck.apply(node._neck.neutral())
            node._ears.apply(node._ears.neutral())
        except Exception:
            pass
        node.close_adapters()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    raw = sys.argv[1:]
    if not raw:
        tags = list(VOCALIZATION_TAGS)
    else:
        bad = [t for t in raw if t not in VOCALIZATION_TAGS]
        if bad:
            print(f"unknown vocalization tag(s): {bad}; valid: {list(VOCALIZATION_TAGS)}",
                  file=sys.stderr)
            sys.exit(2)
        tags = raw
    run(tags)


if __name__ == "__main__":
    main()
