"""Story 6.5 Task — idle drift-to-sleep hardware harness.

Mock companion → engine (real subscribers + render loop) → REAL neck /
ears / eye adapters. Sets a baseline (mood + listening), then goes
QUIET so the idle decay engages and the WHOLE HEAD drifts to sleep:

    current eye → neutral → content → sleepy L1 → L2 → L3 ── then stir

It prints each idle transition (stage eye, droop, head status) so the
on-robot observation has programmatic evidence. Partway through it fires
a `speech_emotion` to show the decay RESETS immediately, then lets it
drift off again.

Uses FAST demo timings by default (decay visible in ~30s instead of the
~2min real cadence). Override on the CLI:
    ... e2e_idle_run.py --after 5 --step-min 3 --step-max 6 --seconds 90

RUN ON THE ROBOT ONLY (drives real servos):

    ssh olaf.local
    cd ~/olaf
    just exp-run test/e2e_idle_run.py
    # or with the long PYTHONPATH form — see e2e_activity_run.py header

Safety: render loop stopped + adapters closed in a `finally`; the head
is left in its last (sleepy) pose. Ctrl-C safe.
"""

from __future__ import annotations

import argparse
import dataclasses
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor

from expression_engine.adapters.ears_adapter import EarsAdapter
from expression_engine.adapters.eye_adapter import EyeAdapter
from expression_engine.adapters.neck_adapter import NeckAdapter
from expression_engine.config import IdleConfig, load_config
from expression_engine.map_loader import load_expression_map
from expression_engine.node import (
    ExpressionEngineNode,
    _default_config_path,
    _default_map_path,
)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mock_publisher import build_envelope  # noqa: E402


def _publish_one(node, topic_key: str, payload: dict) -> None:
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    topic = node.config.topics[topic_key]
    if topic_key in ("mood", "activity"):
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
    else:
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, depth=8)
    pub = node.create_publisher(String, topic, qos)
    msg = String(); msg.data = build_envelope(payload)
    pub.publish(msg)
    time.sleep(0.05)
    node.destroy_publisher(pub)


def run(after: float, step_min: float, step_max: float, seconds: float) -> None:
    config = load_config(_default_config_path())
    # Inject fast demo idle timings so the decay is watchable.
    config = dataclasses.replace(
        config,
        idle=IdleConfig(
            idle_after_seconds=after,
            step_min_seconds=step_min,
            step_max_seconds=step_max,
            drift_amplitude_deg=config.idle.drift_amplitude_deg,
            drift_period_seconds=config.idle.drift_period_seconds,
        ),
    )
    emap = load_expression_map(_default_map_path())

    print("=== Story 6.5 idle drift-to-sleep run ===")
    print(f"  idle_after={after}s  step={step_min}-{step_max}s  watch={seconds}s")
    print("  baseline: mood=happy + listening, then QUIET → decay")

    rclpy.init()
    node = ExpressionEngineNode(config, emap,
                               neck=NeckAdapter(), ears=EarsAdapter(), eye=EyeAdapter())
    node.connect_adapters()
    node.wire_subscriptions()
    node.render_loop.start()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    rl = node.render_loop

    def _spin_for(secs: float) -> None:
        end = time.monotonic() + secs
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    try:
        _publish_one(node, "activity", {"state": "waking", "from_state": "sleeping"})
        _spin_for(0.5)
        _publish_one(node, "mood", {"mood": "happy", "reason": "6.5 idle hw"})
        _publish_one(node, "activity", {"state": "listening", "from_state": "waking"})
        print("  baseline set; going quiet — watch the head drift off ↓")

        fired_reset = False
        last = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            _spin_for(0.2)
            snapshot = (rl._idle.active, rl._last_eye, rl._last_status)
            if snapshot != last:
                last = snapshot
                el = time.monotonic() - t0
                active, eye, status = snapshot
                print(f"  [{el:5.1f}s] idle={'ON ' if active else 'off'} "
                      f"eye={eye} status={status}")
            # Halfway: fire a speech_emotion to demonstrate the reset.
            if not fired_reset and time.monotonic() - t0 > seconds * 0.55:
                fired_reset = True
                print("  -- firing speech_emotion (excited) → should RESET idle --")
                _publish_one(node, "speech_emotion", {
                    "emotion": "excited", "source_tag": "demo",
                    "audio_frame_id": None, "raw_tag": "demo",
                    "resolved_fallback": None})

        print("\n=== done — observe: head drooped to sleep, stirred, reset on speech ===")
    finally:
        node.render_loop.stop()
        node.close_adapters()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--after", type=float, default=5.0, help="idle_after_seconds (demo)")
    ap.add_argument("--step-min", type=float, default=3.0)
    ap.add_argument("--step-max", type=float, default=6.0)
    ap.add_argument("--seconds", type=float, default=90.0, help="total watch time")
    a = ap.parse_args()
    run(a.after, a.step_min, a.step_max, a.seconds)


if __name__ == "__main__":
    main()
