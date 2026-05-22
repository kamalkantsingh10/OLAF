"""Story 7.3 Task 4 — activity → posture end-to-end hardware harness.

Mock companion publisher → engine (real subscribers + render loop) →
REAL neck / ears / eye adapters. Walks the avatar through every leaf
``ActivityState`` ONE AT A TIME and HOLDS each so Kamal can confirm the
authored body posture (neck + ears + eye) reads as distinct on hardware
(Story 7.3 AC#2/#7).

Unlike the vocalization harness (crisp transients on a held baseline),
activity is the absolute BASE layer — there is no speech_emotion /
vocalization overlay here, so what you see is the pure activity pose.
The lifecycle order is a natural narrative so the transitions ease
sensibly:

    starting → waking → listening → working.thinking →
    working.delegating → speaking → going_to_sleep → sleeping

After each state settles, the harness prints the AUTHORED target pose
and the loop's eased neck/ears values as programmatic evidence beside
the human observation.

RUN ON THE ROBOT ONLY (drives real servos):

    ssh olaf.local
    cd ~/olaf
    PYTHONPATH=ros2/src/expression_engine:\\
ros2/src/olaf_drivers/neck_driver:\\
ros2/src/olaf_drivers/head_ears_driver:libs \\
      ~/.local/bin/poetry run python \\
      ros2/src/expression_engine/test/e2e_activity_run.py

    # Or run a subset (leaf labels; use dot for working submodes):
    #   ... e2e_activity_run.py listening speaking working.thinking

Stepping: the walk PAUSES after each state and waits for ENTER, so you
control the pace. At the end the head is left HOLDING its final pose
(no neutral reset) — e.g. the sleep drop persists instead of popping
back up. Ctrl-C is safe (servos hold position in-limits). Authored
poses stay within `config/servo-ids.yaml` limits (the ears adapter
also clamps right_pan ≤50°).

Not a pytest test (no `test_*` prefix). AC#7 visible-distinctness is a
human observation; this harness drives the states + prints evidence.
"""

from __future__ import annotations

import sys
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mock_publisher import build_envelope  # noqa: E402

_NECK = ("pan", "tilt", "roll")
_EARS = ("left_pan", "left_tilt", "right_pan", "right_tilt")

# A baseline mood so the WS2812 strip shows its mood TINT during the lit
# states (listening/working/speaking). happy → warm. Mood no longer
# touches the body (Story 7.4 de-scope) — it only sets the LED tint, so
# the activity postures read at full depth.
_BASELINE_MOOD = "happy"

# Leaf label → activity payload. `from_state` chains the previous state
# (must be non-null for every non-`starting` event; `working_submode`
# is required iff state == "working" — schema.ActivityPayload).
_LIFECYCLE: list[tuple[str, dict]] = [
    ("starting",           {"state": "starting", "from_state": None}),
    ("waking",             {"state": "waking", "from_state": "sleeping"}),
    ("listening",          {"state": "listening", "from_state": "waking"}),
    ("working.thinking",   {"state": "working", "working_submode": "thinking", "from_state": "listening"}),
    ("working.delegating", {"state": "working", "working_submode": "delegating", "from_state": "working"}),
    ("speaking",           {"state": "speaking", "from_state": "working"}),
    ("going_to_sleep",     {"state": "going_to_sleep", "from_state": "speaking"}),
    ("sleeping",           {"state": "sleeping", "from_state": "going_to_sleep"}),
]
_LABELS = [label for label, _ in _LIFECYCLE]


def _publish_one(node, topic_key: str, payload: dict) -> None:
    """Publish ONE schema-3 envelope on `topic_key` via a transient
    publisher created inside the running node."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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


def _leaf_entry(emap, label: str) -> dict:
    if "." in label:
        head, sub = label.split(".", 1)
        return emap.activity[head][sub]
    return emap.activity[label]


def _print_pose_evidence(emap, rl, label: str) -> None:
    entry = _leaf_entry(emap, label)
    pose = entry.get("pose", {})
    neck = pose.get("neck", {})
    ears = pose.get("ears", {})
    eye = entry.get("eye", {})
    # The state portion of the label drives the head system_status
    # (wake level + WS2812 strip) via the render loop (Story 7.3).
    state = label.split(".", 1)[0]
    status = EyeAdapter.status_for_activity(state)
    lit = status in ("listening", "processing", "speaking")
    print("  authored target:")
    print("     neck  " + "  ".join(f"{j}={float(neck.get(j, 0.0)):+6.1f}" for j in _NECK))
    print("     ears  " + "  ".join(f"{j}={float(ears.get(j, 0.0)):+6.1f}" for j in _EARS))
    print(f"     eye   expression={eye.get('expression')!r} intensity={eye.get('intensity', 3)}")
    print(f"     head  system_status={status!r}  strip={'LIT' if lit else 'dark'}")
    # Eased actual (what the loop is driving the servos toward).
    av = rl._a.value
    print("  loop eased actual:")
    print("     neck  " + "  ".join(f"{j}={float(av.get(j, 0.0)):+6.1f}" for j in _NECK))
    print("     ears  " + "  ".join(f"{j}={float(av.get(j, 0.0)):+6.1f}" for j in _EARS))


def run(labels: Sequence[str], settle_s: float = 1.5) -> None:
    config = load_config(_default_config_path())
    emap = load_expression_map(_default_map_path())

    print("=== Story 7.3 activity → posture run (manual step) ===")
    print(f"  states to walk: {list(labels)}")
    print(f"  baseline mood={_BASELINE_MOOD} (LED tint) — press ENTER to advance")

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
    rl = node.render_loop

    def _spin_for(seconds: float) -> None:
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            executor.spin_once(timeout_sec=0.05)

    # label → payload lookup so a CLI subset still sends valid events.
    by_label = dict(_LIFECYCLE)

    try:
        # Baseline mood → WS2812 tint (warm) on the lit states.
        _publish_one(node, "mood", {"mood": _BASELINE_MOOD, "reason": "7.3 hw walk"})
        _spin_for(0.3)

        n = len(labels)
        for i, label in enumerate(labels):
            payload = by_label[label]
            print(f"\n[{i + 1}/{n}] {label}")
            _publish_one(node, "activity", payload)
            _spin_for(settle_s)   # deliver event + let the head ease in
            # The render loop runs on its OWN thread, so the pose keeps
            # easing + holding while we block on input() below.
            _print_pose_evidence(emap, rl, label)
            prompt = (
                "  ▶ ENTER → next state  (Ctrl-C to stop)... "
                if i < n - 1 else
                "  ✓ last state — ENTER to finish (head HOLDS this pose)... "
            )
            try:
                input(prompt)
            except EOFError:
                break

        print("\n=== done — head left in its FINAL pose (no neutral reset) ===")
    finally:
        node.render_loop.stop()
        # Deliberately NOT driving to neutral() — the head HOLDS its last
        # pose (e.g. the sleep drop persists) instead of popping back up to
        # an awake-looking posture. Servos hold position safely in-limits.
        node.close_adapters()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    raw = sys.argv[1:]
    if not raw:
        labels = list(_LABELS)
    else:
        bad = [t for t in raw if t not in _LABELS]
        if bad:
            print(
                f"unknown activity leaf(s): {bad}; valid: {_LABELS}",
                file=sys.stderr,
            )
            sys.exit(2)
        labels = raw
    run(labels)


if __name__ == "__main__":
    main()
