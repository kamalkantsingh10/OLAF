#!/usr/bin/env python3
"""Live monitor for the 4 expression-interface topics — read-only.

Subscribes to mood / activity / speech_emotion / vocalization using the
SAME DDS domain, topic names, and per-topic QoS as the engine itself
(from `expression_engine.toml` + `subscribers._qos_for`), so it sees
EXACTLY what the body sees. Each envelope is parsed with the contract
schema (`schema.parse_event`) and pretty-printed as it arrives.

Crucially it subscribes with the contract QoS, so the latched
`mood`/`activity` topics replay their CURRENT value on connect (a plain
`ros2 topic echo` with default VOLATILE QoS would miss them).

NEVER publishes — pure diagnostic. Run on the Pi (or any host on the
same domain):  `just monitor`  (Ctrl-C to stop).
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from expression_engine.config import load_config
from expression_engine.node import _default_config_path
from expression_engine.schema import parse_event

# Reuse the engine's per-topic QoS so we match the publisher exactly
# (latched mood/activity = TRANSIENT_LOCAL; events = VOLATILE).
from expression_engine.subscribers import _qos_for

_COLOR = {
    "mood": "\033[36m",          # cyan
    "activity": "\033[33m",      # yellow
    "speech_emotion": "\033[35m",  # magenta
    "vocalization": "\033[32m",  # green
}
_RESET = "\033[0m"


def _format(topic_key: str, raw: str) -> str:
    try:
        ev = parse_event(topic_key, raw)
        ts = ev.timestamp.strftime("%H:%M:%S")
        payload = ev.payload.model_dump(exclude_none=True)
        color = _COLOR.get(topic_key, "")
        return f"{ts}  {color}{topic_key:<15}{_RESET} {payload}  (src={ev.source})"
    except Exception as exc:  # malformed / off-contract — show it raw
        return f"  !! {topic_key}: unparseable ({type(exc).__name__}: {exc})  raw={raw!r}"


def main(args=None) -> None:
    config = load_config(_default_config_path())
    rclpy.init(args=args, domain_id=config.domain_id)
    node = Node("expression_topic_monitor")

    held = []
    for key, name in config.topics.items():
        def _cb(msg: String, k: str = key) -> None:
            print(_format(k, msg.data), flush=True)

        held.append(node.create_subscription(String, name, _cb, _qos_for(key)))

    print(
        f"monitoring DDS domain {config.domain_id}: "
        f"{list(config.topics.values())}",
        file=sys.stderr,
    )
    print(
        "(latched mood/activity replay their current value on connect; "
        "Ctrl-C to stop)",
        file=sys.stderr,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C (SIGINT) or kill/systemd (SIGTERM) — clean exit
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
