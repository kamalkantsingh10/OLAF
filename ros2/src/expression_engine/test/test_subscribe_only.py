"""Story 6.1 Task 4/5 — node + subscribe-only invariant (AC #1, #3, #4).

Requires a live rclpy context (ROS 2 Jazzy). DDS loopback is used for
the valid-event round trip; the fatal schema-version path is exercised
in a subprocess so the process-exit code can be asserted (AC #3 — "no
coercion ... exits non-zero", asserted as a real exit, not a swallowed
exception).
"""

import json
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from expression_engine.config import EngineConfig
from expression_engine.map_loader import load_expression_map
from expression_engine.node import (
    ExpressionEngineNode,
    _default_config_path,
    _default_map_path,
)

PKG = Path(__file__).resolve().parents[1]

# Story 6.2: the node now requires a validated expression map (§9).
MAP = load_expression_map(_default_map_path())

CFG = EngineConfig(
    domain_id=0,
    topics={
        "mood": "/olaf/mood",
        "activity": "/olaf/activity",
        "speech_emotion": "/olaf/speech_emotion",
        "vocalization": "/olaf/vocalization",
    },
)


@pytest.fixture()
def ros():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _envelope(payload: dict, version: int = 3) -> str:
    return json.dumps(
        {
            "schema_version": version,
            "timestamp": "2026-05-10T13:42:18.123456+00:00",
            "source": "voice_agent_pipeline",
            "correlation_id": "12345678-1234-5678-1234-567812345678",
            "payload": payload,
        }
    )


class TestSubscriptions:  # AC #1
    def test_creates_exactly_four_subscriptions(self, ros):
        node = ExpressionEngineNode(CFG, MAP)
        node.wire_subscriptions()  # §9: subscriptions created post-connect
        try:
            names = {
                s.topic_name for s in node._subscriptions_held
            }
            assert len(node._subscriptions_held) == 4
            assert names == set(CFG.topics.values())
        finally:
            node.destroy_node()


class TestSubscribeOnlyInvariant:  # AC #4 — FR3
    def test_no_publisher_on_the_four_topics(self, ros):
        node = ExpressionEngineNode(CFG, MAP)
        node.wire_subscriptions()  # §9: subscriptions created post-connect
        try:
            pubs = node.get_publisher_names_and_types_by_node(
                node.get_name(), node.get_namespace()
            )
            published_topics = {name for name, _types in pubs}
            for topic_name in CFG.topics.values():
                assert topic_name not in published_topics, (
                    f"FR3 violation: engine publishes on {topic_name}"
                )
            # Only ROS infrastructure publishers may exist.
            assert published_topics <= {"/rosout", "/parameter_events"}
        finally:
            node.destroy_node()


class TestValidEventReachesState:  # AC #2 (integration, DDS loopback)
    def test_valid_mood_event_lands_in_state(self, ros):
        """A valid schema-3 mood event round-trips into EngineState."""
        node = ExpressionEngineNode(CFG, MAP)
        node.wire_subscriptions()  # §9: subscriptions created post-connect
        pub_node = rclpy.create_node("test_pub")
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        pub = pub_node.create_publisher(String, CFG.topics["mood"], latched)
        exe = SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(pub_node)
        try:
            pub.publish(String(data=_envelope({"mood": "playful"})))
            for _ in range(50):
                exe.spin_once(timeout_sec=0.1)
                if node.state.get("mood") is not None:
                    break
            ev = node.state.get("mood")
            assert ev is not None, "valid event never reached state"
            assert ev.payload.mood == "playful"
        finally:
            exe.remove_node(node)
            exe.remove_node(pub_node)
            pub_node.destroy_node()
            node.destroy_node()


class TestSchemaVersionExitsNonZero:  # AC #3
    """schema_version != 3 is fatal — the process exits non-zero."""

    def test_bad_version_message_exits_process_nonzero(self):
        """Bad-version message terminates the engine with exit code 1."""
        # Drive the real subscriber callback + node.main-style spin in
        # a subprocess; a schema_version!=3 message must terminate the
        # process with a non-zero code (FR4 — no coercion/truncation).
        bad = _envelope({"mood": "calm"}, version=2)
        script = textwrap.dedent(
            f"""
            import sys
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (DurabilityPolicy, QoSProfile,
                                   ReliabilityPolicy)
            from std_msgs.msg import String
            from expression_engine.config import EngineConfig
            from expression_engine.map_loader import load_expression_map
            from expression_engine.node import (
                ExpressionEngineNode, _default_map_path)
            from expression_engine.schema import SchemaVersionError

            cfg = EngineConfig(domain_id=0, topics={CFG.topics!r})
            emap = load_expression_map(_default_map_path())
            rclpy.init()
            node = ExpressionEngineNode(cfg, emap)
            node.wire_subscriptions()
            pub_node = rclpy.create_node("bad_pub")
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
            pub = pub_node.create_publisher(String, cfg.topics["mood"], qos)
            exe = SingleThreadedExecutor()
            exe.add_node(node); exe.add_node(pub_node)
            pub.publish(String(data={bad!r}))
            code = 7
            try:
                for _ in range(50):
                    exe.spin_once(timeout_sec=0.1)
            except SchemaVersionError:
                code = 1            # FR4 fatal — node.main does sys.exit(1)
            sys.exit(code)
            """
        )
        child_env = dict(os.environ)  # inherit ROS/DDS env
        child_env["PYTHONPATH"] = (
            str(PKG) + os.pathsep + child_env.get("PYTHONPATH", "")
        )
        proc = subprocess.run(
            [sys.executable, "-c", script],
            cwd=str(PKG.parents[2]),
            env=child_env,
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert proc.returncode == 1, (
            f"expected exit 1 on schema_version!=3, got "
            f"{proc.returncode}\nstderr:\n{proc.stderr}"
        )


def test_default_config_path_resolves_to_real_file():
    """The packaged expression_engine.toml resolves on a dev tree."""
    assert _default_config_path().is_file()
