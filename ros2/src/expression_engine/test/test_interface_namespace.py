"""Story 7.6 — namespaceable topics + contract QoS profiles.

The expression interface follows the cmd_vel pattern: topic names may
be RELATIVE so the node namespace scopes them per robot, letting two
bodies coexist on one DDS graph. The shipped absolute ``/olaf/*`` names
stay the default (backward-compatible with the current companion).
"""

import textwrap

import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from expression_engine.config import EngineConfig, load_config
from expression_engine.state import EngineState
from expression_engine.subscribers import create_subscriptions

RELATIVE_TOPICS = {
    "mood": "expression/mood",
    "activity": "expression/activity",
    "speech_emotion": "expression/speech_emotion",
    "vocalization": "expression/vocalization",
}


@pytest.fixture()
def ros():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_config_accepts_relative_topic_names(tmp_path):
    toml = tmp_path / "expression_engine.toml"
    toml.write_text(
        textwrap.dedent(
            """
            [dds]
            domain_id = 0
            [topics]
            mood = "expression/mood"
            activity = "expression/activity"
            speech_emotion = "expression/speech_emotion"
            vocalization = "expression/vocalization"
            """
        )
    )
    cfg = load_config(toml)
    assert cfg.topics == RELATIVE_TOPICS


def test_relative_names_scoped_by_node_namespace(ros):
    # The point: a relative name resolves UNDER the node namespace, so
    # /olaf_2 and /olaf_3 bodies subscribe to distinct topics.
    node = rclpy.create_node("eng", namespace="/olaf_2")
    cfg = EngineConfig(domain_id=0, topics=dict(RELATIVE_TOPICS))
    try:
        resolved = {
            s.topic_name for s in create_subscriptions(node, cfg, EngineState())
        }
        assert resolved == {
            "/olaf_2/expression/mood",
            "/olaf_2/expression/activity",
            "/olaf_2/expression/speech_emotion",
            "/olaf_2/expression/vocalization",
        }
    finally:
        node.destroy_node()


def test_default_absolute_names_ignore_namespace(ros):
    # Backward-compat: leading-slash names are absolute → NOT namespaced,
    # so the shipped /olaf/* default keeps working with today's companion.
    node = rclpy.create_node("eng", namespace="/olaf_2")
    cfg = EngineConfig(
        domain_id=0,
        topics={
            "mood": "/olaf/mood",
            "activity": "/olaf/activity",
            "speech_emotion": "/olaf/speech_emotion",
            "vocalization": "/olaf/vocalization",
        },
    )
    try:
        resolved = {
            s.topic_name for s in create_subscriptions(node, cfg, EngineState())
        }
        assert resolved == set(cfg.topics.values())
    finally:
        node.destroy_node()


def test_contract_qos_profiles(ros):
    # Pin the contract QoS (INTERFACE.md table): state topics latched
    # (TRANSIENT_LOCAL, depth 1), event topics volatile (depth 8), all
    # RELIABLE. A producer MUST match these or DDS won't connect.
    node = rclpy.create_node("eng_qos")
    cfg = EngineConfig(domain_id=0, topics=dict(RELATIVE_TOPICS))
    try:
        subs = {
            s.topic_name.rsplit("/", 1)[-1]: s
            for s in create_subscriptions(node, cfg, EngineState())
        }
        for key in ("mood", "activity"):
            q = subs[key].qos_profile
            assert q.reliability == ReliabilityPolicy.RELIABLE
            assert q.durability == DurabilityPolicy.TRANSIENT_LOCAL
            assert q.depth == 1
        for key in ("speech_emotion", "vocalization"):
            q = subs[key].qos_profile
            assert q.reliability == ReliabilityPolicy.RELIABLE
            assert q.durability == DurabilityPolicy.VOLATILE
            assert q.depth == 8
    finally:
        node.destroy_node()
