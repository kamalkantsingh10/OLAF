"""Four topic subscriptions — Story 6.1 Task 3 (AC #1, #2, #3).

Creates exactly four ``std_msgs/String`` subscriptions on the
configured topic names, with per-topic QoS that matches the companion
publisher's contract (brief §A.1/§A.2) so DDS endpoints are compatible
and latched topics replay on late-join:

  RELIABLE on all four (NFR21).
  mood / activity        -> TRANSIENT_LOCAL (latched), depth 1
  speech_emotion / vocalization -> VOLATILE, depth 8

Each message body is the full envelope JSON; it is parsed + validated
by schema.parse_event and handed to EngineState. A wire
``schema_version != 3`` is fatal (FR4): a structured journald line is
logged and the SchemaVersionError propagates out of the executor so
node.main can ``sys.exit(1)`` (systemd restarts — symmetry with the
pipeline). Other validation failures are rejected loudly (structured
error log) and dropped — runtime graceful handling of *unknown names*
is Story 6.2 (FR13), not conflated here.

Subscribe-only is a structural invariant (FR3): this module creates
ONLY subscriptions. It must never construct a publisher.
"""

from __future__ import annotations

import logging

from pydantic import ValidationError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from expression_engine.config import EngineConfig
from expression_engine.logging_setup import log_event
from expression_engine.schema import SchemaVersionError, parse_event
from expression_engine.state import EngineState

#: Per-topic QoS, re-derived from companion brief §A.2. Latched topics
#: use TRANSIENT_LOCAL depth 1 so a re-connecting engine learns the
#: current mood/activity at connect; audio-anchored topics are VOLATILE
#: depth 8. All RELIABLE (§A.1, NFR21).
_LATCHED = ("mood", "activity")


def _qos_for(topic_key: str) -> QoSProfile:
    if topic_key in _LATCHED:
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


def _make_callback(topic_key: str, state: EngineState):
    def _on_message(msg: String) -> None:
        try:
            event = parse_event(topic_key, msg.data)
        except SchemaVersionError as exc:
            # FR4 fatal contract breach — loud, then propagate so
            # node.main exits non-zero (no coercion, no truncation).
            log_event(
                logging.CRITICAL,
                "schema_version_rejected",
                topic=topic_key,
                found=exc.found,
                supported=exc.supported,
                source=exc.source,
            )
            raise
        except (ValidationError, ValueError) as exc:
            # Malformed / short / invariant-violating event. Reject
            # loudly and drop — do NOT corrupt state, do NOT exit
            # (runtime unknown-name fallback is Story 6.2 / FR13).
            log_event(
                logging.ERROR,
                "event_rejected",
                topic=topic_key,
                error=type(exc).__name__,
                detail=str(exc),
            )
            return
        state.apply(topic_key, event)

    return _on_message


def create_subscriptions(
    node: Node, config: EngineConfig, state: EngineState
) -> list:
    """Create the four validated subscriptions; return them.

    Creates ONLY subscriptions (FR3 subscribe-only invariant). The
    returned handles are retained by the caller (the node) so they are
    not garbage-collected.
    """
    subs = []
    for topic_key, topic_name in config.topics.items():
        subs.append(
            node.create_subscription(
                String,
                topic_name,
                _make_callback(topic_key, state),
                _qos_for(topic_key),
            )
        )
        log_event(
            logging.INFO,
            "subscribed",
            topic=topic_key,
            name=topic_name,
            latched=topic_key in _LATCHED,
        )
    return subs
