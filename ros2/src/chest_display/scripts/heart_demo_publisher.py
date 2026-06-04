#!/usr/bin/env python3
"""Story 8.4 demo — publish a sequence of activity + speech_emotion events so you
can watch the chest heart react on the panel.

Run from a ROS-sourced shell (so it joins the robot's DDS domain):
    python3 ~/olaf/ros2/src/chest_display/scripts/heart_demo_publisher.py

Publishes std_msgs/String EventEnvelope JSON on /olaf/activity + /olaf/speech_emotion
(the same topics the engine consumes). The chest app subscribes and drives the heart.
"""

import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def _env(payload: dict) -> str:
    return json.dumps(
        {
            "schema_version": 3,
            "timestamp": "2026-06-03T00:00:00+00:00",
            "source": "heart_demo",
            "correlation_id": "00000000-0000-0000-0000-000000000000",
            "payload": payload,
        }
    )


# (activity_state, speech_emotion, hold_seconds) — what the heart should do.
SEQUENCE = [
    ("listening", None, 4),      # calm ~66 bpm, calm image
    ("speaking", "excited", 6),  # fast ~115 bpm, big throb
    ("speaking", "sad", 6),      # slow ~58 bpm, BROKEN-HEART image
    ("speaking", "angry", 5),    # fast ~110 bpm, hard throb
    ("sleeping", None, 7),       # slowest ~45 bpm, SLEEPY image
    ("listening", None, 5),      # back to calm
]


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("heart_demo")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1,
    )
    act_pub = node.create_publisher(String, "/olaf/activity", qos)
    sp_pub = node.create_publisher(String, "/olaf/speech_emotion", qos)
    time.sleep(0.5)  # discovery settle

    try:
        for state, emo, hold in SEQUENCE:
            print(f"-> activity={state:10s} speech_emotion={emo}")
            act_pub.publish(String(data=_env({"state": state})))
            if emo is not None:
                sp_pub.publish(String(data=_env({"emotion": emo})))
            end = time.time() + hold
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.05)
        print("done — heart should have walked: calm -> excited -> broken/sad -> angry -> sleepy -> calm")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
