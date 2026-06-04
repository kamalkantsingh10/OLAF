"""ROS link for the chest app (Story 8.4).

Subscribes to the companion's ``activity`` + ``speech_emotion`` topics — the SAME
topics the expression engine consumes — and exposes the latest values. The chest
app polls ``spin_once`` once per frame (no extra thread).

Graceful: if rclpy / ROS isn't available (e.g. run without ROS sourced), this is
a no-op stub and ``available`` is False, so the chest app still runs and the heart
stays on its default calm beat. This sets the pattern the log panels will reuse.
"""

from __future__ import annotations

import json


class EmotionLink:
    def __init__(
        self,
        activity_topic: str = "/olaf/activity",
        speech_topic: str = "/olaf/speech_emotion",
        node_name: str = "chest_display",
    ):
        self.activity_state = None
        self.working_submode = None
        self.speech_emotion = None
        self._ok = False
        self._error = None
        self._node = None
        self._rclpy = None
        try:
            import rclpy
            from std_msgs.msg import String
        except Exception as exc:  # ROS not sourced — degrade gracefully
            self._error = f"{type(exc).__name__}: {exc}"
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._rclpy = rclpy
            self._node = rclpy.create_node(node_name)
            self._node.create_subscription(String, activity_topic, self._on_activity, 10)
            self._node.create_subscription(String, speech_topic, self._on_speech, 10)
            self._ok = True
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"

    @staticmethod
    def _payload(data: str) -> dict:
        try:
            obj = json.loads(data)
            return obj.get("payload", {}) if isinstance(obj, dict) else {}
        except Exception:
            return {}

    def _on_activity(self, msg) -> None:
        p = self._payload(msg.data)
        state = p.get("state")
        if state is not None:
            self.activity_state = state
            # Emotion is per-utterance: when speech ends the heart returns to the
            # activity's calm/neutral beat, and the next emotion can ONLY come from
            # a fresh speech_emotion publish (no leftover leaking into the next turn).
            if state != "speaking":
                self.speech_emotion = None
        self.working_submode = p.get("working_submode", self.working_submode)

    def _on_speech(self, msg) -> None:
        p = self._payload(msg.data)
        self.speech_emotion = p.get("emotion", self.speech_emotion)

    def poll(self) -> None:
        if self._ok:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)

    @property
    def available(self) -> bool:
        return self._ok

    @property
    def error(self):
        return self._error

    def close(self) -> None:
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
