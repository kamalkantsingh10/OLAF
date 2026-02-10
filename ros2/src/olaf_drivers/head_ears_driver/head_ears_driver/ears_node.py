"""ROS2 node for OLAF ear servo control.

Subscribes to:
    /olaf/ears/expression (Expression) — apply emotion preset with intensity
    /olaf/ears/pose (EarsPose) — direct angle control

Publishes:
    /olaf/ears/status (String) — periodic feedback (position, load, voltage, temp)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from olaf_interfaces.msg import Expression, EarsPose

from head_ears_driver.ears_servo_driver import EarsServoDriver
from head_ears_driver.expressions import get_preset, PRESETS


class EarsNode(Node):
    """ROS2 node controlling 4x SCS0009 ear servos."""

    def __init__(self) -> None:
        super().__init__("ears_node")

        # Declare parameters
        self.declare_parameter("config_path", "")
        config_path = self.get_parameter("config_path").get_parameter_value().string_value

        # Initialize servo driver
        try:
            if config_path:
                self._driver = EarsServoDriver(config_path)
            else:
                self._driver = EarsServoDriver()
            self.get_logger().info("Ears servo driver initialized")
        except ConnectionError as e:
            self.get_logger().error(f"Failed to initialize ears driver: {e}")
            raise

        # Subscribers
        self._emote_sub = self.create_subscription(
            Expression, "/olaf/ears/expression", self._emote_callback, 10
        )
        self._pose_sub = self.create_subscription(
            EarsPose, "/olaf/ears/pose", self._pose_callback, 10
        )

        # Status publisher (1 Hz)
        self._status_pub = self.create_publisher(String, "/olaf/ears/status", 10)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"Ears node ready. Emotions: {list(PRESETS.keys())}"
        )

    def _emote_callback(self, msg: Expression) -> None:
        """Apply emotion preset to ears."""
        intensity = msg.intensity / 5.0  # Convert 1-5 scale to 0.0-1.0
        angles = get_preset(msg.emotion_type, intensity)
        if angles is None:
            self.get_logger().warning(f"Unknown emotion_type: {msg.emotion_type}")
            return

        self.get_logger().debug(
            f"Emote: type={msg.emotion_type} intensity={msg.intensity} -> {angles}"
        )
        self._apply_angles(angles)

    def _pose_callback(self, msg: EarsPose) -> None:
        """Apply direct angle control."""
        angles = {
            "left_pan": msg.left_pan,
            "left_tilt": msg.left_tilt,
            "right_pan": msg.right_pan,
            "right_tilt": msg.right_tilt,
        }
        speed_pct = msg.speed_pct
        self.get_logger().debug(f"Pose: {angles} speed_pct={speed_pct}")
        self._apply_angles(angles, speed_pct)

    def _apply_angles(self, angles: dict[str, float], speed_pct: float = 0.0) -> None:
        """Send angles to all 4 servos."""
        for servo_name, degrees in angles.items():
            ok = self._driver.move(servo_name, degrees, speed_pct)
            if not ok:
                self.get_logger().warning(f"Move failed: {servo_name} -> {degrees}")

    def _publish_status(self) -> None:
        """Publish periodic feedback from all servos."""
        status_parts = []
        for name in ["left_pan", "left_tilt", "right_pan", "right_tilt"]:
            fb = self._driver.read_all_feedback(name)
            pos = fb["position_deg"]
            pos_str = f"{pos:.1f}" if pos is not None else "ERR"
            status_parts.append(f"{name}={pos_str}deg")

        msg = String()
        msg.data = " | ".join(status_parts)
        self._status_pub.publish(msg)

    def destroy_node(self) -> None:
        """Clean shutdown: center ears and close driver."""
        self.get_logger().info("Shutting down ears node, centering ears...")
        self._driver.center_all()
        self._driver.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EarsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
