"""ROS2 driver node for OLAF Head ESP32 module.

Translates ROS2 topic messages into I2C register writes
to control eye expressions, blink, look direction, and
system status on the Head ESP32.

Subscribes to:
    /olaf/head/expression (std_msgs/String) — set eye expression by name
    /olaf/head/blink (std_msgs/Bool) — trigger blink animation
    /olaf/head/look (geometry_msgs/Point) — set look direction (x, y)
    /olaf/head/status (std_msgs/String) — set system status
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point

from head_ears_driver.head_i2c_client import HeadI2CClient, EXPRESSION_MAP, STATUS_MAP


class HeadNode(Node):
    """ROS2 node controlling Head ESP32 via I2C."""

    def __init__(self) -> None:
        super().__init__("head_node")

        # Initialize I2C client
        self._i2c = HeadI2CClient()
        try:
            self._i2c.open()
            self.get_logger().info("Head I2C client connected at 0x08")
        except OSError as e:
            self.get_logger().error(f"Failed to open I2C bus: {e}")
            raise

        # Subscribers
        self._expr_sub = self.create_subscription(
            String, "/olaf/head/expression", self._expression_callback, 10
        )
        self._blink_sub = self.create_subscription(
            Bool, "/olaf/head/blink", self._blink_callback, 10
        )
        self._look_sub = self.create_subscription(
            Point, "/olaf/head/look", self._look_callback, 10
        )
        self._status_sub = self.create_subscription(
            String, "/olaf/head/status", self._status_callback, 10
        )

        self.get_logger().info(
            f"Head node ready. Expressions: {list(EXPRESSION_MAP.keys())} | "
            f"Statuses: {list(STATUS_MAP.keys())}"
        )

    def _expression_callback(self, msg: String) -> None:
        """Set eye expression from string name.

        Accepts optional intensity suffix: "happy:4" sets happy at intensity 4.
        Default intensity is 3.
        """
        parts = msg.data.strip().split(":")
        expression = parts[0].lower()
        intensity = 3
        if len(parts) > 1:
            try:
                intensity = int(parts[1])
            except ValueError:
                self.get_logger().warning(f"Invalid intensity '{parts[1]}', using default 3")

        ok = self._i2c.set_expression(expression, intensity)
        if ok:
            self.get_logger().info(f"Expression: {expression} intensity={intensity}")
        else:
            self.get_logger().warning(f"Failed to set expression: {msg.data}")

    def _blink_callback(self, msg: Bool) -> None:
        """Trigger blink animation when True received."""
        if msg.data:
            ok = self._i2c.trigger_blink()
            if ok:
                self.get_logger().debug("Blink triggered")
            else:
                self.get_logger().warning("Failed to trigger blink")

    def _look_callback(self, msg: Point) -> None:
        """Set look direction from Point (x=horizontal, y=vertical, z ignored)."""
        x = int(max(-100, min(100, msg.x)))
        y = int(max(-100, min(100, msg.y)))
        ok = self._i2c.set_look_direction(x, y)
        if not ok:
            self.get_logger().warning(f"Failed to set look direction: x={x} y={y}")

    def _status_callback(self, msg: String) -> None:
        """Set system status from string name."""
        status = msg.data.strip().lower()
        ok = self._i2c.set_system_status(status)
        if ok:
            self.get_logger().info(f"System status: {status}")
        else:
            self.get_logger().warning(f"Failed to set status: {msg.data}")

    def destroy_node(self) -> None:
        """Clean shutdown: close I2C bus."""
        self.get_logger().info("Shutting down head node...")
        self._i2c.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
