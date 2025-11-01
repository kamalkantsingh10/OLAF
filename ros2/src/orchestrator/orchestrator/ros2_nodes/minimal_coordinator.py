#!/usr/bin/env python3
"""
Minimal Coordinator Node - Expression Coordinator

This node cycles through emotional expressions, publishing to the head driver
node via ROS2 topics. It serves as a validation tool for the full pipeline
(Orchestrator → ROS2 → I2C → ESP32 → SPI eye displays).

Blink animations are handled autonomously by the ESP32 firmware, so this node
only controls expression changes.

Node: /olaf/minimal_coordinator
Publishes:
  - /olaf/head/expression (std_msgs/String): Expression commands in format "emotion:intensity"

Author: OLAF Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random


class MinimalCoordinator(Node):
    """
    Minimal coordinator node that cycles through expressions.

    This node demonstrates the full end-to-end pipeline by:
    - Cycling through 7 expressions every 10 seconds
    - Randomly varying intensity (2-4) for each expression
    - Publishing commands to the head driver node

    Note: Blink animations are handled autonomously by ESP32 firmware.
    """

    # Expression sequence to cycle through
    EXPRESSIONS = ['neutral', 'happy', 'curious', 'thinking', 'confused', 'sad', 'excited']

    # Timing constant for expression cycling
    CYCLE_INTERVAL_SEC = 10.0

    # Intensity range for expressions
    INTENSITY_MIN = 2
    INTENSITY_MAX = 4

    def __init__(self):
        """Initialize the minimal coordinator node."""
        super().__init__('minimal_coordinator')

        # Create expression publisher
        self.expression_pub = self.create_publisher(
            String,
            '/olaf/head/expression',
            10
        )

        # Initialize expression cycle index
        self.expression_index = 0

        # Create timer for expression cycling
        self.expression_timer = self.create_timer(
            self.CYCLE_INTERVAL_SEC,
            self.expression_callback
        )

        # Log startup information
        self.get_logger().info('Minimal Coordinator started')
        self.get_logger().info(
            f'Cycling expressions every {self.CYCLE_INTERVAL_SEC} seconds'
        )
        self.get_logger().info('Blinks handled autonomously by firmware')

        # Publish initial expression immediately
        self.expression_callback()

    def expression_callback(self):
        """
        Timer callback for expression cycling.

        Publishes the current expression with random intensity, then advances
        to the next expression in the sequence.
        """
        # Get current expression
        expression = self.EXPRESSIONS[self.expression_index]

        # Generate random intensity (2-4)
        intensity = random.randint(self.INTENSITY_MIN, self.INTENSITY_MAX)

        # Create and publish expression message
        # Format: "expression:intensity" (e.g., "happy:3")
        msg = String()
        msg.data = f"{expression}:{intensity}"
        self.expression_pub.publish(msg)

        # Log expression change
        self.get_logger().info(
            f"Expression: {expression}, Intensity: {intensity}"
        )

        # Cycle to next expression (wrap around at end)
        self.expression_index = (self.expression_index + 1) % len(self.EXPRESSIONS)


def main(args=None):
    """
    Main entry point for the minimal coordinator node.

    Args:
        args: Command-line arguments (optional)
    """
    rclpy.init(args=args)
    coordinator = MinimalCoordinator()

    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
