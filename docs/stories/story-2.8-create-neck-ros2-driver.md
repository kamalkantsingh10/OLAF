# Story 2.8: Create Neck ROS2 Driver Node

**Epic:** Epic 2 - Neck Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 5-7 hours

---

## User Story

**As a** builder,
**I want** a ROS2 driver node that translates ROS2 topics into I2C commands for Neck module,
**so that** I can control the module using standard ROS2 pub/sub.

---

## Acceptance Criteria

1. ✅ Python ROS2 node created in `ros2/src/olaf_drivers/neck_driver/`
2. ✅ Node subscribes to topics: `/neck/position` (pan/tilt/roll command), `/neck/kickstand` (deploy/retract command)
3. ✅ Node publishes sensor data: `/neck/presence` (detection state from 2 sensors)
4. ✅ I2C communication implemented using `smbus2` library
5. ✅ ROS2 messages translated to I2C register writes to address 0x09
6. ✅ Node launches successfully and appears in `ros2 node list`
7. ✅ Manual topic publish triggers expected hardware response

---

## Implementation Steps

### 1. Create Driver Node

**neck_driver/neck_driver_node.py:**

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import smbus2

I2C_BUS = 1
I2C_ADDRESS = 0x09

# Register map
REG_PAN_POSITION = 0x10
REG_TILT_POSITION = 0x11
REG_ROLL_POSITION = 0x12
REG_MOVEMENT_SPEED = 0x13
REG_KICKSTAND_CMD = 0x20
REG_PRESENCE_FRONT = 0x30
REG_PRESENCE_REAR = 0x31


class NeckDriverNode(Node):
    def __init__(self):
        super().__init__('neck_driver_node')

        self.bus = smbus2.SMBus(I2C_BUS)
        self.verify_module()

        # Subscribers
        self.position_sub = self.create_subscription(
            String, '/neck/position', self.position_callback, 10
        )
        self.kickstand_sub = self.create_subscription(
            Bool, '/neck/kickstand', self.kickstand_callback, 10
        )

        # Publishers
        self.presence_pub = self.create_publisher(
            String, '/neck/presence', 10
        )

        # Timer for presence sensor polling
        self.sensor_timer = self.create_timer(0.5, self.publish_presence)

        self.get_logger().info('Neck driver node initialized')

    def verify_module(self):
        device_id = self.bus.read_byte_data(I2C_ADDRESS, 0x00)
        if device_id == 0x02:
            self.get_logger().info('Neck module detected')
        else:
            self.get_logger().warn(f'Unexpected device ID: 0x{device_id:02x}')

    def position_callback(self, msg):
        """
        Format: "pan,tilt,roll,speed"
        Example: "128,128,128,150"
        """
        try:
            parts = msg.data.split(',')
            pan, tilt, roll, speed = map(int, parts)

            self.bus.write_byte_data(I2C_ADDRESS, REG_PAN_POSITION, pan)
            self.bus.write_byte_data(I2C_ADDRESS, REG_TILT_POSITION, tilt)
            self.bus.write_byte_data(I2C_ADDRESS, REG_ROLL_POSITION, roll)
            self.bus.write_byte_data(I2C_ADDRESS, REG_MOVEMENT_SPEED, speed)

            self.get_logger().info(f'Neck position: pan={pan}, tilt={tilt}, roll={roll}')
        except Exception as e:
            self.get_logger().error(f'Failed to parse position command: {e}')

    def kickstand_callback(self, msg):
        """Deploy (True) or retract (False) kickstand."""
        cmd = 1 if msg.data else 0
        self.bus.write_byte_data(I2C_ADDRESS, REG_KICKSTAND_CMD, cmd)
        self.get_logger().info(f'Kickstand: {"DEPLOY" if msg.data else "RETRACT"}')

    def publish_presence(self):
        """Read and publish presence sensor states."""
        try:
            front = self.bus.read_byte_data(I2C_ADDRESS, REG_PRESENCE_FRONT)
            rear = self.bus.read_byte_data(I2C_ADDRESS, REG_PRESENCE_REAR)

            msg = String()
            msg.data = f'front:{front},rear:{rear}'
            self.presence_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to read sensors: {e}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = NeckDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Update Setup Files

**setup.py:** Add entry point for `neck_driver_node`

### 3. Build and Test

```bash
cd ~/olaf/ros2
colcon build --packages-select neck_driver
source install/setup.bash

# Run node
ros2 run neck_driver neck_driver_node

# Test commands:
ros2 topic pub --once /neck/position std_msgs/msg/String "{data: '128,128,128,150'}"
ros2 topic pub --once /neck/kickstand std_msgs/msg/Bool "{data: true}"
ros2 topic echo /neck/presence
```

---

## Testing & Validation

Test all topics work as expected, servos move, kickstand deploys, presence sensors read correctly.

---

## Dependencies

**Before this story:**
- Story 2.7: Develop Neck ESP32 Firmware ✅

**After this story:**
- Story 2.9: Mount Neck Module to Robot Frame

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
