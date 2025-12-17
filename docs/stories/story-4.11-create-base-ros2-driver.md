# Story 4.11: Create Base ROS2 Driver Node

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** a ROS2 driver node that provides standard robot navigation interfaces for the Base module,
**so that** I can control the robot using cmd_vel and publish odometry data.

---

## Acceptance Criteria

1. ✅ Python ROS2 node created in `ros2/src/olaf_drivers/base_driver/`
2. ✅ Subscribes to `/cmd_vel` (geometry_msgs/Twist)
3. ✅ Publishes `/odom` (nav_msgs/Odometry)
4. ✅ Publishes `/base/imu` (sensor_msgs/Imu)
5. ✅ Publishes `/base/battery` (sensor_msgs/BatteryState)
6. ✅ I2C communication to Base ESP32 (0x0B)
7. ✅ Balancing enable/disable service
8. ✅ Velocity commands translate correctly to motor commands

---

## Implementation Steps

### 1. Create ROS2 Package

```bash
cd ~/olaf/ros2/src/olaf_drivers
mkdir -p base_driver/base_driver
touch base_driver/base_driver/base_driver_node.py
chmod +x base_driver/base_driver/base_driver_node.py
```

### 2. Implement Driver Node

**Create `base_driver/base_driver_node.py`:**

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from std_srvs.srv import SetBool
import smbus2
import math

I2C_BUS = 1
I2C_ADDRESS = 0x0B

# Registers
REG_DEVICE_ID = 0x00
REG_BALANCE_ENABLE = 0x10
REG_TARGET_VELOCITY = 0x11
REG_TARGET_OMEGA = 0x12
REG_BATTERY_VOLTAGE = 0x20
REG_PITCH = 0x30
REG_PITCH_RATE = 0x31

class BaseDriverNode(Node):
    def __init__(self):
        super().__init__('base_driver_node')

        self.bus = smbus2.SMBus(I2C_BUS)
        self.verify_module()

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/base/imu', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/base/battery', 10)

        # Services
        self.balance_srv = self.create_service(
            SetBool, '/base/enable_balance', self.enable_balance_callback)

        # Timers
        self.create_timer(0.05, self.publish_state)  # 20 Hz

        self.get_logger().info('Base driver initialized')

    def verify_module(self):
        device_id = self.bus.read_byte_data(I2C_ADDRESS, REG_DEVICE_ID)
        if device_id == 0x04:
            self.get_logger().info('Base module detected')
        else:
            self.get_logger().error(f'Unexpected device ID: 0x{device_id:02x}')

    def cmd_vel_callback(self, msg):
        # Convert Twist to robot velocity (m/s) and angular velocity (rad/s)
        linear_vel = msg.linear.x  # Forward velocity
        angular_vel = msg.angular.z  # Rotation (not used in balancing phase 1)

        # Convert m/s to motor command (-10 to +10 m/s)
        # Scale and offset to fit in uint8 (0-255)
        vel_cmd = int((linear_vel * 12.8) + 128)
        vel_cmd = max(0, min(255, vel_cmd))

        self.bus.write_byte_data(I2C_ADDRESS, REG_TARGET_VELOCITY, vel_cmd)

    def enable_balance_callback(self, request, response):
        enable = 1 if request.data else 0
        self.bus.write_byte_data(I2C_ADDRESS, REG_BALANCE_ENABLE, enable)

        response.success = True
        response.message = f'Balancing {"enabled" if enable else "disabled"}'
        self.get_logger().info(response.message)
        return response

    def publish_state(self):
        # Publish odometry (simplified - no encoder feedback yet)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        # TODO: Integrate encoder feedback for accurate odometry
        self.odom_pub.publish(odom)

        # Publish IMU
        pitch_raw = self.bus.read_byte_data(I2C_ADDRESS, REG_PITCH)
        pitch = (pitch_raw - 128) * 0.5  # Convert to degrees
        pitch_rate_raw = self.bus.read_byte_data(I2C_ADDRESS, REG_PITCH_RATE)
        pitch_rate = (pitch_rate_raw - 128) * 2.0  # Convert to deg/s

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        imu.angular_velocity.x = math.radians(pitch_rate)
        # TODO: Add full orientation quaternion
        self.imu_pub.publish(imu)

        # Publish battery
        voltage_raw = self.bus.read_byte_data(I2C_ADDRESS, REG_BATTERY_VOLTAGE)
        voltage = voltage_raw * 0.2  # Scale to voltage

        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.voltage = voltage
        battery.percentage = (voltage - 30.0) / (42.0 - 30.0)  # 30-42V range
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_pub.publish(battery)

def main(args=None):
    rclpy.init(args=args)
    node = BaseDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Test Driver Node

```bash
# Build and source
cd ~/olaf/ros2
colcon build --packages-select base_driver
source install/setup.bash

# Run driver
ros2 run base_driver base_driver_node

# Enable balancing
ros2 service call /base/enable_balance std_srvs/srv/SetBool "{data: true}"

# Send velocity command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Monitor state
ros2 topic echo /base/imu
ros2 topic echo /base/battery
```

---

## Testing & Validation

**Test 1: cmd_vel Control**
```bash
# Robot moves forward at commanded velocity
```

**Test 2: Balancing Service**
```bash
# Service call enables/disables balancing
```

**Test 3: State Publishing**
```bash
# IMU and battery topics publish at 20 Hz
```

---

## Dependencies

**Before this story:**
- Story 4.10: Fine-Tune Self-Balancing PID ✅

**After this story:**
- Story 4.12: Mount Base Module and Connect All Modules

---

## References

- [ROS2 Navigation](https://navigation.ros.org/)
- [cmd_vel Interface](https://wiki.ros.org/cmd_vel)

---

## Notes

- **cmd_vel Standard:** Universal ROS navigation interface
- **Odometry:** Phase 1 simplified, Phase 2 will add encoder-based odometry
- **Future:** Add TF transforms, costmaps, Nav2 integration

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
