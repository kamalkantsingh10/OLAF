# Story 3.7: Create Torso ROS2 Driver Node

**Epic:** Epic 3 - Torso Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** a ROS2 driver node that translates ROS2 topics into I2C commands for Torso module,
**so that** I can control the heart display and thermal printer using standard ROS2 pub/sub.

---

## Acceptance Criteria

1. ✅ Python ROS2 node created in `ros2/src/olaf_drivers/torso_driver/`
2. ✅ Node subscribes to topics: `/torso/heart` (display command), `/torso/print` (print command)
3. ✅ Node publishes status topic: `/torso/status` (printer ready/busy)
4. ✅ I2C communication implemented using `smbus2` library
5. ✅ ROS2 messages translated to I2C register writes to address 0x09
6. ✅ Node launches successfully and appears in `ros2 node list`
7. ✅ Manual topic publish triggers expected hardware response: heart animates, printer prints

---

## Implementation Steps

### 1. Create ROS2 Package Structure

```bash
cd ~/olaf/ros2/src/olaf_drivers/torso_driver

# Verify package created (from Story 0.2)
ls -la
# Should see: package.xml, setup.py, torso_driver/ directory

# Create driver node file
mkdir -p torso_driver
touch torso_driver/torso_driver_node.py
chmod +x torso_driver/torso_driver_node.py
```

### 2. Update Package Configuration

**Edit `package.xml`:**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>torso_driver</name>
  <version>1.0.0</version>
  <description>ROS2 driver for OLAF Torso module (I2C 0x09)</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Edit `setup.py`:**

```python
from setuptools import setup

package_name = 'torso_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 driver for OLAF Torso module',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'torso_driver_node = torso_driver.torso_driver_node:main',
        ],
    },
)
```

### 3. Implement Driver Node

**Create `torso_driver/torso_driver_node.py`:**

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import smbus2
import time

# I2C configuration
I2C_BUS = 1
I2C_ADDRESS = 0x09

# Register map (from firmware register_map.h)
REG_DEVICE_ID = 0x00
REG_FIRMWARE_VER = 0x01
REG_STATUS = 0x02
REG_HEART_MODE = 0x10
REG_HEART_ANIM_ID = 0x11
REG_HEART_COLOR_R = 0x12
REG_HEART_COLOR_G = 0x13
REG_HEART_COLOR_B = 0x14
REG_HEART_SPEED = 0x15
REG_PRINT_CMD = 0x20
REG_PRINT_STATUS = 0x21
REG_PRINT_BUFFER = 0x22


class TorsoDriverNode(Node):
    def __init__(self):
        super().__init__('torso_driver_node')

        # Initialize I2C bus
        try:
            self.bus = smbus2.SMBus(I2C_BUS)
            self.get_logger().info(f'I2C bus {I2C_BUS} opened successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to open I2C bus: {e}')
            raise

        # Verify module is present
        self.verify_module()

        # Create subscribers
        self.heart_sub = self.create_subscription(
            String,
            '/torso/heart',
            self.heart_callback,
            10
        )

        self.print_sub = self.create_subscription(
            String,
            '/torso/print',
            self.print_callback,
            10
        )

        # Create publishers (for status/diagnostics)
        self.status_pub = self.create_publisher(
            String,
            '/torso/status',
            10
        )

        self.printer_ready_pub = self.create_publisher(
            Bool,
            '/torso/printer_ready',
            10
        )

        # Status update timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Torso driver node initialized')

    def verify_module(self):
        """Verify Torso module is connected and responding."""
        try:
            device_id = self.i2c_read_byte(REG_DEVICE_ID)
            firmware_ver = self.i2c_read_byte(REG_FIRMWARE_VER)

            if device_id != 0x03:
                self.get_logger().warn(f'Unexpected device ID: 0x{device_id:02x}')
            else:
                self.get_logger().info(f'Torso module detected (FW v{firmware_ver >> 4}.{firmware_ver & 0x0F})')
        except Exception as e:
            self.get_logger().error(f'Failed to verify module: {e}')
            raise

    def i2c_write_byte(self, register, value):
        """Write a byte to an I2C register."""
        try:
            self.bus.write_byte_data(I2C_ADDRESS, register, value)
        except Exception as e:
            self.get_logger().error(f'I2C write failed: {e}')

    def i2c_read_byte(self, register):
        """Read a byte from an I2C register."""
        try:
            return self.bus.read_byte_data(I2C_ADDRESS, register)
        except Exception as e:
            self.get_logger().error(f'I2C read failed: {e}')
            return 0

    def heart_callback(self, msg):
        """
        Handle heart display commands.
        Expected format: "mode:params"
        Examples:
          "off"
          "static:255,0,0" (red heart)
          "beat:255,100,100:128" (pink beating heart, medium speed)
          "anim:0:255,0,255:200" (animation 0, magenta, fast)
        """
        command = msg.data.lower()
        self.get_logger().info(f'Heart command received: {command}')

        try:
            parts = command.split(':')
            mode_str = parts[0]

            if mode_str == 'off':
                self.i2c_write_byte(REG_HEART_MODE, 0)

            elif mode_str == 'static':
                # Format: "static:r,g,b"
                if len(parts) > 1:
                    r, g, b = map(int, parts[1].split(','))
                    self.i2c_write_byte(REG_HEART_COLOR_R, r)
                    self.i2c_write_byte(REG_HEART_COLOR_G, g)
                    self.i2c_write_byte(REG_HEART_COLOR_B, b)
                self.i2c_write_byte(REG_HEART_MODE, 1)

            elif mode_str == 'beat':
                # Format: "beat:r,g,b:speed"
                if len(parts) > 1:
                    r, g, b = map(int, parts[1].split(','))
                    self.i2c_write_byte(REG_HEART_COLOR_R, r)
                    self.i2c_write_byte(REG_HEART_COLOR_G, g)
                    self.i2c_write_byte(REG_HEART_COLOR_B, b)
                if len(parts) > 2:
                    speed = int(parts[2])
                    self.i2c_write_byte(REG_HEART_SPEED, speed)
                self.i2c_write_byte(REG_HEART_MODE, 2)

            elif mode_str == 'anim':
                # Format: "anim:id:r,g,b:speed"
                if len(parts) > 1:
                    anim_id = int(parts[1])
                    self.i2c_write_byte(REG_HEART_ANIM_ID, anim_id)
                if len(parts) > 2:
                    r, g, b = map(int, parts[2].split(','))
                    self.i2c_write_byte(REG_HEART_COLOR_R, r)
                    self.i2c_write_byte(REG_HEART_COLOR_G, g)
                    self.i2c_write_byte(REG_HEART_COLOR_B, b)
                if len(parts) > 3:
                    speed = int(parts[3])
                    self.i2c_write_byte(REG_HEART_SPEED, speed)
                self.i2c_write_byte(REG_HEART_MODE, 3)

            else:
                self.get_logger().warn(f'Unknown heart command: {command}')

        except Exception as e:
            self.get_logger().error(f'Failed to parse heart command: {e}')

    def print_callback(self, msg):
        """
        Handle printer commands.
        Text is sent to print buffer, then print command is triggered.
        """
        text = msg.data
        self.get_logger().info(f'Print command received: {text}')

        try:
            # Check if printer is ready
            status = self.i2c_read_byte(REG_PRINT_STATUS)
            if status != 0:
                self.get_logger().warn('Printer is busy, skipping print command')
                return

            # Send text to print buffer
            for char in text:
                self.i2c_write_byte(REG_PRINT_BUFFER, ord(char))
                time.sleep(0.001)  # Small delay between characters

            # Trigger print
            self.i2c_write_byte(REG_PRINT_CMD, 1)

            self.get_logger().info(f'Sent to printer: {text}')

        except Exception as e:
            self.get_logger().error(f'Failed to print: {e}')

    def publish_status(self):
        """Publish module status periodically."""
        try:
            status_byte = self.i2c_read_byte(REG_STATUS)
            printer_status = self.i2c_read_byte(REG_PRINT_STATUS)

            status_msg = String()
            status_msg.data = f'Module: 0x{status_byte:02x}, Printer: {printer_status}'
            self.status_pub.publish(status_msg)

            printer_ready_msg = Bool()
            printer_ready_msg.data = (printer_status == 0)
            self.printer_ready_pub.publish(printer_ready_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to read status: {e}')

    def destroy_node(self):
        """Clean up I2C bus on shutdown."""
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TorsoDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. Build and Source Workspace

```bash
cd ~/olaf/ros2

# Build package
colcon build --packages-select torso_driver

# Source workspace
source install/setup.bash

# Verify package is recognized
ros2 pkg list | grep torso_driver
```

### 5. Test Driver Node

**Terminal 1: Launch Driver Node**
```bash
source ~/olaf/ros2/install/setup.bash
ros2 run torso_driver torso_driver_node

# Expected output:
# [INFO] [torso_driver_node]: I2C bus 1 opened successfully
# [INFO] [torso_driver_node]: Torso module detected (FW v1.0)
# [INFO] [torso_driver_node]: Torso driver node initialized
```

**Terminal 2: Test Heart Commands**
```bash
# Turn heart off
ros2 topic pub --once /torso/heart std_msgs/msg/String "{data: 'off'}"

# Static red heart
ros2 topic pub --once /torso/heart std_msgs/msg/String "{data: 'static:255,0,0'}"

# Beating pink heart
ros2 topic pub --once /torso/heart std_msgs/msg/String "{data: 'beat:255,100,100:128'}"

# Rainbow animation
ros2 topic pub --once /torso/heart std_msgs/msg/String "{data: 'anim:0:255,0,255:200'}"
```

**Test Printer Commands**
```bash
# Print a simple message
ros2 topic pub --once /torso/print std_msgs/msg/String "{data: 'Hello from OLAF!'}"

# Print with newlines
ros2 topic pub --once /torso/print std_msgs/msg/String "{data: 'Line 1\nLine 2\nLine 3'}"
```

**Monitor Status**
```bash
# Watch status updates
ros2 topic echo /torso/status

# Check printer ready state
ros2 topic echo /torso/printer_ready
```

### 6. Create Launch File (Optional)

**Create `launch/torso_driver.launch.py`:**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='torso_driver',
            executable='torso_driver_node',
            name='torso_driver',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x09,
            }]
        ),
    ])
```

**Test launch file:**
```bash
ros2 launch torso_driver torso_driver.launch.py
```

---

## Testing & Validation

**Test 1: Node Startup**
```bash
ros2 run torso_driver torso_driver_node
# Should start without errors, detect module at 0x09
```

**Test 2: Topic Discovery**
```bash
ros2 topic list | grep torso
# Expected:
# /torso/heart
# /torso/print
# /torso/status
# /torso/printer_ready
```

**Test 3: Heart Animation Sequence**
```bash
# Automated test script
for cmd in "off" "static:255,0,0" "beat:0,255,0:150" "off"; do
    echo "Testing: $cmd"
    ros2 topic pub --once /torso/heart std_msgs/msg/String "{data: '$cmd'}"
    sleep 3
done
```

**Test 4: Printer Stress Test**
```bash
# Print multiple messages
for i in {1..5}; do
    ros2 topic pub --once /torso/print std_msgs/msg/String "{data: 'Message $i'}"
    sleep 2  # Wait for printer to be ready
done
```

---

## Troubleshooting

**Issue 1: I2C Permission Denied**
- **Solution:** Add user to i2c group: `sudo usermod -aG i2c $USER`, logout/login

**Issue 2: Module Not Detected**
- **Solution:** Check `i2cdetect -y 1`, verify ESP32 running firmware, check wiring

**Issue 3: Heart Display Not Responding**
- **Solution:** Verify register addresses match firmware, test with `i2cset` manually

**Issue 4: Printer Not Printing**
- **Solution:** Check printer power, verify paper loaded, monitor `/torso/printer_ready` topic

**Issue 5: Characters Missing from Print**
- **Solution:** Increase delay between character writes, check I2C buffer size

---

## Dependencies

**Before this story:**
- Story 3.6: Develop Torso ESP32 Firmware ✅
- Story 0.2: Create ROS2 Workspace ✅
- smbus2 library installed

**After this story:**
- Story 3.8: Install and Configure Raspberry Pi 5 in Torso Module
- Epic 5: End-to-End Demo (uses this driver)

---

## References

- [rclpy Documentation](https://docs.ros2.org/foxy/api/rclpy/)
- [smbus2 Python Library](https://pypi.org/project/smbus2/)
- [ROS2 Topics Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

---

## Notes

- **Message Format:** Using String with colon-separated parameters for simplicity. Phase 2 should use custom message types.
- **Print Buffer:** Limited to 255 characters. Longer messages need to be split.
- **Error Handling:** Add retry logic for I2C failures in production.
- **Latency:** I2C write ~5ms, printer takes 1-2s per line.
- **Future Enhancements:** Add service calls for synchronous printing, action servers for complex animations, image printing support.

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
