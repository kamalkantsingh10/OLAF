# Story 1.7: Create Head+Ears ROS2 Driver Node

**Epic:** Epic 1 - Head+Ears Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 6-8 hours

---

## User Story

**As a** builder,
**I want** a ROS2 driver node that translates ROS2 topics into I2C commands for Head+Ears module,
**so that** I can control the module using standard ROS2 pub/sub.

---

## Acceptance Criteria

1. ✅ Python ROS2 node created in `ros2/src/olaf_drivers/head_ears_driver/`
2. ✅ Node subscribes to topics: `/head_ears/eyes` (display command), `/head_ears/ears` (position command), `/head_ears/projector` (power/focus command)
3. ✅ Node publishes sensor data (if applicable, e.g., camera feed as separate topic)
4. ✅ I2C communication implemented using `smbus2` library
5. ✅ ROS2 messages translated to I2C register writes to address 0x08
6. ✅ Node launches successfully and appears in `ros2 node list`
7. ✅ Manual topic publish triggers expected hardware response: `ros2 topic pub /head_ears/eyes ...` makes eyes blink

---

## Implementation Steps

### 1. Create ROS2 Package Structure

```bash
cd ~/olaf/ros2/src/olaf_drivers/head_ears_driver

# Verify package created (from Story 0.2)
ls -la
# Should see: package.xml, setup.py, head_ears_driver/ directory

# Create driver node file
mkdir -p head_ears_driver
touch head_ears_driver/head_ears_driver_node.py
chmod +x head_ears_driver/head_ears_driver_node.py
```

### 2. Update Package Configuration

**Edit `package.xml`:**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>head_ears_driver</name>
  <version>1.0.0</version>
  <description>ROS2 driver for OLAF Head+Ears module (I2C 0x08)</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

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

package_name = 'head_ears_driver'

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
    description='ROS2 driver for OLAF Head+Ears module',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_ears_driver_node = head_ears_driver.head_ears_driver_node:main',
        ],
    },
)
```

### 3. Install smbus2 for I2C Communication

```bash
# On Raspberry Pi
pip install smbus2

# Or add to requirements.txt if using virtual environment
echo "smbus2" >> ~/olaf/requirements.txt
```

### 4. Implement Driver Node

**Create `head_ears_driver/head_ears_driver_node.py`:**

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Bool
import smbus2
import time

# I2C configuration
I2C_BUS = 1
I2C_ADDRESS = 0x08

# Register map (from firmware register_map.h)
REG_DEVICE_ID = 0x00
REG_FIRMWARE_VER = 0x01
REG_STATUS = 0x02
REG_EYE_MODE = 0x10
REG_EYE_ANIM_ID = 0x11
REG_EYE_BRIGHTNESS = 0x12
REG_EAR_L_BASE_POS = 0x20
REG_EAR_L_TIP_POS = 0x21
REG_EAR_L_SPEED = 0x22
REG_EAR_R_BASE_POS = 0x23
REG_EAR_R_TIP_POS = 0x24
REG_EAR_R_SPEED = 0x25
REG_PROJ_POWER = 0x30
REG_PROJ_FOCUS = 0x31


class HeadEarsDriverNode(Node):
    def __init__(self):
        super().__init__('head_ears_driver_node')

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
        self.eye_sub = self.create_subscription(
            String,
            '/head_ears/eyes',
            self.eye_callback,
            10
        )

        self.ear_sub = self.create_subscription(
            String,
            '/head_ears/ears',
            self.ear_callback,
            10
        )

        self.projector_power_sub = self.create_subscription(
            Bool,
            '/head_ears/projector/power',
            self.projector_power_callback,
            10
        )

        self.projector_focus_sub = self.create_subscription(
            UInt8,
            '/head_ears/projector/focus',
            self.projector_focus_callback,
            10
        )

        # Create publishers (for status/diagnostics)
        self.status_pub = self.create_publisher(
            String,
            '/head_ears/status',
            10
        )

        # Status update timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Head+Ears driver node initialized')

    def verify_module(self):
        """Verify Head+Ears module is connected and responding."""
        try:
            device_id = self.i2c_read_byte(REG_DEVICE_ID)
            firmware_ver = self.i2c_read_byte(REG_FIRMWARE_VER)

            if device_id != 0x01:
                self.get_logger().warn(f'Unexpected device ID: 0x{device_id:02x}')
            else:
                self.get_logger().info(f'Head+Ears module detected (FW v{firmware_ver >> 4}.{firmware_ver & 0x0F})')
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

    def eye_callback(self, msg):
        """
        Handle eye display commands.
        Expected format: "mode:value" (e.g., "open", "blink", "anim:5")
        """
        command = msg.data.lower()
        self.get_logger().info(f'Eye command received: {command}')

        if command == 'off':
            self.i2c_write_byte(REG_EYE_MODE, 0)
        elif command == 'open':
            self.i2c_write_byte(REG_EYE_MODE, 1)
        elif command == 'blink':
            self.i2c_write_byte(REG_EYE_MODE, 2)
        elif command.startswith('anim:'):
            anim_id = int(command.split(':')[1])
            self.i2c_write_byte(REG_EYE_MODE, 3)
            self.i2c_write_byte(REG_EYE_ANIM_ID, anim_id)
        else:
            self.get_logger().warn(f'Unknown eye command: {command}')

    def ear_callback(self, msg):
        """
        Handle ear position commands.
        Expected format: "left_base,left_tip,right_base,right_tip,speed"
        Example: "128,128,128,128,128" (all centered, medium speed)
        """
        try:
            parts = msg.data.split(',')
            if len(parts) != 5:
                self.get_logger().warn(f'Invalid ear command format: {msg.data}')
                return

            left_base, left_tip, right_base, right_tip, speed = map(int, parts)

            self.i2c_write_byte(REG_EAR_L_BASE_POS, left_base)
            self.i2c_write_byte(REG_EAR_L_TIP_POS, left_tip)
            self.i2c_write_byte(REG_EAR_R_BASE_POS, right_base)
            self.i2c_write_byte(REG_EAR_R_TIP_POS, right_tip)
            self.i2c_write_byte(REG_EAR_L_SPEED, speed)
            self.i2c_write_byte(REG_EAR_R_SPEED, speed)

            self.get_logger().info(f'Ear positions set: L({left_base},{left_tip}) R({right_base},{right_tip}) @ speed {speed}')
        except Exception as e:
            self.get_logger().error(f'Failed to parse ear command: {e}')

    def projector_power_callback(self, msg):
        """Handle projector power on/off."""
        power_state = 1 if msg.data else 0
        self.i2c_write_byte(REG_PROJ_POWER, power_state)
        self.get_logger().info(f'Projector power: {"ON" if msg.data else "OFF"}')

    def projector_focus_callback(self, msg):
        """Handle projector focus adjustment (0-255)."""
        self.i2c_write_byte(REG_PROJ_FOCUS, msg.data)
        self.get_logger().info(f'Projector focus set to: {msg.data}')

    def publish_status(self):
        """Publish module status periodically."""
        try:
            status_byte = self.i2c_read_byte(REG_STATUS)
            status_msg = String()
            status_msg.data = f'Status: 0x{status_byte:02x}'
            self.status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to read status: {e}')

    def destroy_node(self):
        """Clean up I2C bus on shutdown."""
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = HeadEarsDriverNode()
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

### 5. Build and Source Workspace

```bash
cd ~/olaf/ros2

# Build package
colcon build --packages-select head_ears_driver

# Source workspace
source install/setup.bash

# Verify package is recognized
ros2 pkg list | grep head_ears_driver
```

### 6. Test Driver Node

**Terminal 1: Launch Driver Node**
```bash
source ~/olaf/ros2/install/setup.bash
ros2 run head_ears_driver head_ears_driver_node

# Expected output:
# [INFO] [head_ears_driver_node]: I2C bus 1 opened successfully
# [INFO] [head_ears_driver_node]: Head+Ears module detected (FW v1.0)
# [INFO] [head_ears_driver_node]: Head+Ears driver node initialized
```

**Terminal 2: Test Eye Commands**
```bash
# Open eyes
ros2 topic pub --once /head_ears/eyes std_msgs/msg/String "{data: 'open'}"

# Blink eyes
ros2 topic pub --once /head_ears/eyes std_msgs/msg/String "{data: 'blink'}"

# Turn eyes off
ros2 topic pub --once /head_ears/eyes std_msgs/msg/String "{data: 'off'}"
```

**Test Ear Commands**
```bash
# Center all servos
ros2 topic pub --once /head_ears/ears std_msgs/msg/String "{data: '128,128,128,128,128'}"

# Perk up left ear
ros2 topic pub --once /head_ears/ears std_msgs/msg/String "{data: '200,200,128,128,150'}"
```

**Test Projector**
```bash
# Turn projector on
ros2 topic pub --once /head_ears/projector/power std_msgs/msg/Bool "{data: true}"

# Adjust focus (near)
ros2 topic pub --once /head_ears/projector/focus std_msgs/msg/UInt8 "{data: 50}"

# Adjust focus (far)
ros2 topic pub --once /head_ears/projector/focus std_msgs/msg/UInt8 "{data: 200}"

# Turn projector off
ros2 topic pub --once /head_ears/projector/power std_msgs/msg/Bool "{data: false}"
```

### 7. Create Launch File (Optional)

**Create `launch/head_ears_driver.launch.py`:**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='head_ears_driver',
            executable='head_ears_driver_node',
            name='head_ears_driver',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x08,
            }]
        ),
    ])
```

**Test launch file:**
```bash
ros2 launch head_ears_driver head_ears_driver.launch.py
```

---

## Testing & Validation

**Test 1: Node Startup**
```bash
ros2 run head_ears_driver head_ears_driver_node
# Should start without errors, detect module at 0x08
```

**Test 2: Topic Discovery**
```bash
ros2 topic list | grep head_ears
# Expected:
# /head_ears/ears
# /head_ears/eyes
# /head_ears/projector/focus
# /head_ears/projector/power
# /head_ears/status
```

**Test 3: Eye Animation Sequence**
```bash
# Automated test script
for cmd in "open" "blink" "off" "open"; do
    echo "Testing: $cmd"
    ros2 topic pub --once /head_ears/eyes std_msgs/msg/String "{data: '$cmd'}"
    sleep 2
done
```

**Test 4: I2C Communication Reliability**
```bash
# Rapid-fire commands (stress test)
for i in {1..100}; do
    ros2 topic pub --once /head_ears/eyes std_msgs/msg/String "{data: 'blink'}" &
done
# Monitor for errors in driver node output
```

---

## Troubleshooting

**Issue 1: I2C Permission Denied**
- **Solution:** Add user to i2c group: `sudo usermod -aG i2c $USER`, logout/login

**Issue 2: Module Not Detected**
- **Solution:** Check `i2cdetect -y 1`, verify ESP32 running firmware, check wiring

**Issue 3: Commands Don't Trigger Hardware**
- **Solution:** Verify register addresses match firmware, check I2C write with `i2cset` manually

**Issue 4: Node Crashes on Startup**
- **Solution:** Check smbus2 installed, verify I2C bus permissions, check for other I2C conflicts

**Issue 5: Messages Not Parsed Correctly**
- **Solution:** Check message format in callback functions, add debug logging

---

## Dependencies

**Before this story:**
- Story 1.6: Develop Head+Ears ESP32 Firmware ✅
- Story 0.2: Create ROS2 Workspace ✅
- smbus2 library installed

**After this story:**
- Story 1.8: Mount Head+Ears Module to Robot Frame
- Epic 5: End-to-End Demo (uses this driver)

---

## References

- [rclpy Documentation](https://docs.ros2.org/foxy/api/rclpy/)
- [smbus2 Python Library](https://pypi.org/project/smbus2/)
- [ROS2 Topics Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

---

## Notes

- **Message Types:** Using String for simplicity in Phase 1. Phase 2 should use custom message types (.msg files).
- **Error Handling:** Add retry logic for I2C failures in production version.
- **Latency:** I2C write takes ~5ms, acceptable for 30Hz control loop.
- **Future Enhancements:** Add service calls for synchronous operations, action servers for complex animations.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
