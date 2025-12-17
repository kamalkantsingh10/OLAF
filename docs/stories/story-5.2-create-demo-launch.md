# Story 5.2: Create Demo Launch File

**Epic:** Epic 5 - End-to-End Demo
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 2-3 hours

---

## User Story

**As a** builder,
**I want** a single launch file that brings up all robot systems for the demo,
**so that** I can start everything with one command.

---

## Acceptance Criteria

1. ✅ ROS2 launch file created that starts all driver nodes
2. ✅ Launch file includes parameters for all modules
3. ✅ Nodes start in correct order with dependencies
4. ✅ Launch file tested and all nodes come up successfully
5. ✅ Single command runs entire system: `ros2 launch olaf_bringup demo.launch.py`

---

## Implementation Steps

### 1. Create Launch Package

```bash
cd ~/olaf/ros2/src
mkdir -p olaf_bringup/launch
```

### 2. Create Demo Launch File

**Create `olaf_bringup/launch/demo.launch.py`:**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Base driver (start first - provides balancing)
        Node(
            package='base_driver',
            executable='base_driver_node',
            name='base_driver',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Torso driver
        Node(
            package='torso_driver',
            executable='torso_driver_node',
            name='torso_driver',
            output='screen'
        ),

        # Neck driver
        Node(
            package='neck_driver',
            executable='neck_driver_node',
            name='neck_driver',
            output='screen'
        ),

        # Head+Ears driver
        Node(
            package='head_ears_driver',
            executable='head_ears_driver_node',
            name='head_ears_driver',
            output='screen'
        ),

        # Wait 5 seconds for all drivers to initialize, then optionally start demo
        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         Node(
        #             package='olaf_demos',
        #             executable='phase1_demo',
        #             name='phase1_demo',
        #             output='screen'
        #         )
        #     ]
        # )
    ])
```

### 3. Create Package Configuration

**Create `olaf_bringup/package.xml`:**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>olaf_bringup</name>
  <version>1.0.0</version>
  <description>OLAF system launch files</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>launch</depend>
  <depend>launch_ros</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Create `olaf_bringup/CMakeLists.txt`:**

```cmake
cmake_minimum_required(VERSION 3.5)
project(olaf_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 4. Build and Test

```bash
cd ~/olaf/ros2
colcon build --packages-select olaf_bringup
source install/setup.bash

# Launch all drivers
ros2 launch olaf_bringup demo.launch.py
```

### 5. Verify All Nodes Running

```bash
# In another terminal
ros2 node list

# Expected output:
# /base_driver
# /torso_driver
# /neck_driver
# /head_ears_driver
```

### 6. Create Startup Script

**Create `~/olaf/scripts/start_demo.sh`:**

```bash
#!/bin/bash
# OLAF Phase 1 Demo Startup Script

echo "Starting OLAF Phase 1 Demo..."

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/olaf/ros2/install/setup.bash

# Launch all drivers
ros2 launch olaf_bringup demo.launch.py
```

```bash
chmod +x ~/olaf/scripts/start_demo.sh
```

---

## Testing & Validation

**Test 1: All Drivers Launch**
```bash
# All 4 driver nodes appear in node list
```

**Test 2: No Startup Errors**
```bash
# No error messages in launch output
```

**Test 3: I2C Devices Detected**
```bash
# All modules respond on I2C bus
```

---

## Dependencies

**Before this story:**
- Story 5.1: Create Demo Script ✅

**After this story:**
- Story 5.3: Test and Record End-to-End Demo

---

## Notes

- **Single Command:** Simplifies demo setup
- **Launch Order:** Base first (critical for power/balancing)
- **Future:** Add parameters for network configuration, logging levels

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
