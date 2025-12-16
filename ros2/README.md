# ROS2 Workspace

OLAF's orchestration layer built on ROS2 Humble. This workspace contains all ROS2 packages for personality coordination, AI integration, navigation, and hardware drivers.

## Structure

```
ros2/
├── src/                        # ROS2 packages
│   ├── olaf_bringup/          # Launch files, system startup
│   ├── olaf_description/      # URDF robot model, visualizations
│   ├── olaf_drivers/          # Hardware driver nodes (I2C bridge)
│   │   ├── head_ears_driver/  # Head+Ears I2C → ROS2 topics
│   │   ├── neck_driver/
│   │   ├── torso_driver/
│   │   └── base_driver/
│   ├── olaf_personality/      # Personality coordination
│   ├── olaf_ai/               # AI integration (Whisper, Claude)
│   └── olaf_navigation/       # SLAM, navigation stack
├── build/                      # Build output (gitignored)
├── install/                    # Install space (gitignored)
└── log/                        # ROS2 logs (gitignored)
```

## Prerequisites

### System Requirements

- **OS:** Ubuntu 22.04 (Raspberry Pi OS Bookworm also supported)
- **ROS2:** Humble Hawksbill
- **Python:** 3.10+

### Installation

Install ROS2 Humble:

```bash
# Follow official guide: https://docs.ros.org/en/humble/Installation.html
# Or use convenience script:
./scripts/setup/setup_ros2_workspace.sh
```

### Python Dependencies

Install via Poetry (from project root):

```bash
poetry install
```

Or via pip:

```bash
pip install -r requirements.txt
```

## Building the Workspace

```bash
cd ros2
colcon build --symlink-install
```

Build specific package:

```bash
colcon build --packages-select olaf_drivers
```

## Running OLAF

### 1. Source the Workspace

```bash
source ros2/install/setup.bash
```

Add to `~/.bashrc` for convenience:

```bash
echo "source ~/Documents/Garage/OLAF/ros2/install/setup.bash" >> ~/.bashrc
```

### 2. Launch the System

```bash
ros2 launch olaf_bringup olaf.launch.py
```

This starts:
- All hardware driver nodes (I2C communication)
- Personality coordination
- AI services (Whisper STT, Claude agent)
- Navigation stack

### 3. Individual Components

Launch specific subsystems:

```bash
# Hardware drivers only
ros2 launch olaf_bringup drivers.launch.py

# Personality + AI
ros2 launch olaf_bringup personality.launch.py

# Navigation only
ros2 launch olaf_bringup navigation.launch.py
```

## Package Overview

### olaf_drivers

I2C hardware abstraction layer. Each driver node:

- Subscribes to semantic ROS2 topics (e.g., `/head_ears/eyes/expression`)
- Translates to I2C register writes
- Publishes sensor data back to ROS2 topics

**Example:** `head_ears_driver` subscribes to `/projector/command`, sends I2C commands to ESP32 (0x08) to control projector power/focus.

### olaf_personality

Coordinates expressive outputs across modules:

- **emotion_engine:** Maps internal emotional state to expressions
- **expression_sync:** Synchronizes eyes, ears, neck movements
- **projector_content:** Decides what to project based on context

### olaf_ai

AI integration:

- **whisper_stt:** Hailo-accelerated speech recognition
- **agent_client:** Claude API client, tool execution
- **conversation_manager:** Context management, conversation history

### olaf_navigation

SLAM and navigation:

- **Cartographer:** SLAM mapping
- **Nav2:** Path planning, obstacle avoidance
- **base_controller:** Converts nav goals to motor commands

### olaf_bringup

Launch files for system startup:

- `olaf.launch.py` - Full system
- `drivers.launch.py` - Hardware drivers only
- `simulation.launch.py` - Gazebo simulation (future)

### olaf_description

URDF robot model for visualization:

- `urdf/olaf.urdf.xacro` - Robot description
- `meshes/` - 3D meshes for RViz
- `config/` - RViz configurations

## Development Workflow

### Create a New Package

```bash
cd ros2/src
ros2 pkg create --build-type ament_python my_package
```

### Run Tests

```bash
cd ros2
colcon test
colcon test-result --verbose
```

### Code Style

Follow ROS2 Python style guide:

```bash
# Install tools
pip install flake8 pylint

# Check code
flake8 src/olaf_drivers/
pylint src/olaf_drivers/
```

## Debugging

### View Active Topics

```bash
ros2 topic list
ros2 topic echo /head_ears/eyes/expression
```

### Monitor Node Status

```bash
ros2 node list
ros2 node info /head_ears_driver
```

### RViz Visualization

```bash
ros2 launch olaf_description view_robot.launch.py
```

### RQT Graph

```bash
rqt_graph
```

## Configuration

ROS2 parameters are in `config/`:

- `config/ros2/drivers.yaml` - Driver node parameters
- `config/ros2/personality.yaml` - Personality settings
- `config/ros2/navigation.yaml` - Nav2 configuration

Load custom config:

```bash
ros2 launch olaf_bringup olaf.launch.py config:=/path/to/custom.yaml
```

## Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [I2C Protocol Specification](../docs/api/i2c-protocol.md)
