# PC Development Setup for OLAF

This guide explains how to set up your PC for OLAF development using a hybrid PC+Pi architecture. With this setup, you develop code on your PC, push via git, and deploy to the Raspberry Pi which handles all hardware control.

**When to use this guide:**
- Part of **Epic 0, Stories 0.2, 0.3, 0.4, 0.7**
- After Raspberry Pi setup is complete (Story 0.1)
- Before starting module development (Epic 1+)

---

## Overview: Hybrid PC+Pi Development Architecture

```
┌─────────────────────────────────────┐
│  YOUR PC (Development)              │
│  ┌───────────────────────────────┐  │
│  │ • Code editing (VS Code)      │  │
│  │ • Git push to deploy          │  │
│  │ • ros2 topic pub/echo         │  │
│  │ • rviz2, rqt visualization    │  │
│  │ • PlatformIO (ESP32 firmware) │  │
│  └───────────────────────────────┘  │
│           ↓ ROS2 DDS (WiFi)         │
│           ↓ ROS_DOMAIN_ID=42        │
└─────────────────────────────────────┘
            ↓
┌─────────────────────────────────────┐
│  RASPBERRY PI 5 (Hardware Control)  │
│  ┌───────────────────────────────┐  │
│  │ ROS2 Driver Nodes             │  │
│  │ • olaf_head (I2C 0x10)        │  │
│  │ • olaf_base (I2C 0x11)        │  │
│  │ • olaf_neck (USB Serial)      │  │
│  │ • olaf_ears (USB Serial)      │  │
│  │ • olaf_indicator (Fusion HAT) │  │
│  └───────────────────────────────┘  │
│           ↓                         │
└─────────────────────────────────────┘
            ↓
    ┌───────────────────────────────────────────────┐
    │ Hardware                                      │
    │ • Head ESP32 (0x10) → 2× GC9A01 OLED eyes    │
    │ • Base ESP32 (0x11) → MPU6050 + ODrive       │
    │ • Waveshare Neck → 3× STS3215 servos         │
    │ • Waveshare Ears → 4× SCS0009 servos         │
    │ • Fusion HAT → WS2812 LEDs, kickstand PWM    │
    └───────────────────────────────────────────────┘
```

**Benefits:**
- ✅ Full IDE experience on PC (VS Code, debugging tools)
- ✅ Direct topic interaction with robot over WiFi
- ✅ No code duplication — same git repo on both machines
- ✅ Faster iteration (edit on PC, test via ROS2 topics)
- ✅ Better visualization tools (rviz2, rqt run better on PC)

---

## Prerequisites

### Hardware Requirements
- PC running Ubuntu 24.04 LTS
- Raspberry Pi 5 with OLAF setup complete
- Both PC and Pi on the same WiFi network

### Software Requirements
- Ubuntu 24.04 LTS (required for ROS2 Jazzy)
- Git
- Python 3.11+

---

## Installation Steps

### 1. Install ROS2 Jazzy on PC

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop (includes rviz2, rqt, etc.)
sudo apt update
sudo apt install ros-jazzy-desktop -y

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep
sudo rosdep init
rosdep update
```

### 2. Configure ROS2 Environment

```bash
# Add to ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc

# Source immediately
source ~/.bashrc
```

**IMPORTANT:** The `ROS_DOMAIN_ID=42` must match your Raspberry Pi.

### 3. Clone OLAF Repository

Clone directly to home directory (same structure as Pi):

```bash
cd ~
git clone https://github.com/kamalkantsingh10/OLAF.git olaf
```

Note: Use HTTPS for simplicity. For SSH (passwordless push): `git clone git@github.com:kamalkantsingh10/OLAF.git olaf`

### 4. Install Python Dependencies

```bash
pip install smbus2 pyserial numpy
```

### 5. Build ROS2 Workspace

```bash
cd ~/olaf/ros2

# Install ROS2 package dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/olaf/ros2/install/setup.bash" >> ~/.bashrc
```

---

## Network Configuration

### 1. Verify ROS2 Domain ID (Both PC and Pi)

```bash
# On PC
echo $ROS_DOMAIN_ID  # Should be 42

# On Pi
ssh kamal@olaf.local "echo \$ROS_DOMAIN_ID"  # Should be 42
```

### 2. Configure Firewall (PC)

```bash
# Allow ROS2 DDS ports
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp

# Or disable firewall for local network (trusted networks only)
sudo ufw disable
```

### 3. Verify Network Connectivity

```bash
# Ping Pi from PC
ping olaf.local

# Test ROS2 cross-machine communication
# On Pi: Start a talker
ssh kamal@olaf.local "ros2 run demo_nodes_cpp talker"

# On PC: Listen (should see messages)
ros2 topic echo /chatter
```

---

## Development Workflow

### Git-Based Deployment

```
┌──────────────────────────────────────────────────────────────┐
│  1. EDIT on PC                                               │
│     code ~/olaf/ros2/src/olaf_head/                          │
│                          ↓                                   │
│  2. COMMIT & PUSH                                            │
│     cd ~/olaf && git add . && git commit -m "feat: ..." && git push │
│                          ↓                                   │
│  3. PULL on Pi                                               │
│     ssh kamal@olaf.local "cd ~/olaf && git pull"             │
│                          ↓                                   │
│  4. BUILD on Pi                                              │
│     ssh kamal@olaf.local "cd ~/olaf/ros2 && colcon build"    │
│                          ↓                                   │
│  5. RUN on Pi                                                │
│     ssh kamal@olaf.local "ros2 run olaf_head head_driver"    │
│                          ↓                                   │
│  6. INTERACT from PC                                         │
│     ros2 topic pub /head/expression ...                      │
└──────────────────────────────────────────────────────────────┘
```

### Quick Deploy Script

Create `~/olaf/scripts/deploy.sh`:

```bash
#!/bin/bash
set -e

PI_HOST="kamal@olaf.local"

echo "=== Deploying to Pi ==="

echo "1. Pulling latest code on Pi..."
ssh ${PI_HOST} "cd ~/olaf && git pull"

echo "2. Building on Pi..."
ssh ${PI_HOST} "cd ~/olaf/ros2 && source /opt/ros/jazzy/setup.bash && colcon build"

echo "=== Deploy complete ==="
echo "Run nodes with: ssh ${PI_HOST} 'cd ~/olaf/ros2 && source install/setup.bash && ros2 run ...'"
```

Make executable: `chmod +x scripts/deploy.sh`

### Daily Development Workflow

**Terminal 1 — Pi Driver Nodes:**
```bash
ssh kamal@olaf.local
cd ~/olaf_ws && source install/setup.bash
ros2 launch olaf_bringup drivers.launch.py
```

**Terminal 2 — PC Topic Interaction:**
```bash
# List available topics
ros2 topic list

# Send expression command to Head
ros2 topic pub /head/expression olaf_interfaces/msg/Expression "{emotion: 1, intensity: 3}" --once

# Echo sensor data from Base
ros2 topic echo /base/imu
```

**Terminal 3 — PC Visualization:**
```bash
rqt_graph  # View node connections
rqt        # All-in-one dashboard
```

---

## ESP32 Firmware Development on PC

### 1. Install PlatformIO

```bash
# Install PlatformIO Core
pip install platformio

# Add udev rules for ESP32
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Add user to dialout group
sudo usermod -aG dialout $USER
# Log out and back in
```

### 2. Build and Upload Firmware

```bash
# Head module (ESP32, I2C 0x10)
cd ~/olaf/modules/head/firmware
pio run                      # Build
pio run --target upload      # Upload (ESP32 connected to PC via USB)
pio device monitor           # Serial monitor

# Base module (ESP32, I2C 0x11)
cd ~/olaf/modules/base/firmware
pio run --target upload
```

---

## ROS2 Topic Reference

### Head Module (I2C 0x10)
```bash
# Subscribe topics (Pi → Head ESP32)
/head/expression    # Set eye expression
/head/blink         # Trigger blink
/head/look          # Set look direction

# Example
ros2 topic pub /head/expression olaf_interfaces/msg/Expression "{emotion: 1, intensity: 3}" --once
```

### Base Module (I2C 0x11)
```bash
# Subscribe topics (Pi → Base ESP32)
/cmd_vel            # Velocity commands (geometry_msgs/Twist)

# Publish topics (Base ESP32 → Pi)
/odom               # Odometry
/imu                # IMU data

# Example
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

### Neck Module (USB Serial)
```bash
/neck/pose          # Set pan/tilt/roll angles
/neck/lookat        # Look at point in space

# Example
ros2 topic pub /neck/pose olaf_interfaces/msg/NeckPose "{pan: 30.0, tilt: 15.0, roll: 0.0}" --once
```

### Ears Module (USB Serial)
```bash
/ears/emote         # Preset emotions
/ears/pose          # Direct servo angles

# Example
ros2 topic pub /ears/emote std_msgs/msg/String "data: 'alert'" --once
```

### Indicator Module (Fusion HAT)
```bash
/indicator/interact # Interaction feedback LEDs
/indicator/status   # Status LEDs
/indicator/pid      # PID visualization LEDs

# Example
ros2 topic pub /indicator/status std_msgs/msg/String "data: 'success'" --once
```

---

## Monitoring and Debugging Tools

### RQt Tools (run on PC)

```bash
rqt_graph           # Visualize node connections
rqt_topic           # Monitor topics
rqt_console         # View logs
rqt_plot            # Plot numeric data
rqt                 # All-in-one dashboard
```

### Command-Line Tools

```bash
ros2 topic list                          # List all topics
ros2 topic echo /head/expression         # Monitor topic
ros2 topic info /head/expression         # Topic details
ros2 topic pub /head/blink std_msgs/msg/Bool "data: true" --once

ros2 node list                           # List all nodes
ros2 node info /olaf_head                # Node details

ros2 service list                        # List services
ros2 param list                          # List parameters
```

---

## Troubleshooting

### Issue: PC cannot see Pi's ROS2 topics

**Check 1: ROS_DOMAIN_ID matches**
```bash
# PC
echo $ROS_DOMAIN_ID  # Should be 42

# Pi
ssh kamal@olaf.local "echo \$ROS_DOMAIN_ID"  # Should be 42
```

**Check 2: Firewall**
```bash
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
```

**Check 3: Network connectivity**
```bash
ping olaf.local
```

**Check 4: ROS_LOCALHOST_ONLY is disabled**
```bash
echo $ROS_LOCALHOST_ONLY  # Should be 0 or empty
```

### Issue: Permission denied when uploading to ESP32

```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

### Issue: I2C errors on Pi

```bash
# Check I2C devices
ssh kamal@olaf.local "i2cdetect -y 1"
# Should show 0x10 (Head) and 0x11 (Base) when ESP32s connected
```

---

## Workspace Structure

Same structure on both PC and Pi — keeps things simple:

```
~/olaf/                             # Git repo (home base)
├── ros2/                           # ROS2 workspace
│   ├── src/
│   │   ├── olaf_bringup/           # Launch files
│   │   ├── olaf_interfaces/        # Custom messages
│   │   ├── olaf_head/              # Head driver node
│   │   ├── olaf_base/              # Base driver node
│   │   ├── olaf_neck/              # Neck driver node
│   │   ├── olaf_ears/              # Ears driver node
│   │   └── olaf_indicator/         # LED driver node
│   ├── build/                      # Colcon build output (gitignored)
│   ├── install/                    # Colcon install output (gitignored)
│   └── log/                        # Colcon logs (gitignored)
├── modules/
│   ├── head/
│   │   └── firmware/               # Head ESP32 firmware
│   └── base/
│       └── firmware/               # Base ESP32 firmware
├── docs/                           # Documentation
└── tools/                          # Diagnostic scripts
```

| Task | Directory |
|------|-----------|
| Git operations | `~/olaf/` |
| ROS2 build | `~/olaf/ros2/` |
| ESP32 firmware | `~/olaf/modules/{module}/firmware/` |

---

## Best Practices

1. **Always source workspace:**
   ```bash
   source ~/olaf_ws/install/setup.bash
   ```

2. **Use `--symlink-install` for faster Python iteration:**
   ```bash
   colcon build --symlink-install
   # Python changes don't require rebuild
   ```

3. **Build specific packages only:**
   ```bash
   colcon build --packages-select olaf_head
   ```

4. **Keep Pi drivers running, iterate via topics from PC**

5. **Use rqt_graph to visualize system state**

6. **Commit frequently, deploy often**

---

## Related Documentation

- **Story 0.1**: Install ROS2 on Pi — `docs/stories/0.1.install-ros2-jazzy.md`
- **Story 0.2**: Install ROS2 on PC — `docs/stories/0.2.install-ros2-pc.md`
- **Story 0.3**: Multi-machine networking — `docs/stories/0.3.configure-multi-machine-ros2.md`
- **Story 0.4**: Git workflow — `docs/stories/0.4.setup-git-workflow.md`
- **Story 0.7**: Dev tools on PC — `docs/stories/0.7.setup-dev-tools-pc.md`
- **Architecture**: `docs/architecture/tech-stack.md`
- **Coding Standards**: `docs/architecture/coding-standards.md`

---

**Last Updated:** 2026-02-01
