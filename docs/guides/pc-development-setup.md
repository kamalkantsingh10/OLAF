# PC Development Setup for OLAF

This guide explains how to set up your PC for OLAF development using a hybrid PC+Pi architecture. With this setup, you can develop and test ROS2 application nodes on your PC while the Raspberry Pi handles hardware driver nodes and ESP32 module communication.

---

## Overview: Hybrid PC+Pi Development Architecture

```
┌─────────────────────────────────────┐
│  YOUR PC (Development)              │
│  ┌───────────────────────────────┐  │
│  │ ROS2 Application Nodes        │  │
│  │ - olaf_personality            │  │
│  │ - olaf_ai                     │  │
│  │ - olaf_navigation             │  │
│  │ - Development tools (rqt)    │  │
│  └───────────────────────────────┘  │
│           ↓ ROS2 DDS (WiFi)         │
└─────────────────────────────────────┘
            ↓
┌─────────────────────────────────────┐
│  RASPBERRY PI (Hardware Control)    │
│  ┌───────────────────────────────┐  │
│  │ ROS2 Driver Nodes ONLY        │  │
│  │ - head_ears_driver            │  │
│  │ - neck_driver                 │  │
│  │ - torso_driver                │  │
│  │ - base_driver                 │  │
│  └───────────────────────────────┘  │
│           ↓ I2C (Wired)             │
└─────────────────────────────────────┘
            ↓
    ┌───────────────────────┐
    │ ESP32 Modules         │
    │ - Head+Ears (0x08)    │
    │ - Neck (0x09)         │
    │ - Torso (0x0A)        │
    │ - Base (0x0B)         │
    └───────────────────────┘
```

**Benefits:**
- ✅ Use your PC's power and development tools
- ✅ Keep existing automation workflows
- ✅ Test without deploying to Pi every time
- ✅ Easy migration to production (same code, different machine)
- ✅ Faster iteration cycle
- ✅ Better debugging tools (RViz, RQt, IDE integration)

---

## Prerequisites

### Hardware Requirements
- PC running Ubuntu 22.04 LTS (native or VM)
- Raspberry Pi 5 with OLAF setup complete
- Both PC and Pi on the same WiFi network
- ESP32 modules flashed with firmware (optional for initial development)

### Software Requirements
- Ubuntu 22.04 LTS (for native ROS2 Humble support)
- Git
- Python 3.10+

---

## Installation Steps

### 1. Install ROS2 Humble on PC

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

# Install ROS2 Humble Desktop (includes RViz, RQt, etc.)
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep
sudo rosdep init
rosdep update
```

### 2. Configure ROS2 Environment

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Source immediately
source ~/.bashrc
```

**IMPORTANT:** The `ROS_DOMAIN_ID` must match your Raspberry Pi (default: 42).

### 3. Clone OLAF Repository

```bash
cd ~
git clone <your-olaf-repo-url> olaf
cd olaf
```

### 4. Install Python Dependencies

```bash
cd ~/olaf

# Install Poetry (if not already installed)
curl -sSL https://install.python-poetry.org | python3 -

# Install dependencies
poetry install

# OR use pip
pip3 install -r requirements.txt  # Create this if needed
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

### 1. Set ROS2 Domain ID (Both PC and Pi)

The `ROS_DOMAIN_ID` isolates your robot from other ROS2 systems on the network.

**On PC:**
```bash
# Add to ~/.bashrc (should already be set if you followed installation)
export ROS_DOMAIN_ID=42
```

**On Pi:**
```bash
# Should already be set by Pi setup
echo $ROS_DOMAIN_ID  # Verify it shows 42
```

### 2. Configure Firewall (PC)

```bash
# On Ubuntu PC, allow ROS2 DDS ports
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp

# Or disable firewall for local network
sudo ufw disable  # Only if on trusted local network
```

### 3. Verify Network Connectivity

**Check if PC can see Pi's ROS2 topics:**

```bash
# On Pi: Start driver nodes
cd ~/olaf/ros2
source install/setup.bash
ros2 launch olaf_bringup drivers.launch.py

# On PC: List topics (should see /olaf/* topics)
ros2 topic list
```

**Expected output:**
```
/olaf/head_ears/eyes/expression
/olaf/head_ears/projector/status
/olaf/neck/position
/olaf/torso/heart/animation
/olaf/base/velocity
/parameter_events
/rosout
```

If you don't see topics, check the troubleshooting section.

---

## Development Workflow

### 1. Start Driver Nodes on Pi

**SSH into Pi:**
```bash
ssh pi@raspberrypi.local
# OR
ssh pi@<pi-ip-address>
```

**Start hardware driver nodes:**
```bash
cd ~/olaf/ros2
source install/setup.bash
ros2 launch olaf_bringup drivers.launch.py
```

**Leave this running** - Pi is now the hardware bridge to ESP32 modules.

### 2. Develop Application Nodes on PC

**Terminal 1 - Personality Coordination:**
```bash
cd ~/olaf/ros2
source install/setup.bash
ros2 launch olaf_bringup personality.launch.py
```

**Terminal 2 - AI Agent:**
```bash
cd ~/olaf/ros2
source install/setup.bash
ros2 run olaf_ai agent_node
```

**Terminal 3 - Navigation (when ready):**
```bash
cd ~/olaf/ros2
source install/setup.bash
ros2 launch olaf_bringup navigation.launch.py
```

### 3. Test Cross-Machine Communication

**Publish from PC, hardware responds on Pi:**

```bash
# On PC: Send expression command
ros2 topic pub /olaf/head_ears/eyes/expression \
  std_msgs/String "data: 'happy'" --once

# Eyes on actual robot should change (if ESP32 connected)
# Or check Pi terminal for driver node logs
```

### 4. Monitor with RQt Tools (PC)

```bash
# Visualize ROS2 graph
rqt_graph

# Monitor topics
rqt_topic

# All-in-one dashboard
rqt
```

### 5. Develop-Test-Deploy Cycle

```bash
# 1. Edit code on PC
code ros2/src/olaf_personality/olaf_personality/emotion_engine.py

# 2. Rebuild (from ros2/ directory)
colcon build --packages-select olaf_personality
source install/setup.bash

# 3. Test on PC (with Pi drivers running)
ros2 run olaf_personality emotion_engine

# 4. Commit changes
git add .
git commit -m "feat: update emotion engine"
git push origin main

# 5. Deploy to Pi
ssh pi@raspberrypi.local
cd ~/olaf/ros2
git pull origin main
colcon build
source install/setup.bash

# 6. Run full system on Pi (production)
ros2 launch olaf_bringup olaf.launch.py
```

---

## ESP32 Firmware Development on PC

You can develop, compile, and upload ESP32 firmware from your PC.

### 1. Install PlatformIO

```bash
# Install PlatformIO Core
pip3 install platformio

# OR install PlatformIO IDE (VS Code extension)
# Search "PlatformIO IDE" in VS Code extensions
```

### 2. Develop Module Firmware

```bash
# Navigate to module
cd ~/olaf/modules/head-ears/firmware

# Open in editor
code .  # VS Code
# OR
vim src/main.cpp
```

### 3. Build Firmware

```bash
cd ~/olaf/modules/head-ears/firmware

# Build
pio run

# Build for specific environment
pio run -e esp32dev-release
```

### 4. Upload to ESP32 (USB Connected to PC)

```bash
# Connect ESP32 to PC via USB

# Check port
ls /dev/ttyUSB*  # Linux
ls /dev/cu.*     # macOS

# Upload (auto-detects port)
pio run --target upload

# Or specify port
pio run --target upload --upload-port /dev/ttyUSB0
```

### 5. Monitor Serial Output

```bash
# Open serial monitor
pio device monitor

# Or with upload
pio run --target upload && pio device monitor
```

### 6. Module-Specific Development

Each module has its own complete firmware:

```bash
# Head+Ears (0x08)
cd modules/head-ears/firmware
pio run --target upload

# Neck (0x09)
cd modules/neck/firmware
pio run --target upload

# Torso (0x0A)
cd modules/torso/firmware
pio run --target upload

# Base (0x0B)
cd modules/base/firmware
pio run --target upload
```

---

## Module Testing Workflow

### 1. Test Individual Modules

```bash
# Test head-ears module
cd ~/olaf/modules/head-ears

# Run module tests
pytest tests/

# Run diagnostics
python3 diagnostics/projector_diagnostic.py
python3 diagnostics/eye_calibration.py
```

### 2. Test System Integration

```bash
# Run system-wide integration tests
cd ~/olaf
pytest tests/integration/

# Test I2C communication (on Pi, with PC monitoring)
python3 tools/diagnostics/i2c_scanner.py
```

### 3. Use Module Scripts

```bash
cd ~/olaf/modules/head-ears

# Build and flash (when scripts are created)
./scripts/build.sh
./scripts/flash.sh
./scripts/test.sh
```

---

## Using Your Existing PC Automation

### Git Hooks

Create `.git/hooks/pre-commit`:

```bash
#!/bin/bash
# Run checks before commit

# Format Python code
black ros2/src/ --check

# Lint code
ruff ros2/src/

# Run unit tests
cd ros2
colcon test --packages-select olaf_personality olaf_ai

# Run pytest
cd ..
pytest tests/unit/
```

Make it executable:
```bash
chmod +x .git/hooks/pre-commit
```

### VS Code Remote SSH

1. Install "Remote - SSH" extension
2. Connect to Pi: `Cmd+Shift+P` → "Remote-SSH: Connect to Host"
3. Enter: `pi@raspberrypi.local`
4. Edit files on Pi directly from PC VS Code

### rsync for Fast Sync

```bash
# Sync code from PC to Pi (faster than git for iteration)
rsync -avz --exclude='build/' --exclude='install/' --exclude='log/' \
  --exclude='.git/' ~/olaf/ros2/ pi@raspberrypi.local:~/olaf/ros2/

# Rebuild on Pi
ssh pi@raspberrypi.local "cd ~/olaf/ros2 && colcon build && source install/setup.bash"
```

### Automated Build Script

Create `scripts/sync_to_pi.sh`:

```bash
#!/bin/bash
set -e

PI_HOST="pi@raspberrypi.local"
PROJECT_DIR="~/olaf"

echo "Syncing to Pi..."
rsync -avz --exclude='build/' --exclude='install/' --exclude='log/' \
  --exclude='.git/' ${PROJECT_DIR}/ros2/ ${PI_HOST}:${PROJECT_DIR}/ros2/

echo "Building on Pi..."
ssh ${PI_HOST} "cd ${PROJECT_DIR}/ros2 && colcon build --symlink-install"

echo "Done! Run 'ssh pi@raspberrypi.local' to test on Pi."
```

---

## Monitoring and Debugging Tools

### RQt Tools (GUI - run on PC)

```bash
# ROS2 graph visualizer
rqt_graph

# Topic monitor
rqt_topic

# Service caller
rqt_service_caller

# Message publisher
rqt_publisher

# Plot topic data
rqt_plot /olaf/base/velocity/linear/x

# All-in-one dashboard
rqt
```

### Command-Line Tools

```bash
# List all topics
ros2 topic list

# Echo topic data
ros2 topic echo /olaf/head_ears/eyes/expression

# Show topic info
ros2 topic info /olaf/head_ears/eyes/expression

# Publish to topic
ros2 topic pub /olaf/head_ears/projector/command \
  std_msgs/String "data: 'ON'" --once

# List all nodes
ros2 node list

# Show node info
ros2 node info /head_ears_driver

# List services
ros2 service list

# Call service
ros2 service call /olaf/head_ears/calibrate std_srvs/Trigger

# Check parameter
ros2 param get /head_ears_driver i2c_address

# Set parameter
ros2 param set /head_ears_driver brightness 200
```

### RViz for Visualization (when URDF is created)

```bash
# Launch robot visualization
ros2 launch olaf_description view_robot.launch.py

# Or manually
rviz2
```

---

## Troubleshooting

### Issue: PC cannot see Pi's ROS2 topics

**Symptoms:**
```bash
ros2 topic list  # Only shows /parameter_events and /rosout
```

**Solutions:**

1. **Check ROS_DOMAIN_ID match**
   ```bash
   # On PC
   echo $ROS_DOMAIN_ID  # Should be 42

   # On Pi
   ssh pi@raspberrypi.local "echo \$ROS_DOMAIN_ID"  # Should be 42
   ```

2. **Check firewall**
   ```bash
   # On PC
   sudo ufw status
   sudo ufw allow 7400:7500/udp
   sudo ufw allow 7400:7500/tcp

   # Or disable temporarily
   sudo ufw disable
   ```

3. **Check network connectivity**
   ```bash
   # Ping Pi from PC
   ping raspberrypi.local

   # Check if on same subnet
   ip addr show  # On PC
   ssh pi@raspberrypi.local "ip addr show"  # On Pi
   ```

4. **Check Pi driver nodes are running**
   ```bash
   ssh pi@raspberrypi.local "ros2 node list"
   # Should show: /head_ears_driver, /neck_driver, etc.
   ```

5. **Configure static DDS peers** (if multicast blocked)

   Create `~/.ros/fastdds.xml` on both PC and Pi:
   ```xml
   <?xml version="1.0" encoding="UTF-8" ?>
   <profiles>
     <transport_descriptors>
       <transport_descriptor>
         <transport_id>udp_transport</transport_id>
         <type>UDPv4</type>
       </transport_descriptor>
     </transport_descriptors>
     <participant profile_name="participant_profile" is_default_profile="true">
       <rtps>
         <builtin>
           <metatrafficUnicastLocatorList>
             <locator>
               <udpv4>
                 <address>192.168.1.100</address>  <!-- PC IP -->
               </udpv4>
             </locator>
             <locator>
               <udpv4>
                 <address>192.168.1.101</address>  <!-- Pi IP -->
               </udpv4>
             </locator>
           </metatrafficUnicastLocatorList>
         </builtin>
       </rtps>
     </participant>
   </profiles>
   ```

   Set environment variable:
   ```bash
   export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
   ```

### Issue: Module build fails on PC

**Symptoms:**
```bash
pio run
# Error: platformio.ini not found
```

**Solution:**
```bash
# Ensure you're in the firmware directory
cd modules/head-ears/firmware
ls platformio.ini  # Should exist

# If missing, check you're in the right module
pwd  # Should end with /modules/{module}/firmware
```

### Issue: Permission denied when uploading to ESP32

**Solution:**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Logout and login again, or:
newgrp dialout

# Verify
groups  # Should include 'dialout'
```

### Issue: I2C errors on Pi

**Solution:**
```bash
# Check I2C bus
ssh pi@raspberrypi.local "i2cdetect -y 1"
# Should show devices at 0x08, 0x09, 0x0A, 0x0B

# Check Pi user is in i2c group
ssh pi@raspberrypi.local "groups"  # Should include 'i2c'
```

### Issue: Python module not found

**Solution:**
```bash
cd ~/olaf

# Install dependencies
poetry install
# OR
pip3 install -r requirements.txt

# Verify
python3 -c "import olaf_personality"  # Should not error
```

---

## Production Deployment

When ready to deploy full system to Pi:

### 1. Test Full System on Pi

```bash
ssh pi@raspberrypi.local
cd ~/olaf/ros2
source install/setup.bash

# Run everything on Pi
ros2 launch olaf_bringup olaf.launch.py
```

### 2. Verify All Nodes Running

```bash
ros2 node list
# Expected:
# /head_ears_driver
# /neck_driver
# /torso_driver
# /base_driver
# /personality_coordinator
# /emotion_engine
# /ai_agent
# etc.
```

### 3. Create Systemd Service (Auto-Start on Boot)

```bash
# On Pi
sudo nano /etc/systemd/system/olaf.service
```

```ini
[Unit]
Description=OLAF Robot Full System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/olaf/ros2
Environment="ROS_DOMAIN_ID=42"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/pi/olaf/ros2/install/setup.bash && ros2 launch olaf_bringup olaf.launch.py"
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

```bash
# Enable and start service
sudo systemctl enable olaf.service
sudo systemctl start olaf.service

# Check status
sudo systemctl status olaf.service

# View logs
sudo journalctl -u olaf.service -f
```

---

## Workflow Summary

### **Daily Development (PC):**

1. **Morning:** Start Pi driver nodes
   ```bash
   ssh pi@raspberrypi.local
   cd ~/olaf/ros2 && source install/setup.bash
   ros2 launch olaf_bringup drivers.launch.py
   ```

2. **PC Development:**
   ```bash
   # Edit code
   code ros2/src/olaf_personality/

   # Build
   cd ros2 && colcon build --symlink-install

   # Test
   ros2 launch olaf_bringup personality.launch.py

   # Monitor
   rqt_graph
   ```

3. **Commit & Deploy:**
   ```bash
   git commit -am "feat: new emotion behavior"
   git push

   # Sync to Pi
   ./scripts/sync_to_pi.sh
   ```

### **Firmware Development:**

```bash
# Edit module firmware
cd modules/head-ears/firmware
code src/

# Build and upload
pio run --target upload

# Monitor
pio device monitor

# Test
cd ../../tests
pytest test_projector.py
```

### **Integration Testing:**

```bash
# System-wide tests
pytest tests/integration/

# Module tests
cd modules/head-ears
pytest tests/
```

---

## Best Practices

1. **Always source workspace:**
   ```bash
   source ~/olaf/ros2/install/setup.bash
   ```

2. **Use `--symlink-install` for faster iteration:**
   ```bash
   colcon build --symlink-install
   # Python changes don't require rebuild
   ```

3. **Build specific packages only:**
   ```bash
   colcon build --packages-select olaf_personality
   ```

4. **Use RQt for debugging, not terminal spam:**
   ```bash
   rqt_console  # View logs
   rqt_graph    # Visualize connections
   ```

5. **Keep Pi drivers running, iterate on PC**

6. **Test on PC first, deploy to Pi when stable**

---

## Next Steps

1. ✅ Complete Pi setup (install ROS2, flash ESP32 modules)
2. ✅ Install ROS2 on PC (this guide)
3. ✅ Verify network communication (test topic echo)
4. ✅ Start developing application nodes on PC
5. ✅ Test with Pi driver nodes
6. ✅ Create launch files in `ros2/src/olaf_bringup/launch/`
7. ✅ Deploy to Pi when ready

For module-specific guides, see `modules/{module}/README.md`.

For system architecture, see `docs/architecture.md`.

For wiring and assembly, see `hardware/wiring/` and `docs/guides/wiring-guide.md`.
