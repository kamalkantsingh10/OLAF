# PC Development Setup for OLAF

This guide explains how to set up your PC for OLAF development using the hybrid PC+Pi architecture. With this setup, you can develop and test application nodes on your PC while the Raspberry Pi handles hardware driver nodes.

---

## Overview: Hybrid PC+Pi Development Architecture

```
┌─────────────────────────────────────┐
│  YOUR PC (Development)              │
│  ┌───────────────────────────────┐  │
│  │ ROS2 Application Nodes        │  │
│  │ - personality_coordinator     │  │
│  │ - ai_agent_node              │  │
│  │ - Development tools (rqt)    │  │
│  └───────────────────────────────┘  │
│           ↓ ROS2 DDS (WiFi)         │
└─────────────────────────────────────┘
            ↓
┌─────────────────────────────────────┐
│  RASPBERRY PI (Hardware Control)    │
│  ┌───────────────────────────────┐  │
│  │ ROS2 Driver Nodes ONLY        │  │
│  │ - head_driver_node            │  │
│  │ - ears_neck_driver_node       │  │
│  │ - body_driver_node            │  │
│  │ - base_driver_node            │  │
│  └───────────────────────────────┘  │
│           ↓ I2C (Wired)             │
└─────────────────────────────────────┘
            ↓
    ┌───────────────┐
    │ ESP32 Modules │
    │ (I2C Slaves)  │
    └───────────────┘
```

**Benefits:**
- Use your PC's power and development tools
- Keep your existing automation workflows
- Test without deploying to Pi every time
- Easy migration to production (same code, different machine)

---

## Prerequisites

### Hardware Requirements
- PC running Ubuntu 22.04 LTS (native or VM) **OR** Windows/Mac with Docker
- Raspberry Pi 5 with OLAF setup complete (Story 1.3)
- Both PC and Pi on the same WiFi network

### Software Requirements
- Ubuntu 22.04 LTS (for native ROS2 Humble support)
- OR Docker Desktop (for Windows/Mac users)

---

## Installation Options

Choose one of the following based on your platform:

### Option A: Ubuntu 22.04 (Native) - Recommended

#### 1. Install ROS2 Humble on PC

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

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep
sudo rosdep init
rosdep update
```

#### 2. Configure ROS2 Environment

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Source immediately
source ~/.bashrc
```

**IMPORTANT:** The `ROS_DOMAIN_ID` must match your Raspberry Pi (default: 42).

#### 3. Install Python Dependencies

```bash
cd ~/olaf
pip3 install -r orchestrator/requirements.txt
```

---

### Option B: Docker (Windows/Mac/Linux)

#### 1. Install Docker Desktop

Download and install from: https://www.docker.com/products/docker-desktop

#### 2. Create ROS2 Humble Docker Image

Create `Dockerfile` in your OLAF repository:

```dockerfile
FROM ros:humble

# Install development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-rqt* \
    && rm -rf /var/lib/apt/lists/*

# Set ROS2 domain ID (must match Pi)
ENV ROS_DOMAIN_ID=42

# Set working directory
WORKDIR /workspace

# Source ROS2 in every bash session
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
```

#### 3. Build and Run Docker Container

```bash
# Build image
docker build -t olaf-ros2-dev .

# Run container (Linux)
docker run -it --rm \
  --network host \
  -v $(pwd):/workspace \
  olaf-ros2-dev

# Run container (Windows/Mac - requires explicit port mapping)
docker run -it --rm \
  -p 11311:11311 \
  -v $(pwd):/workspace \
  olaf-ros2-dev
```

---

## Network Configuration

### 1. Set ROS2 Domain ID (Both PC and Pi)

The `ROS_DOMAIN_ID` isolates your robot from other ROS2 systems on the network.

**On PC:**
```bash
# Add to ~/.bashrc (Ubuntu) or set in Docker
export ROS_DOMAIN_ID=42
```

**On Pi:**
```bash
# Should already be set by install_ros2_humble_pi.sh
echo $ROS_DOMAIN_ID  # Verify it shows 42
```

### 2. Verify Network Connectivity

**Check if PC can see Pi's ROS2 topics:**

```bash
# On Pi: Start driver nodes
ros2 launch olaf_orchestrator drivers_only.launch.py

# On PC: List topics (should see /olaf/* topics)
ros2 topic list
```

Expected output:
```
/olaf/head/expression
/olaf/ears_neck/gesture
/olaf/body/heart_rate
/olaf/base/velocity
/parameter_events
/rosout
```

If you don't see topics, check troubleshooting section below.

---

## Development Workflow

### 1. Clone Repository on PC

```bash
cd ~
git clone <your-olaf-repo-url> olaf
cd olaf
```

### 2. Start Driver Nodes on Pi

**SSH into Pi:**
```bash
ssh pi@raspberrypi.local
# OR
ssh pi@<pi-ip-address>
```

**Start driver nodes:**
```bash
cd ~/olaf
ros2 launch olaf_orchestrator drivers_only.launch.py
```

**Leave this running** - Pi is now the hardware bridge.

### 3. Develop on PC

**On PC, run application nodes:**

```bash
cd ~/olaf

# Run personality coordinator (example)
ros2 launch olaf_orchestrator app_nodes.launch.py
```

### 4. Test Cross-Machine Communication

**Publish from PC, receive on Pi:**

```bash
# On PC: Publish test message
ros2 topic pub /olaf/test std_msgs/String "data: 'hello from PC'"

# On Pi: Echo topic
ros2 topic echo /olaf/test
# Should see: data: 'hello from PC'
```

### 5. Develop-Test-Deploy Cycle

```bash
# 1. Edit code on PC
vim orchestrator/ros2_nodes/personality/personality_coordinator_node.py

# 2. Test on PC (with Pi drivers running)
ros2 run olaf_orchestrator personality_coordinator_node

# 3. Commit changes
git add .
git commit -m "feat: update personality coordinator"
git push origin main

# 4. Deploy to Pi
ssh pi@raspberrypi.local
cd ~/olaf
git pull origin main

# 5. Run full system on Pi (production)
ros2 launch olaf_orchestrator olaf_full.launch.py
```

---

## ESP32 Firmware Development on PC

You can also compile and upload ESP32 firmware from your PC:

### 1. Install PlatformIO

```bash
# Install PlatformIO Core
pip3 install platformio
```

### 2. Compile Firmware

```bash
cd modules/head
pio run
```

### 3. Upload to ESP32 (USB Connected to PC)

```bash
# Connect ESP32 via USB to PC
pio run -t upload
```

### 4. Deploy Compiled Binaries to Pi (for OTA)

```bash
# Copy .bin files to Pi's OTA server
scp .pio/build/esp32/firmware.bin pi@raspberrypi.local:~/olaf/orchestrator/ota_server/firmware_binaries/
```

---

## Using Your Existing PC Automation

You can integrate OLAF development into your existing PC workflows:

### Git Hooks

```bash
# .git/hooks/pre-commit (on PC)
#!/bin/bash
# Run linters before commit
ruff orchestrator/
black orchestrator/ --check
pytest tests/unit/
```

### VS Code Remote SSH

1. Install "Remote - SSH" extension
2. Connect to Pi: `ssh pi@raspberrypi.local`
3. Edit files on Pi directly from PC VS Code

### rsync for Fast Sync

```bash
# Sync code from PC to Pi (faster than git for iteration)
rsync -avz --exclude='build/' --exclude='.git/' \
  ~/olaf/ pi@raspberrypi.local:~/olaf/
```

---

## Monitoring and Debugging Tools

### RQT Tools (GUI - run on PC)

```bash
# ROS2 graph visualizer
rqt_graph

# Topic monitor
rqt_topic

# Service caller
rqt_service_caller

# All-in-one dashboard
rqt
```

### Command-Line Tools

```bash
# List all topics
ros2 topic list

# Echo topic data
ros2 topic echo /olaf/head/expression

# Show topic info
ros2 topic info /olaf/head/expression

# Publish to topic
ros2 topic pub /olaf/head/expression olaf_interfaces/Expression "{emotion_type: 1, intensity: 3, duration_ms: 2000}"

# List all nodes
ros2 node list

# Show node info
ros2 node info /olaf/personality_coordinator
```

---

## Troubleshooting

### Issue: PC cannot see Pi's ROS2 topics

**Symptoms:**
```bash
ros2 topic list  # Only shows /parameter_events and /rosout
```

**Possible Causes & Solutions:**

1. **ROS_DOMAIN_ID mismatch**
   ```bash
   # On PC
   echo $ROS_DOMAIN_ID  # Should be 42

   # On Pi
   echo $ROS_DOMAIN_ID  # Should also be 42
   ```

2. **Firewall blocking DDS multicast**
   ```bash
   # On Ubuntu PC, allow ROS2 DDS ports
   sudo ufw allow 7400:7500/udp
   sudo ufw allow 7400:7500/tcp
   ```

3. **Network not allowing multicast**
   - Some corporate/guest WiFi networks block multicast
   - Solution: Use same local network or configure static DDS peers

4. **Docker network mode (Windows/Mac)**
   - `--network host` only works on Linux
   - Windows/Mac requires explicit port mapping or DDS peer configuration

### Issue: Permission denied on Pi I2C

**Solution:** Already handled by setup script, but verify:
```bash
# On Pi
groups  # Should include 'i2c'
ls -l /dev/i2c-1  # Should show 'crw-rw---- 1 root i2c'
```

### Issue: Module not found errors on PC

**Solution:** Install Python dependencies:
```bash
cd ~/olaf
pip3 install -r orchestrator/requirements.txt
```

---

## Production Deployment

When ready to deploy to Pi for production:

### 1. Test Full System on Pi

```bash
# On Pi, run everything
ros2 launch olaf_orchestrator olaf_full.launch.py
```

### 2. Verify All Nodes Running

```bash
ros2 node list
# Expected:
# /olaf/head_driver
# /olaf/ears_neck_driver
# /olaf/body_driver
# /olaf/base_driver
# /olaf/personality_coordinator
# /olaf/ai_agent
# etc.
```

### 3. Create Systemd Service (Auto-Start on Boot)

```bash
# Create service file
sudo nano /etc/systemd/system/olaf.service
```

```ini
[Unit]
Description=OLAF Robot Full System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/olaf
Environment="ROS_DOMAIN_ID=42"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && ros2 launch olaf_orchestrator olaf_full.launch.py"
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
# Enable and start service
sudo systemctl enable olaf.service
sudo systemctl start olaf.service

# Check status
sudo systemctl status olaf.service
```

---

## Summary

**PC Development:**
- Install ROS2 Humble (Ubuntu native or Docker)
- Set `ROS_DOMAIN_ID=42`
- Run application nodes: `ros2 launch olaf_orchestrator app_nodes.launch.py`
- Use your existing PC automation and tools

**Pi Hardware Control:**
- Run setup script: `./tools/setup/install_ros2_humble_pi.sh`
- Run driver nodes: `ros2 launch olaf_orchestrator drivers_only.launch.py`
- Provides I2C bridge to ESP32 modules

**Production:**
- Deploy to Pi: `git pull origin main`
- Run full system: `ros2 launch olaf_orchestrator olaf_full.launch.py`
- Same code, no changes needed!

---

## Next Steps

1. Complete Pi setup (Story 1.3)
2. Install ROS2 on PC (this guide)
3. Verify network communication (test topic echo)
4. Start developing application nodes on PC
5. Test with Pi driver nodes
6. Deploy to Pi when ready

For questions or issues, see `tools/setup/README.md` or check the troubleshooting section above.
