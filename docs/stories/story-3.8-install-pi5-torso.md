# Story 3.8: Install and Configure Raspberry Pi 5 in Torso Module

**Epic:** Epic 3 - Torso Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** to install Raspberry Pi 5 with Hailo AI Kit in the Torso enclosure and configure it as the main compute unit,
**so that** it can run ROS2, process AI workloads, and coordinate all robot modules.

---

## Acceptance Criteria

1. ✅ Raspberry Pi 5 (8GB model recommended) physically mounted in Torso enclosure
2. ✅ Hailo-8L AI Kit (M.2 module) installed and recognized by Pi
3. ✅ Adequate cooling solution installed (heatsink + fan)
4. ✅ MicroSD card (64GB+) flashed with Ubuntu 22.04 for ROS2 Humble
5. ✅ Pi boots successfully and connects to network (Wi-Fi or Ethernet)
6. ✅ ROS2 Humble installed and configured (from Story 0.1)
7. ✅ All I2C connections working: Head+Ears (0x08), Neck (0x0A), Torso (0x09), Base (0x0B)
8. ✅ Hailo drivers installed and AI inference tested
9. ✅ Power distribution configured: Pi receives 5V from Base module

---

## Implementation Steps

### 1. Prepare Raspberry Pi 5 Hardware

**Components:**
- Raspberry Pi 5 (8GB model recommended for AI workloads)
- Hailo-8L AI Kit (M.2 2242 module)
- MicroSD card: 64GB or larger (SanDisk Extreme recommended)
- Active cooler: Official Pi 5 Active Cooler or equivalent
- Power supply: 5V/5A USB-C (27W) - will come from Base module via buck converter

**Install Hailo AI Kit:**
```bash
# 1. Remove M.2 slot cover from Pi 5
# 2. Insert Hailo-8L module into M.2 slot (2242 size)
# 3. Secure with standoff and screw
# 4. Install active cooler on Pi 5 CPU
# 5. Connect fan to Pi 5 fan header (4-pin PWM)
```

### 2. Flash Operating System to MicroSD Card

**On your development computer:**

```bash
# Download Raspberry Pi Imager
# https://www.raspberrypi.com/software/

# Or use dd/balenaEtcher with Ubuntu 22.04 ARM64 image
# https://ubuntu.com/download/raspberry-pi

# Recommended: Use Pi Imager to flash Ubuntu Server 22.04.3 LTS (64-bit)
# Settings during flash:
#   - Set hostname: olaf-pi5
#   - Enable SSH
#   - Configure Wi-Fi (SSID + password)
#   - Set username: olaf, password: [your password]
#   - Set locale/timezone
```

**Alternative manual method:**
```bash
# Download Ubuntu 22.04 for Raspberry Pi
wget https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.3-preinstalled-server-arm64+raspi.img.xz

# Flash to SD card (replace /dev/sdX with your SD card device)
xzcat ubuntu-22.04.3-preinstalled-server-arm64+raspi.img.xz | sudo dd of=/dev/sdX bs=4M status=progress

# Sync and eject
sync
sudo eject /dev/sdX
```

### 3. Initial Pi Boot and Network Setup

**Insert SD card and power on Pi:**

```bash
# Wait 1-2 minutes for first boot (system resizes partitions)

# Find Pi on network
nmap -sn 192.168.1.0/24 | grep -B 2 "Raspberry Pi"
# Or check your router's DHCP client list

# SSH into Pi
ssh olaf@olaf-pi5.local
# Or use IP address: ssh olaf@192.168.1.XXX

# Update system
sudo apt update
sudo apt upgrade -y
```

### 4. Install Hailo Drivers and Dependencies

**On Raspberry Pi 5:**

```bash
# Install Hailo drivers (official package)
wget https://hailo.ai/wp-content/uploads/2023/11/hailort_4.17.0_arm64.deb
sudo dpkg -i hailort_4.17.0_arm64.deb
sudo apt-get install -f

# Verify Hailo device is recognized
hailortcli fw-control identify
# Expected output: Hailo-8L device info

# Install Python Hailo API
pip3 install hailort

# Test inference (download sample model)
wget https://hailo.ai/wp-content/uploads/2023/11/yolov5m.hef
hailortcli run yolov5m.hef

# Expected: Model runs, outputs inference results
```

### 5. Install ROS2 Humble

**Follow Story 0.1 instructions on Pi:**

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop (or Base for headless)
sudo apt update
sudo apt install ros-humble-desktop -y  # Or ros-humble-ros-base for headless

# Source ROS2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify ROS2 installation
ros2 --version
# Expected: ros2 cli version: 0.18.x
```

### 6. Configure I2C Interface

**Enable I2C on Raspberry Pi 5:**

```bash
# Enable I2C interface
sudo raspi-config
# Navigate to: Interfacing Options -> I2C -> Enable

# Or edit config directly
sudo nano /boot/firmware/config.txt
# Add: dtparam=i2c_arm=on

# Reboot
sudo reboot

# After reboot, verify I2C devices
sudo apt install i2c-tools
i2cdetect -y 1

# Expected: Detect devices at addresses 0x08, 0x09, 0x0A, 0x0B
# (Addresses will show once ESP32 modules are connected)
```

### 7. Mount Pi in Torso Enclosure

**Physical mounting:**

```bash
# 1. Position Pi mounting plate inside Torso enclosure
#    - Near top for better airflow
#    - Away from Torso PCB (thermal separation)
#    - Fan facing ventilation holes

# 2. Secure Pi to mounting plate with M2.5 standoffs
#    - Use 10mm standoffs for clearance
#    - Verify fan can spin freely

# 3. Connect cables:
#    - Power: USB-C from Base module (5V/5A buck converter)
#    - I2C: GPIO 2 (SDA), GPIO 3 (SCL) to Torso PCB
#    - HDMI: To projector (via Head+Ears module)
#    - USB: OAK-D-Pro camera (via Head+Ears module)
#    - Ethernet: Optional external connection

# 4. Cable management:
#    - Use zip ties to secure cables to enclosure walls
#    - Avoid blocking ventilation holes
#    - Leave slack for maintenance access
```

### 8. Configure Power Management

**On Raspberry Pi 5:**

```bash
# Set CPU governor for performance (AI workloads)
sudo apt install cpufrequtils
sudo cpufreq-set -g performance

# Make persistent
sudo nano /etc/default/cpufrequtils
# Add: GOVERNOR="performance"

# Configure fan control (optional)
sudo nano /boot/firmware/config.txt
# Add:
dtoverlay=gpio-fan,gpiopin=14,temp=65000

# This enables fan at 65°C

# Disable screen blanking (if using HDMI projector)
sudo nano /etc/default/grub
# Edit: GRUB_CMDLINE_LINUX_DEFAULT="consoleblank=0"
sudo update-grub
```

### 9. Clone OLAF Workspace to Pi

**Transfer OLAF workspace:**

```bash
# On Pi
mkdir -p ~/olaf
cd ~/olaf

# Clone from git (if using version control)
git clone https://github.com/yourusername/olaf.git .

# Or rsync from development machine
# On dev machine:
rsync -avz --exclude 'build' --exclude 'install' --exclude 'log' \
  ~/olaf/ olaf@olaf-pi5.local:~/olaf/

# Build ROS2 workspace on Pi
cd ~/olaf/ros2
colcon build
source install/setup.bash
```

### 10. Test All Module Connections

**Verify I2C communication with all modules:**

```bash
# Scan I2C bus
i2cdetect -y 1

# Expected devices:
# 0x08: Head+Ears module
# 0x09: Torso module (ESP32 on Torso PCB)
# 0x0A: Neck module
# 0x0B: Base module

# Test each module
# Head+Ears
i2cget -y 1 0x08 0x00  # Should return 0x01 (device ID)

# Torso
i2cget -y 1 0x09 0x00  # Should return 0x03 (device ID)

# Neck
i2cget -y 1 0x0A 0x00  # Should return 0x02 (device ID)

# Base
i2cget -y 1 0x0B 0x00  # Should return 0x04 (device ID)
```

**Test ROS2 drivers:**

```bash
# Launch all driver nodes
ros2 run head_ears_driver head_ears_driver_node &
ros2 run torso_driver torso_driver_node &
ros2 run neck_driver neck_driver_node &
ros2 run base_driver base_driver_node &

# Verify all nodes running
ros2 node list

# Test topic communication
ros2 topic pub --once /head_ears/eyes std_msgs/msg/String "{data: 'blink'}"
ros2 topic pub --once /torso/heart std_msgs/msg/String "{data: 'beat:255,0,0:128'}"
```

---

## Testing & Validation

**Test 1: Pi Boot and SSH**
```bash
# Pi should boot within 30 seconds
# SSH should connect: ssh olaf@olaf-pi5.local
```

**Test 2: Hailo AI Kit Recognition**
```bash
hailortcli fw-control identify
# Should show Hailo-8L device info
```

**Test 3: ROS2 Functionality**
```bash
ros2 topic list
# Should show ROS2 topics
```

**Test 4: I2C Communication**
```bash
i2cdetect -y 1
# Should detect all 4 module addresses
```

**Test 5: Thermal Performance**
```bash
# Run stress test
sudo apt install stress
stress --cpu 4 --timeout 60s

# Monitor temperature
watch -n 1 vcgencmd measure_temp

# Temperature should stay below 80°C with active cooling
```

**Test 6: AI Inference Performance**
```bash
# Run Hailo benchmark
hailortcli benchmark yolov5m.hef

# Expected: 50+ FPS for YOLOv5m on Hailo-8L
```

---

## Troubleshooting

**Issue 1: Pi Won't Boot**
- **Solution:** Check SD card is properly seated, verify power supply (5V/5A minimum), try reflashing SD card

**Issue 2: Hailo Not Detected**
- **Solution:** Verify M.2 module fully seated, check `lspci` output, reinstall Hailo drivers

**Issue 3: I2C Devices Not Detected**
- **Solution:** Check I2C enabled in config.txt, verify wiring (SDA/SCL), check pull-up resistors (4.7kΩ)

**Issue 4: Pi Overheating (>85°C)**
- **Solution:** Verify fan connected and spinning, add more ventilation holes to enclosure, reduce CPU load

**Issue 5: ROS2 Nodes Crash**
- **Solution:** Check system resources (`htop`), increase swap size, reduce AI model complexity

**Issue 6: Network Connectivity Issues**
- **Solution:** Check Wi-Fi credentials, verify antenna connected, use Ethernet cable temporarily

---

## Dependencies

**Before this story:**
- Story 3.5: Assemble Torso Enclosure ✅
- Story 3.7: Create Torso ROS2 Driver Node ✅
- Raspberry Pi 5 and Hailo AI Kit purchased

**After this story:**
- Story 3.9: Mount Torso Module to Robot Frame
- All driver nodes can run on Pi

---

## References

- [Raspberry Pi 5 Documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi-5.html)
- [Hailo-8L AI Kit Setup](https://hailo.ai/products/hailo-8l-ai-accelerator-kit/)
- [Ubuntu on Raspberry Pi](https://ubuntu.com/download/raspberry-pi)
- [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

---

## Notes

- **Pi 5 vs Pi 4:** Pi 5 recommended for 2-3x performance improvement, PCIe support for Hailo
- **Memory:** 8GB model recommended for AI workloads + ROS2
- **Storage:** 64GB SD card minimum, 128GB+ recommended for datasets/logs
- **Power:** Pi 5 requires 5V/5A (27W max). Base module must provide adequate power via buck converter
- **Cooling:** Active cooling mandatory for AI workloads (Hailo can generate significant heat)
- **Network:** Wi-Fi for development, Ethernet for production (lower latency)
- **Hailo Performance:** 13 TOPS inference, can run multiple models concurrently
- **Future Enhancements:** Add SSD via USB for faster storage, coral TPU for additional AI acceleration

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
