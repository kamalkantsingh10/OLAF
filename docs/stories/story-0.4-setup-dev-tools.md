# Story 0.4: Set Up Development Tools and Dependencies

**Epic:** Epic 0 - ROS2 Foundation Setup
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 1-2 hours

---

## User Story

**As a** builder,
**I want** all necessary development tools and Python dependencies installed,
**so that** I can efficiently develop and test ROS2 nodes.

---

## Acceptance Criteria

1. ✅ Python 3.11+ is installed and set as default
2. ✅ PlatformIO is installed for ESP32 firmware development: `pip install platformio`
3. ✅ Git is configured with user name and email
4. ✅ Required Python packages installed: `rclpy`, `odrive`, `numpy`
5. ✅ VS Code (or preferred IDE) is installed with ROS2 and PlatformIO extensions
6. ✅ USB permissions configured for OAK-D-Pro camera and ESP32 flashing

---

## Implementation Steps

### 1. Verify Python Version

```bash
# Check Python version
python3 --version

# Should be Python 3.11 or higher
# Raspberry Pi OS 64-bit typically comes with Python 3.11+

# Set python3 as default (if not already)
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 1
```

### 2. Install Essential Build Tools

```bash
# Install build essentials
sudo apt update
sudo apt install -y build-essential cmake git wget curl

# Install Python development headers
sudo apt install -y python3-dev python3-pip python3-venv

# Verify installations
gcc --version
cmake --version
git --version
pip3 --version
```

### 3. Configure Git

```bash
# Set Git user name and email
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Set default editor (optional)
git config --global core.editor "nano"

# Verify configuration
git config --list | grep user

# Expected output:
# user.name=Your Name
# user.email=your.email@example.com
```

### 4. Install PlatformIO for ESP32 Development

```bash
# Install PlatformIO CLI
pip3 install platformio

# Verify installation
pio --version

# Update PlatformIO core
pio upgrade

# Install ESP32 platform (this will be used later for firmware)
pio platform install espressif32

# Verify ESP32 platform installed
pio platform list | grep espressif32
```

### 5. Install Required Python Packages

```bash
# Create virtual environment (optional but recommended)
python3 -m venv ~/olaf_venv
source ~/olaf_venv/bin/activate

# OR install system-wide (simpler for embedded device)
# Install core ROS2 Python dependencies (should already be installed via ROS2)
pip3 install rclpy

# Install OLAF-specific dependencies
pip3 install smbus2      # I2C communication (already done in Story 0.3)
pip3 install odrive      # ODrive motor controller
pip3 install numpy       # Numerical computing (for IMU data processing)
pip3 install pyserial    # Serial communication (for ODrive UART)

# Additional useful packages
pip3 install matplotlib  # Plotting (for PID tuning, sensor visualization)
pip3 install pyyaml      # YAML parsing (for config files)

# Verify installations
python3 << 'EOF'
import smbus2
import odrive
import numpy
import serial
import yaml
print("All packages imported successfully!")
EOF
```

### 6. Install VS Code (Optional but Recommended)

```bash
# Download VS Code for ARM64
wget https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-arm64 -O vscode_arm64.deb

# Install VS Code
sudo apt install ./vscode_arm64.deb

# Clean up
rm vscode_arm64.deb

# Launch VS Code
code
```

**Alternative:** Use lightweight editor like nano, vim, or connect remotely via SSH + VS Code Remote

### 7. Install VS Code Extensions

If using VS Code, install these extensions:

```bash
# Install extensions via CLI
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension platformio.platformio-ide
code --install-extension ms-iot-vscode.vscode-ros

# OR install manually via VS Code Extensions marketplace:
# - Python (Microsoft)
# - C/C++ (Microsoft)
# - PlatformIO IDE (PlatformIO)
# - ROS (Microsoft)
```

**VS Code Settings for ROS2:**

Create `.vscode/settings.json` in your workspace:

```bash
mkdir -p ~/olaf/.vscode
cat > ~/olaf/.vscode/settings.json << 'EOF'
{
  "python.autoComplete.extraPaths": [
    "/opt/ros/humble/lib/python3.11/site-packages",
    "/opt/ros/humble/local/lib/python3.11/dist-packages"
  ],
  "python.analysis.extraPaths": [
    "/opt/ros/humble/lib/python3.11/site-packages",
    "/opt/ros/humble/local/lib/python3.11/dist-packages"
  ],
  "ros.distro": "humble",
  "files.associations": {
    "*.repos": "yaml",
    "*.world": "xml",
    "*.xacro": "xml"
  }
}
EOF
```

### 8. Configure USB Permissions

```bash
# Add user to dialout group (for serial devices like ESP32, ODrive)
sudo usermod -a -G dialout $USER

# Create udev rules for ESP32 (CP2102 USB-to-serial chip)
sudo bash -c 'cat > /etc/udev/rules.d/99-platformio-udev.rules' << 'EOF'
# ESP32 USB-to-Serial (CP2102)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666"

# ESP32 USB-to-Serial (CH340)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666"

# FTDI USB-to-Serial
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666"

# ODrive (if using USB connection)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d32", MODE:="0666"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Log out and back in for group changes to take effect
# OR reboot
sudo reboot
```

### 9. Create Development Environment Aliases

```bash
# Add helpful aliases to .bashrc
cat >> ~/.bashrc << 'EOF'

# OLAF Development Aliases
alias olaf-cd='cd ~/olaf'
alias olaf-ros='cd ~/olaf/ros2'
alias olaf-build='cd ~/olaf/ros2 && colcon build && source install/setup.bash'
alias olaf-clean='cd ~/olaf/ros2 && rm -rf build/ install/ log/ && colcon build'
alias olaf-i2c='python3 ~/olaf/tools/diagnostics/i2c_scanner.py'
alias ros-source='source /opt/ros/humble/setup.bash && source ~/olaf/ros2/install/setup.bash'

# PlatformIO aliases
alias pio-build='pio run'
alias pio-upload='pio run --target upload'
alias pio-monitor='pio device monitor'
EOF

# Reload .bashrc
source ~/.bashrc
```

### 10. Verify Development Environment

```bash
# Test Python
python3 --version

# Test PlatformIO
pio --version

# Test Git
git --version

# Test ROS2
ros2 --version

# Test I2C tools
i2cdetect -V

# Test serial access (should not give permission error)
ls -l /dev/ttyUSB0
# OR (if CP2102 connected):
ls -l /dev/ttyUSB*

# Test Python imports
python3 << 'EOF'
import rclpy
import smbus2
import odrive
import numpy
import serial
print("✓ All Python dependencies OK")
EOF
```

---

## Testing & Validation

**Test 1: Python Environment**
```bash
python3 --version
pip3 list | grep -E "rclpy|smbus2|odrive|numpy|serial"

# All packages should be listed
```

**Test 2: PlatformIO ESP32**
```bash
# Create test ESP32 project
mkdir -p ~/test_esp32
cd ~/test_esp32
pio init --board esp32dev

# Build test project
pio run

# Should compile without errors
```

**Test 3: Git Configuration**
```bash
git config --get user.name
git config --get user.email

# Should return your configured name and email
```

**Test 4: USB Permissions**
```bash
# Check group membership
groups $USER | grep dialout

# Should include 'dialout'

# Check udev rules
cat /etc/udev/rules.d/99-platformio-udev.rules

# Should show ESP32 rules
```

**Test 5: ROS2 Build**
```bash
cd ~/olaf/ros2
colcon build

# Should build without errors
```

---

## Development Workflow Setup

### Recommended Directory Structure

```
~/olaf/
├── .vscode/              # VS Code settings
├── modules/              # Module firmware (ESP32)
│   ├── head-ears/
│   ├── neck/
│   ├── torso/
│   └── base/
├── ros2/                 # ROS2 workspace
│   └── src/
│       ├── olaf_bringup/
│       ├── olaf_description/
│       └── olaf_drivers/
├── tools/                # Development tools
│   └── diagnostics/
├── config/               # Configuration files
│   └── i2c/
└── docs/                 # Documentation
```

### ESP32 Firmware Development Workflow

```bash
# Navigate to module firmware
cd ~/olaf/modules/head-ears/firmware

# Build firmware
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor

# Or combine: build + upload + monitor
pio run --target upload && pio device monitor
```

### ROS2 Development Workflow

```bash
# Navigate to ROS2 workspace
cd ~/olaf/ros2

# Build packages
colcon build

# Build specific package
colcon build --packages-select head_ears_driver

# Source workspace
source install/setup.bash

# Run driver node
ros2 run head_ears_driver head_ears_driver_node
```

---

## Troubleshooting

**Issue 1: PlatformIO Install Fails**
- **Solution:** Ensure pip is up-to-date: `pip3 install --upgrade pip`
- Try installing in user space: `pip3 install --user platformio`

**Issue 2: Permission Denied on `/dev/ttyUSB0`**
- **Solution:** Add user to dialout group: `sudo usermod -a -G dialout $USER`
- Log out and back in

**Issue 3: ODrive Python Package Install Fails**
- **Solution:** Install dependencies first:
  ```bash
  sudo apt install libusb-1.0-0-dev
  pip3 install odrive
  ```

**Issue 4: VS Code Can't Find ROS2 Packages**
- **Solution:** Set Python path in `.vscode/settings.json` (see Step 7)
- Restart VS Code after changes

**Issue 5: `colcon build` Fails with Import Errors**
- **Solution:** Ensure ROS2 environment is sourced: `source /opt/ros/humble/setup.bash`
- Check `setup.py` in packages has correct dependencies

---

## Dependencies

**Before this story:**
- Story 0.1: Install ROS2 Humble ✅
- Story 0.2: Create ROS2 Workspace ✅
- Story 0.3: Configure I2C Tools ✅

**After this story:**
- Epic 1: Head+Ears Module Build (firmware development begins)
- All subsequent epics (full dev environment ready)

---

## References

- [PlatformIO Documentation](https://docs.platformio.org/)
- [ODrive Python Tools](https://docs.odriverobotics.com/v/latest/api/odrive.html)
- [ROS2 Python Client Library (rclpy)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [VS Code ROS Extension](https://marketplace.visualstudio.com/items?itemName=ms-iot-vscode.vscode-ros)

---

## Notes

- **Python Version:** Python 3.11 is default on Raspberry Pi OS (Bookworm)
- **Virtual Environments:** Optional for this project (embedded device, single user)
- **VS Code Performance:** Pi 5 handles VS Code well; consider remote SSH for Pi 4
- **PlatformIO Cache:** First build downloads toolchains (~500MB), subsequent builds are fast
- **ODrive Library:** Only needed when Base module ODrive is connected

### Package Versions Used (Reference)

```
rclpy==3.3.x (from ROS2 Humble)
smbus2==0.4.2
odrive==0.6.x
numpy==1.24.x
pyserial==3.5
```

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
