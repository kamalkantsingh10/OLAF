# Quick Development Setup Guide

**Get OLAF development environment ready in 10 minutes.**

This guide covers the minimal setup needed to start developing ROS2 nodes and ESP32 firmware for OLAF.

---

## 📋 What You'll Install

- **Poetry** - Python dependency manager (one command installs everything)
- **VS Code** - IDE with PlatformIO extension for ESP32
- **USB Permissions** - Serial access for flashing ESP32 modules

**That's it!** Poetry handles all Python packages automatically.

---

## ✅ Prerequisites

Before starting, you need:

- [ ] **Linux system** (Ubuntu 22.04 / Raspberry Pi OS recommended)
- [ ] **Python 3.10+** installed
- [ ] **Git** installed
- [ ] **ROS2 Humble** installed (see [story-0.1-install-ros2-humble.md](../stories/story-0.1-install-ros2-humble.md))

**Quick check:**
```bash
python3 --version  # Should show 3.10 or higher
git --version      # Should show installed
ros2 --version     # Should show Humble
```

---

## 🚀 Step 1: Install Poetry

Poetry manages all Python dependencies from a single `pyproject.toml` file.

```bash
# Install Poetry
curl -sSL https://install.python-poetry.org | python3 -

# Add to PATH
export PATH="$HOME/.local/bin:$PATH"
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

# Verify installation
poetry --version
```

**Expected output:** `Poetry (version 1.x.x)`

💡 **What is Poetry?** Think of it as `npm` for Python - one command to install all project dependencies.

---

## 📦 Step 2: Install Project Dependencies

```bash
# Navigate to OLAF repository
cd ~/olaf

# Install ALL dependencies (this takes 2-3 minutes)
poetry install
```

**What gets installed:**
- ✅ `smbus2` - I2C communication with ESP32 modules
- ✅ `anthropic`, `openai` - AI integration
- ✅ `odrive`, `pyserial` - Motor controller communication
- ✅ `numpy`, `pyyaml` - Data processing & config
- ✅ `pytest`, `ruff`, `black`, `mypy` - Testing & code quality tools

**Verify installation:**
```bash
poetry run python -c "import smbus2, anthropic, odrive; print('✅ All packages installed!')"
```

---

## 💻 Step 3: Install VS Code & Extensions

### Install VS Code

**Option A: Ubuntu/PC**
```bash
# Download and install
sudo snap install code --classic
```

**Option B: Raspberry Pi**
```bash
sudo apt install code
```

**Option C: Already have VS Code?** Skip to extensions.

### Install Required Extensions

Open VS Code and install these 4 extensions:

**Method 1: Via Command Line**
```bash
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension platformio.platformio-ide
code --install-extension ms-iot-vscode.vscode-ros
```

**Method 2: Via VS Code GUI**
1. Open VS Code
2. Click **Extensions** icon (left sidebar, or `Ctrl+Shift+X`)
3. Search and install:
   - `Python` by Microsoft
   - `C/C++` by Microsoft
   - `PlatformIO IDE` by PlatformIO
   - `ROS` by Microsoft

**Verify:**
```bash
code --list-extensions | grep -E "python|platformio|ros"
```

---

## 🔌 Step 4: Configure USB Permissions

This allows you to flash ESP32 modules without `sudo`.

```bash
# Add yourself to dialout group
sudo usermod -a -G dialout $USER

# Create udev rules for ESP32
sudo bash -c 'cat > /etc/udev/rules.d/99-esp32.rules' << 'EOF'
# ESP32 USB-to-Serial Chips
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666"
EOF

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

⚠️ **IMPORTANT:** Log out and back in for group changes to take effect.

**After re-login, verify:**
```bash
groups | grep dialout
```
Should show `dialout` in the list.



**PlatformIO will auto-detect the project.**

**Use the GUI buttons at the bottom of VS Code:**
- ✅ **Build** (checkmark icon)
- ⬆️ **Upload** (arrow icon)
- 🔌 **Serial Monitor** (plug icon)

**No terminal commands needed!**

### ROS2 Development
```bash
cd ~/olaf/ros2

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Run a node
ros2 run head_ears_driver head_ears_driver_node
```

### Code Quality Checks
```bash
cd ~/olaf

# Format code
poetry run black ros2/src/

# Lint code
poetry run ruff ros2/src/

# Type check
poetry run mypy ros2/src/

# Run tests
poetry run pytest
```

---