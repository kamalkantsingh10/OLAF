# Story 0.4: Set Up Development Tools and Dependencies

**Epic:** Epic 0 - ROS2 Foundation Setup
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 30 minutes

---

## User Story

**As a** builder,
**I want** minimal essential development tools installed,
**so that** I can efficiently develop ROS2 nodes and ESP32 firmware with a clean, manageable setup.

---

## Acceptance Criteria

1. ✅ Python 3.10+ is installed and verified
2. ✅ Git is configured with user name and email
3. ✅ Poetry is installed for Python dependency management
4. ✅ All Python dependencies installed via Poetry
5. ✅ VS Code with PlatformIO extension (for ESP32 firmware control)
6. ✅ USB permissions configured for ESP32 flashing

---

## Implementation Steps

### 1. Prerequisites: Python and Git

```bash
# Check Python version (should be 3.10+)
python3 --version

# Install Git if not present
sudo apt update
sudo apt install -y git

# Configure Git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Verify
git config --list | grep user
```

### 2. Install Poetry (Python Dependency Manager)

```bash
# Install Poetry
curl -sSL https://install.python-poetry.org | python3 -

# Add Poetry to PATH
export PATH="$HOME/.local/bin:$PATH"
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

# Verify installation
poetry --version
```

**Why Poetry?** Single source of truth for dependencies, automatic virtual environment management, consistent across team.

### 3. Install Project Dependencies

```bash
cd ~/olaf

# Install all dependencies (production + dev tools)
poetry install

# This installs:
# - smbus2 (I2C communication)
# - anthropic, openai (AI integration)
# - odrive (motor controller)
# - pytest, ruff, black, mypy (dev tools)
# - All other dependencies from pyproject.toml
```

### 4. Install VS Code with Extensions

```bash
# Install VS Code (if not already installed)
# Download from: https://code.visualstudio.com/

# Or on Pi/Ubuntu:
sudo apt install -y code

# Install VS Code extensions (via GUI or CLI):
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension platformio.platformio-ide
code --install-extension ms-iot-vscode.vscode-ros
```

**Important:** We use PlatformIO IDE **extension**, NOT the CLI. This gives you GUI control over ESP32 builds and uploads.

**VS Code Settings for ROS2:**

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
  "python.defaultInterpreterPath": "${workspaceFolder}/.venv/bin/python",
  "ros.distro": "humble",
  "files.associations": {
    "*.repos": "yaml",
    "*.world": "xml",
    "*.xacro": "xml"
  }
}
EOF
```

### 5. Configure USB Permissions for ESP32

```bash
# Add user to dialout group (for serial access)
sudo usermod -a -G dialout $USER

# Create udev rules for ESP32
sudo bash -c 'cat > /etc/udev/rules.d/99-platformio-udev.rules' << 'EOF'
# ESP32 USB-to-Serial (CP2102, CH340, FTDI)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# IMPORTANT: Log out and back in for group changes to take effect
echo "Log out and back in for USB permissions to take effect"
```

### 6. Verify Installation

```bash
cd ~/olaf

# Verify Poetry environment
poetry run python --version

# Test Python imports
poetry run python << 'EOF'
import smbus2
import anthropic
import odrive
import yaml
print("✓ All Python dependencies installed")
EOF

# Verify VS Code extensions
code --list-extensions | grep -E "python|platformio|ros"

# Check USB permissions (after re-login)
groups | grep dialout
```

---

## Testing & Validation

**Test 1: Poetry Environment**
```bash
cd ~/olaf
poetry run python --version  # Should show Python 3.10+
poetry show | head -n 5       # Lists installed packages
```

**Test 2: Python Dependencies**
```bash
poetry run python -c "import smbus2, anthropic, odrive; print('✓ All imports OK')"
```

**Test 3: Git Configuration**
```bash
git config --get user.name
git config --get user.email
```

**Test 4: VS Code Extensions**
```bash
code --list-extensions | grep platformio  # Should show platformio.platformio-ide
```

**Test 5: USB Permissions (after re-login)**
```bash
groups | grep dialout  # Should include 'dialout'
```

**Test 6: ROS2 Build**
```bash
cd ~/olaf/ros2
colcon build  # Should build without errors
```

---

## ESP32 Firmware Development with PlatformIO Extension

### Using PlatformIO IDE (VS Code Extension)

1. **Open module in VS Code:**
   ```bash
   cd ~/olaf/modules/head-ears/firmware
   code .
   ```

2. **PlatformIO will auto-detect `platformio.ini`**

3. **Use the GUI buttons:**
   - Click **Build** (checkmark icon) in bottom toolbar
   - Click **Upload** (arrow icon) to flash ESP32
   - Click **Serial Monitor** (plug icon) to view output

4. **No CLI commands needed!** The extension handles everything.

### Alternative: Quick CLI Access (if needed)

```bash
# If you ever need CLI access, Poetry can add it
poetry add --group dev platformio

# Then use:
poetry run pio run              # Build
poetry run pio run --target upload  # Upload
```

**Recommendation:** Stick with VS Code extension for visual feedback and control.

---

## Development Workflow

### ROS2 Development

```bash
cd ~/olaf/ros2

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Run node
ros2 run head_ears_driver head_ears_driver_node
```

### Running Python Tools

```bash
cd ~/olaf

# Use Poetry to run scripts
poetry run python tools/diagnostics/i2c_scanner.py

# Or activate virtual environment
poetry shell
python tools/diagnostics/i2c_scanner.py
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

## Troubleshooting

**Issue 1: Poetry Install Fails**
- **Solution:** Update pip first:
  ```bash
  python3 -m pip install --upgrade pip
  curl -sSL https://install.python-poetry.org | python3 -
  ```

**Issue 2: Permission Denied on `/dev/ttyUSB0`**
- **Solution:** Ensure you're in dialout group and logged out/in
  ```bash
  sudo usermod -a -G dialout $USER
  # Then log out and back in
  groups | grep dialout  # Verify
  ```

**Issue 3: VS Code Can't Find ROS2 Packages**
- **Solution:** Verify `.vscode/settings.json` has correct Python paths
- Set Poetry interpreter: `Cmd+Shift+P` → "Python: Select Interpreter" → Choose `.venv/bin/python`
- Restart VS Code

**Issue 4: `colcon build` Fails**
- **Solution:** Source ROS2 environment first:
  ```bash
  source /opt/ros/humble/setup.bash
  cd ~/olaf/ros2
  colcon build
  ```

**Issue 5: PlatformIO Extension Not Detecting ESP32**
- **Solution:**
  - Check USB cable is connected
  - Verify USB permissions (Issue 2)
  - Click **PlatformIO** icon in left sidebar → Devices → Should show `/dev/ttyUSB0`

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

- [Poetry Documentation](https://python-poetry.org/docs/)
- [PlatformIO IDE Extension](https://docs.platformio.org/en/latest/integration/ide/vscode.html)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [VS Code ROS Extension](https://marketplace.visualstudio.com/items?itemName=ms-iot-vscode.vscode-ros)

---

## Notes

- **Poetry Advantages:** Single `pyproject.toml` manages all dependencies, automatic virtual environment, lock file for reproducibility
- **VS Code Extension vs CLI:** Extension provides GUI, build status, serial monitor. Easier for beginners. CLI optional via Poetry if needed.
- **Python Version:** Python 3.10+ required (3.11 default on Raspberry Pi OS Bookworm)
- **PlatformIO Toolchain:** First build downloads ESP32 toolchain (~500MB), subsequent builds are fast
- **Dependencies:** All Python deps in `pyproject.toml` - single source of truth

### Minimal Setup Summary

```bash
# Three commands to get started:
1. curl -sSL https://install.python-poetry.org | python3 -  # Install Poetry
2. poetry install                                           # Install all deps
3. code --install-extension platformio.platformio-ide       # ESP32 extension
```

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
