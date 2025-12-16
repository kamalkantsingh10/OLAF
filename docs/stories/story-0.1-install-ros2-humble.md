# Story 0.1: Install ROS2 Humble on Raspberry Pi

**Epic:** Epic 0 - ROS2 Foundation Setup
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 2-3 hours

---

## User Story

**As a** builder,
**I want** ROS2 Humble installed on Raspberry Pi 5 with all dependencies,
**so that** I can develop ROS2 driver nodes for OLAF modules.

---

## Acceptance Criteria

1. ✅ Raspberry Pi OS (Debian 12 Bookworm, 64-bit) is installed and updated
2. ✅ ROS2 Humble Hawksbill is installed following official Pi installation guide
3. ✅ Core ROS2 packages are verified (`ros-humble-desktop` or `ros-humble-base`)
4. ✅ `rosdep` is initialized and dependencies are installed
5. ✅ ROS2 environment is sourced in `.bashrc` for automatic activation
6. ✅ Basic ROS2 commands work: `ros2 topic list`, `ros2 node list`

---

## Implementation Steps

### 1. Prepare Raspberry Pi OS

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Verify OS version
lsb_release -a
# Should show: Debian 12 (bookworm), 64-bit
```

### 2. Set Locale

```bash
# Ensure UTF-8 locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify
locale
```

### 3. Add ROS2 APT Repository

```bash
# Add ROS2 GPG key
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**Note:** For Raspberry Pi OS (Debian-based), you may need to use the Ubuntu focal repository. Check official ROS2 documentation for latest instructions.

### 4. Install ROS2 Humble

```bash
# Update apt cache
sudo apt update

# Install ROS2 Humble base (smaller footprint, recommended for embedded)
sudo apt install ros-humble-ros-base

# OR install ROS2 Humble desktop (includes visualization tools, GUI)
# sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools
```

### 5. Initialize rosdep

```bash
# Initialize rosdep
sudo rosdep init
rosdep update
```

### 6. Source ROS2 Environment

```bash
# Source setup file (temporary, for current session)
source /opt/ros/humble/setup.bash

# Add to .bashrc for automatic sourcing on login
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 7. Verify Installation

```bash
# Check ROS2 environment variables
printenv | grep ROS

# Verify ROS2 commands work
ros2 --help
ros2 topic list
ros2 node list

# Run demo (optional)
ros2 run demo_nodes_cpp talker
# In another terminal:
ros2 run demo_nodes_cpp listener
```

---

## Testing & Validation

**Test 1: ROS2 Environment Variables**
```bash
printenv | grep ROS_DISTRO
# Expected output: ROS_DISTRO=humble
```

**Test 2: ROS2 CLI Commands**
```bash
ros2 topic list
# Should return list of topics (even if empty)

ros2 node list
# Should return list of nodes (even if empty)
```

**Test 3: Demo Nodes**
```bash
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2 (SSH or separate session)
ros2 run demo_nodes_cpp listener

# Verify: listener should print messages from talker
```

---

## Troubleshooting

**Issue 1: GPG Key Errors**
- **Symptom:** `GPG error: ... NO_PUBKEY`
- **Solution:** Re-add ROS2 GPG key with correct permissions
  ```bash
  sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  sudo chmod 644 /usr/share/keyrings/ros-archive-keyring.gpg
  ```

**Issue 2: Repository Not Found**
- **Symptom:** `Failed to fetch http://packages.ros.org/ros2/ubuntu`
- **Solution:** Raspberry Pi OS uses Debian, not Ubuntu. Check ROS2 docs for Debian-specific instructions or use Ubuntu ARM image

**Issue 3: `rosdep init` Permission Denied**
- **Symptom:** `ERROR: cannot download default sources list`
- **Solution:** Run with sudo: `sudo rosdep init`

**Issue 4: ROS2 Commands Not Found After Install**
- **Symptom:** `bash: ros2: command not found`
- **Solution:** Source setup file: `source /opt/ros/humble/setup.bash`

---

## Dependencies

**Before this story:**
- Raspberry Pi 5 with Raspberry Pi OS (64-bit) installed
- Internet connection for downloading packages
- SD card with at least 16GB (32GB recommended)

**After this story:**
- Story 0.2: Create ROS2 Workspace with Module-First Structure

---

## References

- [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [ROS2 on Raspberry Pi](https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

---

## Notes

- **Hardware:** Raspberry Pi 5 16GB is recommended for OLAF (ROS2 + Hailo AI Kit)
- **Installation Size:** ROS2 Humble base (~500MB), desktop (~2GB)
- **Performance:** Pi 5 handles ROS2 well; Pi 4 8GB is minimum for this project
- **Alternative:** Consider using pre-built ROS2 Docker images for faster setup

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
