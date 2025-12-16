# Epic 0: ROS2 Foundation Setup - Story Index

**Epic Goal:** Establish the ROS2 Humble development environment on Raspberry Pi 5 with proper workspace structure, I2C tools, and module discovery capability, enabling incremental testing as each module is built.

**Status:** Not Started
**Total Stories:** 4
**Estimated Effort:** 5-8 hours

---

## Stories

### ✅ [Story 0.1: Install ROS2 Humble on Raspberry Pi](story-0.1-install-ros2-humble.md)
**Priority:** High | **Effort:** 2-3 hours
- Install Raspberry Pi OS (Debian 12 Bookworm, 64-bit)
- Install ROS2 Humble Hawksbill
- Configure rosdep and verify installation
- **Outcome:** ROS2 Humble operational on Raspberry Pi 5

### ✅ [Story 0.2: Create ROS2 Workspace with Module-First Structure](story-0.2-create-ros2-workspace.md)
**Priority:** High | **Effort:** 1-2 hours
- Create workspace at `~/olaf/ros2/src`
- Create packages: `olaf_bringup`, `olaf_description`, `olaf_drivers`
- Create 4 driver packages (head_ears, neck, torso, base)
- Build workspace and verify packages discoverable
- **Outcome:** ROS2 workspace structured for OLAF modules

### ✅ [Story 0.3: Configure I2C Communication Tools](story-0.3-configure-i2c.md)
**Priority:** High | **Effort:** 1 hour
- Enable I2C interface on Raspberry Pi
- Install i2c-tools and smbus2 Python library
- Create I2C scanner diagnostic script
- Document module addresses (0x08-0x0B)
- **Outcome:** I2C ready for module communication testing

### ✅ [Story 0.4: Set Up Development Tools and Dependencies](story-0.4-setup-dev-tools.md)
**Priority:** High | **Effort:** 1-2 hours
- Verify Python 3.11+ and configure Git
- Install PlatformIO for ESP32 firmware development
- Install Python dependencies (odrive, numpy, pyserial)
- Configure USB permissions for ESP32 flashing
- (Optional) Install VS Code with ROS2/PlatformIO extensions
- **Outcome:** Complete development environment for firmware + ROS2

---

## Epic Completion Criteria

- [x] ROS2 Humble installed and functional
- [x] ROS2 workspace created with module-first structure
- [x] All 6 packages build successfully (olaf_bringup, olaf_description, 4 drivers)
- [x] I2C interface enabled and testable
- [x] I2C scanner script works
- [x] PlatformIO installed for ESP32 development
- [x] All Python dependencies installed
- [x] USB permissions configured for ESP32 flashing
- [x] Development aliases and workflow documented

---

## Prerequisites

- Raspberry Pi 5 16GB (or Pi 4 8GB minimum)
- Raspberry Pi OS 64-bit (Debian 12 Bookworm) installed
- Internet connection for package downloads
- SD card: 32GB+ recommended
- (Optional) USB keyboard, mouse, monitor for initial setup

---

## Next Steps After Epic 0

Once Epic 0 is complete, you'll be ready to start building modules:

**Recommended Path:**
1. **Epic 1: Head+Ears Module Build** - Most visible progress (eyes blinking!)
2. **Epic 4: Base Module Build** - Power hub enables other modules
3. **Epic 2: Neck Module Build** - Adds expressive movement
4. **Epic 3: Torso Module Build** - Integrates Pi and heart display
5. **Epic 5: End-to-End Demo** - Validate full system

**Alternative Path (Power First):**
1. **Epic 4: Base Module Build** - Get power distribution working
2. **Epic 1: Head+Ears Module Build** - Most personality
3. **Epic 2/3** - Neck/Torso in any order
4. **Epic 5: End-to-End Demo**

---

## Useful Commands Reference

```bash
# ROS2
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 node list

# Workspace
cd ~/olaf/ros2
colcon build
source install/setup.bash

# I2C
i2cdetect -y 1
python3 ~/olaf/tools/diagnostics/i2c_scanner.py

# PlatformIO (ESP32)
pio run                    # Build
pio run --target upload    # Flash
pio device monitor         # Serial monitor

# Git
git status
git add .
git commit -m "message"
git push
```

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
