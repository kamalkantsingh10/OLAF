# Story 0.2: Create ROS2 Workspace with Module-First Structure

**Epic:** Epic 0 - ROS2 Foundation Setup
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 1-2 hours

---

## User Story

**As a** builder,
**I want** a ROS2 workspace structured according to the module-first architecture,
**so that** each module's ROS2 driver node is organized and discoverable.

---

## Acceptance Criteria

1. ✅ Workspace directory created at `~/olaf/ros2/` with `src/` subdirectory
2. ✅ Following packages created under `src/`: `olaf_bringup`, `olaf_description`, `olaf_drivers`
3. ✅ Under `olaf_drivers/`, create placeholder packages: `head_ears_driver`, `neck_driver`, `torso_driver`, `base_driver`
4. ✅ Each driver package has proper `package.xml` and `setup.py` (Python) or `CMakeLists.txt` (C++)
5. ✅ Workspace builds successfully: `colcon build` completes without errors
6. ✅ Workspace can be sourced: `source install/setup.bash` works

---

## Implementation Steps

### 1. Create Workspace Directory Structure

```bash
# Navigate to home directory
cd ~

# Create ROS2 workspace matching project structure
mkdir -p olaf/ros2/src
cd olaf/ros2
```

### 2. Create Core Packages

```bash
cd ~/olaf/ros2/src

# Create bringup package (launch files, configs)
ros2 pkg create olaf_bringup --build-type ament_python --dependencies rclpy

# Create description package (URDF, robot models)
ros2 pkg create olaf_description --build-type ament_cmake

# Create drivers parent directory
mkdir -p olaf_drivers
```

### 3. Create Module Driver Packages

```bash
cd ~/olaf/ros2/src/olaf_drivers

# Create Head+Ears driver package
ros2 pkg create head_ears_driver --build-type ament_python --dependencies rclpy std_msgs

# Create Neck driver package
ros2 pkg create neck_driver --build-type ament_python --dependencies rclpy std_msgs

# Create Torso driver package
ros2 pkg create torso_driver --build-type ament_python --dependencies rclpy std_msgs

# Create Base driver package
ros2 pkg create base_driver --build-type ament_python --dependencies rclpy std_msgs geometry_msgs
```

### 4. Verify Package Structure

```bash
cd ~/olaf/ros2/src
tree -L 3

# Expected structure:
# .
# ├── olaf_bringup/
# │   ├── package.xml
# │   ├── setup.py
# │   ├── setup.cfg
# │   └── olaf_bringup/
# │       └── __init__.py
# ├── olaf_description/
# │   ├── package.xml
# │   └── CMakeLists.txt
# └── olaf_drivers/
#     ├── head_ears_driver/
#     │   ├── package.xml
#     │   ├── setup.py
#     │   └── head_ears_driver/
#     ├── neck_driver/
#     ├── torso_driver/
#     └── base_driver/
```

### 5. Build Workspace

```bash
cd ~/olaf/ros2

# Install colcon (if not already installed)
sudo apt install python3-colcon-common-extensions

# Build all packages
colcon build

# Expected output:
# Starting >>> olaf_bringup
# Starting >>> olaf_description
# Starting >>> head_ears_driver
# Starting >>> neck_driver
# Starting >>> torso_driver
# Starting >>> base_driver
# Finished <<< [packages] [time]
```

### 6. Source Workspace

```bash
# Source the workspace (temporary, current session)
source ~/olaf/ros2/install/setup.bash

# Add to .bashrc for automatic sourcing (optional, recommended later)
# echo "source ~/olaf/ros2/install/setup.bash" >> ~/.bashrc
```

**Note:** Don't add workspace sourcing to `.bashrc` yet—do this after Phase 1 is complete to avoid issues during development.

### 7. Verify Packages Are Discoverable

```bash
# List available packages
ros2 pkg list | grep olaf

# Expected output:
# base_driver
# head_ears_driver
# neck_driver
# olaf_bringup
# olaf_description
# torso_driver

# Check package paths
ros2 pkg prefix olaf_bringup
ros2 pkg prefix head_ears_driver
```

---

## Testing & Validation

**Test 1: Workspace Structure**
```bash
cd ~/olaf/ros2
ls -la

# Should show:
# build/
# install/
# log/
# src/
```

**Test 2: Package Discovery**
```bash
source ~/olaf/ros2/install/setup.bash
ros2 pkg list | grep olaf | wc -l

# Expected: 6 packages (olaf_bringup, olaf_description, 4 drivers)
```

**Test 3: Build Clean**
```bash
cd ~/olaf/ros2
rm -rf build/ install/ log/
colcon build

# Should complete without errors
```

**Test 4: Package Executables (after adding nodes)**
```bash
# This will work after driver nodes are implemented
ros2 run head_ears_driver head_ears_driver_node
```

---

## Package Details

### olaf_bringup
**Purpose:** Launch files, system-wide configurations, demo scripts

**Key Files:**
- `launch/` - ROS2 launch files (Python)
- `config/` - YAML configuration files
- `scripts/` - Demo and utility scripts

### olaf_description
**Purpose:** Robot model descriptions, URDF, visualization

**Key Files:**
- `urdf/` - URDF/xacro robot models
- `meshes/` - 3D models for visualization
- `rviz/` - RViz configuration files

### olaf_drivers/[module]_driver
**Purpose:** ROS2 driver nodes for each hardware module

**Key Files:**
- `[module]_driver/[module]_driver_node.py` - Main driver node
- `launch/` - Module-specific launch files
- `config/` - Module-specific configs

---

## Troubleshooting

**Issue 1: `colcon: command not found`**
- **Solution:** Install colcon: `sudo apt install python3-colcon-common-extensions`

**Issue 2: Build Fails with Python Import Errors**
- **Solution:** Ensure `setup.py` has correct package name and entry points

**Issue 3: Packages Not Discoverable After Build**
- **Solution:** Source workspace: `source ~/olaf/ros2/install/setup.bash`

**Issue 4: Permission Denied on Build**
- **Solution:** Check workspace ownership: `ls -la ~/olaf/ros2`
  - Fix if needed: `sudo chown -R $USER:$USER ~/olaf/ros2`

**Issue 5: Conflicting Package Names**
- **Solution:** ROS2 package names must be unique system-wide. Prefix with `olaf_` to avoid conflicts.

---

## Dependencies

**Before this story:**
- Story 0.1: Install ROS2 Humble on Raspberry Pi ✅

**After this story:**
- Story 0.3: Configure I2C Communication Tools
- Epic 1+: Module firmware and driver node implementation

---

## References

- [ROS2 Workspace Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [Creating a Package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Colcon Build System](https://colcon.readthedocs.io/)

---

## Notes

- **Build Time:** Initial build is fast (~30 seconds) since packages are empty
- **Python vs C++:** Using Python for driver nodes (easier I2C integration, faster development)
- **Workspace Overlay:** This workspace overlays `/opt/ros/humble/` (underlay)
- **Git Integration:** Add `build/`, `install/`, `log/` to `.gitignore`

### Add to `.gitignore`

```bash
# Add ROS2 build artifacts to gitignore
echo "ros2/build/" >> ~/olaf/.gitignore
echo "ros2/install/" >> ~/olaf/.gitignore
echo "ros2/log/" >> ~/olaf/.gitignore
```

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
