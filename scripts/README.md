# Scripts

Build automation, deployment, and utility scripts for OLAF development.

## Structure

```
scripts/
├── setup/      # Environment setup
├── build/      # Build automation
├── flash/      # Firmware flashing
├── test/       # Testing scripts
└── deploy/     # OTA deployment
```

## Setup Scripts

**Location:** `scripts/setup/`

Install dependencies and configure the development environment:

```bash
# Install all dependencies (ROS2, PlatformIO, Python packages)
./scripts/setup/install_dependencies.sh

# Configure I2C bus for Raspberry Pi
./scripts/setup/configure_i2c.sh

# Initialize ROS2 workspace
./scripts/setup/setup_ros2_workspace.sh
```

## Build Scripts

**Location:** `scripts/build/`

Build firmware and ROS2 packages:

```bash
# Build all firmware modules
./scripts/build/build_all_firmware.sh

# Build ROS2 workspace
./scripts/build/build_ros2.sh

# Clean all build artifacts
./scripts/build/clean_all.sh
```

## Flash Scripts

**Location:** `scripts/flash/`

Flash firmware to ESP32 modules:

```bash
# Flash individual modules
./scripts/flash/flash_head_ears.sh
./scripts/flash/flash_neck.sh
./scripts/flash/flash_torso.sh
./scripts/flash/flash_base.sh

# Flash all modules sequentially
./scripts/flash/flash_all.sh
```

**Requirements:**
- ESP32 connected via USB
- PlatformIO installed
- Correct USB port permissions (add user to `dialout` group)

## Test Scripts

**Location:** `scripts/test/`

Run system tests:

```bash
# Test I2C module communication
./scripts/test/test_i2c_modules.sh

# Test projector control
./scripts/test/test_projector.sh

# Run all integration tests
./scripts/test/run_all_tests.sh
```

## Deploy Scripts

**Location:** `scripts/deploy/`

Deploy OTA firmware updates:

```bash
# Deploy firmware update to specific module
./scripts/deploy/deploy_ota_update.sh head-ears

# Deploy to all modules
./scripts/deploy/deploy_ota_update.sh all
```

## Usage Examples

### First-Time Setup

```bash
# 1. Install dependencies
./scripts/setup/install_dependencies.sh

# 2. Build everything
./scripts/build/build_all_firmware.sh
./scripts/build/build_ros2.sh

# 3. Flash firmware to modules
./scripts/flash/flash_all.sh
```

### Daily Development

```bash
# Build and flash specific module
cd firmware/modules/head-ears
pio run --target upload

# Or use script
./scripts/flash/flash_head_ears.sh

# Test changes
./scripts/test/test_i2c_modules.sh
```

### Continuous Integration

All scripts are designed to work in CI pipelines:

```yaml
# Example GitHub Actions workflow
- name: Build Firmware
  run: ./scripts/build/build_all_firmware.sh

- name: Run Tests
  run: ./scripts/test/run_all_tests.sh
```

## Script Conventions

- **Exit codes:** 0 = success, non-zero = failure
- **Logging:** Scripts output to stdout/stderr, use `set -x` for debugging
- **Error handling:** Use `set -e` to exit on first error
- **Parameters:** Scripts accept parameters where applicable (e.g., module name)

## Troubleshooting

### Flash Fails: "Permission Denied"

Add user to dialout group:

```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

### ROS2 Build Fails: Package Not Found

Source ROS2 setup:

```bash
source /opt/ros/humble/setup.bash
```

### I2C Test Fails: Device Not Found

Check I2C bus configuration:

```bash
i2cdetect -y 1
```

Should show devices at 0x08, 0x09, 0x0A, 0x0B.

## Contributing

When adding new scripts:

1. Place in appropriate category folder
2. Make executable: `chmod +x script.sh`
3. Add shebang: `#!/bin/bash`
4. Include usage help: `./script.sh --help`
5. Document in this README
