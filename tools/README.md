# Tools

Development and debugging utilities for OLAF.

## Structure

```
tools/
├── diagnostics/    # System diagnostics
├── calibration/    # Sensor/servo calibration
├── simulators/     # Hardware-in-the-loop simulators
└── utils/          # General utilities
```

## Diagnostics

**Location:** `tools/diagnostics/`

System health checks and debugging:

### i2c_scanner.py

Scan I2C bus for connected modules:

```bash
python3 tools/diagnostics/i2c_scanner.py
```

Output:
```
Found devices at: 0x08 (Head+Ears), 0x09 (Neck), 0x0A (Torso), 0x0B (Base)
```

### module_health_check.py

Check health status of all modules:

```bash
python3 tools/diagnostics/module_health_check.py
```

Reports:
- Firmware version
- Uptime
- Error counts
- Temperature
- I2C communication status

### odrive_diagnostic.py

ODrive motor controller diagnostics:

```bash
python3 tools/diagnostics/odrive_diagnostic.py
```

Checks:
- Motor calibration status
- Encoder alignment
- Current limits
- Bus voltage

## Calibration

**Location:** `tools/calibration/`

Calibrate sensors and actuators:

### servo_calibrator.py

Calibrate servo ranges and positions:

```bash
# Interactive calibration
python3 tools/calibration/servo_calibrator.py --module head-ears

# Auto-calibrate with defaults
python3 tools/calibration/servo_calibrator.py --module neck --auto
```

Saves calibration to `config/firmware/{module}_servo_cal.json`

### imu_calibrator.py

Calibrate BNO085 IMU (9-axis AHRS with on-chip sensor fusion):

```bash
python3 tools/calibration/imu_calibrator.py
```

BNO085 calibration procedure:
1. Place robot stationary on flat surface for ~30 seconds (accel/gyro auto-calibrate)
2. Slowly rotate robot in figure-8 pattern for magnetometer calibration
3. Script saves calibration data to ESP32 flash via `sh2_saveDcdNow()`
4. Tare sets current upright orientation as pitch=0° reference

### camera_calibrator.py

Calibrate RGBD camera intrinsics:

```bash
python3 tools/calibration/camera_calibrator.py --checkerboard 9x6 --size 0.025
```

Uses checkerboard pattern for calibration.

## Simulators

**Location:** `tools/simulators/`

Test without physical hardware:

### i2c_module_simulator.py

Simulate ESP32 modules for development:

```bash
# Simulate all modules
python3 tools/simulators/i2c_module_simulator.py

# Simulate specific module
python3 tools/simulators/i2c_module_simulator.py --module head-ears
```

Emulates I2C slave behavior, useful for:
- ROS2 driver development without hardware
- Integration testing
- CI/CD pipelines

## Utilities

**Location:** `tools/utils/`

General-purpose utilities:

### log_analyzer.py

Analyze ROS2 and firmware logs:

```bash
python3 tools/utils/log_analyzer.py logs/olaf_2024-01-15.log
```

Extracts:
- Error patterns
- Performance metrics
- Event timelines

## Usage Examples

### Initial Setup

```bash
# 1. Scan for modules
python3 tools/diagnostics/i2c_scanner.py

# 2. Check module health
python3 tools/diagnostics/module_health_check.py

# 3. Calibrate servos
python3 tools/calibration/servo_calibrator.py --module head-ears
python3 tools/calibration/servo_calibrator.py --module neck

# 4. Calibrate IMU (for base module)
python3 tools/calibration/imu_calibrator.py
```

### Development Without Hardware

```bash
# Start I2C simulator
python3 tools/simulators/i2c_module_simulator.py &

# Launch ROS2 drivers (will connect to simulator)
ros2 launch olaf_bringup drivers.launch.py
```

### Troubleshooting

```bash
# Module not responding?
python3 tools/diagnostics/i2c_scanner.py
python3 tools/diagnostics/module_health_check.py

# Motors behaving oddly?
python3 tools/diagnostics/odrive_diagnostic.py

# Check logs
python3 tools/utils/log_analyzer.py logs/latest.log
```

## Requirements

Install Python dependencies:

```bash
pip install smbus2 pyserial numpy opencv-python
```

Or via Poetry (from project root):

```bash
poetry install
```

## Adding New Tools

When creating new tools:

1. Place in appropriate category folder
2. Make executable: `chmod +x tool.py`
3. Add shebang: `#!/usr/bin/env python3`
4. Include `--help` argument
5. Document usage in this README

Example template:

```python
#!/usr/bin/env python3
"""
Brief description of tool.
"""
import argparse

def main():
    parser = argparse.ArgumentParser(description="Tool description")
    parser.add_argument("--option", help="Option description")
    args = parser.parse_args()

    # Tool logic here

if __name__ == "__main__":
    main()
```

## Resources

- [I2C Protocol Specification](../docs/api/i2c-protocol.md)
- [Calibration Guide](../docs/guides/calibration.md)
