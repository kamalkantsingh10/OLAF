# Ears + Neck Module

**I2C Address:** 0x09

## Overview

The Ears + Neck module provides unified upper-body control for OLAF's expressive movements. It consolidates ear and neck servos under a single ESP32-S3 controller, enabling coordinated gestures and synchronized motion.

## Hardware Specifications

### Microcontroller
- **ESP32-S3-WROOM-2 (N16R8)**
  - 16MB Flash, 8MB PSRAM
  - Dual UART for independent servo bus control
  - I2C slave interface (0x09) for orchestrator communication

### Servo Controllers

**Controller A (UART1):** Ear Servos
- 4× Feetech SCS0009 servos
- Configuration: 2-DOF per ear (left/right)
- Range: 2 ears × 2 degrees of freedom = 4 servos total
- Use case: Expressive ear movements (perking, drooping, rotating)

**Controller B (UART2):** Neck Servos
- 3× Feetech STS3215 servos
- Configuration: Pan (yaw), Tilt (pitch), Roll
- Range: Full 3-DOF head orientation control
- Use case: Head tracking, nodding, shaking, tilting

### Pin Assignments

| Function | Pin | Notes |
|----------|-----|-------|
| I2C SDA | GPIO 8 | I2C slave to orchestrator |
| I2C SCL | GPIO 9 | I2C slave to orchestrator |
| UART1 TX | GPIO 17 | Servo Controller A (ears) |
| UART1 RX | GPIO 18 | Servo Controller A (ears) |
| UART2 TX | GPIO 21 | Servo Controller B (neck) |
| UART2 RX | GPIO 20 | Servo Controller B (neck) |

## I2C Register Map

### Common Registers (0x00-0x0F)
Standard module registers defined in `shared/i2c_registers.h`:
- `0x00`: Module ID (read-only, returns 0x09)
- `0x01`: Firmware version
- `0x02`: Status byte
- `0x03`: Error code
- `0x04`: Command trigger
- `0x05-0x08`: Command arguments

### Module-Specific Registers (0x10+)

| Register | Name | Type | Description |
|----------|------|------|-------------|
| 0x10 | EAR_LEFT_POS_H | R/W | Left ear horizontal position (0-255) |
| 0x11 | EAR_LEFT_POS_V | R/W | Left ear vertical position (0-255) |
| 0x12 | EAR_RIGHT_POS_H | R/W | Right ear horizontal position (0-255) |
| 0x13 | EAR_RIGHT_POS_V | R/W | Right ear vertical position (0-255) |
| 0x20 | NECK_PAN | R/W | Neck pan/yaw position (0-255) |
| 0x21 | NECK_TILT | R/W | Neck tilt/pitch position (0-255) |
| 0x22 | NECK_ROLL | R/W | Neck roll position (0-255) |
| 0x30 | GESTURE_ID | W | Trigger coordinated gesture (0-99) |

## Firmware Architecture

### Main Controller (`ears_neck_controller.ino`)
- I2C slave interrupt handling
- Servo position command dispatching
- Status reporting to orchestrator

### Drivers
- `feetech_scs.cpp/h`: SCS0009 servo protocol (Controller A)
- `feetech_sts.cpp/h`: STS3215 servo protocol (Controller B)
- `i2c_slave.cpp/h`: I2C slave register interface

### Control Loops
- **Servo Update Loop:** 50Hz position updates to maintain smooth motion
- **Gesture Sequencer:** Executes multi-step coordinated movements
- **Safety Monitor:** Servo position limits, temperature monitoring

## Setup Instructions

### PlatformIO Upload
```bash
cd modules/ears-neck
pio run --target upload
```

### Pin Connections
1. Connect I2C bus (SDA/SCL) to Raspberry Pi 5 orchestrator
2. Connect UART1 (GPIO 17/18) to Feetech servo bus A (4× SCS0009 ear servos)
3. Connect UART2 (GPIO 21/20) to Feetech servo bus B (3× STS3215 neck servos)
4. Ensure common ground between ESP32, servos, and orchestrator
5. Power servos with appropriate voltage (typically 6-12V depending on servo specs)

## Standalone Testing

### Test Mode (No Orchestrator)
The firmware includes a standalone test mode for verification without the orchestrator:

```cpp
// In firmware/ears_neck_controller.ino
#define STANDALONE_TEST 1
```

### Test Sequence
1. Upload firmware with `STANDALONE_TEST` enabled
2. Open serial monitor (115200 baud)
3. Observe automated test sequence:
   - Center all servos (home position)
   - Cycle each ear servo individually
   - Cycle neck pan/tilt/roll servos
   - Execute sample coordinated gesture
4. Verify smooth motion and no mechanical binding

### Manual Control
Send serial commands for manual testing:
```
EL90    # Move left ear horizontal to 90°
ER45    # Move right ear vertical to 45°
NP180   # Pan neck to 180°
NT90    # Tilt neck to 90°
NR0     # Roll neck to 0°
G05     # Execute gesture ID 5
```

## Troubleshooting

### Common Issues

**Servo not responding:**
- Check UART TX/RX connections (correct polarity)
- Verify servo power supply voltage and current capacity
- Test servo individually with Feetech debugging software

**I2C communication failure:**
- Verify I2C address (0x09) not conflicting with other devices
- Check pull-up resistors on SDA/SCL lines (typically 4.7kΩ)
- Use `i2cdetect -y 1` on orchestrator to scan for device

**Jerky or erratic motion:**
- Reduce command update rate if servo bus bandwidth saturated
- Check for loose connections on UART lines
- Verify servo firmware is up-to-date

**Module reports STATUS_ERROR:**
- Read error code from register 0x03
- Check serial monitor for detailed error messages
- Common errors: servo timeout, position limit exceeded, over-temperature

### Debug Steps
1. Enable verbose logging: Set `#define DEBUG_LEVEL 2` in firmware
2. Monitor serial output for UART communication errors
3. Use oscilloscope to verify UART signal integrity (9600 baud for Feetech protocol)
4. Test servos independently using Feetech FD software
5. Verify ESP32 I2C slave functionality with `i2c-tools` commands

## Coordinated Gestures

The module supports pre-programmed gesture sequences for common expressions:

| ID | Gesture | Description |
|----|---------|-------------|
| 0 | Home | Return all servos to neutral position |
| 1 | Curious | Perk ears up, tilt head slightly |
| 2 | Alert | Ears forward, neck straight |
| 3 | Sad | Droop ears, tilt head down |
| 4 | Happy | Wiggle ears, gentle head bob |
| 5 | Confused | Tilt head, alternate ear positions |

Gestures can be triggered via register 0x30 or orchestrator animation sequences.

## References

- [Feetech SCS0009 Datasheet](https://www.feetechrc.com/en/products/scs0009)
- [Feetech STS3215 Datasheet](https://www.feetechrc.com/en/products/sts3215)
- [I2C Register Map Standard](../../docs/architecture/data-models.md)
- [Module Architecture](../../docs/architecture/components.md#ears--neck-module-shared-esp32)
