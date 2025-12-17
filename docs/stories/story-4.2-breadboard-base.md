# Story 4.2: Breadboard Base Components and Test Connectivity

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 10-14 hours

---

## User Story

**As a** builder,
**I want** to breadboard all Base module components and verify connectivity before PCB design,
**so that** I can test the ODrive motor controller, MPU6050 IMU, and power distribution system work together.

---

## Acceptance Criteria

1. ✅ ODrive S1 controller connected to hoverboard motors and powered
2. ✅ Motors spin in both directions under ODrive control
3. ✅ ODrive calibration completed successfully for both motors
4. ✅ MPU6050 IMU connected to ESP32 via I2C and reading accelerometer/gyro data
5. ✅ ESP32 connected to Pi via I2C (slave address 0x0B)
6. ✅ Power distribution breadboarded: 36V battery → buck converters → 5V (Pi) and 12V/24V (ODrive)
7. ✅ Emergency stop circuit tested (physical button cuts motor power)
8. ✅ All wiring documented with photos and schematic

---

## Implementation Steps

### 1. Gather All Components

**Major components:**
- ODrive S1 motor controller (or ODrive 3.6)
- 2× Hoverboard BLDC motors (from Story 4.1)
- 36V hoverboard battery (from Story 4.1)
- ESP32 development board
- MPU6050 IMU breakout board
- Buck converter: 36V → 5V, 5A (for Pi 5)
- Buck converter: 36V → 24V, 3A (for ODrive, if S1 doesn't accept 36V directly)
- XT60 connectors
- Emergency stop button (large red mushroom button)
- Breadboard (full-size, 830 tie-points)
- Jumper wires (male-male, male-female)
- 10kΩ resistors (×2 for I2C pull-ups)

**Tools:**
- Multimeter
- Soldering iron (for connectors)
- Wire strippers
- Crimping tool
- Heat shrink tubing

### 2. Set Up ODrive and Install Software

**On your development computer:**

```bash
# Install ODrive tools
pip3 install odrive

# Connect ODrive to computer via USB
# Verify ODrive detected
odrivetool

# Expected output:
# ODrive control utility v0.6.x
# Please connect your ODrive.
# Connected to ODrive [serial_number] as odrv0

# Exit odrivetool
quit()
```

### 3. Wire ODrive to Motors

**Motor wiring (ODrive S1):**

```bash
# ODrive S1 has two motor outputs: M0 and M1
# Each motor needs:
#   - 3× Phase wires (A, B, C)
#   - 5× Hall sensor wires (VCC, GND, HA, HB, HC)

# Left Motor (M0):
#   ODrive M0 Phase A → Motor Phase A (check color from Story 4.1 docs)
#   ODrive M0 Phase B → Motor Phase B
#   ODrive M0 Phase C → Motor Phase C
#   ODrive M0 Hall VCC (5V) → Motor Hall VCC (red wire)
#   ODrive M0 Hall GND → Motor Hall GND (black wire)
#   ODrive M0 Hall A → Motor Hall A
#   ODrive M0 Hall B → Motor Hall B
#   ODrive M0 Hall C → Motor Hall C

# Right Motor (M1):
#   [Same wiring to M1 port]

# Phase wire gauge: 12-14 AWG (high current)
# Hall sensor wire gauge: 22-26 AWG (signal only)

# Connectors:
#   - Phase wires: Solder directly to ODrive screw terminals or use ring terminals
#   - Hall sensors: Use JST connectors or Dupont connectors
```

**Important notes:**
- Phase wire order (A, B, C) can be any order initially - ODrive calibration will determine correct direction
- Hall sensor polarity critical: VCC and GND must be correct or sensors will be damaged
- Double-check all connections before powering on

### 4. Set Up Power Distribution

**Power wiring diagram:**

```
36V Battery (via XT60 connector)
    │
    ├─→ Emergency Stop Button (NC contact)
    │
    ├─→ ODrive DC Power Input (36V, 10A fuse)
    │
    ├─→ Buck Converter 1 (36V → 5V, 5A) → Raspberry Pi USB-C
    │
    └─→ Buck Converter 2 (36V → 12V, 1A) → ESP32 + IMU (via voltage regulator)
```

**Buck converter setup:**

```bash
# Buck Converter 1 (for Raspberry Pi):
#   - Input: 36V from battery
#   - Output: 5.0V (adjust with trim pot)
#   - Current: 5A minimum (Pi 5 requires up to 5A under load)

# Adjust output voltage:
#   1. Connect input (36V) WITHOUT connecting output
#   2. Measure output voltage with multimeter
#   3. Turn trim pot clockwise to increase, counterclockwise to decrease
#   4. Set to exactly 5.0V (use precision: 4.95-5.05V acceptable)
#   5. Connect to dummy load (10Ω resistor) and verify voltage stays stable

# Buck Converter 2 (for ESP32):
#   - Input: 36V from battery
#   - Output: 12V or 5V (depends on ESP32 board voltage regulator)
#   - Current: 1A sufficient
#   - Adjust to 12V (ESP32 boards with 7805 regulator accept 12V)
#   - Or adjust to 5V (ESP32 boards with 3.3V regulator only)
```

**ODrive power:**

```bash
# ODrive S1: Accepts 12-56V DC
#   - Connect 36V battery directly to ODrive DC input
#   - Add 10A fuse in series (protection)
#   - Use XT60 or XT90 connector for high-current connection

# Polarity: Double-check V+ and GND markings on ODrive
#   - Reversed polarity will destroy ODrive (no protection diode)
```

**Emergency stop circuit:**

```bash
# Use NC (normally closed) contact on E-stop button
# Wire in series with battery positive lead to ODrive
# When button pressed: Circuit opens, motors lose power immediately
# When button released: Circuit closed, normal operation

# Emergency Stop Button:
#   Battery V+ → E-stop NC contact → ODrive V+
#   (GND remains connected)

# Position button for easy access during testing
```

### 5. Connect MPU6050 IMU to ESP32

**I2C wiring (MPU6050 to ESP32):**

```bash
# MPU6050 pins:
#   VCC → ESP32 3.3V (NOT 5V - MPU6050 is 3.3V device)
#   GND → ESP32 GND
#   SCL → ESP32 GPIO 22 (I2C clock)
#   SDA → ESP32 GPIO 21 (I2C data)
#   INT → (optional) ESP32 GPIO 4 (interrupt pin for motion detection)

# I2C address: 0x68 (default) or 0x69 (if AD0 pin pulled high)

# Pull-up resistors:
#   - Some MPU6050 breakout boards have built-in 10kΩ pull-ups
#   - If not, add external 10kΩ resistors:
#     - SDA → 10kΩ → 3.3V
#     - SCL → 10kΩ → 3.3V
```

### 6. Connect ESP32 to Raspberry Pi (I2C)

**I2C wiring (ESP32 to Pi):**

```bash
# ESP32 as I2C slave (address 0x0B):
#   ESP32 SDA (GPIO 21) → Pi GPIO 2 (SDA1)
#   ESP32 SCL (GPIO 22) → Pi GPIO 3 (SCL1)
#   ESP32 GND → Pi GND

# Note: ESP32 and Pi share same I2C bus as other modules
#   - Head+Ears: 0x08
#   - Torso: 0x09
#   - Neck: 0x0A
#   - Base: 0x0B

# Logic level:
#   - Pi uses 3.3V I2C
#   - ESP32 uses 3.3V I2C
#   - Compatible, no level shifter needed

# Pull-up resistors:
#   - Should already be on Pi I2C bus from previous stories
#   - If not, add 4.7kΩ pull-ups to 3.3V
```

### 7. Test ODrive Motor Control

**Basic motor test:**

```bash
# Connect ODrive to computer via USB
# Power on ODrive (36V from battery)

# Launch ODrive tool
odrivetool

# Configure motor 0 (left motor)
odrv0.axis0.motor.config.pole_pairs = 15  # Hoverboard motors typically 15 pole pairs
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4.0
odrv0.axis0.motor.config.requested_current_range = 25  # Amps
odrv0.axis0.motor.config.current_control_bandwidth = 100

# Configure encoder (hall sensors)
odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis0.encoder.config.cpr = 90  # 15 pole pairs × 6 = 90
odrv0.axis0.encoder.config.bandwidth = 100

# Save configuration
odrv0.save_configuration()
odrv0.reboot()

# Wait for reboot, reconnect
odrivetool

# Run motor calibration
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION

# Wait for calibration to complete (motor will beep/twitch)
# Check for errors
odrv0.axis0.motor.error
# Expected: 0 (no errors)

# Run encoder calibration
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION

# Check encoder calibrated
odrv0.axis0.encoder.is_ready
# Expected: True

# Enter closed-loop control mode
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Set control mode to velocity
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

# Test motor spin (1 revolution per second)
odrv0.axis0.controller.input_vel = 1

# Motor should spin slowly!
# Reverse direction
odrv0.axis0.controller.input_vel = -1

# Stop motor
odrv0.axis0.controller.input_vel = 0

# Exit closed loop
odrv0.axis0.requested_state = AXIS_STATE_IDLE

# Repeat for motor 1 (right motor)
# [Same configuration and calibration steps for odrv0.axis1]
```

### 8. Test IMU Data Acquisition

**ESP32 firmware for IMU test:**

```cpp
// test_imu.ino
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to g's and deg/s
  float accelX = ax / 16384.0;  // ±2g range
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  float gyroX = gx / 131.0;     // ±250 deg/s range
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  Serial.print("Accel X: "); Serial.print(accelX);
  Serial.print(" Y: "); Serial.print(accelY);
  Serial.print(" Z: "); Serial.print(accelZ);
  Serial.print(" | Gyro X: "); Serial.print(gyroX);
  Serial.print(" Y: "); Serial.print(gyroY);
  Serial.print(" Z: "); Serial.println(gyroZ);

  delay(100);  // 10 Hz update rate
}
```

Upload to ESP32 and verify data in serial monitor.

### 9. Test I2C Communication (ESP32 to Pi)

**On Raspberry Pi:**

```bash
# Scan I2C bus
i2cdetect -y 1

# Expected: 0x0B shows up (Base module ESP32)

# Test read device ID (once ESP32 firmware implements I2C slave)
i2cget -y 1 0x0B 0x00
# Expected: 0x04 (Base module device ID)
```

### 10. Document Breadboard Setup

**Take photos and create schematic:**

```markdown
# Base Module Breadboard Setup

## Power Distribution
- Battery: 36V, 4.4Ah
- ODrive: 36V direct (10A fuse)
- Pi: 5V/5A (buck converter)
- ESP32: 12V/1A (buck converter → onboard regulator)

## Motor Connections
- Left motor (M0): Phase A/B/C + Hall sensors
- Right motor (M1): Phase A/B/C + Hall sensors

## ESP32 Connections
- I2C to Pi: GPIO 21 (SDA), GPIO 22 (SCL), address 0x0B
- I2C to IMU: GPIO 21 (SDA), GPIO 22 (SCL), IMU at 0x68
- Serial to ODrive: GPIO 16 (TX), GPIO 17 (RX) (for UART control)

## IMU
- MPU6050: 3.3V, I2C 0x68

## Emergency Stop
- Red button: NC contact in series with battery V+ to ODrive

## Photos
[Insert breadboard photos here]
```

---

## Testing & Validation

**Test 1: Power Distribution**
```bash
# Measure voltages with multimeter:
#   - Battery: 36-42V
#   - ODrive input: 36-42V
#   - Pi buck converter output: 5.0V ± 0.05V
#   - ESP32 buck converter output: 12V or 5V (depending on board)
```

**Test 2: ODrive Motor Calibration**
```bash
# Both motors calibrate successfully
# No errors: odrv0.axis0.motor.error == 0
```

**Test 3: Motor Spin Test**
```bash
# Motors spin smoothly in both directions
# Speed control responsive
# Emergency stop cuts power immediately
```

**Test 4: IMU Data**
```bash
# MPU6050 outputs reasonable values:
#   - Accel Z ≈ 1.0g (when flat)
#   - Gyro ≈ 0 deg/s (when stationary)
```

**Test 5: I2C Communication**
```bash
# Pi detects ESP32 at 0x0B
# ESP32 can read IMU at 0x68
```

---

## Troubleshooting

**Issue 1: ODrive Not Detected**
- **Solution:** Check USB cable, try different port, reinstall drivers

**Issue 2: Motor Calibration Fails**
- **Solution:** Check phase wire connections, verify hall sensor polarity, reduce current limit

**Issue 3: IMU Not Detected (0x68)**
- **Solution:** Check I2C wiring, verify 3.3V power, check pull-up resistors, try address 0x69

**Issue 4: Buck Converter Output Voltage Unstable**
- **Solution:** Add output capacitor (100µF), check input voltage, reduce load

**Issue 5: Emergency Stop Doesn't Cut Power**
- **Solution:** Verify NC contact used (not NO), check wiring continuity

---

## Dependencies

**Before this story:**
- Story 4.1: Source and Disassemble Hoverboard ✅

**After this story:**
- Story 4.3: Design Power Distribution System
- Story 4.4: Design Base Custom PCB

---

## References

- [ODrive Documentation](https://docs.odriverobotics.com/)
- [MPU6050 Library](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
- [Buck Converter Basics](https://www.ti.com/lit/an/slva059/slva059.pdf)

---

## Notes

- **ODrive Calibration:** Must be done for each motor before first use. Saved to ODrive memory.
- **Motor Direction:** Phase wire order determines direction. Can swap any two phases to reverse.
- **IMU Orientation:** Mount IMU flat with Z-axis pointing up for simplest pitch/roll calculations.
- **Power Budget:** Pi (25W) + ODrive (200W peak) + ESP32 (5W) = ~230W. Battery runtime: ~30 minutes continuous operation.
- **Future Enhancements:** Add current sensors to monitor battery usage, voltage divider for battery voltage monitoring.

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
