# Neck Module Assembly

## Overview

The Neck module provides expressive 3-DOF head movement using a parallel linkage mechanism with 3× STS3215 servos plus 1× kickstand servo.

**Detailed Kinematics:** See [Neck Mechanism Kinematics](./articles/neck-mechanism-kinematics.md) for complete design documentation, control modes, and expressive motion examples.

**Configuration:** See [config.yaml](./config.yaml) for servo IDs, motion ranges (yaw ±75°, pitch ±45°, roll ±30°), and all configurable parameters.

---

## Components

### Electronics
- **1× ESP32-S3-DevKitC-1** (N16R8) - Module controller
- **4× Feetech STS3215 Servos** - Yaw, 2× tilt (left/right), kickstand
- **1× Serial Bus Servo Controller (STSC)** - Controls all 4 servos
- **2× 24GHz mmWave Presence Sensors (HLK-LD2461)** - Human detection
- **1× WS2812B LED Strip (8 LEDs)** - Status display
- **1× 36V→12V Buck Converter (HLK-PM12)** - Servo power

### Mechanical (3D Printed)
- **1× Neck Base Platform** - Mounts to torso, houses yaw servo
- **2× Tilt Servo Mounts** - Parallel mounting (left/right)
- **2× Linkage Rods** - Connect tilt servos to head platform
- **1× Head Platform** - Receives head module, driven by linkages
- **4× Servo Horns** - Attach to servo shafts

---

## Assembly Steps

[To be documented during Story 2.4]

## Testing

Before assembling the final PCB, breadboard each component individually to verify functionality. This incremental approach catches issues early when they're easier to debug. The firmware is designed to support this workflow—you can enable one component at a time, test it, then move to the next.

### Breadboard Testing Workflow

**Start small, build confidence, then integrate.** Each component gets its own test phase before combining them.

#### Phase 1: mmWave Sensor (Current)

**Goal:** Verify UART communication with HLK-LD2461 sensor at 256000 baud.

**Wiring:**
```
HLK-LD2461        ESP32-S3
----------        --------
VCC       →       5V (⚠️ 330mA - ensure power supply can handle this)
GND       →       GND
TX        →       GPIO 39 (RX)
RX        →       GPIO 38 (TX)
```

**Test Procedure:**
1. Navigate to firmware directory: `cd modules/neck/firmware`
2. Build and upload: `~/.platformio/penv/bin/pio run -t upload -t monitor`
3. Watch for sensor data in Serial Monitor (115200 baud)
4. Expected: Hex data stream like `0xAA 0x55 0x12 ...` when motion detected
5. Wave hand near sensor - data rate should increase

**Success Criteria:**
- [ ] Serial monitor shows initialization message
- [ ] Hex data appears when motion detected
- [ ] No "timeout" or "not responding" errors
- [ ] Steady 5V power (measure with multimeter: 4.75-5.25V)

**Troubleshooting:**
- No data? Check 5V supply and verify TX/RX are crossed (TX→RX, RX→TX)
- Garbage data? Verify 256000 baud rate
- Intermittent? Check breadboard connections and common ground

---

#### Phase 2: Servo Controller (Next)

**Goal:** Control 4× STS3215 servos via Serial Bus Controller (pan, tilt, roll, kickstand).

**Setup:**
1. Edit `src/main.cpp`: Uncomment `#define TEST_SERVOS`
2. Connect servo controller to GPIO 17/18 (UART1)
3. Power servos with external 12V supply (⚠️ never connect servo motor power to ESP32!)
4. Rebuild: `~/.platformio/penv/bin/pio run -t upload -t monitor`

**Test Procedure:**
- Send position commands to each servo ID (1-4)
- Verify smooth movement and position feedback
- Check current draw stays below 4A during motion

---

#### Phase 3: LED Status Strip (Future)

**Goal:** Drive 8× WS2812B LEDs on GPIO 1 for system status display.

**Setup:**
1. Uncomment `#define TEST_LEDS` in `src/main.cpp`
2. Wire LED strip DIN to GPIO 1 via 470Ω resistor
3. Add 100µF capacitor between LED VCC and GND
4. Rebuild and upload

**Test Procedure:**
- Verify all 8 LEDs light up in sequence
- Test color accuracy (red, green, blue)
- Check for flickering or dead pixels

---

#### Phase 4: I2C Slave (Final)

**Goal:** Enable Raspberry Pi communication as I2C slave at address 0x09.

**Setup:**
1. Uncomment `#define TEST_I2C_SLAVE`
2. Connect to Pi: GPIO 8→Pi GPIO 2 (SDA), GPIO 9→Pi GPIO 3 (SCL)
3. Verify 4.7kΩ pull-up resistors on SDA/SCL
4. On Pi, run: `i2cdetect -y 1` (should show 0x09)

**Test Procedure:**
- Send test command from Pi
- Verify ESP32 receives and responds
- Test all I2C registers defined in protocol

---

### Quick Commands

All commands assume you're in `modules/neck/firmware/`:

```bash
# Build only
~/.platformio/penv/bin/pio run

# Upload and monitor
~/.platformio/penv/bin/pio run -t upload -t monitor

# Clean build (if issues)
~/.platformio/penv/bin/pio run -t clean

# Debug build (verbose logging)
~/.platformio/penv/bin/pio run -e esp32s3-debug -t upload -t monitor
```

**Exit monitor:** `Ctrl+C`

---

### Enabling Components

The test firmware uses conditional compilation. Edit `firmware/src/main.cpp` and uncomment flags:

```cpp
// Component Enable Flags - Uncomment to test
#define TEST_MMWAVE_SENSOR      // ✓ HLK-LD2461 mmWave sensor
// #define TEST_SERVOS          // Servo controller (uncomment when ready)
// #define TEST_LEDS            // WS2812B LED strip
// #define TEST_I2C_SLAVE       // I2C slave mode
```

After changing flags, rebuild: `~/.platformio/penv/bin/pio run -t upload`

---

### Component Test Checklist

Track your progress as you breadboard each component:

- [ ] **mmWave Sensor** - Data received at 256000 baud, motion detection works
- [ ] **Servo Controller** - All 4 servos respond to position commands
- [ ] **LED Strip** - All 8 LEDs light up, colors accurate
- [ ] **I2C Slave** - Pi detects 0x09, can read/write registers
- [ ] **Integration** - All components work simultaneously without conflicts

**Integration Test:** Once all components pass individually, enable all flags and verify they coexist without I2C/UART/power conflicts. Check current draw doesn't exceed power budget.
