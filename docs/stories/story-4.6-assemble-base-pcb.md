# Story 4.6: Assemble and Test Base PCB

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 6-8 hours

---

## User Story

**As a** builder,
**I want** to solder all components to the Base PCB and verify functionality,
**so that** the Base module electronics are complete and tested.

---

## Acceptance Criteria

1. ✅ All components soldered to PCB (ESP32, MPU6050, MOSFET, connectors, passives)
2. ✅ Solder joints inspected (no cold joints, bridges, or shorts)
3. ✅ Power-on test successful (no smoke, correct voltages)
4. ✅ ESP32 programmed and responding on I2C (address 0x0B)
5. ✅ MPU6050 IMU reading data correctly
6. ✅ Voltage monitoring circuit functional (reads battery voltage)
7. ✅ All connectors tested (XT60, JST, headers)
8. ✅ Reverse polarity protection verified

---

## Implementation Steps

### 1. Prepare Workspace and Tools

**Required tools:**
- Soldering iron (temperature-controlled, 350°C)
- Solder (60/40 or lead-free, 0.8mm diameter)
- Flux (rosin-based)
- Solder wick (desoldering braid)
- Tweezers
- Multimeter
- Magnifying glass or microscope
- Isopropyl alcohol (90%+) for cleaning
- Anti-static mat and wrist strap

### 2. Solder Components in Order (Small to Large)

**Soldering sequence:**

```bash
# Step 1: Resistors (R1, R2, R3)
#   - Bend leads, insert through holes
#   - Solder on back side
#   - Trim excess leads

# Step 2: Ceramic capacitors (C4-C8, 0.1µF)
#   - Insert, solder, trim

# Step 3: Electrolytic capacitors (C1-C3, 100µF)
#   - Observe polarity (longer lead = positive)
#   - Insert, solder, trim

# Step 4: LED (LED1)
#   - Longer lead = anode (positive)
#   - Insert, solder, trim

# Step 5: MOSFET (Q1, IRF4905)
#   - Bend leads to fit footprint
#   - Ensure correct orientation (check datasheet)
#   - Solder, optionally add heatsink

# Step 6: Pin headers (P1, P2)
#   - Insert, hold straight while soldering
#   - Solder one pin first, adjust angle, then solder rest

# Step 7: JST connector (J4)
#   - Insert, solder all pins

# Step 8: Screw terminals (J3, ×2)
#   - Insert, solder securely

# Step 9: XT60 connectors (J1, J2)
#   - Large pads, high heat required
#   - Use higher temp (380°C) and thick solder wire
#   - Hold connector while cooling (prevents shift)

# Step 10: Fuse holders (F1, F2, F3)
#   - Insert, solder all pins

# Step 11: ESP32 module (U1)
#   - Use pin headers (solder headers first, then ESP32)
#   - Or solder ESP32 directly for lower profile
#   - Ensure orientation correct (match silkscreen)

# Step 12: MPU6050 breakout (U2)
#   - Use pin headers or solder directly
#   - Orientation matters (check I2C address markings)
```

### 3. Inspect Solder Joints

**Quality checklist:**

```bash
✓ Shiny solder (not dull/grainy = cold joint)
✓ Cone shape around pin (not blob)
✓ Good wetting (solder flows onto pad and pin)
✓ No bridges between adjacent pins
✓ No flux residue (clean with isopropyl alcohol)
✓ All pins soldered (no missed connections)
✓ Leads trimmed flush (no sharp edges)
```

**Rework if needed:**
- Cold joints: Reheat with flux
- Bridges: Remove excess solder with wick
- Missing joints: Add solder

### 4. Clean PCB

```bash
# Apply isopropyl alcohol (90%+) with brush
# Scrub gently to remove flux residue
# Let dry completely (5 minutes)
# Inspect under magnification for cleanliness
```

### 5. Visual Inspection and Continuity Check

**Pre-power checks:**

```bash
# Check 1: No shorts between power rails
#   Set multimeter to continuity mode
#   Test: Battery (+) pad to GND pad
#   Expected: Open circuit (no beep)

# Check 2: Ground plane continuity
#   Test: Any GND pad to any other GND pad
#   Expected: Continuity (beep)

# Check 3: Component polarity
#   Electrolytic caps: (+) marking on PCB matches cap marking
#   MOSFET: Source, Drain, Gate on correct pins (check datasheet)
#   LED: Anode to (+), cathode to (-)
```

### 6. Power-On Test (No Battery Yet)

**Bench power supply test:**

```bash
# Use adjustable bench power supply (safer than battery)
# Set to 36V, current limit 0.5A

# Connect to XT60 battery input
# Power on slowly, monitor current draw

# Expected current: <100mA (ESP32 + IMU idle)
# If current spikes: POWER OFF immediately, check for shorts

# Measure voltages at test points:
#   Main bus (+): 36V
#   ESP32 VIN: 5V (after buck converter)
#   ESP32 3.3V: 3.3V
#   MPU6050 VCC: 3.3V

# If voltages correct: SUCCESS
# If voltages wrong: Debug power distribution
```

### 7. Program ESP32 Firmware

**Upload test firmware:**

```cpp
// base_test.ino
#include <Wire.h>
#include <MPU6050.h>

#define I2C_ADDRESS 0x0B

MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // Initialize I2C as slave
  Wire.begin(I2C_ADDRESS);

  // Initialize MPU6050
  Wire.beginTransmission(0x68);
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 OK");
  } else {
    Serial.println("MPU6050 FAIL");
  }

  Serial.println("Base PCB Test v1.0");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("IMU: ");
  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.println(az);

  delay(100);
}
```

```bash
# Upload via USB
pio run --target upload

# Monitor serial output
pio device monitor

# Expected:
# Base PCB Test v1.0
# MPU6050 OK
# IMU: [values changing when board moved]
```

### 8. Test I2C Communication with Pi

**On Raspberry Pi:**

```bash
# Connect Base PCB I2C to Pi via JST connector
# Power Base PCB with bench supply

# Scan I2C bus
i2cdetect -y 1

# Expected: 0x0B appears (Base ESP32)

# Test read device ID
i2cget -y 1 0x0B 0x00
# Expected: 0x04
```

### 9. Test Voltage Monitoring

```bash
# On ESP32:
#   Read ADC value from voltage divider
#   Calculate battery voltage: Vbat = ADC × 11

# With 36V input:
#   ADC should read ~3.27V
#   Calculated: 3.27 × 11 = 35.97V (close to 36V)

# Test with different voltages (30V, 42V)
#   Verify readings accurate within ±5%
```

### 10. Test Reverse Polarity Protection

**WARNING: Only test once!**

```bash
# With bench power supply OFF
# Reverse polarity: Connect (+) to (-), (-) to (+)
# Set current limit to 0.1A (protection)
# Power on

# Expected: No current flow (MOSFET blocks)
# Multimeter on output: 0V (protection working)

# If protection fails: Components may be damaged
# This is why we test with current-limited supply first
```

---

## Testing & Validation

**Test 1: Power Rails**
```bash
# All voltages correct: 36V, 5V, 3.3V
```

**Test 2: ESP32 Firmware**
```bash
# Uploads successfully, runs test code
```

**Test 3: I2C Communication**
```bash
# Pi detects Base at 0x0B
# ESP32 reads MPU6050 at 0x68
```

**Test 4: IMU Data**
```bash
# Reasonable values, responds to movement
```

---

## Troubleshooting

**Issue 1: No Power (0V on all rails)**
- **Solution:** Check reverse polarity MOSFET orientation, verify input voltage

**Issue 2: ESP32 Won't Boot**
- **Solution:** Check 3.3V rail, verify EN pin pulled high, check USB connection

**Issue 3: MPU6050 Not Detected**
- **Solution:** Check I2C wiring, verify 3.3V power, try address 0x69

**Issue 4: Voltage Monitoring Incorrect**
- **Solution:** Verify resistor values (100kΩ, 10kΩ), check ADC pin connection

---

## Dependencies

**Before this story:**
- Story 4.5: Order and Receive Base PCB ✅

**After this story:**
- Story 4.7: Assemble Base Platform with Skateboard Suspension

---

## References

- [Soldering Tutorial](https://www.youtube.com/watch?v=VxMV6wGS3NY)
- [IPC Solder Joint Standards](https://www.ipc.org/ipc-standards)

---

## Notes

- **Take Your Time:** Rushing leads to mistakes. 6-8 hours is reasonable for first-time PCB assembly.
- **Test Early:** Power-on test catches major issues before full assembly.
- **Use Flux:** Makes soldering easier and creates better joints.
- **Heat Management:** Don't overheat components (especially ESP32). 2-3 seconds per joint max.
- **Backup PCBs:** If you ordered 5 boards, you have spares for mistakes.

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
