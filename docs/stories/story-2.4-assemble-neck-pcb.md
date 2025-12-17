# Story 2.4: Assemble and Test Neck PCB

**Epic:** Epic 2 - Neck Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** all components soldered to Neck PCB and tested for functionality,
**so that** I have a working circuit board ready for firmware development.

---

## Acceptance Criteria

1. ✅ All components soldered: ESP32 module, connectors for servos/sensors, logic level converter, passive components
2. ✅ Visual inspection confirms proper solder joints (no cold joints, bridges)
3. ✅ Power-on test confirms correct voltage rails (7.4V, 5V, 3.3V) with no excessive current draw
4. ✅ I2C address configured as 0x09 in firmware
5. ✅ Component-level tests pass: all 4 servos respond on UART bus, presence sensors provide readings
6. ✅ PCB mounted on temporary test fixture for firmware development

---

## Implementation Steps

### 1. Solder Components (Same Process as Story 1.4)

**Order:** Resistors → Capacitors → Voltage regulator → Logic level converter → ESP32 → Connectors

**Critical Components:**
- **1000µF Capacitor:** Ensure polarity correct (+ to 7.4V, - to GND)
- **Logic Level Converter:** Verify orientation (3.3V side to ESP32, 5V side to servo)
- **Fuse:** Solder fuse holder, install 3A fuse

### 2. Power-On Test (No Load)

```bash
# Bench supply: 7.4V @ 500mA limit to servo power input
#                5V @ 500mA limit to logic power input
# Turn on, verify:
# - 7.4V rail present at servo connectors
# - 5V rail stable
# - 3.3V output from regulator (3.2-3.4V)
# - Current draw <200mA (idle, no components connected)
```

### 3. Connect and Test ESP32

Upload blink test from breadboard (Story 2.1), verify:
- ESP32 boots and runs
- Serial output visible
- LED blinks

### 4. Test Servo Communication

```bash
# Connect all 4 servos to daisy-chain connectors
# Upload servo test code from Story 2.1
# Expected: All servos respond to unique IDs, smooth movement
# Measure current: peak <4A during movement
```

### 5. Test Presence Sensors

```bash
# Connect sensors to J7, J8
# Upload sensor test code
# Expected: Digital readings toggle when presence detected
```

### 6. Test I2C Communication

```bash
# Connect to Raspberry Pi I2C bus
# On Pi: i2cdetect -y 1
# Expected: Address 0x09 shows up
```

---

## Testing & Validation

**Test 1: Servo Power Delivery Under Load**
```bash
# Command all 4 servos to move simultaneously
# Measure voltage at servo connectors: should stay >7.0V (minimal sag)
# If voltage sags to <6.5V, check:
#   - Power supply current capacity
#   - Power trace resistance
#   - Bulk capacitor (1000µF) installed correctly
```

**Test 2: Thermal Check**
```bash
# Run servos continuously for 15 minutes
# Touch components:
#   - Voltage regulator: Warm OK (<60°C)
#   - Fuse: Cool to warm
#   - Power traces: Should not be hot
# If excessive heat, check for shorts or undersized traces
```

---

## Troubleshooting

**Issue: Servos Don't Respond After PCB Assembly**
- **Solution:**
  - Check logic level converter orientation and connections
  - Verify servo daisy-chain wiring (DATA OUT → DATA IN)
  - Test UART signal with oscilloscope (should see 5V TTL pulses)
  - Revert to breadboard servo test to isolate PCB vs servo issue

**Issue: Excessive Voltage Drop on Servo Power**
- **Solution:**
  - Check solder joints on power traces (reflow if necessary)
  - Verify 2oz copper was used (measure trace resistance: should be <0.1Ω)
  - Add wire jumpers parallel to power traces if needed (temporary fix)

---

## Dependencies

**Before this story:**
- Story 2.3: Order and Receive Neck PCB from Elecrow ✅
- All components from BOM purchased

**After this story:**
- Story 2.5: Design and 3D Print Neck Enclosure and Servo Mounts
- Story 2.7: Develop Neck ESP32 Firmware (can work in parallel)

---

## References

- [Servo Current Draw Measurement](https://learn.adafruit.com/adafruit-motor-selection-guide/rc-servos)
- [High-Current PCB Design](https://www.protoexpress.com/blog/current-carrying-capacity-pcb-traces/)

---

## Notes

- **Current Measurement:** Use clamp meter or multimeter in series to measure actual servo current during movement.
- **Servo Power Quality:** Clean, stable 7.4V critical for smooth servo operation. Any sag causes jitter.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
