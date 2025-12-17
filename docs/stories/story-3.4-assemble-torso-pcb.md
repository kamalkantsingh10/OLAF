# Story 3.4: Assemble and Test Torso PCB

**Epic:** Epic 3 - Torso Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 3-5 hours

---

## User Story

**As a** builder,
**I want** all components soldered to Torso PCB and tested for functionality,
**so that** I have a working circuit board ready for firmware development.

---

## Acceptance Criteria

1. ✅ All components soldered: ESP32, connectors, voltage regulator, passives
2. ✅ Visual inspection confirms proper solder joints
3. ✅ Power-on test: 5V input yields stable 3.3V output
4. ✅ I2C address configured as 0x0A
5. ✅ Component tests pass: display shows animations, printer outputs test page
6. ✅ PCB mounted on test fixture

---

## Implementation Steps

### 1. Solder Components

Order: Resistors → Capacitors → Regulator → ESP32 → Connectors

### 2. Power-On Test

```bash
# Bench supply: 5V @ 500mA
# Verify 3.3V output from regulator
# Current draw <200mA (idle, no peripherals)
```

### 3. Test Display

Connect 2.8" display to 8-pin header, upload test code from Story 3.1, verify heart animation.

### 4. Test Printer

Connect thermal printer to 4-pin header (5V, GND, TX, RX), upload print test, verify text output.

### 5. Test I2C

Connect to Pi, run `i2cdetect -y 1`, verify address 0x0A appears.

---

## Testing & Validation

**Test 1:** Display animates at 10+ FPS
**Test 2:** Printer outputs clear text
**Test 3:** I2C communication works

---

## Troubleshooting

**Issue: Display Blank**
**Solution:** Check SPI wiring, TFT_eSPI config

**Issue: Printer No Output**
**Solution:** Verify baud rate (19200), separate 5V supply with adequate current

---

## Dependencies

**Before:** Story 3.3 ✅
**After:** Story 3.5, Story 3.6 (parallel)

---

**Created:** 2025-12-16
