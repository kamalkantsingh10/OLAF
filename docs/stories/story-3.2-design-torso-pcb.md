# Story 3.2: Design Torso Custom PCB in Fritzing

**Epic:** Epic 3 - Torso Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** a custom PCB designed in Fritzing integrating ESP32 and all Torso components,
**so that** I can replace the breadboard with a compact, reliable circuit board.

---

## Acceptance Criteria

1. ✅ Fritzing schematic created with ESP32 module, 2.8" display connection (SPI), thermal printer connection (UART/parallel)
2. ✅ PCB layout designed with proper component placement, trace routing, and mounting holes
3. ✅ Power input connector included (receives 5V from Base module power distribution)
4. ✅ I2C connector included (SDA, SCL, GND for connection to Pi I2C bus)
5. ✅ Pin assignments documented in wiring diagram
6. ✅ Design reviewed for electrical correctness
7. ✅ Gerber files exported and ready for manufacturing

---

## Implementation Steps

### 1. Create Fritzing Schematic

**Components:**
- ESP32-WROOM-32
- AMS1117-3.3 voltage regulator (5V → 3.3V)
- Display connector (8-pin header for SPI)
- Printer connector (4-pin for UART: TX, RX, VCC, GND)
- I2C connector (4-pin header)
- Power input (screw terminal, 5V + GND)
- Capacitors: 10µF (×3), 100nF (×6)
- Pull-up resistors: 4.7kΩ (×2 for I2C)
- Programming header (6-pin)

**Connections:**
```
Power: 5V input → Regulator → 3.3V rail
Display: ESP32 SPI (GPIO18/23) + DC/CS/RST → 8-pin header
Printer: ESP32 UART2 (GPIO16/17) → 4-pin header (with 5V pass-through)
I2C: ESP32 GPIO21/22 → Pi (with pull-ups)
```

### 2. PCB Layout

**Board Size:** 80mm × 60mm
**Placement:** ESP32 center, connectors on edges, decoupling caps near ICs

### 3. Create Wiring Diagram

`modules/torso/wiring.md`:

```markdown
# Torso Module Wiring

**I2C Address:** 0x0A

## Pin Assignments
- **Display (SPI):** SCK=18, MOSI=23, DC=21, CS=5, RST=16
- **Printer (UART2):** TX=16, RX=17
- **I2C:** SDA=21, SCL=22 (4.7kΩ pull-ups)
- **Power:** 5V input from Base module

## Power Budget
- ESP32: 500mA @ 3.3V
- Display: 100mA @ 3.3V
- Printer: 1.5A peak @ 5V
**Total: 5V/2A**
```

### 4. Export Gerbers

Save to `modules/torso/hardware/gerbers/`, create ZIP for Elecrow.

---

## Testing & Validation

Verify Gerbers in online viewer, check power trace widths (20 mil for 5V to printer).

---

## Dependencies

**Before:** Story 3.1 ✅
**After:** Story 3.3

---

**Created:** 2025-12-16
