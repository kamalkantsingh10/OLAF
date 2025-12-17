# Story 2.2: Design Neck Custom PCB in Fritzing

**Epic:** Epic 2 - Neck Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 5-7 hours

---

## User Story

**As a** builder,
**I want** a custom PCB designed in Fritzing integrating ESP32 and all Neck components,
**so that** I can replace the breadboard with a compact, reliable circuit board.

---

## Acceptance Criteria

1. ✅ Fritzing schematic created with ESP32 module, servo UART bus (daisy-chain topology), presence sensor connections
2. ✅ PCB layout designed with proper component placement, trace routing, and mounting holes
3. ✅ Power distribution traces sized appropriately for servo current draw (5V rail, beefy traces)
4. ✅ Pin assignments documented in wiring diagram (saved as `modules/neck/wiring.md`)
5. ✅ Design reviewed for electrical correctness (no shorts, proper pull-ups for UART)
6. ✅ Gerber files exported and ready for manufacturing

---

## Implementation Steps

### 1. Create Fritzing Project

```bash
cd ~/olaf/modules/neck/hardware
# Create new Fritzing project: neck-pcb.fzz
```

### 2. Design Schematic

**Components:**

1. **ESP32-WROOM-32 Module**
2. **Voltage Regulator:** AMS1117-3.3 (if using 5V input) or direct 3.3V input
3. **4× Servo Connectors:** 3-pin headers (Signal, VCC, GND) - but only 1 signal line daisy-chained
4. **2× Presence Sensor Connectors:** 3-pin headers (VCC, GND, OUT)
5. **I2C Connector:** 4-pin header (VCC, GND, SDA, SCL) to Raspberry Pi
6. **Power Input:** 2× screw terminals (7.4V for servos, 5V for logic)
7. **Resistors:** 4.7kΩ (×2 for I2C pull-ups), 1kΩ (optional UART pull-up)
8. **Capacitors:** 10µF electrolytic (×3), 100nF ceramic (×6), 1000µF electrolytic (servo power)
9. **Logic Level Converter:** BSS138-based bidirectional shifter (3.3V ↔ 5V) for servo UART
10. **Programming Header:** 6-pin (FTDI compatible)

**Schematic Connections:**

```
Power Section:
- J1 (Screw Terminal): 7.4V input for servos → Fuse (3A) → C1 (1000µF) → Servo connectors VCC
- J2 (Screw Terminal): 5V input for logic → Regulator (AMS1117-3.3) → ESP32 VCC
- Common GND between all power rails

ESP32 Connections:
- GPIO32 (UART1 TX/RX): Logic Level Converter → Servo Data Line (daisy-chained)
- GPIO25: Presence Sensor 1 OUT
- GPIO26: Presence Sensor 2 OUT
- GPIO21: I2C SDA (with 4.7kΩ pull-up to 3.3V)
- GPIO22: I2C SCL (with 4.7kΩ pull-up to 3.3V)
- GPIO1/3: Programming UART (TX/RX to FTDI header)
- GPIO0: Boot mode (pull-down button for programming)
- EN: Reset (pull-up + reset button)

Servo Daisy Chain:
- J3 (Servo 1 - Pan): Signal IN from logic level converter
- J4 (Servo 2 - Tilt): Signal daisy-chained from J3
- J5 (Servo 3 - Roll): Signal daisy-chained from J4
- J6 (Servo 4 - Kickstand): Signal daisy-chained from J5
- All servos share 7.4V VCC and GND

Presence Sensors:
- J7 (Sensor 1 - Front): VCC (5V), GND, OUT → GPIO25
- J8 (Sensor 2 - Rear): VCC (5V), GND, OUT → GPIO26

I2C Bus:
- J9 (4-pin header): 3.3V, GND, SDA (GPIO21), SCL (GPIO22)
```

**Wiring Diagram Document:**

Create `modules/neck/wiring.md`:

```markdown
# Neck Module Wiring Diagram

**I2C Address:** 0x09

## ESP32 Pin Assignments

### Power
- 7.4V Input (J1): Servo power from Base module
- 5V Input (J2): Logic power from Base module
- 3.3V Regulator: AMS1117-3.3 (5V → 3.3V for ESP32)
- GND: Common ground

### Servo UART Bus (All 4 servos on single line)
- GPIO32: UART1 TX/RX (half-duplex) → Logic level converter → Servo data line
- Servo IDs:
  - ID 1: Pan (left/right head rotation)
  - ID 2: Tilt (up/down head pitch)
  - ID 3: Roll (head tilt side-to-side)
  - ID 4: Kickstand (deploy/retract)
- Servo Power: 7.4V (2S LiPo voltage), 5A maximum

### Presence Sensors
- GPIO25: Front sensor OUT (digital input, active HIGH)
- GPIO26: Rear sensor OUT (digital input, active HIGH)
- Sensor Power: 5V

### I2C Bus (to Raspberry Pi)
- SDA: GPIO21 (with 4.7kΩ pull-up to 3.3V)
- SCL: GPIO22 (with 4.7kΩ pull-up to 3.3V)

### Programming
- UART0 TX: GPIO1
- UART0 RX: GPIO3
- Boot: GPIO0 (pull low for programming mode)
- Enable: EN (pull low to reset)

## Power Budget
- ESP32: 500mA @ 3.3V
- Servos (4×): 4A peak @ 7.4V (1A per servo)
- Presence Sensors (2×): 50mA @ 5V
- **Total: 5V/1A, 7.4V/4A**

## Connectors
- J1: Servo power input 7.4V (screw terminal, 2-pin)
- J2: Logic power input 5V (screw terminal, 2-pin)
- J3-J6: Servo connectors (3-pin headers, 2.54mm pitch, daisy-chained data)
- J7-J8: Presence sensor connectors (3-pin headers)
- J9: I2C to Pi (4-pin header: VCC, GND, SDA, SCL)
- J10: Programming header (6-pin, 2.54mm pitch)
```

### 3. PCB Layout Design

**Board Specifications:**
- Size: 90mm × 70mm
- Thickness: 1.6mm
- Copper: 1oz (may need 2oz for servo power traces)
- Layers: 2 (top + bottom)

**Component Placement:**

```
Top Layer Layout:
┌─────────────────────────────────┐
│  J1(7.4V)  J2(5V)  [ESP32]      │
│                                  │
│  [Fuse]  [1000µF Cap]           │
│                                  │
│  [Logic    J3 ──┐               │
│   Level    J4 ──┼── Servo       │
│   Conv]    J5 ──┤   Connectors  │
│            J6 ──┘               │
│                                  │
│  J7(Front)  J8(Rear)  J9(I2C)   │
│  Sensors            J10(Prog)   │
└─────────────────────────────────┘
```

**Trace Width Guidelines:**
- 7.4V servo power: 30 mil (0.76mm) minimum for 4A
- 5V logic power: 20 mil (0.5mm)
- 3.3V rail: 20 mil
- GND: Use ground plane on bottom layer
- UART data line: 10 mil (keep short, away from power)
- I2C lines: 10 mil (route together, away from servo power)

**Critical Routing:**
- Servo power from J1 to all servo connectors: Wide traces, star topology from bulk capacitor
- UART data line: Single trace from logic level converter daisy-chained through all servo connectors
- I2C lines: Keep SDA and SCL together, minimize length, route away from noisy servo traces
- Ground plane: Flood bottom layer, connect all GND pads with vias

**Mounting Holes:**
- 4× M3 holes at corners (3.2mm diameter)
- Clearance: 5mm from edge, 3mm from traces

### 4. Design Rule Check

```bash
# In Fritzing:
# Routing → Design Rules Check
# Fix all errors:
# - Minimum trace width: 10 mil
# - Minimum clearance: 10 mil
# - No unrouted connections
# - No overlapping traces
```

### 5. Add Silkscreen Labels

```
Top Silkscreen:
- Component references (U1, J1, C1, etc.)
- Pin 1 indicators (dot or square pad)
- Polarity markings on electrolytic capacitors (+/-)
- Servo connector labels (Pan, Tilt, Roll, Kick)
- Sensor labels (Front, Rear)
- Module name: "OLAF Neck Module v1.0"
- I2C address: "I2C: 0x09"
```

### 6. Export Gerber Files

```bash
# File → Export → for Production → Extended Gerber (RS-274X)
# Save to: ~/olaf/modules/neck/hardware/gerbers/

# Files generated:
# - neck-pcb.GTL (top copper)
# - neck-pcb.GBL (bottom copper)
# - neck-pcb.GTS (top soldermask)
# - neck-pcb.GBS (bottom soldermask)
# - neck-pcb.GTO (top silkscreen)
# - neck-pcb.GBO (bottom silkscreen)
# - neck-pcb.TXT (drill file)
# - neck-pcb.GML (board outline)

# Create ZIP for Elecrow upload
cd ~/olaf/modules/neck/hardware/gerbers
zip neck-pcb-gerbers.zip *.GTL *.GBL *.GTS *.GBS *.GTO *.GBO *.TXT *.GML
```

### 7. Pre-Manufacturing Review

**Checklist:**

```markdown
## Electrical Review
- [ ] All nets connected (no airwires)
- [ ] Power rails isolated (no shorts: 7.4V, 5V, 3.3V, GND separate)
- [ ] Decoupling capacitors on ESP32 VCC pins
- [ ] Pull-up resistors on I2C lines (4.7kΩ to 3.3V)
- [ ] Logic level converter properly wired (3.3V ↔ 5V)
- [ ] Servo daisy-chain data path continuous
- [ ] Fuse on 7.4V servo power line
- [ ] Bulk capacitor (1000µF) on servo power near connectors

## Mechanical Review
- [ ] Mounting holes (4× M3) with proper clearance
- [ ] Board dimensions fit enclosure (from Story 2.5)
- [ ] Connector positions allow cable routing
- [ ] Silkscreen labels readable and accurate

## Manufacturing Review
- [ ] Trace widths meet minimums (10 mil signal, 30 mil power)
- [ ] Clearances meet minimums (10 mil)
- [ ] Via sizes manufacturable (0.8mm drill)
- [ ] No acute angle traces (use 45° or curves)
- [ ] Gerber files verified in online viewer
```

---

## Testing & Validation

**Test 1: Gerber Viewer Check**
```bash
# Upload ZIP to: https://www.pcbway.com/project/OnlineGerberViewer.html
# Verify:
# - All layers present
# - Servo power traces thick and continuous
# - No missing drill holes
# - Silkscreen labels correct
```

**Test 2: Power Trace Width Calculation**
```bash
# Use trace width calculator: https://www.4pcb.com/trace-width-calculator.html
# Input:
# - Current: 4A (servo power)
# - Copper thickness: 1oz
# - Temperature rise: 10°C
# Result: ~30 mil minimum → Design uses 30 mil ✓
```

**Test 3: BOM Completeness**
```bash
# Export BOM from Fritzing
# Cross-reference with breadboard components (Story 2.1)
# Verify all parts available for purchase
```

---

## Troubleshooting

**Issue 1: Logic Level Converter Not in Fritzing Library**
- **Solution:** Search Fritzing forums for "BSS138 level shifter", download custom part, or use generic IC footprint

**Issue 2: Servo Daisy-Chain Routing Difficult**
- **Solution:** Place servo connectors in linear arrangement (J3→J4→J5→J6), use single trace connecting all

**Issue 3: Ground Plane Not Filling**
- **Solution:** Check polygon settings (minimum clearance, orphan removal), use smaller clearance values

**Issue 4: Power Traces Flagged as Too Narrow**
- **Solution:** Manually widen traces to 30 mil, re-run DRC

---

## Dependencies

**Before this story:**
- Story 2.1: Breadboard Neck Components and Test Connectivity ✅
- Pin assignments verified from breadboard test

**After this story:**
- Story 2.3: Order and Receive Neck PCB from Elecrow

---

## References

- [Fritzing PCB Design Tutorial](https://fritzing.org/learning/)
- [Trace Width Calculator](https://www.4pcb.com/trace-width-calculator.html)
- [Logic Level Shifting](https://learn.sparkfun.com/tutorials/bi-directional-logic-level-converter-hookup-guide)
- [Feetech Servo Wiring Guide](http://www.feetechrc.com/en_product.html)

---

## Notes

- **Servo Power Traces:** 4A current requires 30 mil minimum with 1oz copper. Consider 2oz copper for better margin.
- **Daisy Chain:** Single UART line simplifies PCB routing vs separate lines per servo.
- **Separate Power Inputs:** 7.4V for servos (high current), 5V for logic (clean power for ESP32).
- **Bulk Capacitor:** 1000µF critical for servo transient currents, place close to servo connectors.
- **I2C Address:** 0x09 hardcoded in firmware (or configurable via jumpers if needed).

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
