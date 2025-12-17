# Story 1.2: Design Head+Ears Custom PCB in Fritzing

**Epic:** Epic 1 - Head+Ears Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 6-8 hours

---

## User Story

**As a** builder,
**I want** a custom PCB designed in Fritzing integrating ESP32 and all Head+Ears components,
**so that** I can replace the breadboard with a compact, reliable circuit board.

---

## Acceptance Criteria

1. ✅ Fritzing schematic created with ESP32 module, OLED connections (SPI), servo UART buses, optocoupler circuit, focus servo connection
2. ✅ PCB layout designed with proper component placement, trace routing, and mounting holes
3. ✅ Power distribution traces sized appropriately (5V, 3.3V rails)
4. ✅ Pin assignments documented in wiring diagram (saved as `modules/head-ears/wiring.md`)
5. ✅ Design reviewed for electrical correctness (no shorts, proper pull-ups/pull-downs)
6. ✅ Gerber files exported and ready for manufacturing

---

## Implementation Steps

### 1. Install and Set Up Fritzing

```bash
# Download Fritzing from official website (or use Flatpak)
# Linux:
flatpak install flathub org.fritzing.Fritzing
flatpak run org.fritzing.Fritzing

# Or download from: https://fritzing.org/download/
```

**Create New Project:**
1. Open Fritzing
2. File → New
3. Save as: `~/olaf/modules/head-ears/hardware/head-ears-pcb.fzz`

### 2. Create Schematic (Schematic View)

**Add Components:**

1. **ESP32 Module (Main Controller)**
   - Search: "ESP32-WROOM-32"
   - If not available, import custom part from Fritzing forums
   - Place in center of schematic

2. **Power Input (From Base Module)**
   - Add: 2-pin screw terminal (5V + GND)
   - Connect to voltage regulator if using 12V input, or directly to 5V rail if Base provides regulated 5V

3. **Voltage Regulator (Optional)**
   - Add: AMS1117-3.3 or LM1117-3.3 (for 3.3V rail from 5V)
   - Input: 5V rail
   - Output: 3.3V rail
   - Add decoupling capacitors: 10µF input, 10µF output

4. **2× OLED Displays (Eyes)**
   - Add: 2× 7-pin headers (VCC, GND, SCK, MOSI, DC, CS, RST)
   - Wire to ESP32 SPI pins:
     - OLED1: DC=GPIO21, CS=GPIO5, RST=GPIO16
     - OLED2: DC=GPIO22, CS=GPIO17, RST=GPIO16 (shared)
     - SCK=GPIO18, MOSI=GPIO23 (shared)
   - Pull-up resistors (4.7kΩ) on CS lines (optional but recommended)

5. **4× Servo Connectors (Ears)**
   - Add: 2× 3-pin headers (Signal, VCC, GND) for left ear bus
   - Add: 2× 3-pin headers for right ear bus
   - Wire:
     - Left ear signal → GPIO26 (UART1)
     - Right ear signal → GPIO27 (UART2)
     - VCC → 6V rail (separate power input)
     - Add bulk capacitor: 1000µF electrolytic across 6V rail

6. **Projector Power Circuit (Optocoupler + MOSFET)**
   - Add: PC817 optocoupler
   - Add: IRLZ44N MOSFET (logic-level, N-channel)
   - Add: Resistors: 220Ω (optocoupler LED), 10kΩ (MOSFET pull-down)
   - Add: 2-pin screw terminal for projector power output
   - Wire:
     - ESP32 GPIO25 → 220Ω → Optocoupler LED anode
     - LED cathode → GND
     - Optocoupler transistor collector → MOSFET gate
     - Transistor emitter → GND
     - 10kΩ: MOSFET gate to GND
     - MOSFET drain → Projector negative terminal
     - Projector positive → External 12V supply

7. **Focus Servo Connector**
   - Add: 3-pin header (Signal, VCC, GND)
   - Wire:
     - Signal → GPIO32 (PWM)
     - VCC → 5V rail
     - GND → GND

8. **I2C Bus Connector (To Raspberry Pi)**
   - Add: 4-pin header (VCC, GND, SDA, SCL)
   - Wire:
     - SDA → ESP32 GPIO21 (I2C SDA)
     - SCL → ESP32 GPIO22 (I2C SCL)
     - Add 4.7kΩ pull-up resistors on SDA and SCL to 3.3V
   - VCC → 3.3V (for reference, Pi provides pull-ups)
   - GND → GND

9. **Decoupling Capacitors**
   - Add: 100nF ceramic capacitors near ESP32 VCC pins
   - Add: 10µF electrolytic on 3.3V rail
   - Add: 10µF electrolytic on 5V rail

10. **Programming Header**
    - Add: 6-pin header for FTDI (TX, RX, GND, 3.3V, GPIO0, EN)
    - Wire to ESP32 UART0 and boot pins

**Schematic Review Checklist:**
- [ ] All power connections made (5V, 3.3V, GND)
- [ ] Decoupling capacitors on all IC power pins
- [ ] Pull-up/pull-down resistors where needed
- [ ] No floating inputs
- [ ] Correct polarity on polarized components
- [ ] Pin assignments match breadboard test (Story 1.1)

### 3. Design PCB Layout (PCB View)

**Board Dimensions:**
- Size: 100mm × 80mm (custom shape optional)
- Thickness: 1.6mm
- Copper: 1oz (standard)

**Component Placement:**

1. **Top Layer:**
   - ESP32 module (center)
   - Voltage regulator (near power input)
   - All connectors around edges:
     - Power input (left edge)
     - I2C connector (right edge)
     - OLED connectors (top edge, spaced for cable routing)
     - Servo connectors (bottom edge)
     - Projector connectors (left edge, near power)

2. **Component Grouping:**
   - Keep OLED traces short (minimize SPI noise)
   - Separate digital (ESP32, OLEDs) from power (MOSFET, servos)
   - Place bulk capacitors near connector inputs

**Routing Guidelines:**

1. **Power Traces:**
   - 5V rail: 20 mil (0.5mm) minimum
   - 3.3V rail: 20 mil minimum
   - GND: Use ground plane (copper pour) on bottom layer
   - Servo power (6V): 30 mil (higher current for servos)

2. **Signal Traces:**
   - Digital signals (GPIO, SPI, I2C): 10 mil minimum
   - Keep I2C traces away from SPI to minimize crosstalk
   - Route UART traces with ground return nearby

3. **Mounting Holes:**
   - 4× M3 mounting holes at corners (3.2mm diameter)
   - Clearance: 5mm from edge, 3mm from traces

4. **Copper Pours:**
   - Bottom layer: GND plane
   - Top layer: 3.3V or 5V pour (optional, avoid under ESP32 antenna area)
   - Thermal relief on through-hole pads

**Design Rule Check:**
- Minimum trace width: 10 mil
- Minimum clearance: 10 mil
- Via size: 0.8mm drill, 1.2mm pad
- No acute angles in traces (use 45° or curved)

### 4. Document Pin Assignments

Create `modules/head-ears/wiring.md`:

```markdown
# Head+Ears Module Wiring Diagram

**I2C Address:** 0x08

## ESP32 Pin Assignments

### Power
- 5V Input: Screw terminal J1 (from Base module)
- 3.3V Regulator: AMS1117-3.3
- GND: Common ground

### OLED Displays (SPI)
- SCK: GPIO18 (shared)
- MOSI: GPIO23 (shared)
- Left Eye (OLED1):
  - DC: GPIO21
  - CS: GPIO5
  - RST: GPIO16 (shared)
- Right Eye (OLED2):
  - DC: GPIO22
  - CS: GPIO17
  - RST: GPIO16 (shared)

### Ear Servos (UART, Feetech STS3215)
- Left Ear Bus: GPIO26 (UART1, half-duplex)
  - Servo 1 (ID=1): Base joint
  - Servo 2 (ID=2): Tip joint
- Right Ear Bus: GPIO27 (UART2, half-duplex)
  - Servo 3 (ID=3): Base joint
  - Servo 4 (ID=4): Tip joint
- Power: 6V rail (separate input J2), 2A minimum

### Projector Control
- Power Control: GPIO25 → Optocoupler → MOSFET (12V switching)
- Focus Servo: GPIO32 (PWM, 50Hz)

### I2C Bus (to Raspberry Pi)
- SDA: GPIO21 (with 4.7kΩ pull-up to 3.3V)
- SCL: GPIO22 (with 4.7kΩ pull-up to 3.3V)

### Programming
- UART0 TX: GPIO1
- UART0 RX: GPIO3
- Boot: GPIO0 (pull low for programming)
- Enable: EN (pull low to reset)

## Power Budget
- ESP32: 500mA @ 3.3V
- OLEDs: 100mA each @ 3.3V
- Ear Servos: 1.5A peak @ 6V
- Focus Servo: 500mA @ 5V
- Projector: External 12V supply (varies by model)
- **Total: 5V/1A, 6V/2A, 12V/varies**

## Connectors
- J1: Power input 5V (2-pin screw terminal)
- J2: Servo power 6V (2-pin screw terminal)
- J3: I2C to Pi (4-pin header: VCC, GND, SDA, SCL)
- J4-J5: OLED connectors (7-pin headers)
- J6-J9: Servo connectors (3-pin headers, 2.54mm pitch)
- J10: Projector power output (2-pin screw terminal)
- J11: Focus servo (3-pin header)
- J12: Programming header (6-pin, 2.54mm pitch)
```

### 5. Export Gerber Files for Manufacturing

**In Fritzing:**
1. Switch to PCB view
2. File → Export → for Production → Extended Gerber (RS-274X)
3. Save to: `~/olaf/modules/head-ears/hardware/gerbers/`

**Gerber Files Generated:**
- `head-ears-pcb.GTL` - Top copper layer
- `head-ears-pcb.GBL` - Bottom copper layer
- `head-ears-pcb.GTS` - Top soldermask
- `head-ears-pcb.GBS` - Bottom soldermask
- `head-ears-pcb.GTO` - Top silkscreen
- `head-ears-pcb.GBO` - Bottom silkscreen
- `head-ears-pcb.TXT` - Drill file
- `head-ears-pcb.GML` - Board outline

**Also Export:**
- File → Export → as Image → PCB (PNG) - for documentation
- File → Export → Bill of Materials - save as `BOM.csv`

### 6. Pre-Manufacturing Checklist

Create `modules/head-ears/hardware/pre-order-checklist.md`:

```markdown
# Pre-Order PCB Checklist

## Electrical Review
- [ ] All nets connected (no airwires)
- [ ] Power and ground connections verified
- [ ] Decoupling capacitors on all ICs
- [ ] Pull-up resistors on I2C lines
- [ ] No shorts between power rails (DRC pass)
- [ ] Pin assignments match firmware (cross-reference Story 1.1)

## Mechanical Review
- [ ] Mounting holes present and clearances correct
- [ ] Board dimensions fit enclosure design
- [ ] Connector locations allow cable routing
- [ ] Component heights don't exceed enclosure depth
- [ ] Silkscreen labels readable (component references, pin 1 markers)

## Manufacturing Review
- [ ] Trace widths meet minimums (10 mil for signal, 20+ mil for power)
- [ ] Clearances meet minimums (10 mil)
- [ ] Via sizes are manufacturable (0.8mm drill minimum)
- [ ] No acute angle traces
- [ ] Gerber files generated and checked

## Design Rule Check (DRC)
- [ ] Run Fritzing DRC: Routing → Design Rules Check
- [ ] Fix all errors and warnings
- [ ] Re-export Gerbers after fixes
```

Run through checklist before proceeding to Story 1.3.

---

## Testing & Validation

**Test 1: Schematic Review**
- Print schematic on paper
- Trace each net from source to destination
- Verify against breadboard wiring (Story 1.1)

**Test 2: PCB Layout Visual Inspection**
- Check component placement (no overlaps, adequate spacing)
- Verify connector orientation (pin 1 markers)
- Ensure mounting holes have clearance

**Test 3: Gerber File Verification**
- Use online Gerber viewer (e.g., https://www.pcbway.com/project/OnlineGerberViewer.html)
- Check all layers load correctly
- Verify board outline and drill holes
- Measure critical dimensions (mounting hole spacing)

**Test 4: Bill of Materials**
- Export BOM from Fritzing
- Cross-reference with components from Story 1.1
- Verify part numbers, quantities, and availability

---

## Troubleshooting

**Issue 1: ESP32 Part Not in Fritzing Library**
- **Solution:** Download community parts from Fritzing forums
  - Visit: https://forum.fritzing.org/
  - Search: "ESP32-WROOM-32"
  - Download `.fzpz` file and import: Parts → Import

**Issue 2: DRC Errors (Clearance Violations)**
- **Solution:**
  - Increase trace spacing (manual routing)
  - Use thinner traces for low-current signals
  - Re-route problematic areas
  - Check settings: Routing → Design Rules → Clearance (set to 10 mil)

**Issue 3: Ground Plane Not Connecting**
- **Solution:**
  - Verify ground plane is set to GND net
  - Use thermals on through-hole pads
  - Check for isolated copper islands (add vias or traces to connect)

**Issue 4: Gerber Export Fails or Files Missing**
- **Solution:**
  - Update Fritzing to latest version
  - Try: File → Export → for Production → ZIP file (includes all Gerbers)
  - Manually verify all required files present

**Issue 5: Mounting Hole Locations Don't Match Enclosure**
- **Solution:**
  - Create cardboard template of enclosure mounting points
  - Measure with calipers
  - Update PCB layout to match (may require component re-placement)

**Issue 6: Trace Width Calculator**
- **Solution:** Use external calculator for high-current traces
  - Visit: https://www.4pcb.com/trace-width-calculator.html
  - Input: current, copper thickness, temperature rise
  - Update trace widths in Fritzing accordingly

---

## Dependencies

**Before this story:**
- Story 1.1: Breadboard Head+Ears Components and Test Connectivity ✅
- Fritzing installed
- All component datasheets available

**After this story:**
- Story 1.3: Order and Receive Head+Ears PCB from Elecrow

---

## References

- [Fritzing Documentation](https://fritzing.org/learning/)
- [PCB Design Tutorial](https://learn.sparkfun.com/tutorials/using-eagle-schematic)
- [Trace Width Calculator](https://www.4pcb.com/trace-width-calculator.html)
- [Gerber File Format](https://www.ucamco.com/en/gerber)
- [ESP32 Pinout Reference](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/)
- [Elecrow PCB Specifications](https://www.elecrow.com/pcb-manufacturing.html)

---

## Notes

- **Fritzing Limitations:** Fritzing is beginner-friendly but less powerful than KiCAD. For complex designs (Base module), may need KiCAD upgrade.
- **Component Footprints:** Verify footprints match actual components (especially ESP32 module and connectors) using datasheets.
- **Prototyping:** First PCB order should be small quantity (5-10 boards) to allow design iteration.
- **Design Time:** Schematic: 2-3 hours, PCB layout: 3-4 hours, review: 1 hour.
- **Silkscreen:** Add component labels, pin numbers, polarity markers, module name, version number (v1.0).
- **Version Control:** Commit `.fzz` file and Gerbers to Git with descriptive message.
- **Testing Pads:** Consider adding test points (vias or pads) for critical signals (5V, 3.3V, GND, I2C lines) for multimeter probing.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
