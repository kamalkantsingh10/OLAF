# Story 4.4: Design Base Custom PCB in Fritzing

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 8-12 hours

---

## User Story

**As a** builder,
**I want** a custom PCB that integrates ESP32, MPU6050 IMU, power distribution, and connectors for ODrive and modules,
**so that** the Base module has a clean, reliable electrical system without breadboard wiring.

---

## Acceptance Criteria

1. ✅ Fritzing schematic created with all Base module components
2. ✅ PCB layout designed with proper trace widths for high-current paths
3. ✅ ESP32 footprint added with all GPIO connections
4. ✅ MPU6050 IMU footprint added (I2C connection)
5. ✅ Power distribution section: XT60 input, fuse holders, screw terminals for buck converters
6. ✅ Connectors: I2C to Pi, ODrive UART, emergency stop, voltage monitoring
7. ✅ Reverse polarity protection circuit (MOSFET or diode)
8. ✅ Copper pour for ground plane (2oz copper)
9. ✅ Gerber files exported and verified
10. ✅ Bill of Materials (BOM) created with all component part numbers

---

## Implementation Steps

### 1. Create Fritzing Project

```bash
cd ~/olaf/modules/base/pcb
mkdir -p base-pcb
cd base-pcb

# Launch Fritzing
fritzing base-pcb.fzz
```

### 2. Add Components to Schematic

**Major components:**
- ESP32-WROOM-32 module
- MPU6050 IMU breakout
- IRF4905 P-channel MOSFET (reverse polarity protection)
- 3× Fuse holders (15A, 5A, 2A)
- XT60 connectors (×2): Battery input, ODrive output
- Screw terminals (5mm pitch): Buck converter connections
- JST-XH connectors (4-pin): I2C to Pi
- 2.54mm headers: UART to ODrive, emergency stop
- Voltage divider resistors: 100kΩ, 10kΩ (1%)
- Capacitors: 100µF (×3), 0.1µF (×5)
- LEDs: Power indicator, status (optional)

### 3. Design Schematic (Key Sections)

**Power input section:**
```
Battery XT60 (+) → IRF4905 Drain
IRF4905 Source → Main Bus (+)
IRF4905 Gate → Battery (-)
Main Bus (+) → Fuse Holder 1 (15A) → ODrive XT60 (+)
Main Bus (+) → Fuse Holder 2 (5A) → Buck Conv 1 (+)
Main Bus (+) → Fuse Holder 3 (2A) → Buck Conv 2 (+)
Battery XT60 (-) → Main Ground
```

**ESP32 section:**
```
ESP32 VIN → 5V from buck converter (via LDO)
ESP32 GND → Ground plane
ESP32 GPIO 21 (SDA) → I2C bus (with 4.7kΩ pull-up)
ESP32 GPIO 22 (SCL) → I2C bus (with 4.7kΩ pull-up)
ESP32 GPIO 16 (TX) → ODrive UART RX
ESP32 GPIO 17 (RX) → ODrive UART TX
ESP32 GPIO 34 (ADC) → Voltage divider (battery monitoring)
ESP32 EN → 10kΩ pull-up to 3.3V
```

**MPU6050 section:**
```
MPU6050 VCC → ESP32 3.3V
MPU6050 GND → Ground
MPU6050 SDA → I2C bus (shared with ESP32)
MPU6050 SCL → I2C bus
```

**I2C connector to Pi:**
```
4-pin JST-XH:
  Pin 1: SDA (to Pi GPIO 2)
  Pin 2: SCL (to Pi GPIO 3)
  Pin 3: 3.3V
  Pin 4: GND
```

### 4. Design PCB Layout

**Board dimensions:**
- Size: 120mm × 100mm (fits standard enclosure)
- Layers: 2-layer (top and bottom)
- Copper weight: 2oz (thick traces for high current)

**Component placement:**
```
Top view layout:

┌─────────────────────────────────────────┐
│  XT60 IN    FUSES (×3)      XT60 OUT    │
│  Battery    15A 5A 2A       ODrive      │
│                                          │
│  MOSFET     ┌─────────────┐             │
│  (reverse   │  ESP32-32   │  MPU6050    │
│  polarity)  │  Module     │  IMU        │
│             └─────────────┘             │
│                                          │
│  Buck Conv  Buck Conv     I2C JST       │
│  Terminals  Terminals     Connector     │
│  (5V)       (12V)                       │
│                                          │
│  UART       E-Stop        Status LED    │
│  Header     Header                      │
└─────────────────────────────────────────┘

Ground plane covers remaining area (bottom layer)
```

**Trace width guidelines:**
```
High current (>5A):     3mm (120 mil) or thicker
Medium current (1-5A):  1.5mm (60 mil)
Low current (<1A):      0.5mm (20 mil)
Signal traces:          0.3mm (12 mil)
```

**Critical routing:**
- Battery (+) to ODrive: 3mm trace, shortest path
- Ground return: 3mm trace or copper pour
- I2C traces: Keep short (<50mm), route together (matched length)
- UART traces: Keep short, add series resistors (100Ω) for protection
- Voltage divider: Route ADC trace away from noisy signals

### 5. Add Copper Pours and Ground Plane

```bash
# In Fritzing PCB view:
# 1. Select "Copper Fill" tool
# 2. Draw polygon around entire board perimeter
# 3. Set "Ground Fill" in properties
# 4. Repeat for top layer (partial pour around components)
# 5. Bottom layer: full ground plane

# Thermal reliefs:
#   - Ground pins connect via thermal relief (X pattern)
#   - Prevents excessive heat sink during soldering
```

### 6. Add Mounting Holes

```bash
# 4× M3 mounting holes at corners
# Position: 5mm from edges
# Hole diameter: 3.2mm (clearance for M3 screw)
# Copper clearance: 2mm (prevent shorts)
```

### 7. Add Silkscreen Labels

```bash
# Label all connectors:
#   - "BAT IN XT60" (battery input)
#   - "ODrive XT60" (motor controller output)
#   - "5V Buck" (screw terminal)
#   - "12V Buck" (screw terminal)
#   - "I2C to Pi" (JST connector)
#   - "UART ODrive" (header)
#   - "E-STOP" (header)

# Label pin functions:
#   - "+" and "-" on screw terminals
#   - Pin numbers on headers
#   - GPIO numbers on ESP32

# Add version and date:
#   "OLAF Base PCB v1.0"
#   "2025-12-17"
```

### 8. Run Design Rule Check

```bash
# In Fritzing:
# Routing → Design Rules Check (DRC)

# Check for:
#   - Trace-to-trace spacing: Minimum 0.3mm
#   - Trace-to-edge spacing: Minimum 1mm
#   - Drill hole size: Minimum 0.3mm
#   - Trace width: Minimum 0.3mm (except high current)
#   - Copper pour clearance: 0.5mm

# Fix any violations before export
```

### 9. Export Gerber Files

```bash
# In Fritzing:
# File → Export → for Production → Extended Gerber

# Files generated:
#   - base-pcb.gbl (bottom copper)
#   - base-pcb.gtl (top copper)
#   - base-pcb.gbs (bottom soldermask)
#   - base-pcb.gts (top soldermask)
#   - base-pcb.gbo (bottom silkscreen)
#   - base-pcb.gto (top silkscreen)
#   - base-pcb.gm1 (board outline)
#   - base-pcb.txt (drill file)

# Verify Gerber files with online viewer:
# https://www.pcbway.com/project/OnlineGerberViewer.html
```

### 10. Create Bill of Materials (BOM)

**Create `modules/base/pcb/base-pcb-bom.csv`:**

```csv
Designator,Component,Value,Package,Quantity,Source,Part Number,Cost
U1,ESP32-WROOM-32,ESP32,Module,1,Amazon,ESP32-DevKitC,$8
U2,MPU6050,IMU,Breakout,1,Amazon,GY-521,$3
Q1,IRF4905,P-MOSFET,TO-220,1,Digikey,IRF4905PBF,$1.50
F1,Fuse Holder,15A,5mm,1,Amazon,Generic,$1
F2,Fuse Holder,5A,5mm,1,Amazon,Generic,$1
F3,Fuse Holder,2A,5mm,1,Amazon,Generic,$1
J1,XT60,Connector,XT60,1,Amazon,XT60 Male,$2
J2,XT60,Connector,XT60,1,Amazon,XT60 Female,$2
J3,Screw Terminal,2-pos,5mm,2,Amazon,KF128,$1
J4,JST-XH,4-pin,2.54mm,1,Amazon,JST-XH-4,$0.50
P1,Pin Header,6-pin,2.54mm,1,Amazon,Male Header,$0.50
P2,Pin Header,2-pin,2.54mm,1,Amazon,Male Header,$0.50
R1,Resistor,100kΩ 1%,1/4W,1,Digikey,MFR-25FBF,$0.10
R2,Resistor,10kΩ 1%,1/4W,1,Digikey,MFR-25FBF,$0.10
R3,Resistor,4.7kΩ,1/4W,2,Digikey,Generic,$0.10
C1-C3,Capacitor,100µF 50V,Electrolytic,3,Digikey,Generic,$0.50
C4-C8,Capacitor,0.1µF 50V,Ceramic,5,Digikey,Generic,$0.10
LED1,LED,Red,3mm,1,Amazon,Generic,$0.10
─────────────────────────────────────────────────────────────
Total:,,,,,,$22.50
```

---

## Testing & Validation

**Test 1: Gerber File Verification**
```bash
# Upload Gerbers to online viewer
# Check all layers align correctly
# Verify no missing traces or pads
```

**Test 2: Trace Width Calculation**
```bash
# Use trace width calculator:
# https://www.4pcb.com/trace-width-calculator.html
# Input: 2oz copper, 10A current, 10°C rise
# Result: Minimum 3mm trace width (matches design)
```

**Test 3: Component Footprint Verification**
```bash
# Print PCB at 1:1 scale
# Place physical components on paper
# Verify all footprints match actual parts
```

---

## Troubleshooting

**Issue 1: Trace Width Calculator Shows Insufficient Width**
- **Solution:** Increase trace width or reduce current, use thicker copper (2oz confirmed)

**Issue 2: Components Don't Fit on PCB**
- **Solution:** Increase board size or use smaller components, consider double-sided assembly

**Issue 3: Gerber Export Fails**
- **Solution:** Check Fritzing version, verify all components have proper footprints

**Issue 4: DRC Shows Errors**
- **Solution:** Fix violations one by one, adjust trace routing or component placement

---

## Dependencies

**Before this story:**
- Story 4.3: Design Power Distribution System ✅

**After this story:**
- Story 4.5: Order and Receive Base PCB from Elecrow

---

## References

- [Fritzing PCB Design Tutorial](https://fritzing.org/learning/tutorials)
- [PCB Trace Width Calculator](https://www.4pcb.com/trace-width-calculator.html)
- [Gerber File Format](https://en.wikipedia.org/wiki/Gerber_format)

---

## Notes

- **2oz Copper:** Essential for high-current traces (10A+). Standard PCBs use 1oz copper.
- **Ground Plane:** Reduces noise, improves heat dissipation, simplifies routing.
- **Component Placement:** Keep high-current components near edges for heat dissipation.
- **I2C Pull-ups:** 4.7kΩ standard value for 100kHz-400kHz I2C.
- **Future Enhancements:** Add status LEDs for each voltage rail, add test points for debugging, consider 4-layer PCB for better EMI performance.

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
