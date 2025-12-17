# Story 1.9: Mount Projector with 3.7V Buck Converter

**Epic:** Epic 1 - Head+Ears Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 6-10 hours

---

## User Story

**As a** builder,
**I want** the floor projector mounted with proper power supply using a 3.7V buck converter,
**so that** the projector operates reliably from the robot's power system with correct voltage regulation.

---

## Acceptance Criteria

1. ✅ 3.7V buck converter selected and sourced (adjustable output, 1-2A capacity)
2. ✅ Buck converter mounting bracket fabricated (metal or 3D printed)
3. ✅ Input power connection designed (12V or 5V from Base power distribution)
4. ✅ Output voltage verified and adjusted to exactly 3.7V with multimeter
5. ✅ Projector power cable fabricated with proper connector
6. ✅ Buck converter mounted securely to Head+Ears module structure
7. ✅ Optocoupler circuit integrated for ESP32 on/off control
8. ✅ Thermal management verified (buck converter heat dissipation adequate)
9. ✅ Projector powers on/off reliably via ESP32 GPIO commands
10. ✅ Video feed from Pi (HDMI) displays correctly on floor surface

---

## Implementation Steps

### 1. Select and Source Buck Converter

**Specifications:**
- **Input Voltage:** 12V or 5V (depending on Base power distribution)
- **Output Voltage:** Adjustable, target 3.7V
- **Output Current:** 1-2A (check projector specifications)
- **Efficiency:** >85%
- **Features:** Adjustable potentiometer, short-circuit protection

**Recommended Options:**
```
Option 1: LM2596 DC-DC Buck Converter Module
- Input: 4.5V-40V
- Output: 1.25V-37V (adjustable)
- Current: 3A max
- Cost: ~$2-5
- Pros: Cheap, widely available, easy to adjust

Option 2: MP1584EN Buck Converter Module
- Input: 4.5V-28V
- Output: 0.8V-20V (adjustable)
- Current: 3A max
- Cost: ~$1-3
- Pros: Compact, efficient (>90%)

Option 3: XL4015 Buck Converter Module (if higher current needed)
- Input: 8V-36V
- Output: 1.25V-35V (adjustable)
- Current: 5A max
- Cost: ~$3-8
- Pros: Higher current capacity
```

### 2. Calculate Power Requirements

```bash
# Measure projector power consumption
# Connect projector to lab power supply set to 3.7V
# Measure current draw:
# - Idle (no video): typically 200-500mA
# - Active (displaying video): typically 500-1000mA
# - Peak (bright scenes): up to 1.5A

# Example calculation:
# Projector: 3.7V @ 1A = 3.7W
# Buck converter efficiency: 85%
# Input power required: 3.7W / 0.85 = 4.35W
# If input is 12V: 4.35W / 12V = 363mA from 12V rail
# If input is 5V: 4.35W / 5V = 870mA from 5V rail
```

**Decision:** Use 12V input if current draw < 500mA, otherwise use 5V input to reduce load on 12V rail.

### 3. Design Buck Converter Mounting Bracket

**Option A: Metal Fabrication**
```
Materials:
- Aluminum angle bracket or flat bar (1-2mm thickness)
- M3 screws and standoffs for buck converter mounting
- M4 screws for bracket attachment to module structure

Fabrication:
1. Cut aluminum to size (approx 40mm × 60mm)
2. Drill mounting holes for buck converter (check module dimensions)
3. Drill attachment holes for module structure
4. Deburr edges with file
5. Optional: Paint or anodize for corrosion resistance
```

**Option B: 3D Printed Bracket**
```
Design in OnShape:
- Base plate with buck converter mounting holes
- Attachment points for module structure
- Cable routing channels
- Ventilation slots for heat dissipation

Print settings:
- Material: PETG (better heat resistance than PLA)
- Layer height: 0.2mm
- Infill: 40-50% (structural strength)
- Perimeters: 3-4 (strong walls)
```

### 4. Fabricate Projector Power Cable

**Cable Assembly:**
```
Components needed:
- Wire: 22-24 AWG silicone wire (red + black)
- Connector: Match projector input (micro-USB, barrel jack, or custom)
- Heat shrink tubing
- Solder and soldering iron

Steps:
1. Identify projector power connector type
2. Cut wire to appropriate length (150-200mm)
3. Strip wire ends (5mm)
4. Solder red wire to buck converter V+ output
5. Solder black wire to buck converter GND output
6. Solder connector to wire ends
7. Apply heat shrink tubing over connections
8. Test continuity with multimeter
```

### 5. Adjust Buck Converter Output Voltage

```bash
# CRITICAL: Adjust voltage BEFORE connecting to projector

# Setup:
1. Connect buck converter input to lab power supply (12V or 5V)
2. Do NOT connect projector yet
3. Connect multimeter to buck converter output (V+ and GND)

# Adjustment:
1. Turn buck converter potentiometer counterclockwise (lowest voltage)
2. Power on input supply
3. Slowly turn potentiometer clockwise while monitoring voltage
4. Stop when multimeter reads exactly 3.70V
5. Mark potentiometer position with marker
6. Power off and verify voltage remains at 3.70V after power cycle
7. Apply nail polish or glue to potentiometer to lock setting

# Tolerance: ±0.05V (3.65V - 3.75V acceptable)
```

### 6. Integrate Optocoupler Control Circuit

**Wiring:**
```
ESP32 GPIO → Current-limiting resistor (220Ω) → Optocoupler LED+ → GND
Optocoupler Collector → Buck converter ENABLE pin (or input power line)
Optocoupler Emitter → GND

Note: Some buck converters don't have ENABLE pin.
Alternative: Use optocoupler to switch input power via MOSFET or relay.
```

**Circuit Diagram (ASCII):**
```
       ESP32 GPIO (3.3V)
            |
          [220Ω]
            |
    +-------+-------+
    |  Optocoupler  |
    |   (PC817)     |
    +-------+-------+
            |
    +-------+-------+
    |  MOSFET       |  (IRLZ44N or similar)
    |  Gate         |
    +-------+-------+
    |       |       |
   12V   Drain   Source
  Input    |       |
           +---Buck Converter Vin
                   Buck Converter GND → Common GND
```

### 7. Mount and Connect All Components

**Assembly Order:**
```
1. Mount buck converter to fabricated bracket
2. Attach bracket to Head+Ears module structure
3. Connect input power (12V or 5V from Base power distribution)
4. Connect output power cable to projector
5. Route cables neatly with cable ties
6. Test voltage at projector connector (should be 3.7V)
7. Connect ESP32 GPIO to optocoupler circuit
8. Test on/off control via I2C commands
```

### 8. Test Projector Operation

```bash
# Test 1: Manual Power On
1. Apply power to buck converter input
2. Verify 3.7V at projector connector
3. Projector should power on and display startup screen

# Test 2: ESP32 Control
1. Send I2C command to ESP32: PROJECTOR_ON
2. ESP32 GPIO should go HIGH, optocoupler switches, projector powers on
3. Send I2C command: PROJECTOR_OFF
4. ESP32 GPIO goes LOW, projector powers off

# Test 3: Video Display
1. Connect HDMI cable from Pi to projector
2. Enable Pi HDMI output
3. Display test pattern on projector
4. Verify image quality, focus, and position on floor
```

### 9. Verify Thermal Management

```bash
# Monitor buck converter temperature during operation
# Use IR thermometer or thermal camera

# Acceptable temperatures:
# - Ambient: 20-25°C
# - Buck converter IC: <80°C (under load)
# - Heatsink (if present): <70°C

# If temperatures exceed limits:
# - Add heatsink to buck converter IC
# - Improve airflow (ventilation holes in bracket)
# - Reduce input voltage (if possible)
# - Increase wire gauge to reduce resistance losses
```

---

## Testing & Validation

**Test 1: Output Voltage Accuracy**
```bash
# Equipment: Digital multimeter
# Procedure:
1. Power on buck converter
2. Measure output voltage at projector connector
3. Verify: 3.65V - 3.75V (target: 3.70V)
4. Load test: Connect projector and measure voltage under load
5. Verify voltage drop < 0.1V under full load
```

**Test 2: On/Off Control Reliability**
```bash
# Procedure:
1. Send PROJECTOR_ON command 50 times
2. Verify projector powers on each time
3. Send PROJECTOR_OFF command 50 times
4. Verify projector powers off each time
5. No failures = PASS
```

**Test 3: Video Quality**
```bash
# Procedure:
1. Display test pattern from Pi (checkerboard, color bars)
2. Verify no flickering, distortion, or color shifts
3. Adjust projector focus for sharp image
4. Measure brightness and contrast
5. Test in various ambient lighting conditions
```

**Test 4: Thermal Stability**
```bash
# Procedure:
1. Run projector continuously for 2 hours
2. Monitor buck converter temperature every 15 minutes
3. Verify temperature stabilizes below 80°C
4. Check for any hot spots or discoloration
```

**Test 5: Integration with ROS2**
```bash
# From Pi terminal:
ros2 topic pub /head_ears/projector std_msgs/String "data: 'ON'" --once
# Projector should power on

ros2 topic pub /head_ears/projector std_msgs/String "data: 'OFF'" --once
# Projector should power off

# Check video feed:
ros2 run rqt_image_view rqt_image_view
# Should see projector displaying Pi screen
```

---

## Troubleshooting

**Issue 1: Buck Converter Output Voltage Drifts**
- **Symptom:** Voltage changes over time or with load
- **Solution:**
  - Check input voltage stability (measure at buck converter input)
  - Re-adjust potentiometer and lock with glue
  - Add output capacitor (100-470µF) for stability
  - Replace buck converter if faulty

**Issue 2: Projector Doesn't Power On**
- **Symptom:** No startup screen, no LED indicators
- **Solution:**
  - Verify 3.7V at projector connector with multimeter
  - Check cable continuity (no breaks)
  - Verify projector polarity (+ and - correct)
  - Test projector with lab power supply directly
  - Check fuse inside projector (if accessible)

**Issue 3: Buck Converter Overheating**
- **Symptom:** IC temperature > 90°C, thermal shutdown
- **Solution:**
  - Add heatsink to buck converter IC
  - Improve ventilation (add cooling fan if needed)
  - Reduce input voltage to minimize power dissipation
  - Use higher efficiency buck converter (MP1584EN vs LM2596)
  - Check for short circuit on output

**Issue 4: Optocoupler Control Not Working**
- **Symptom:** Projector stays on/off regardless of GPIO state
- **Solution:**
  - Verify GPIO output with multimeter (should toggle 0V/3.3V)
  - Check current-limiting resistor (220Ω present?)
  - Test optocoupler with LED (should light up when GPIO HIGH)
  - Verify MOSFET gate voltage (should toggle)
  - Check MOSFET connections (Drain/Source/Gate correct?)

**Issue 5: Video Flickering or Distortion**
- **Symptom:** Image flickers, lines, or color shifts
- **Solution:**
  - Check HDMI cable quality (try different cable)
  - Verify buck converter output ripple (use oscilloscope)
  - Add output capacitor (100µF, low ESR) for ripple reduction
  - Check ground connections (common ground between Pi and projector)
  - Ensure buck converter switching frequency doesn't interfere with HDMI

**Issue 6: Projector Focus Servo Doesn't Move**
- **Symptom:** Linear servo for focus control not responding
- **Solution:**
  - This is handled in Story 1.6 (firmware), but verify:
  - Servo gets power (5V, not 3.7V from buck converter)
  - ESP32 PWM signal correct (50Hz, 1-2ms pulse width)
  - Servo mechanically free (no binding)

---

## Safety Warnings

**⚠️ ELECTRICAL SAFETY:**
- Always disconnect power before making connections
- Double-check polarity before connecting projector
- Use insulated tools when working with live circuits
- Never touch buck converter IC while powered (hot surface)

**⚠️ FIRE HAZARD:**
- Buck converter can overheat if improperly configured
- Monitor temperature during first power-on
- Use proper wire gauge (22-24 AWG for 1-2A)
- Secure all connections with heat shrink tubing

**⚠️ ESD PROTECTION:**
- Projector electronics are ESD-sensitive
- Ground yourself before handling projector
- Avoid touching connector pins

---

## Dependencies

**Before this story:**
- Story 1.4: Assemble and Test Head+Ears PCB ✅
- Story 1.5: Design and 3D Print Head+Ears Enclosure ✅
- Story 1.6: Develop Head+Ears ESP32 Firmware ✅ (optocoupler control implemented)

**After this story:**
- Story 1.8: Mount Head+Ears Module to Robot Frame
- Projector fully operational and integrated

---

## References

- [LM2596 Buck Converter Datasheet](https://www.ti.com/lit/ds/symlink/lm2596.pdf)
- [MP1584EN Buck Converter Datasheet](https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MP1584EN/document_id/388/)
- [Optocoupler PC817 Datasheet](https://www.sharpsma.com/webfm_send/1103)
- [MOSFET IRLZ44N Datasheet](https://www.infineon.com/dgdl/irlz44n.pdf?fileId=5546d462533600a40153567217c32725)
- [Buck Converter Output Ripple Reduction](https://www.ti.com/lit/an/slva630/slva630.pdf)

---

## Notes

- **Projector Power:** Most mini projectors use 3.7V (single Li-ion cell voltage)
- **Buck Converter Selection:** LM2596 is more common but less efficient than MP1584EN
- **Heat Dissipation:** Buck converter dissipates (Vin - Vout) × Iout × (1 - efficiency)
  - Example: (12V - 3.7V) × 1A × 0.15 = 1.25W heat generated
- **Potentiometer Locking:** Use nail polish or hot glue to prevent drift
- **Output Capacitor:** Add 100µF low-ESR capacitor near projector for stability
- **Switching Frequency:** Most buck converters operate at 50-500kHz
- **HDMI Interference:** If video has issues, add ferrite beads to HDMI cable

### Alternative: Pre-Made 3.7V Power Supply

If buck converter adjustment is problematic, consider:
- **Single-cell Li-ion battery holder** (3.7V nominal, 4.2V full charge)
  - Pros: No adjustment needed, stable voltage
  - Cons: Additional battery to manage, needs charging circuit
- **USB power bank with 3.7V output** (rare, most are 5V)
- **LD1117-3.3 LDO regulator** (if projector accepts 3.3V instead of 3.7V)
  - Pros: Simple, no switching noise
  - Cons: Low efficiency, heat dissipation

### Future Enhancements (Phase 2)

- **Auto-focus feedback:** Use distance sensor to auto-adjust focus based on floor distance
- **Brightness control:** PWM dimming for projector LED
- **Thermal monitoring:** Add temperature sensor to buck converter
- **Power monitoring:** Measure projector current draw for diagnostics

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
