# Story 1.4: Assemble and Test Head+Ears PCB

**Epic:** Epic 1 - Head+Ears Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** all components soldered to Head+Ears PCB and tested for functionality,
**so that** I have a working circuit board ready for firmware development.

---

## Acceptance Criteria

1. ✅ All components soldered: ESP32 module, connectors for OLEDs/servos/projector, optocoupler, passive components
2. ✅ Visual inspection confirms proper solder joints (no cold joints, bridges)
3. ✅ Power-on test confirms correct voltage rails (5V, 3.3V) with no excessive current draw
4. ✅ I2C address configured as 0x08 in firmware (or hardware jumpers if applicable)
5. ✅ Component-level tests pass: OLEDs display test pattern, servos respond to commands, projector power circuit switches, focus servo moves
6. ✅ PCB mounted on temporary test fixture for firmware development

---

## Implementation Steps

### 1. Gather Components and Tools

**Components (from BOM):**
- ESP32-WROOM-32 module (or pre-soldered ESP32 dev board)
- AMS1117-3.3 voltage regulator
- PC817 optocoupler
- IRLZ44N MOSFET (N-channel, logic-level)
- Resistors: 220Ω, 10kΩ, 4.7kΩ (×4 for I2C pull-ups)
- Capacitors: 10µF electrolytic (×4), 100nF ceramic (×6), 1000µF electrolytic (servo power)
- Connectors: 2-pin screw terminals (×3), 3-pin headers (×5), 4-pin header, 7-pin headers (×2)
- 2× OLED displays (128×64, SPI)
- 4× Feetech STS3215 servos (ear actuators)
- 1× Standard servo (focus control)

**Tools:**
- Soldering iron (temperature-controlled, 350°C for lead-free)
- Solder (0.8mm diameter, 60/40 or lead-free)
- Flux pen or paste
- Solder wick (desoldering braid)
- Tweezers (fine-tip)
- Multimeter
- Magnifying glass or microscope
- Isopropyl alcohol (90%+ for cleaning)
- Helping hands or PCB holder

### 2. Solder Passive Components First

**Order of Assembly:**

1. **Smallest to Largest:** Resistors → Capacitors → ICs → Connectors
2. **Low Profile First:** Prevents components from blocking access

**Start with Resistors:**
```bash
# Resistor placement (all SMD or through-hole depending on design):
# R1: 220Ω (optocoupler LED current limiting)
# R2: 10kΩ (MOSFET gate pull-down)
# R3-R6: 4.7kΩ (I2C pull-ups: 2× on SDA, 2× on SCL)

# Soldering technique:
# 1. Insert component leads through PCB
# 2. Bend leads slightly on back to hold in place
# 3. Solder one lead
# 4. Check alignment, adjust if needed
# 5. Solder second lead
# 6. Trim excess lead with flush cutters
```

**Capacitors:**
```bash
# Ceramic capacitors (100nF): C1-C6 near ESP32 and regulator VCC pins
# Electrolytic capacitors (observe polarity!):
#   C7-C8: 10µF (regulator input/output)
#   C9-C10: 10µF (power rail decoupling)
#   C11: 1000µF (servo power supply, large physical size)

# POLARITY CHECK: Electrolytic capacitors have negative lead marked
# PCB has "+" symbol, negative goes to GND
```

### 3. Solder ICs and Active Components

**Voltage Regulator (AMS1117-3.3):**
```bash
# 3-pin package (IN, GND, OUT)
# Check datasheet for pin orientation
# Apply heat to pad and component lead simultaneously
# Use flux for better solder flow
```

**Optocoupler (PC817):**
```bash
# 4-pin DIP package
# Check orientation: Pin 1 marked with dot or notch
# Match to PCB silkscreen
```

**MOSFET (IRLZ44N):**
```bash
# TO-220 package: Gate, Drain, Source
# Check pinout in datasheet
# May need heatsink if switching high current (not necessary for projector control)
```

**ESP32 Module:**
```bash
# If using pre-soldered dev board: solder headers
# If using bare ESP32-WROOM-32:
#   - Most challenging component (small pitch)
#   - Use flux liberally
#   - Solder one corner pin first (alignment)
#   - Solder opposite corner
#   - Complete remaining pins
#   - Check for bridges with magnifying glass
#   - Use solder wick to remove bridges
```

### 4. Solder Connectors

**Screw Terminals (Power Input, Projector Output):**
```bash
# Large through-hole pads
# Ensure terminals are flush against PCB before soldering
# Orientation matters: wire entry side should match silkscreen
```

**Pin Headers (OLEDs, Servos, I2C, Programming):**
```bash
# Can solder headers straight, or use sockets for removable connections
# For OLEDs: 7-pin headers (or solder OLED displays directly)
# For servos: 3-pin headers (standard servo connector pitch: 2.54mm)
# For I2C: 4-pin header (to Raspberry Pi)
# For programming: 6-pin header (FTDI or USB-Serial adapter)
```

### 5. Post-Soldering Inspection and Cleaning

**Visual Inspection:**
```bash
# Use magnifying glass or microscope (10× magnification)
# Check for:
#   - Cold solder joints (dull, grainy appearance) → Re-heat
#   - Solder bridges (shorts between adjacent pads) → Remove with wick
#   - Missing solder joints → Re-solder
#   - Component orientation (polarized parts: capacitors, ICs)
```

**Cleaning:**
```bash
# Remove flux residue with isopropyl alcohol (IPA)
# Use soft brush or cotton swabs
# Allow PCB to dry completely before power-on test
```

### 6. Power-On Test (No Load)

**CRITICAL SAFETY: Do this BEFORE connecting expensive components (ESP32)**

**Test Setup:**
```bash
# 1. Bench power supply set to 5V with current limit (500mA)
# 2. Connect to 5V power input screw terminal
# 3. Turn on power supply
# 4. Observe current draw:
#    - Initial: <100mA (quiescent current for regulator)
#    - If >200mA immediately, turn off and check for shorts
```

**Voltage Rail Check:**
```bash
# Use multimeter in DC voltage mode
# Measure between GND and:
#   - 5V rail: Should read 4.9-5.1V
#   - 3.3V rail (regulator output): Should read 3.2-3.4V
# If voltages incorrect:
#   - Check regulator orientation
#   - Check for shorts with continuity mode
#   - Verify input capacitors installed correctly
```

### 7. Connect and Test ESP32

**Upload Blink Test:**
```bash
cd ~/olaf/modules/head-ears/firmware/breadboard_test
# Modify code to use PCB pinout (if different from breadboard)

# Connect FTDI programmer to programming header
# Upload blink code (toggle GPIO2, ESP32 onboard LED)
pio run --target upload

# Expected: ESP32 LED blinks
# If upload fails:
#   - Check FTDI connections (TX→RX, RX→TX)
#   - Hold GPIO0 low (to GND) during upload for boot mode
#   - Press RESET button after upload starts
```

### 8. Test Individual Subsystems

**Test OLEDs:**
```bash
# Connect OLEDs to 7-pin headers
# Upload OLED test code from Story 1.1
# Expected: Both displays show graphics, blink animation

# If one or both don't work:
#   - Check SPI pin connections (SCK, MOSI, DC, CS, RST)
#   - Verify OLED power (3.3V, GND)
#   - Try different OLED library or display type setting
```

**Test Servo Connectors:**
```bash
# Connect test servo to focus servo header (GPIO32)
# Upload servo sweep code
# Expected: Servo moves smoothly through 0-180°

# Test ear servo buses:
#   - Connect one Feetech servo to left ear bus (GPIO26)
#   - Upload UART servo test code
#   - Expected: Servo responds to position commands
```

**Test Projector Power Circuit:**
```bash
# Connect LED + resistor to projector power output (simulate load)
# Upload GPIO toggle code for GPIO25
# Expected: LED turns on/off with GPIO state
# Measure MOSFET drain voltage with multimeter (should switch)
```

**Test I2C Communication:**
```bash
# On Raspberry Pi (if available), or use I2C master device
# Connect I2C header to Pi
# Run I2C scan:
i2cdetect -y 1

# Expected: Address 0x08 shows up (once ESP32 firmware implements I2C slave)
# If not detected:
#   - Check pull-up resistors (4.7kΩ on SDA, SCL)
#   - Verify ESP32 I2C slave code running
#   - Check wiring (SDA GPIO21, SCL GPIO22)
```

### 9. Create Test Fixture

**Temporary Mounting:**
```bash
# Options:
# 1. Acrylic or wood board with standoffs (M3 screws, 10mm spacers)
# 2. 3D printed test jig
# 3. Breadboard mounting plate

# Fixture should:
#   - Securely hold PCB
#   - Allow access to all connectors
#   - Provide cable management for OLED, servos, power
#   - Not short any exposed traces on PCB bottom
```

---

## Testing & Validation

**Test 1: Solder Joint Quality**
```bash
# Visual: All joints shiny, concave fillet shape
# Mechanical: Gently tug component leads (should not move)
# Electrical: Continuity between component pin and PCB pad
```

**Test 2: Power Consumption**
```bash
# Measure current at 5V input with multimeter
# ESP32 idle: ~80-150mA
# ESP32 + OLEDs: ~250-350mA
# ESP32 + OLEDs + servos idle: ~300-400mA
# If current >500mA with no servos moving, investigate short
```

**Test 3: Component Functionality**
```bash
# Checklist:
# [ ] ESP32 boots and runs code
# [ ] 3.3V regulator outputs stable voltage
# [ ] Both OLEDs display graphics
# [ ] Servo connectors deliver PWM signals
# [ ] Projector power circuit switches
# [ ] I2C bus detected by Pi (once firmware ready)
```

**Test 4: Thermal Check**
```bash
# Run all components for 15 minutes
# Touch components to check temperature:
#   - Voltage regulator: Warm (40-60°C acceptable)
#   - ESP32: Warm (30-50°C acceptable)
#   - MOSFETs: Cool to warm (<50°C)
# If any component >80°C, turn off and investigate
```

---

## Troubleshooting

**Issue 1: Short Circuit on Power-On**
- **Symptom:** Current limit trips immediately, voltage sags to 0V
- **Solution:**
  - Turn off power
  - Check multimeter continuity between 5V and GND (should be open)
  - Inspect for solder bridges between power traces
  - Check capacitor polarity (reversed electrolytic can short)
  - Remove components one-by-one to isolate short

**Issue 2: 3.3V Regulator Not Outputting Voltage**
- **Symptom:** 5V present, but 3.3V rail reads 0V or very low
- **Solution:**
  - Check regulator orientation (compare to datasheet)
  - Verify input and output capacitors installed (10µF each)
  - Check for solder bridge on regulator pins
  - Test regulator with multimeter in diode mode (should show voltage drop)
  - Replace regulator if defective

**Issue 3: ESP32 Won't Upload Firmware**
- **Symptom:** Serial port not detected, or upload fails
- **Solution:**
  - Check USB cable (must be data cable, not charge-only)
  - Install ESP32 USB-to-Serial drivers (CP2102 or CH340)
  - Hold GPIO0 to GND while pressing RESET (enters bootloader mode)
  - Verify TX/RX connections (TX→RX, RX→TX, crossed)
  - Try lower baud rate: 115200 instead of 460800

**Issue 4: OLEDs Don't Display**
- **Symptom:** Blank screens on one or both OLEDs
- **Solution:**
  - Check OLED power (3.3V, GND with multimeter)
  - Verify SPI connections (use logic analyzer if available)
  - Check DC, CS, RST pins (unique per OLED)
  - Try different OLED library (SSD1306 vs SH1106)
  - Test OLED on breadboard to verify not defective
  - Check solder joints on ESP32 SPI pins (GPIO18, GPIO23)

**Issue 5: Servos Jitter or Don't Move**
- **Symptom:** Servos vibrate, move erratically, or don't respond
- **Solution:**
  - Check servo power supply (should be 6V for Feetech, 5V for focus servo)
  - Verify common ground between ESP32 and servo power supply
  - Add bulk capacitor (1000µF) to servo power rail
  - Check signal connection (GPIO26/27 for ear servos, GPIO32 for focus)
  - Test servo with external servo tester to verify not defective
  - For Feetech: verify UART baud rate and servo ID configured

**Issue 6: I2C Not Detected by Raspberry Pi**
- **Symptom:** `i2cdetect` doesn't show address 0x08
- **Solution:**
  - Check ESP32 I2C slave firmware is running
  - Verify I2C pins: SDA=GPIO21, SCL=GPIO22
  - Check pull-up resistors (4.7kΩ to 3.3V)
  - Measure SDA/SCL voltage with multimeter (should be ~3.3V when idle)
  - Try I2C scanner code on ESP32 (in master mode) to verify I2C working
  - Check for solder bridges or broken traces on I2C header

---

## Dependencies

**Before this story:**
- Story 1.3: Order and Receive Head+Ears PCB from Elecrow ✅
- All components purchased (per BOM)
- Soldering tools and workspace ready

**After this story:**
- Story 1.5: Design and 3D Print Head+Ears Enclosure
- Story 1.6: Develop Head+Ears ESP32 Firmware

---

## References

- [PCB Soldering Tutorial](https://learn.adafruit.com/adafruit-guide-excellent-soldering)
- [SMD Soldering Techniques](https://www.sparkfun.com/tutorials/36)
- [ESP32 Hardware Design Guidelines](https://www.espressif.com/sites/default/files/documentation/esp32_hardware_design_guidelines_en.pdf)
- [Common PCB Assembly Mistakes](https://www.digikey.com/en/maker/blogs/common-pcb-assembly-mistakes)

---

## Notes

- **Soldering Time:** Allow 3-4 hours for careful assembly, 1-2 hours for testing
- **Component Substitutions:** If exact parts unavailable, check datasheets for pin-compatible replacements
- **Rework:** Keep solder wick and desoldering pump handy for mistakes
- **ESP32 Module Options:** Pre-soldered dev boards easier than bare modules (but larger)
- **Connector Choice:** Consider using JST connectors instead of screw terminals for cleaner wiring
- **Test Points:** Add wire loops to critical nodes (5V, 3.3V, GND, I2C) for easy multimeter probing
- **Documentation:** Take photos at each assembly stage for troubleshooting and replication
- **Quality Control:** Don't rush assembly. Better to spend extra hour on careful soldering than debug issues later

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
