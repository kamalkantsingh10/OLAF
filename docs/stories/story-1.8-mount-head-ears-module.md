# Story 1.8: Mount Head+Ears Module to Robot Frame

**Epic:** Epic 1 - Head+Ears Module Build
**Status:** Not Started
**Priority:** Medium
**Estimated Effort:** 3-4 hours

---

## User Story

**As a** builder,
**I want** Head+Ears module physically mounted to robot frame with all cables connected,
**so that** the module is integrated into the robot structure.

---

## Acceptance Criteria

1. ✅ Module mounted securely to robot frame using 3D printed brackets and hardware
2. ✅ OLED displays positioned correctly for "eyes" appearance
3. ✅ Ear servos mounted with full range of motion (no mechanical interference)
4. ✅ Floor projector mounted and aimed at floor surface
5. ✅ OAK-D-Pro camera mounted with clear field of view
6. ✅ I2C cable connected to Pi (appropriate length, strain relief)
7. ✅ Power cables connected to Base module power distribution
8. ✅ Cable management ensures no loose wires interfering with movement
9. ✅ Module responds to ROS2 commands when powered: `ros2 topic pub` tests pass

---

## Implementation Steps

### 1. Prepare Mounting Hardware

**Hardware Checklist:**
```bash
# Screws and Standoffs:
- M3 screws: 10mm (×4 for PCB), 15mm (×4 for enclosure cover)
- M4 or M5 screws: 20mm (×2 for neck connection)
- M3 standoffs: 10mm (×4 for PCB mounting)
- M2 screws: 6mm (×8 for OLED mounts)
- M2.5 screws: 8mm (×4 for camera)

# Tools:
- Screwdrivers: Phillips #1, flathead 2mm
- Hex keys: 2mm, 2.5mm, 3mm
- Wire cutters and strippers
- Zip ties (various sizes)
- Heat shrink tubing (optional)
- Multimeter for continuity checks
```

### 2. Mount PCB in Enclosure

```bash
# Steps:
# 1. Insert M3 standoffs into base plate mounting holes
# 2. Place PCB on standoffs, align mounting holes
# 3. Secure with M3 screws from top
# 4. Verify PCB is level and secure (no wobble)
# 5. Check clearance between PCB bottom and enclosure base (no shorts)
```

### 3. Install OLED Displays

```bash
# 1. Connect OLED ribbon cables or wires to PCB (7-pin connectors)
# 2. Place OLEDs in bezels (front of enclosure)
# 3. Secure with M2 screws or snap-fit clips
# 4. Verify OLEDs are aligned (same height, parallel)
# 5. Power on, test display: should show test pattern
```

### 4. Mount Ear Servos and Structures

```bash
# Left Ear Assembly:
# 1. Attach base servo to left servo bracket (enclosure side wall)
# 2. Secure bracket to enclosure with screws
# 3. Connect servo horn to ear base structure
# 4. Attach tip servo to ear base structure
# 5. Connect servo horn to ear tip structure
# 6. Connect servo cables to PCB (left ear bus connector)
# 7. Test range of motion: command servos through full range, check for interference

# Right Ear Assembly:
# 1. Repeat steps for right ear
# 2. Ensure symmetry with left ear (mirror positions)

# Final Check:
# - Both ears move smoothly without binding
# - No contact with enclosure walls
# - Cables have slack for movement (not taut)
```

### 5. Mount OAK-D-Pro Camera

```bash
# 1. Position camera on top of enclosure (forward-facing mount)
# 2. Secure with M2.5 screws or camera-specific bracket
# 3. Ensure camera is level (use bubble level or visual alignment)
# 4. Camera field of view: aimed forward, slightly downward (5-10°)
# 5. Route USB cable from camera through enclosure (rear exit)
# 6. Add strain relief at cable exit point (zip tie or grommet)
```

### 6. Mount Floor Projector

```bash
# 1. Install projector in bottom mount bracket
# 2. Angle projector 45-60° downward (projects onto floor ~30cm in front)
# 3. Secure projector with screws or custom clamp (depends on projector model)
# 4. Connect focus servo linkage to projector focus mechanism
# 5. Test focus servo: verify smooth adjustment without binding
# 6. Route HDMI cable from rear of enclosure (runs to Pi in Torso)
# 7. Connect power cable from projector power circuit (PCB GPIO25 output)
```

### 7. Cable Routing and Management

**Power Cables:**
```bash
# From Base module power distribution:
# - 5V power: 18-20 AWG wire, 1m length (estimated, measure actual)
# - GND: Same gauge as 5V
# - Route along robot frame edge, zip tie every 10cm
# - Connect to Head+Ears PCB power input screw terminal
# - Verify polarity (5V to +, GND to -)
```

**I2C Bus Cable:**
```bash
# From Raspberry Pi (in Torso) to Head+Ears:
# - 4-wire cable: VCC (3.3V), GND, SDA, SCL
# - Use shielded twisted pair or ribbon cable
# - Length: ~50cm (measure along cable route)
# - Route separately from power cables (minimize EMI)
# - Connect to Head+Ears PCB I2C header (4-pin)
# - Label wires for future troubleshooting
```

**USB Cable (Camera to Pi):**
```bash
# - OAK-D-Pro to Raspberry Pi: USB 3.0 cable, ~60cm
# - Route along same path as I2C cable
# - Secure with zip ties, ensure no sharp bends
# - Connect to Pi USB 3.0 port (blue port)
```

**HDMI Cable (Pi to Projector):**
```bash
# - Raspberry Pi to floor projector: HDMI micro to standard, ~60cm
# - Route from Torso to Head+Ears (alongside other cables)
# - Secure cable near projector to prevent strain on connector
```

**Servo and OLED Cables (Internal):**
```bash
# - Already connected to PCB (done in Steps 3-4)
# - Use cable channels in enclosure to organize
# - Ensure no cables block ventilation holes (ESP32 cooling)
# - Secure with small zip ties or hot glue dots
```

### 8. Attach Top Cover

```bash
# 1. Route all external cables through rear exit before closing
# 2. Place top cover on enclosure
# 3. Align with mounting holes or snap-fit tabs
# 4. Secure with M3 screws (×4 at corners)
# 5. Verify cover is flush, no gaps
# 6. Check camera is still visible through top cutout
```

### 9. Connect Module to Neck/Frame

**Mounting Points:**
```bash
# Rear of Head+Ears enclosure has 2× mounting holes (M4 or M5)
# These connect to top of Neck module (Story 2.9)

# Temporary Mounting (before Neck module complete):
# - Use temporary stand or bracket to hold Head+Ears upright
# - Clamp to workbench or use tripod mount
# - This allows testing before full robot assembly
```

### 10. Power-On and Functional Test

**Pre-Power Checklist:**
```bash
# Visual inspection:
# [ ] No loose wires or components
# [ ] No shorts visible (use flashlight, inspect solder joints)
# [ ] All connectors firmly seated
# [ ] Cable routing clear of moving parts (ears)
# [ ] Enclosure cover secure
```

**Power-On Sequence:**
```bash
# 1. Connect 5V power cable to bench supply (not robot yet)
# 2. Set current limit: 2A
# 3. Turn on power
# 4. Observe current draw: should be <1A (idle)
# 5. Check ESP32 boots (LED blink or serial output)
# 6. Verify OLEDs light up (may show test pattern or blank)
```

**ROS2 Communication Test:**
```bash
# On Raspberry Pi:
source ~/olaf/ros2/install/setup.bash

# Launch driver node
ros2 run head_ears_driver head_ears_driver_node

# In another terminal, test commands:
# Eyes
ros2 topic pub --once /head_ears/eyes std_msgs/msg/String "{data: 'open'}"
# Expected: OLEDs display open eyes

# Ears
ros2 topic pub --once /head_ears/ears std_msgs/msg/String "{data: '128,128,128,128,128'}"
# Expected: Servos move to center positions

# Projector
ros2 topic pub --once /head_ears/projector/power std_msgs/msg/Bool "{data: true}"
# Expected: Projector powers on

# Focus
ros2 topic pub --once /head_ears/projector/focus std_msgs/msg/UInt8 "{data: 128}"
# Expected: Focus servo moves to center
```

**Endurance Test:**
```bash
# Run all components for 30 minutes:
# - Eyes blinking (mode=2)
# - Ears moving in slow pattern
# - Projector on

# Monitor:
# - Temperature (touch ESP32, voltage regulator - should be warm, not hot)
# - Current draw (should be stable)
# - No intermittent failures or resets
```

---

## Testing & Validation

**Test 1: Mechanical Stability**
```bash
# Gently shake module (simulates robot movement)
# Check for:
# - No rattling components
# - No loose wires
# - Enclosure cover stays secure
# - Servos don't slip or bind
```

**Test 2: Cable Stress Test**
```bash
# Move ears through full range of motion repeatedly (50 cycles)
# Verify:
# - No cable pinching
# - Servo cables don't disconnect
# - No reduction in range of motion
```

**Test 3: OLED Visibility**
```bash
# View OLEDs from multiple angles:
# - Front (0°): Clear, bright
# - Side (45°): Still visible
# - Top (looking down): Check alignment
```

**Test 4: Camera Field of View**
```bash
# On Pi, run camera test:
python3 depthai-python/examples/ColorCamera/rgb_preview.py

# Verify:
# - Camera image is level (horizon straight if aimed forward)
# - Field of view covers intended area (forward and slightly down)
# - No obstructions (enclosure parts, wires) in view
```

**Test 5: Projector Aim**
```bash
# Turn on projector with test image
# Verify:
# - Projects onto floor in front of robot (~30-50cm ahead)
# - Image is in focus (adjust with focus servo)
# - Sufficient brightness for visibility
```

---

## Troubleshooting

**Issue 1: Module Won't Power On**
- **Solution:** Check power cable polarity, verify 5V at screw terminal with multimeter, check fuse/switch on Base module

**Issue 2: I2C Communication Fails After Mounting**
- **Solution:** Check for broken wire in I2C cable, verify connections at both ends, try shorter cable, check for EMI from power cables

**Issue 3: Ears Hit Enclosure When Moving**
- **Solution:** Adjust servo brackets (move outward), trim enclosure cutouts, limit software range of motion in firmware

**Issue 4: OLEDs Dim or Not Visible**
- **Solution:** Increase brightness in firmware (REG_EYE_BRIGHTNESS), check OLED power (3.3V), verify SPI connections

**Issue 5: Camera Not Detected by Pi**
- **Solution:** Check USB cable connection, try different USB port, verify camera power LED, check depthai drivers

**Issue 6: Projector Focus Doesn't Work**
- **Solution:** Check servo linkage (may have slipped or binding), verify focus servo PWM signal with oscilloscope, adjust linkage design

**Issue 7: High Current Draw (>2A Idle)**
- **Solution:** Disconnect components one at a time to isolate culprit, check for shorts, verify servo power supply separate from ESP32

---

## Dependencies

**Before this story:**
- Story 1.4: Assemble and Test Head+Ears PCB ✅
- Story 1.5: Design and 3D Print Head+Ears Enclosure ✅
- Story 1.6: Develop Head+Ears ESP32 Firmware ✅
- Story 1.7: Create Head+Ears ROS2 Driver Node ✅
- Raspberry Pi installed in Torso (or available for testing)
- Base module power distribution available (or use bench supply)

**After this story:**
- Epic 2: Neck Module Build (Head+Ears will mount to Neck)
- Epic 5: End-to-End Demo (tests this module with others)

---

## References

- [Cable Management Best Practices](https://www.allaboutcircuits.com/technical-articles/cable-management-in-robotics/)
- [Strain Relief Techniques](https://www.digikey.com/en/articles/proper-cable-strain-relief)
- [ROS2 Launch System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)

---

## Notes

- **Modularity:** Design cable connections to allow easy module removal (use connectors, not soldered wires).
- **Labeling:** Label all cables at both ends (e.g., "5V from Base", "I2C to Pi") for future troubleshooting.
- **Documentation:** Take photos of final assembly (cable routing, component positions) for reference.
- **Temporary Mounting:** Until Neck module complete, use temporary stand to hold Head+Ears for testing.
- **Weight:** Final assembled module should be <1kg (weigh and document).
- **Aesthetic:** Consider cable sleeving or braiding for cleaner appearance (optional, post-Phase 1).
- **Vibration:** Add rubber dampeners between PCB standoffs and enclosure if vibration is issue during movement.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
