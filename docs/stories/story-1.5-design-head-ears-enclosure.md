# Story 1.5: Design and 3D Print Head+Ears Enclosure

**Epic:** Epic 1 - Head+Ears Module Build
**Status:** Not Started
**Priority:** Medium
**Estimated Effort:** 8-12 hours (design), 12-24 hours (printing)

---

## User Story

**As a** builder,
**I want** 3D printed enclosure and mounting brackets designed in OnShape and printed,
**so that** I can protect the PCB and mount OLEDs/servos/projector/camera to the robot.

---

## Acceptance Criteria

1. ✅ OnShape CAD model created for Head+Ears enclosure with space for PCB, OLED mounts, ear servo brackets, projector mount, camera mount
2. ✅ Design includes cable routing channels and mounting holes for robot frame attachment
3. ✅ STL files exported for 3D printing
4. ✅ Parts printed on FDM printer with appropriate settings (layer height, infill, supports)
5. ✅ Printed parts fit together with PCB, OLEDs, servos, projector, and camera
6. ✅ Assembly instructions drafted with photos (saved as `modules/head-ears/assembly.md`)

---

## Implementation Steps

### 1. Set Up OnShape Account and Project

```bash
# Create free OnShape account (education or maker tier)
# Visit: https://www.onshape.com/en/sign-up

# Create new document:
# Name: "OLAF Head+Ears Module v1.0"
# Make public or private (public allows community sharing)
```

### 2. Define Design Requirements and Measurements

**Measurements to Take:**
```bash
# PCB dimensions (from Story 1.2):
# - Length: 100mm
# - Width: 80mm
# - Thickness: 1.6mm
# - Mounting holes: 4× M3, positions documented in wiring.md

# Component heights (measure with calipers):
# - ESP32 module: ~5mm
# - Tallest component (screw terminals): ~10mm
# - OLED displays: 128×64mm active area, ~3mm thick
# - Servo dimensions: Feetech STS3215 ~40×20×38mm
# - OAK-D-Pro camera: ~98×30×26mm (check datasheet)
# - Projector dimensions: varies by model, measure actual unit
```

**Design Constraints:**
- Enclosure must allow airflow for ESP32 cooling
- OLED eyes must be visible from front
- Ear servos need full range of motion (no mechanical interference)
- Camera must have unobstructed field of view (120° FOV typical)
- Projector must aim downward at floor (45-60° angle)
- Total weight budget: <1kg (to not overload neck servos)

### 3. Design Main Enclosure Body

**OnShape Sketch:**

1. **Base Plate (Bottom):**
   ```
   - Rectangle: 120mm × 100mm (larger than PCB for mounting ears)
   - Add 4× mounting holes for PCB standoffs (M3, match PCB positions)
   - Add 2× mounting holes for neck connection (M4 or M5, rear edge)
   - Thickness: 3mm
   ```

2. **Enclosure Walls:**
   ```
   - Extrude walls from base plate perimeter: 50mm height
   - Add cutouts:
     - Front: 2× OLED windows (30×25mm each, spaced for eyes)
     - Rear: Cable routing slot (20×10mm)
     - Sides: Ear servo mounting points (slots or pockets)
     - Top: Camera mount (100×30mm opening)
     - Bottom: Projector mount (circular or rectangular, angled)
   - Wall thickness: 2-3mm (balance strength vs print time)
   ```

3. **Top Cover:**
   ```
   - Plate: 120mm × 100mm
   - Cutout for camera (centered, 100×30mm)
   - Snap-fit tabs or screw mounts to attach to walls
   - Ventilation holes (5mm diameter, grid pattern for ESP32 cooling)
   - Thickness: 2mm
   ```

### 4. Design OLED Eye Mounts

**Eye Socket Design:**
```
# Component: 2× OLED bezels (one per eye)
# - Pocket to hold OLED display (128×64mm PCB dimensions)
# - Retention clips or screw mounts (M2 screws)
# - Integration with front wall cutouts
# - Light seals to prevent backlight bleed
# - Optional: diffuser mount for softer eye appearance
```

**OnShape Steps:**
1. Create new part: "OLED_Bezel_Left"
2. Sketch rectangle matching OLED PCB outline + 2mm clearance
3. Extrude 5mm deep pocket
4. Add mounting screw holes (M2, match OLED PCB holes)
5. Add snap-fit clips on rear to attach to main enclosure
6. Mirror for right eye bezel

### 5. Design Ear Servo Brackets

**Ear Mechanism Design:**

**Considerations:**
- 2-DOF per ear: base joint (up/down), tip joint (forward/back or rotation)
- Servos: Feetech STS3215, dimensions ~40×20×38mm
- Range of motion: Base ±45°, tip ±30° typical
- Linkage: Can use servo horn + 3D printed ear structure

**OnShape Parts:**

1. **Servo Bracket (4× total, 2 per ear):**
   ```
   - Pocket to hold servo body (friction fit or screw mount)
   - Mounting tab to attach to main enclosure side wall
   - Wire routing channel
   - Thickness: 3mm
   ```

2. **Ear Base Structure:**
   ```
   - Connects to base servo horn
   - Holds tip servo in position
   - Organic ear shape (triangular or rounded)
   - Lightweight (hollow or low infill acceptable)
   - Length: 80-100mm (proportional to head size)
   ```

3. **Ear Tip (Articulated):**
   ```
   - Connects to tip servo horn
   - Completes ear shape
   - Length: 40-60mm
   ```

**Assembly:**
```
Base servo → Ear base structure → Tip servo → Ear tip
Entire assembly mounts to side of main enclosure
```

### 6. Design Camera and Projector Mounts

**OAK-D-Pro Camera Mount:**
```
# Top of enclosure, forward-facing
# - Bracket to hold camera (friction fit or screw mount M2.5)
# - Ensures camera level and aimed forward
# - USB cable routing to rear (runs down to Pi in Torso)
# - Adjustable tilt (optional, use set screw or ratchet)
```

**Floor Projector Mount:**
```
# Bottom of enclosure, angled downward
# - Mounting bracket for projector body (dimensions vary by model)
# - Angle: 45-60° from horizontal (projects onto floor in front of robot)
# - Focus servo integration: mechanical linkage to projector focus ring
# - HDMI cable routing from rear (connected to Pi)
# - Power cable routing from projector power circuit on PCB
```

**Focus Servo Linkage (Critical):**
```
# Mechanical connection: servo horn → linkage arm → projector focus ring
# Design options:
#   1. Flexible cable (like bicycle brake cable)
#   2. Rigid rod with universal joints
#   3. Gear train (servo gear → projector focus gear)
# Must be tested for smooth operation without backlash
```

### 7. Export STL Files for Printing

**OnShape Export:**
1. Right-click each part → Export → Format: STL
2. Units: Millimeters
3. Resolution: Fine (smaller triangles, smoother curves)
4. Save to: `~/olaf/modules/head-ears/hardware/3d-models/stl/`

**Parts List:**
```
- head-ears-main-enclosure.stl
- head-ears-top-cover.stl
- oled-bezel-left.stl
- oled-bezel-right.stl
- ear-base-servo-bracket.stl (×2)
- ear-tip-servo-bracket.stl (×2)
- ear-base-structure-left.stl
- ear-base-structure-right.stl
- ear-tip-left.stl
- ear-tip-right.stl
- camera-mount-bracket.stl
- projector-mount-bracket.stl
- focus-servo-linkage-parts.stl (depending on design)
```

### 8. Prepare Files for 3D Printing

**Slicing Software (PrusaSlicer, Cura, etc.):**

```bash
# Install PrusaSlicer (or your preferred slicer)
# Open STL files

# Print Settings:
# - Layer height: 0.2mm (balance quality vs speed)
# - Infill: 20% (adequate strength, lightweight)
# - Wall thickness: 3-4 perimeters (strong walls)
# - Supports: Auto-generate where needed (OLED bezels, servo brackets)
# - Adhesion: Brim or raft (prevents warping on large parts)
# - Material: PLA or PETG
#   - PLA: Easy to print, good for prototyping
#   - PETG: Stronger, better heat resistance

# Print Time Estimates:
# - Main enclosure: 10-15 hours
# - Ear parts (all): 8-12 hours
# - Small parts (bezels, brackets): 2-4 hours each
# - Total: 24-48 hours depending on printer speed
```

### 9. 3D Print All Parts

**Printing Strategy:**
```bash
# Print smallest parts first (test printer settings):
# 1. Servo brackets → Quick, test fitment
# 2. OLED bezels → Test OLED fit
# 3. Large parts (main enclosure, ear structures) → Long prints, do overnight

# Quality Check After Each Print:
# - Layer adhesion (no delamination)
# - Dimensional accuracy (measure with calipers, ±0.2mm tolerance)
# - No warping or curling
# - Supports removed cleanly
```

### 10. Test Fit and Iterate

**Assembly Dry-Fit:**
```bash
# Without glue or permanent fasteners:
# 1. Place PCB in main enclosure → Check clearances
# 2. Mount OLEDs in bezels → Verify tight fit, no rattling
# 3. Attach servos to brackets → Check screw holes align
# 4. Assemble ear structures → Test range of motion
# 5. Mount camera and projector → Verify cable routing

# Issues Found:
# - Tolerances too tight? → Scale parts by 101% and reprint
# - Parts don't fit? → Modify OnShape design, re-export STL, reprint
# - Weak structures? → Increase infill or wall thickness
```

### 11. Create Assembly Instructions

**Document:** `modules/head-ears/assembly.md`

```markdown
# Head+Ears Module Assembly Instructions

## Tools Required
- Screwdrivers: Phillips #1, flathead 2mm
- Hex keys: 2mm, 2.5mm (for M2, M3 screws)
- Calipers (for verification)
- Hot glue gun (optional, for cable management)

## Hardware Required
- M3 screws: 10mm (×4 for PCB standoffs), 15mm (×4 for cover)
- M3 standoffs: 10mm (×4)
- M2 screws: 6mm (×8 for OLED mounts)
- M2.5 screws: 8mm (×4 for camera mount)
- M4 or M5 screws: 20mm (×2 for neck connection)
- Zip ties for cable management

## Assembly Steps
1. Install PCB in main enclosure using standoffs
2. Connect OLEDs to PCB, mount in bezels
3. Attach servo brackets to enclosure walls
4. Install ear servos in brackets
5. Attach ear structures to servo horns
6. Mount camera bracket and install OAK-D-Pro
7. Mount projector and configure focus linkage
8. Route all cables through channels
9. Attach top cover
10. Test all components before final mounting

## Photos
[Include photos of each assembly step]
```

---

## Testing & Validation

**Test 1: PCB Fit**
```bash
# PCB should sit flush on standoffs
# All mounting holes aligned
# No interference with walls or components
```

**Test 2: OLED Visibility**
```bash
# OLEDs powered on, displaying test pattern
# Viewable from front at 0-45° angles
# No light bleed around edges
# Bezels secure, no rattling
```

**Test 3: Ear Range of Motion**
```bash
# Command servos through full range
# No mechanical interference with enclosure walls
# Smooth motion, no binding
# Servo brackets don't flex under load
```

**Test 4: Cable Routing**
```bash
# All cables (OLED, servo, power, I2C, USB, HDMI) fit through channels
# No pinching or sharp bends
# Strain relief at connection points
# Organized, not tangled
```

**Test 5: Weight Check**
```bash
# Weigh complete assembly on scale
# Target: <1kg (to not overload neck servos)
# If over weight, identify areas to hollow out or reduce infill
```

---

## Troubleshooting

**Issue 1: Parts Don't Fit (Too Tight)**
- **Solution:** Scale STL by 101-102% before slicing, reprint

**Issue 2: Parts Too Loose**
- **Solution:** Scale STL by 98-99%, or add shims/padding

**Issue 3: Layers Delaminating During Print**
- **Solution:** Increase bed temperature, reduce cooling fan speed, check filament dry

**Issue 4: Supports Difficult to Remove**
- **Solution:** Reduce support density in slicer (10-15%), use support interface layers

**Issue 5: Ear Servos Interfere with Enclosure**
- **Solution:** Increase clearance in CAD model, reprint walls or brackets

**Issue 6: Projector Focus Linkage Doesn't Work**
- **Solution:** Redesign linkage mechanism, consider simpler servo mount directly on projector focus ring

---

## Dependencies

**Before this story:**
- Story 1.2: PCB dimensions and mounting hole positions known
- Story 1.4: Component dimensions verified during assembly
- Access to 3D printer and filament

**After this story:**
- Story 1.6: Develop Head+Ears ESP32 Firmware (can work in parallel)
- Story 1.8: Mount Head+Ears Module to Robot Frame

---

## References

- [OnShape Learning Center](https://learn.onshape.com/)
- [3D Printing Basics](https://www.prusa3d.com/page/3d-printing-basics_277/)
- [PrusaSlicer Manual](https://help.prusa3d.com/tag/prusaslicer)
- [Servo Bracket Design Tips](https://www.thingiverse.com/search?q=servo+bracket)

---

## Notes

- **Design Time:** Expect 2-3 iterations to get perfect fit. First version may need adjustments.
- **Print Material:** PLA easiest, PETG more durable. ABS possible but requires heated enclosure.
- **Infill Pattern:** Gyroid or honeycomb for strength with low weight.
- **Post-Processing:** Sand surfaces for smooth finish, acetone vapor smoothing for ABS.
- **Alternative:** If no 3D printer available, laser-cut acrylic enclosure (2D design instead of 3D).
- **Community Sharing:** Upload OnShape design and STLs to Thingiverse, Printables, or GitHub for open-source collaboration.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
