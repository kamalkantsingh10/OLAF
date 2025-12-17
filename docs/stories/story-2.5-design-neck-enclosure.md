# Story 2.5: Design and 3D Print Neck Enclosure and Servo Mounts

**Epic:** Epic 2 - Neck Module Build
**Status:** Not Started
**Priority:** Medium
**Estimated Effort:** 10-14 hours (design + print)

---

## User Story

**As a** builder,
**I want** 3D printed enclosure, servo mounting brackets, and kickstand mechanism designed in OnShape and printed,
**so that** I can protect the PCB and mount servos/sensors to the robot.

---

## Acceptance Criteria

1. ✅ OnShape CAD model created for Neck enclosure with space for PCB, 3-DOF servo mounting, kickstand servo mount, presence sensor mounts
2. ✅ Design includes mechanical linkages for pan/tilt/roll motion and kickstand deployment mechanism
3. ✅ Design ensures full range of motion for all servos without mechanical interference
4. ✅ STL files exported for 3D printing
5. ✅ Parts printed on FDM printer with appropriate settings
6. ✅ Printed parts fit together with PCB, servos, sensors, and allow smooth motion
7. ✅ Assembly instructions drafted with photos (saved as `modules/neck/assembly.md`)

---

## Implementation Steps

### 1. Define Design Requirements

**Measurements:**
- PCB: 90mm × 70mm × 1.6mm
- Servos (STS3215): 40mm × 20mm × 38mm (×4)
- Presence sensors: ~30mm × 15mm × 15mm (×2)
- Range of motion:
  - Pan: ±90° (180° total)
  - Tilt: +60° / -30° (90° total)
  - Roll: ±20° (40° total)
  - Kickstand: 90° deployment arc

**Mechanical Load:**
- Head+Ears module weight: ~800g (from Story 1.8)
- Kickstand must support full robot weight: ~10kg
- Safety factor: 2× (design for 20kg kickstand load)

### 2. Design Main Enclosure (OnShape)

**Base Plate:**
```
Dimensions: 100mm × 80mm × 3mm
Features:
- 4× M3 holes for PCB standoffs (match PCB layout)
- 2× M5 holes for mounting to Torso module (bottom)
- 2× M4 holes for Head+Ears module connection (top)
- Cable routing slot (20mm × 10mm)
```

**Enclosure Walls:**
```
Height: 40mm
Wall thickness: 2.5mm
Cutouts:
- Side slots for servo horn access
- Rear cable exit (30mm × 15mm)
- Sensor mounting windows (front and rear)
```

**Top Plate:**
```
Dimensions: 100mm × 80mm × 2mm
Features:
- Central mounting point for Head+Ears module
- Bearing or bushing for pan rotation (if designing full gimbal)
- Ventilation holes (5mm grid)
```

### 3. Design Pan/Tilt/Roll Mechanism

**Option A: Stacked Servo Configuration**
```
Pan (Servo 1, bottom):
  ↓ Servo horn connects to U-bracket
Tilt (Servo 2, middle):
  ↓ Attached to U-bracket, rotates head pitch
Roll (Servo 3, top):
  ↓ Attached to tilt platform, rotates head roll
```

**Parts to Design:**
1. **Pan Base Mount:** Holds Servo 1, attaches to Neck enclosure top
2. **Pan-Tilt U-Bracket:** Connects pan servo horn to tilt servo body
3. **Tilt-Roll Adapter:** Connects tilt servo horn to roll servo body
4. **Roll-Head Connector:** Attaches roll servo horn to Head+Ears mounting holes

**Design Guidelines:**
- Servo horns: Use 25T spline (Feetech standard)
- Clearances: 2mm minimum between moving parts
- Fasteners: M3 screws for servo mounting
- Material: PLA or PETG, 30% infill for strength

### 4. Design Kickstand Mechanism (Story 2.6 reference)

**Linkage Design:**
```
Servo 4 (Kickstand) positioned on side of Neck enclosure
  ↓ Servo horn (0° = retracted, 90° = deployed)
  ↓ Linkage arm (50mm length, 3mm thick)
  ↓ Kickstand leg (metal rod, 150mm length, 10mm diameter aluminum)
  ↓ Ground pad (metal plate, 50mm × 50mm × 3mm steel)
```

**Parts:**
1. **Servo Bracket:** Holds kickstand servo to enclosure
2. **Linkage Arm:** 3D printed, connects servo horn to kickstand leg
3. **Leg Pivot Mount:** Bearing or bushing for smooth rotation
4. **Metal Components:** Sourced separately (see Story 2.6)

### 5. Design Sensor Mounts

**Front Sensor Mount:**
- Bracket holds sensor facing forward
- Angled 0-10° downward (detect humans at ground level)
- Snap-fit or screw mount to front of enclosure

**Rear Sensor Mount:**
- Bracket holds sensor facing backward
- Coverage: 180° rear hemisphere

### 6. Export STL Files

```bash
# OnShape: Right-click each part → Export → STL
# Save to: ~/olaf/modules/neck/hardware/3d-models/stl/

Parts list:
- neck-main-enclosure.stl
- neck-top-plate.stl
- pan-base-mount.stl
- pan-tilt-u-bracket.stl
- tilt-roll-adapter.stl
- roll-head-connector.stl
- kickstand-servo-bracket.stl
- kickstand-linkage-arm.stl
- sensor-mount-front.stl
- sensor-mount-rear.stl
```

### 7. 3D Print All Parts

**Print Settings:**
- Layer height: 0.2mm
- Infill: 30% (structural parts), 20% (enclosure)
- Wall thickness: 4 perimeters
- Supports: Auto-generate for overhangs >45°
- Material: PETG preferred (stronger than PLA)

**Print Order:**
1. Small parts first (sensor mounts, linkage arm)
2. Test fit before printing large parts
3. Main enclosure last (longest print, ~12 hours)

### 8. Assembly and Fit Test

```bash
# Dry-fit sequence:
# 1. Install PCB in enclosure with standoffs
# 2. Mount pan servo to base mount
# 3. Attach U-bracket to pan servo horn
# 4. Mount tilt servo to U-bracket
# 5. Attach tilt-roll adapter to tilt servo horn
# 6. Mount roll servo to adapter
# 7. Attach roll-head connector
# 8. Test range of motion (no binding, interference)
# 9. Mount kickstand servo and test deployment
# 10. Install sensors in mounts
```

---

## Testing & Validation

**Test 1: Servo Range of Motion**
```bash
# Command each servo through full range
# Verify: No contact with enclosure or other servos
# Check for binding, strange noises, or resistance
```

**Test 2: Head+Ears Load Test**
```bash
# Attach Head+Ears module (or equivalent weight ~800g)
# Command pan/tilt/roll through full range
# Verify: Servos can hold position without drooping
# If drooping, increase servo torque or redesign for better load distribution
```

**Test 3: Kickstand Deployment**
```bash
# Deploy kickstand (servo to 90°)
# Apply downward force (10kg simulated robot weight)
# Verify: Linkage holds, no bending or breaking
# Check servo for overheating or stall
```

---

## Troubleshooting

**Issue: Servos Bind During Movement**
- **Solution:** Increase clearances in CAD, sand/file contact points, check servo horn alignment

**Issue: Parts Don't Fit Together**
- **Solution:** Measure with calipers, scale STL by 99-101%, reprint problematic parts

**Issue: Linkages Flex Under Load**
- **Solution:** Increase wall thickness, use higher infill (40-50%), switch to PETG or ABS

**Issue: Kickstand Servo Stalls Under Load**
- **Solution:** Increase mechanical advantage (longer linkage arm), use dual-servo design, or metal gear servo

---

## Dependencies

**Before this story:**
- Story 2.2: PCB dimensions known
- Story 2.4: Component dimensions verified
- Head+Ears module dimensions (Story 1.5)

**After this story:**
- Story 2.6: Design Kickstand Mechanism (metal components)
- Story 2.9: Mount Neck Module to Robot Frame

---

## References

- [Servo Gimbal Design](https://www.thingiverse.com/search?q=servo+gimbal)
- [OnShape Mechanical Design](https://learn.onshape.com/)
- [3D Printing for Robotics](https://www.prusa3d.com/category/robotics/)

---

## Notes

- **Gimbal vs Fixed:** Could design full 3-axis gimbal (more complex) or simpler stacked servo approach. Start simple.
- **Bearing vs Bushings:** Can use printed bushings for cost savings, but metal bearings smoother.
- **Print Time:** Total ~20-30 hours for all parts.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
