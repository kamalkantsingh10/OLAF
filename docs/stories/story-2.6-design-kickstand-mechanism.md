# Story 2.6: Design Kickstand Mechanism with Single Servo

**Epic:** Epic 2 - Neck Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 6-8 hours

---

## User Story

**As a** builder,
**I want** a kickstand mechanism designed that deploys/retracts using a single STS3215 servo,
**so that** the robot can transition between stationary mode (kickstand down) and balancing mode (kickstand up).

---

## Acceptance Criteria

1. ✅ OnShape CAD model created for kickstand mechanism including servo mount, linkage/lever arm, and ground contact pad
2. ✅ Mechanism design allows single servo rotation to deploy kickstand downward (stationary mode) and retract upward (balancing mode)
3. ✅ Deployed kickstand position is stable and can support robot weight (load calculation documented)
4. ✅ Retracted kickstand position clears ground and doesn't interfere with base module wheels
5. ✅ Servo mounting position integrated into Neck module mechanical design
6. ✅ Mechanical range of motion matches servo rotation limits (no over-travel)
7. ✅ Bill of materials includes both 3D printed parts and metal components
8. ✅ STL files exported for 3D printed kickstand components
9. ✅ Metal parts specified with dimensions for fabrication or sourcing
10. ✅ Parts printed/fabricated and mechanically tested: smooth deploy/retract, stable under load
11. ✅ Kickstand design documented with photos, BOM, servo angle specifications, and load test results

---

## Implementation Steps

### 1. Define Kickstand Requirements

**Functional Requirements:**
- Deploy time: <2 seconds (servo speed)
- Retract time: <2 seconds
- Ground clearance (retracted): >50mm (clear base wheels)
- Deployed length: 200-250mm (depends on robot height)
- Load capacity: 10kg minimum (full robot weight)
- Safety factor: 2× (design for 20kg)

**Servo Specifications (STS3215):**
- Stall torque: 15 kg·cm @ 7.4V
- Speed: 0.13 sec/60° @ 7.4V
- Rotation range: 0-4095 (270° mechanical, programmable limits)

### 2. Calculate Mechanical Advantage

**Leverage Analysis:**

```
Servo torque: 15 kg·cm = 1.5 N·m
Robot weight: 10 kg × 9.8 m/s² = 98 N
Required leverage: 98 N / (1.5 N·m) = 65.3 (force ratio)

If kickstand leg is 200mm from pivot:
  Required servo arm length: 200mm / 65.3 = 3.1mm (too short, inefficient)

Better approach: Use linkage multiplier
Servo arm: 25mm (typical servo horn radius)
Linkage ratio: 1:4 (servo moves 25mm, kickstand moves 100mm)
Effective leverage: 25mm × 4 = 100mm effective arm
Force at kickstand: 1.5 N·m / 0.1 m = 15 N × 4 = 60 N (6.1 kg holding force)

Conclusion: Need mechanical advantage of ~1.6× additional
Solution: Angle linkage for mechanical advantage or use dual-servo
```

**Design Decision:** Use angled linkage with lock-out mechanism (over-center locking) for maximum stability when deployed.

### 3. Design Linkage Mechanism (OnShape)

**Components:**

**A. Servo Mount Bracket (3D Printed)**
```
Material: PETG
Dimensions: 60mm × 40mm × 5mm base
Features:
- Servo pocket (40mm × 20mm × 38mm) with screw holes
- Mounting tabs to attach to Neck enclosure side wall
- Cable routing channel
```

**B. Servo Horn Adapter (3D Printed)**
```
Fits Feetech 25T spline servo horn
Has pivot point for linkage arm connection
M3 screw attachment point
```

**C. Linkage Arm (3D Printed or Metal)**
```
Material: PETG with 50% infill, or aluminum flat bar
Length: 60mm
Width: 15mm
Thickness: 5mm (3D printed) or 3mm (metal)
Features:
- Pivot holes at both ends (4mm diameter for M3 bolt with bearing)
- Over-center locking geometry (slight arc)
```

**D. Kickstand Leg Pivot Bracket (3D Printed)**
```
Attaches to Neck enclosure
Holds pivot point for kickstand leg
Metal bushing or bearing insert (8mm OD, 3mm ID)
```

**E. Kickstand Leg (Metal - Aluminum Rod)**
```
Material: Aluminum round rod, 10mm diameter
Length: 150mm
Features:
- Drilled hole at top (4mm) for pivot bolt
- Threaded hole at bottom (M6) for ground pad attachment
- Optional: Knurling or grip texture
```

**F. Ground Contact Pad (Metal - Steel Plate)**
```
Material: Steel plate, 3mm thickness
Dimensions: 50mm × 50mm
Features:
- Central threaded hole (M6) for leg attachment
- Rubber pad on bottom (optional, anti-slip)
- Chamfered edges (safety, aesthetics)
```

### 4. Design Deployment Kinematics

**Servo Positions:**

```
Retracted (balancing mode):
  Servo angle: 0° (position 512 in firmware)
  Linkage: Pulls kickstand leg vertical/upward
  Leg position: Against Neck enclosure, clear of ground

Deployed (stationary mode):
  Servo angle: 90° (position 2560 in firmware)
  Linkage: Pushes kickstand leg downward/outward
  Leg position: Extended to ground, angled ~70° from horizontal
  Locking: Over-center linkage locks in position (gravity assists)
```

**Over-Center Locking:**
- When fully deployed, linkage passes center line of pivot points
- Gravity pulls kickstand down, creating self-locking force
- Servo doesn't need continuous power to hold deployed position
- To retract, servo applies brief torque to break lock, then pulls

### 5. Create BOM with Metal Parts

**Bill of Materials:**

| Item | Description | Material | Quantity | Source |
|------|-------------|----------|----------|--------|
| Servo Mount Bracket | 3D printed | PETG | 1 | Print |
| Servo Horn Adapter | 3D printed | PETG | 1 | Print |
| Linkage Arm | 3D printed or metal | PETG/Aluminum | 1 | Print or fabricate |
| Pivot Bracket | 3D printed | PETG | 1 | Print |
| **Kickstand Leg** | **Aluminum rod** | **Al 6061** | **1** | **McMaster-Carr #8974K21** |
| **Ground Pad** | **Steel plate** | **Steel** | **1** | **McMaster-Carr #8982K18** |
| Pivot Bolts | M3 × 20mm | Steel | 3 | Hardware store |
| Bearings (optional) | 8mm OD, 3mm ID | Steel | 2 | McMaster-Carr |
| Ground Pad Thread | M6 threaded insert | Brass | 1 | McMaster-Carr |
| Rubber Pad (optional) | Anti-slip rubber | Rubber | 1 | Hardware store |

**Metal Part Specifications:**

**Kickstand Leg:**
- Aluminum round rod, 10mm diameter, 150mm length
- Drill 4mm hole at one end (pivot point), 10mm from edge
- Tap M6 thread at opposite end (ground pad attachment)
- Finish: Anodized (optional) or raw aluminum

**Ground Pad:**
- Steel plate, 50mm × 50mm × 3mm thickness
- Drill and tap M6 central hole
- Chamfer edges (1mm @ 45°)
- Finish: Powder coat or paint (rust prevention)

### 6. Export STL Files and Fabrication Drawings

**3D Printed Parts:**
```bash
# Export from OnShape
- kickstand-servo-bracket.stl
- kickstand-servo-horn-adapter.stl
- kickstand-linkage-arm.stl (if not using metal)
- kickstand-pivot-bracket.stl
```

**Metal Part Drawings:**
Create PDF drawings with dimensions for machine shop or DIY fabrication:
```
- kickstand-leg-drawing.pdf (rod length, hole positions, thread spec)
- ground-pad-drawing.pdf (plate dimensions, hole position)
```

### 7. Fabricate/Source Metal Components

**Option A: Purchase Pre-Cut**
- Order aluminum rod cut to length from supplier
- Order steel plate cut to size
- Drill holes at home with drill press

**Option B: Machine Shop**
- Provide drawings to local machine shop
- Cost estimate: $20-40 for both parts

**Option C: DIY Fabrication**
- Purchase raw stock from hardware store
- Cut with hacksaw or angle grinder
- Drill with drill press
- Tap threads with M6 tap and tap handle

### 8. Assemble and Test Mechanism

**Assembly Steps:**
```bash
# 1. Install servo in bracket
# 2. Attach servo bracket to Neck enclosure
# 3. Connect servo horn adapter to servo
# 4. Attach linkage arm to servo horn adapter (M3 bolt with bearing)
# 5. Install pivot bracket on Neck enclosure
# 6. Connect kickstand leg to pivot bracket (M3 bolt with bearing)
# 7. Connect linkage arm to kickstand leg
# 8. Attach ground pad to bottom of kickstand leg (M6 bolt)
# 9. Test deploy/retract motion by hand (no servo power)
# 10. Connect servo, test with firmware commands
```

**Load Test:**
```bash
# Place robot on kickstand (deployed position)
# Apply downward force gradually up to 15kg
# Verify:
#   - Linkage doesn't bend or break
#   - Servo holds position without overheating
#   - Ground pad doesn't slip on floor
#   - No plastic parts cracking under load
```

### 9. Document Design

Create `modules/neck/kickstand-design.md`:

```markdown
# Kickstand Mechanism Design

## Servo Angles
- **Retracted:** 0° (position 512) - kickstand up, clear of ground
- **Deployed:** 90° (position 2560) - kickstand down, supporting robot

## Load Capacity
- **Tested:** 15kg (1.5× robot weight)
- **Design limit:** 20kg (2× safety factor)

## Metal Components
- **Leg:** Aluminum rod, 10mm Ø, 150mm length
- **Pad:** Steel plate, 50mm × 50mm × 3mm

## Assembly Photos
[Include photos of assembled mechanism]

## Maintenance
- Check M3 bolts for tightness monthly
- Lubricate pivot points with dry lube (PTFE spray)
- Inspect ground pad for wear, replace if damaged
```

---

## Testing & Validation

**Test 1: Deployment Smoothness**
```bash
# Command servo to deploy/retract 50 times
# Verify: Smooth motion, no binding, consistent end positions
```

**Test 2: Lock Stability**
```bash
# Deploy kickstand, turn off servo power
# Apply lateral force (push robot sideways)
# Verify: Over-center lock holds, kickstand doesn't retract
```

**Test 3: Ground Clearance**
```bash
# Retract kickstand fully
# Measure ground clearance: should be >50mm from lowest point to floor
# Place robot on Base module, verify kickstand doesn't contact wheels
```

**Test 4: Repeated Load Cycles**
```bash
# Deploy kickstand
# Apply 10kg load for 10 seconds
# Retract
# Repeat 20 times
# Inspect for: Bent linkages, cracked plastic, servo overheating, loose bolts
```

---

## Troubleshooting

**Issue: Servo Can't Deploy Kickstand (Stalls)**
- **Solution:** Reduce friction at pivot points (add bearings), increase servo torque setting, check linkage for binding

**Issue: Kickstand Slips on Floor When Loaded**
- **Solution:** Add rubber pad to ground contact, increase ground pad area, roughen surface texture

**Issue: Linkage Arm Bends Under Load**
- **Solution:** Use metal linkage arm instead of 3D printed, increase arm width/thickness

**Issue: Kickstand Doesn't Lock in Deployed Position**
- **Solution:** Adjust linkage geometry to ensure over-center locking (linkage should pass pivot centerline when deployed)

**Issue: Ground Pad Contacts at Angle (Unstable)**
- **Solution:** Adjust kickstand leg length or deployment angle to ensure pad contacts floor flat

---

## Dependencies

**Before this story:**
- Story 2.4: Assemble and Test Neck PCB ✅
- Story 2.5: Design and 3D Print Neck Enclosure (servo mount locations known)

**After this story:**
- Story 2.7: Develop Neck ESP32 Firmware (kickstand servo commands)
- Story 2.9: Mount Neck Module to Robot Frame

---

## References

- [Linkage Mechanism Design](https://www.tec-science.com/mechanical-power-transmission/linkage-mechanism/)
- [Over-Center Locking Mechanisms](https://en.wikipedia.org/wiki/Toggle_mechanism)
- [Servo Torque Calculations](https://www.servocity.com/how-to-calculate-servo-torque)
- [McMaster-Carr Industrial Supply](https://www.mcmaster.com/)

---

## Notes

- **Over-Center Critical:** Ensures kickstand stays deployed without continuous servo power, saving battery and reducing servo wear.
- **Metal Components Essential:** 3D printed leg would flex/break under robot weight. Aluminum provides strength with low weight.
- **Safety Factor:** 2× ensures mechanism won't fail even if robot gains weight (additional modules, batteries, etc.).
- **Alternative Design:** Could use scissor-lift or telescoping leg for adjustable height, but single-servo linkage is simplest.
- **Cost:** Metal components add ~$30-50 to module cost, but essential for durability.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
