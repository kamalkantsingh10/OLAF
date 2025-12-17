# Story 4.7: Assemble Base Platform with Skateboard Suspension

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** Medium
**Estimated Effort:** 6-8 hours

---

## User Story

**As a** builder,
**I want** to assemble the Base platform using skateboard trucks for suspension and mount the hoverboard motors,
**so that** the robot has a stable, shock-absorbing base for balancing and movement.

---

## Acceptance Criteria

1. ✅ Skateboard iron trucks (2×) sourced and tested for smooth pivoting
2. ✅ Base platform constructed (wood/aluminum plate, ~400mm × 300mm)
3. ✅ Trucks mounted to platform with appropriate hardware
4. ✅ Hoverboard motors (with wheels) attached to truck axles
5. ✅ Motor wiring routed cleanly to top of platform
6. ✅ Battery mounting bracket installed on platform
7. ✅ Base PCB mounting location prepared
8. ✅ Platform weight distribution balanced (center of gravity aligned)
9. ✅ Wheels spin freely, trucks pivot smoothly (±20° tilt)
10. ✅ Mechanical assembly documented with photos

---

## Implementation Steps

### 1. Source Skateboard Trucks

**Requirements:**
- Type: Iron trucks (metal, durable) - NOT plastic
- Width: 5-6 inches (127-152mm)
- Mounting: Standard 4-hole pattern (skateboard standard)
- Bushings: Medium-soft for suspension compliance
- Cost: $15-30 per pair

**Where to buy:**
- Skateboard shops (used trucks acceptable)
- Amazon: Search "skateboard trucks 5 inch"
- eBay: Used trucks often cheaper

**Recommended brands:**
- Independent, Thunder, Venture (quality brands)
- Generic brands acceptable if metal construction

### 2. Prepare Base Platform

**Material options:**

```
Option A: Plywood (easier, lighter)
  - 1/2" (12mm) Baltic birch plywood
  - Dimensions: 400mm × 300mm
  - Weight: ~1kg
  - Cost: $10

Option B: Aluminum plate (stronger, heavier)
  - 3mm aluminum sheet
  - Dimensions: 400mm × 300mm
  - Weight: ~2.5kg
  - Cost: $30

Recommended: Plywood for Phase 1 (easier to work with)
```

**Cut platform:**

```bash
# If using plywood:
#   Mark 400mm × 300mm rectangle
#   Cut with circular saw or jigsaw
#   Sand edges smooth

# Round corners (optional):
#   20mm radius reduces sharp edges
#   Safer and looks better
```

### 3. Mark and Drill Truck Mounting Holes

**Truck mounting pattern:**

```
Skateboard truck mounting: 4 holes per truck
Hole spacing: Standard skateboard pattern
  - 2 holes: 38mm apart (1.5")
  - 2 pairs: 57mm apart (2.25")

Position on platform:
  Front truck: 50mm from front edge, centered
  Rear truck: 50mm from rear edge, centered
  (Symmetrical placement)

Drill:
  - Hole diameter: 5mm (for M4 bolts)
  - 8 holes total (4 per truck)
  - Drill from top, countersink if using flat-head bolts
```

### 4. Mount Trucks to Platform

**Hardware:**

```bash
# Per truck (×2):
#   4× M4 × 40mm bolts
#   8× M4 washers
#   4× M4 lock nuts

# Assembly:
#   1. Place truck on underside of platform
#   2. Insert bolts from top through platform into truck holes
#   3. Add washer on bolt (under platform)
#   4. Thread lock nut onto bolt
#   5. Tighten securely (3-4 Nm torque)

# Test:
#   Truck should pivot freely ±20°
#   No wobble or loose bolts
```

### 5. Attach Hoverboard Motors to Truck Axles

**Motor mounting:**

```bash
# Option A: Direct axle mount (if motors have axle bore)
#   - Remove skateboard wheels from trucks
#   - Insert truck axle through motor hub center
#   - Secure with axle nut (M8 typically)
#   - Add washer for bearing support

# Option B: Custom adapter plates (if motors don't fit axle)
#   - 3D print or laser-cut adapter plate
#   - Plate bolts to truck axle end (4× M5 bolts)
#   - Motor bolts to adapter plate
#   - Dimensions: Custom based on motor hub size

# Recommended: Option A if motor hub bore = axle diameter
#   Most hoverboard motors fit 8mm axles

# Alignment:
#   Motors should be level (not tilted)
#   Wheels should be perpendicular to platform
#   Both wheels should touch ground simultaneously when flat
```

### 6. Route Motor Wiring

**Cable management:**

```bash
# Each motor has 8 wires:
#   3× Phase wires (thick, 12-14 AWG)
#   5× Hall sensor wires (thin, 22-26 AWG)

# Routing path:
#   1. Motors (bottom) → Through platform hole → Top surface
#   2. Drill 25mm diameter holes near each motor mount
#   3. Use rubber grommets to protect wires from sharp edges
#   4. Coil excess wire on top of platform
#   5. Secure with zip ties to platform (strain relief)

# Wire length:
#   Left motor: ~400mm to ODrive
#   Right motor: ~400mm to ODrive
#   Allow extra length for truck pivot movement (~50mm slack)
```

### 7. Install Battery Mounting Bracket

**Battery mounting:**

```bash
# Battery position: Center of platform (optimal weight distribution)
# Mounting: Velcro straps or metal bracket

# Option A: Velcro straps (removable)
#   - 2× 50mm wide Velcro straps
#   - Drill 4× holes for strap anchors
#   - Battery sits on platform, straps over top
#   - Easy removal for charging

# Option B: Metal bracket (permanent)
#   - Bend 1mm aluminum sheet into U-shape
#   - Bolt bracket to platform (4× M4 bolts)
#   - Battery slides into bracket
#   - More secure but harder to remove

# Recommended: Velcro straps (Phase 1 flexibility)

# Position verification:
#   Place battery on platform
#   Check center of gravity with motors attached
#   Platform should balance horizontally
#   If tilts: Adjust battery position forward/back
```

### 8. Prepare Base PCB Mounting

**PCB location:**

```bash
# Position: Top of platform, rear section
#   - Behind battery (away from weight center)
#   - Near motors (shorter ODrive wiring)
#   - Accessible for debugging

# Mounting:
#   - 4× M3 × 10mm standoffs
#   - Standoffs bolt through platform (M3 nuts underneath)
#   - PCB screws onto standoffs (M3 × 6mm screws)

# Drill 4× 3.2mm holes matching PCB mounting holes
# Position standoffs, test-fit PCB before final assembly
```

### 9. Test Mechanical Assembly

**Functional tests:**

```bash
# Test 1: Wheel spin
#   - Lift platform off ground
#   - Spin each wheel by hand
#   - Should rotate freely (slight resistance from motor magnets)
#   - No grinding or wobble

# Test 2: Truck pivot
#   - Tilt platform left/right
#   - Trucks should pivot ±20° smoothly
#   - Bushings provide damped return to center
#   - No squeaking (add lubricant if needed)

# Test 3: Weight distribution
#   - Place fully assembled (battery + PCB) on flat surface
#   - Both wheels should contact ground equally
#   - No rocking or instability
#   - If unbalanced: Adjust battery position

# Test 4: Structural integrity
#   - Apply downward force (~10kg) on platform center
#   - No flexing or cracking of platform
#   - Truck bolts remain tight
#   - If flex: Add crossbrace underneath
```

### 10. Document Assembly

**Create `modules/base/mechanical_assembly.md`:**

```markdown
# Base Platform Mechanical Assembly

## Components
- Platform: 400mm × 300mm plywood (12mm thick)
- Trucks: 5" skateboard iron trucks (×2)
- Motors: Hoverboard BLDC hub motors (×2)
- Battery: 36V 4.4Ah (center-mounted)
- PCB: Base control board (rear-mounted)

## Dimensions
- Wheelbase: 300mm (front to rear truck)
- Track width: 300mm (left to right wheel)
- Platform height: 120mm (with wheels)

## Weight
- Platform assembly: ~4kg
- Battery: 2.5kg
- PCB + components: 0.5kg
- Total base weight: ~7kg

## Truck Setup
- Bushing stiffness: Medium (allows ±20° tilt)
- Axle nuts: Hand-tight + 1/4 turn (prevents wheel from falling off, allows spin)

## Motor Mounting
- Direct axle mount (8mm axle through motor hub)
- Secured with axle nut + washer

## Cable Routing
- Motor wires: Through 25mm grommeted holes
- Coiled on platform top with 50mm slack
- Zip-tied for strain relief

## Photos
[Insert assembly step photos]

## Notes
- Platform balances horizontally with battery centered
- Trucks provide passive suspension (absorbs shocks)
- Estimated payload capacity: 10kg (Torso + Neck + Head)
```

---

## Testing & Validation

**Test 1: Wheels Spin Freely**
```bash
# Both wheels rotate with minimal resistance
```

**Test 2: Trucks Pivot Smoothly**
```bash
# ±20° tilt in both directions
# Returns to center due to bushing tension
```

**Test 3: Platform Stable**
```bash
# Sits level on flat ground
# No wobble when pressed
```

**Test 4: Weight Capacity**
```bash
# Apply 10kg load (simulating upper modules)
# No structural failure or excessive flex
```

---

## Troubleshooting

**Issue 1: Wheels Wobble**
- **Solution:** Tighten axle nuts, check bearings in motor hub, verify truck alignment

**Issue 2: Platform Tilts to One Side**
- **Solution:** Adjust battery position, verify both wheels same diameter, check truck mounting

**Issue 3: Trucks Too Stiff (Won't Pivot)**
- **Solution:** Loosen truck kingpin nut, replace bushings with softer ones

**Issue 4: Platform Flexes Under Load**
- **Solution:** Use thicker plywood (3/4"), add crossbrace underneath, switch to aluminum

**Issue 5: Motor Wires Get Pinched**
- **Solution:** Add more slack, reroute away from moving parts, use rubber grommets

---

## Dependencies

**Before this story:**
- Story 4.1: Source and Disassemble Hoverboard ✅
- Story 4.6: Assemble and Test Base PCB ✅

**After this story:**
- Story 4.8: Configure ODrive for Hoverboard Motors

---

## References

- [Skateboard Truck Anatomy](https://www.warehouseskateboards.com/help/Skateboard-Trucks-Buying-Guide)
- [DIY Self-Balancing Robot](https://www.instructables.com/Self-Balancing-Robot/)

---

## Notes

- **Skateboard Suspension:** Trucks provide ~20mm vertical travel, absorbs small bumps and allows tilting for balance
- **Iron Trucks Critical:** Plastic trucks will break under robot weight (7kg base + 10kg payload = 17kg total)
- **Wheel Alignment:** Must be parallel and perpendicular to platform for straight movement
- **Center of Gravity:** Keep battery and heavy components centered to minimize torque on motors
- **Alternative Designs:** Could use custom machined axles, but skateboard trucks are cheap, proven, and easily replaceable
- **Future Upgrades:** Add spring-damper system for better shock absorption, upgrade to larger wheels for outdoor use

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
