# Story 3.5: Assemble Torso Enclosure with Kitchen Bin and Metal Reinforcement

**Epic:** Epic 3 - Torso Module Build
**Status:** Not Started
**Priority:** Medium
**Estimated Effort:** 6-8 hours

---

## User Story

**As a** builder,
**I want** a durable kitchen bin enclosure reinforced with metal flat bars to house Raspberry Pi 5, Torso PCB, display, and printer,
**so that** all components are protected and organized with a strong, practical structure.

---

## Acceptance Criteria

1. ✅ Kitchen bin selected with appropriate size for all components
2. ✅ Metal flat bars sourced (steel or aluminum) for structural reinforcement
3. ✅ Ventilation holes cut/drilled for Pi and Hailo Kit thermal management
4. ✅ Cutouts made for: front-facing display, thermal printer output slot, power input, cable routing
5. ✅ Metal flat bars attached to kitchen bin for reinforcement
6. ✅ Mounting brackets/points fabricated for robot frame attachment (top to Neck, bottom to Base)
7. ✅ Internal mounting system created for Pi, PCB, display, and printer
8. ✅ Power input connector accessible from Base module
9. ✅ All components fit in enclosure with adequate cooling and cable management
10. ✅ Assembly instructions drafted with photos

---

## Implementation Steps

### 1. Select Kitchen Bin

**Requirements:**
- Size: ~300mm (H) × 200mm (W) × 200mm (D) minimum
- Material: Durable plastic (HDPE or PP)
- Lid: Removable or hinged for access
- Cost: $10-20

**Recommended:** Rectangular wastebasket or small storage bin from hardware store

### 2. Source Metal Reinforcement

**Materials:**
- Steel or aluminum flat bars: 25mm × 3mm × 500mm (×4 pieces for vertical corners)
- Alternative: Angle iron (L-shaped) for better rigidity
- Source: Hardware store or McMaster-Carr

**Attachment:**
- Epoxy (strong, permanent): JB Weld or equivalent
- Bolts/rivets (removable): M4 bolts with nuts, or pop rivets
- Combination: Epoxy + bolts for maximum strength

### 3. Cut Ventilation Holes

**Locations:**
- Sides: 10-15 holes, 10mm diameter, grid pattern
- Top: 5-8 holes near Pi mounting area
- Bottom: Optional (if not sealed for floor clearance)

**Tools:**
- Step drill bit (10mm) or hole saw
- Power drill
- Deburring tool or file (smooth edges)

**Pattern:**
```
Ventilation Grid (side view):
○ ○ ○ ○ ○
○ ○ ○ ○ ○
○ ○ ○ ○ ○

Spacing: 30mm between centers
Total airflow: ~600mm² (adequate for Pi 5)
```

### 4. Cut Component Cutouts

**Front Panel:**
```
Display Window: 90mm × 90mm (centered, 100mm from bottom)
- Use rotary tool (Dremel) or jigsaw
- Cut slightly smaller than display, file to fit
- Smooth edges with file or sandpaper
```

**Side Panel (Printer):**
```
Printer Output Slot: 60mm × 15mm (horizontal slot)
- Position 50mm from bottom (accessible height)
- Allow paper to feed out smoothly
```

**Rear Panel:**
```
Power Input Connector: 20mm × 20mm (bottom corner)
Cable Routing Slots: 30mm × 15mm (×2-3 for I2C, USB, HDMI)
```

**Top Panel:**
```
Neck Mounting Holes: 2× 6mm diameter (M5 bolts)
- Position matching Neck module bottom mounting points
- Reinforce with metal washers on inside
```

**Bottom Panel:**
```
Base Mounting Holes: 2× 6mm diameter (M5 bolts)
- Position matching Base module top mounting points
```

### 5. Attach Metal Reinforcement

**Vertical Corner Bars:**
```bash
# Position flat bars at 4 corners (inside or outside of bin)
# Mark drill holes every 100mm
# Drill through plastic bin (4mm holes)
# Attach with M4 bolts + nuts (or rivets)
# If using epoxy: Clean surfaces with isopropyl alcohol, apply epoxy, clamp for 24h
```

**Horizontal Cross-Braces (Optional):**
```
Add bars across top and bottom edges for extra rigidity
Useful if bin material is thin/flexible
```

### 6. Create Internal Mounting System

**Raspberry Pi Mount:**
```
Option A: 3D print mounting plate with standoffs
Option B: Cut acrylic/wood plate, attach with adhesive standoffs
- Plate dimensions: 100mm × 80mm
- Standoffs: M2.5 × 10mm for Pi mounting holes
- Attach plate to bin wall with strong adhesive or screws
```

**Torso PCB Mount:**
```
Similar to Pi: mounting plate or direct standoffs to bin wall
Position near bottom, away from Pi (thermal separation)
```

**Display Mount:**
```
Secure display behind front panel cutout
Use hot glue or small brackets
Ensure display is flush with or slightly recessed from panel
```

**Printer Mount:**
```
Secure printer near side slot
Use Velcro strips or 3D printed bracket
Ensure paper path is clear to output slot
Include mechanism to open printer for paper loading
```

### 7. Create Mounting Points for Frame

**Top (Neck Connection):**
```
Metal plate: 80mm × 60mm × 3mm steel
- Drill 2× M5 holes matching Neck module
- Bolt plate to top of bin (through plastic + reinforcement bars)
- This distributes head/neck weight to bin structure
```

**Bottom (Base Connection):**
```
Similar metal plate on bottom
Matches Base module top mounting points
Secure with bolts through bottom panel
```

### 8. Assemble and Test Fit

**Assembly Order:**
```bash
# 1. Install metal reinforcement bars
# 2. Cut all holes and cutouts
# 3. Install internal mounting plates/standoffs
# 4. Mount Raspberry Pi (verify boot, cooling adequate)
# 5. Mount Torso PCB
# 6. Install display (test visibility, secure)
# 7. Install printer (test paper feed, output slot alignment)
# 8. Route cables (keep organized, use cable ties)
# 9. Verify all components fit with lid/access panel
# 10. Test power-on (Pi boots, display shows, printer accessible)
```

### 9. Cable Management

```
Power cables (from Base): Route through bottom, secure to wall with zip ties
I2C/USB/HDMI (to Head+Ears): Route through top, bundle together
Internal cables: Use cable channels or adhesive clips
Strain relief: Rubber grommets at entry/exit points
```

### 10. Document Assembly

Create `modules/torso/assembly.md`:

```markdown
# Torso Enclosure Assembly

## Kitchen Bin Specs
- Model: [brand/model]
- Dimensions: [H×W×D]
- Material: HDPE plastic

## Metal Reinforcement
- 4× steel flat bars (25mm × 3mm × 500mm)
- Attachment: M4 bolts + epoxy
- Weight added: ~500g

## Components Installed
- Raspberry Pi 5 + Hailo AI Kit
- Torso ESP32 PCB
- 2.8" heart display
- Thermal printer

## Ventilation
- 20× 10mm holes (grid pattern)
- Airflow verified with temperature monitoring

## Photos
[Include assembly photos at each step]
```

---

## Testing & Validation

**Test 1: Structural Integrity**
```bash
# Apply 5kg downward force on top (simulates neck/head weight)
# Verify: No flexing, cracking, or deformation
# Metal bars should prevent bin from bowing
```

**Test 2: Thermal Management**
```bash
# Run Pi 5 + Hailo at full load for 30 minutes
# Monitor temperature:
#   - Pi CPU: <80°C acceptable (with heatsink + fan)
#   - Hailo: <75°C
# If temps exceed limits, add more ventilation or active cooling (fan)
```

**Test 3: Component Accessibility**
```bash
# Verify can access:
#   - Pi SD card slot (for firmware updates)
#   - Printer paper compartment (for reloading)
#   - Torso PCB (for debugging/maintenance)
# Lid or access panel opens easily
```

**Test 4: Cable Routing**
```bash
# All cables reach destinations without strain
# No sharp bends or pinch points
# Organized, not tangled
```

---

## Troubleshooting

**Issue: Bin Flexes Under Weight**
- **Solution:** Add more reinforcement bars (horizontal cross-braces), use thicker metal (5mm instead of 3mm)

**Issue: Pi Overheats**
- **Solution:** Add more ventilation holes, install active cooling fan (40mm × 10mm, 5V), consider heatsink upgrade

**Issue: Display Not Visible Through Cutout**
- **Solution:** Enlarge cutout, adjust display mounting position, add bezel or frame for cleaner appearance

**Issue: Printer Paper Jams at Output Slot**
- **Solution:** Smooth slot edges, enlarge slot slightly, ensure printer is level and aligned

**Issue: Metal Bars Interfere with Components**
- **Solution:** Position bars at corners only, avoid center areas where components mount

---

## Dependencies

**Before this story:**
- Story 3.2: PCB dimensions known
- Raspberry Pi 5 dimensions known
- Display and printer dimensions measured

**After this story:**
- Story 3.8: Install and Configure Raspberry Pi 5 in Torso Module
- Story 3.9: Mount Torso Module to Robot Frame

---

## References

- [Kitchen Bin Selection Guide](https://www.ikea.com/us/en/cat/waste-bins-10552/)
- [Metal Reinforcement Techniques](https://makezine.com/projects/metal-reinforced-enclosures/)
- [Raspberry Pi Cooling Solutions](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#cooling)

---

## Notes

- **Kitchen Bin Advantage:** Off-the-shelf, durable, large internal volume, cheap ($10-20 vs $50+ custom metal enclosure)
- **Metal Reinforcement Critical:** Plastic alone won't support neck/head weight (totaling ~1.5kg + torso components). Metal provides rigidity.
- **Aesthetics:** Bin can be painted, wrapped, or decorated. Consider spray paint (Rust-Oleum) for custom color.
- **Waterproofing:** Optional silicone sealant around cutouts if robot will operate in humid/wet environments (not required for indoor use).
- **Alternative Materials:** Could use metal toolbox or project enclosure, but kitchen bin offers better size/cost ratio.
- **Weight:** Complete Torso module ~3-4kg (bin + reinforcement + Pi + components). Acceptable for Base module to support.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
