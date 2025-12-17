# Story 2.3: Order and Receive Neck PCB from Elecrow

**Epic:** Epic 2 - Neck Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 2-3 weeks (lead time), 1 hour (ordering)

---

## User Story

**As a** builder,
**I want** the Neck PCB ordered from Elecrow and delivered,
**so that** I can proceed with assembly.

---

## Acceptance Criteria

1. ✅ Gerber files uploaded to Elecrow website
2. ✅ PCB specifications confirmed (layer count, thickness, color, quantity, copper weight)
3. ✅ Order placed and payment completed
4. ✅ PCB received and visually inspected for manufacturing defects
5. ✅ Continuity test confirms no shorts between power rails
6. ✅ PCB dimensions and mounting holes match design specifications

---

## Implementation Steps

### 1. Prepare and Upload Gerbers

```bash
cd ~/olaf/modules/neck/hardware/gerbers
zip neck-pcb-gerbers.zip *.GTL *.GBL *.GTS *.GBS *.GTO *.GBO *.TXT *.GML

# Verify ZIP contents
unzip -l neck-pcb-gerbers.zip
```

### 2. Configure Elecrow Order

**PCB Specifications:**
- **Dimensions:** 90mm × 70mm
- **Layers:** 2
- **Quantity:** 5 or 10
- **Thickness:** 1.6mm
- **Color:** Green (or preference)
- **Surface Finish:** HASL (lead-free)
- **Copper Weight:** 2oz (for servo power traces) - IMPORTANT for high current
- **Min Track/Spacing:** 6/6 mil

**Cost Estimate:** ~$25-40 + shipping

### 3. Order and Track

Follow same process as Story 1.3:
1. Upload gerbers to Elecrow
2. Review preview carefully (check servo power traces visible and thick)
3. Place order
4. Document order details in `modules/neck/hardware/pcb-order-log.md`
5. Track delivery (2-3 weeks)

### 4. Inspection on Receipt

**Critical Checks:**
```bash
# Continuity tests (multimeter):
# 1. 7.4V to GND: Open (>10MΩ)
# 2. 5V to GND: Open
# 3. 3.3V to GND: Open
# 4. GND pads: Continuity (0Ω) - ground plane connected
# 5. Servo data line: Trace continuous through all connectors

# Dimensional check:
# - Board: 90mm × 70mm (±0.2mm)
# - Mounting holes: 3.2mm diameter (±0.1mm)
```

---

## Testing & Validation

Same as Story 1.3 (Gerber preview, power rail isolation, visual inspection)

---

## Troubleshooting

**Issue: High-Current Traces Look Too Thin**
- **Solution:** Verify 2oz copper option selected. If standard 1oz shipped, may need to add solder to beef up traces or re-order.

---

## Dependencies

**Before this story:**
- Story 2.2: Design Neck Custom PCB in Fritzing ✅

**After this story:**
- Story 2.4: Assemble and Test Neck PCB

---

## References

- [Elecrow PCB Manufacturing](https://www.elecrow.com/pcb-manufacturing.html)
- [Copper Weight Selection Guide](https://www.4pcb.com/pcb-design-specifications.html)

---

## Notes

- **2oz Copper Critical:** Servo power traces need extra thickness for 4A current. Don't skip this spec.
- **Lead Time:** Same 2-3 weeks as Head+Ears PCB. Can order in parallel.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
