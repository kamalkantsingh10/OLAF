# Story 4.5: Order and Receive Base PCB from Elecrow

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 2-3 weeks lead time

---

## User Story

**As a** builder,
**I want** to order the Base PCB from Elecrow and receive manufactured boards,
**so that** I can proceed with PCB assembly.

---

## Acceptance Criteria

1. ✅ Gerber files uploaded to Elecrow website
2. ✅ PCB specifications configured: 2-layer, 2oz copper, 1.6mm thickness, HASL finish
3. ✅ Order placed with tracking number
4. ✅ PCBs received within 2-3 weeks
5. ✅ Visual inspection: No manufacturing defects
6. ✅ Continuity test: All traces intact
7. ✅ Dimensional check: Mounting holes align with design

---

## Implementation Steps

### 1. Prepare Gerber Files

```bash
cd ~/olaf/modules/base/pcb/base-pcb

# Zip Gerber files
zip base-pcb-gerbers.zip *.gbl *.gtl *.gbs *.gts *.gbo *.gto *.gm1 *.txt

# Verify zip contains all files:
unzip -l base-pcb-gerbers.zip
```

### 2. Upload to Elecrow

```
Website: https://www.elecrow.com/pcb-manufacturing.html

Steps:
1. Click "Add Gerber File"
2. Upload base-pcb-gerbers.zip
3. Wait for automatic detection (30 seconds)
4. Verify board dimensions: 120mm × 100mm
```

### 3. Configure PCB Specifications

```
PCB Specifications:
─────────────────────────────────────────
Base Material:        FR-4
Layers:               2
Dimensions:           120mm × 100mm
PCB Thickness:        1.6mm
Copper Weight:        2 oz (70µm)
Surface Finish:       HASL (Lead-Free)
Soldermask Color:     Green
Silkscreen Color:     White
Min Track/Spacing:    6/6 mil (0.15mm)
Min Hole Size:        0.3mm
Castellated Holes:    No
Edge Connector:       No
─────────────────────────────────────────
```

### 4. Select Quantity and Lead Time

```
Quantity:    5 pcs (minimum for prototyping)
Lead Time:   7-10 days (production + shipping)
Shipping:    DHL Express (faster, ~$20)
             or China Post (slower, ~$5)

Cost Estimate:
  PCB (5 pcs):     $15-25
  Shipping:        $5-20
  Total:           $20-45
```

### 5. Place Order

```bash
# Review order summary
# Add to cart
# Proceed to checkout
# Enter shipping address
# Select payment method (PayPal/Credit Card)
# Complete payment
# Note order number and tracking number (arrives via email)
```

### 6. Track Shipment

```bash
# Production: 7-10 days
# Shipping: 5-15 days (depends on method)
# Total: 2-3 weeks typically

# Track on Elecrow website or DHL/postal service
```

### 7. Receive and Inspect PCBs

**Visual inspection checklist:**

```bash
✓ Board dimensions correct (120mm × 100mm)
✓ All mounting holes present (4× M3)
✓ No cracks or chips
✓ Soldermask evenly applied (no bare copper)
✓ Silkscreen legible
✓ No bridged traces
✓ Pads intact and well-defined
✓ XT60 connector holes correct size
✓ Screw terminal holes aligned
```

### 8. Electrical Testing

**Continuity test:**

```bash
# Use multimeter in continuity mode

# Test 1: Power traces
#   Battery (+) pad → ODrive (+) pad (via fuse holder)
#   Should beep (continuity)

# Test 2: Ground plane
#   Any GND pad → Any other GND pad
#   Should beep

# Test 3: Isolated traces
#   (+) trace → GND
#   Should NOT beep (open circuit)

# Test 4: I2C traces
#   ESP32 SDA pad → JST SDA pad
#   Should beep
```

### 9. Dimensional Verification

```bash
# Measure with calipers:
#   Board length: 120mm ± 0.5mm
#   Board width: 100mm ± 0.5mm
#   Mounting hole spacing: [design values] ± 0.2mm
#   Mounting hole diameter: 3.2mm ± 0.1mm

# Fit test:
#   Place M3 screws through mounting holes
#   Should fit easily without force
```

### 10. Document Receipt

**Create `modules/base/pcb/manufacturing_notes.md`:**

```markdown
# Base PCB Manufacturing Notes

## Order Details
- Manufacturer: Elecrow
- Order Date: 2025-12-17
- Order Number: [number]
- Quantity: 5 pcs
- Lead Time: 14 days
- Cost: $35 (including shipping)

## Specifications
- 2-layer, 2oz copper, HASL finish
- Dimensions: 120mm × 100mm
- Green soldermask, white silkscreen

## Inspection Results
- Visual: Pass ✓
- Continuity: Pass ✓
- Dimensional: Pass ✓

## Photos
[Include photos of received PCBs]

## Notes
- All 5 boards identical, no defects
- Ready for assembly (Story 4.6)
```

---

## Testing & Validation

**Test 1: Visual Inspection**
```bash
# No manufacturing defects visible
# All silkscreen labels legible
```

**Test 2: Electrical Continuity**
```bash
# All expected connections have continuity
# No unintended shorts between traces
```

**Test 3: Dimensional Check**
```bash
# Board dimensions within tolerance
# Mounting holes align with design
```

---

## Troubleshooting

**Issue 1: PCBs Not Received After 3 Weeks**
- **Solution:** Contact Elecrow support, check tracking number, request replacement if lost

**Issue 2: Manufacturing Defects Found**
- **Solution:** Document with photos, contact Elecrow for replacement/refund

**Issue 3: Wrong Dimensions**
- **Solution:** Check Gerber files match design, contact Elecrow, order replacement

**Issue 4: Traces Bridged**
- **Solution:** Attempt to cut bridge with knife, if unsuccessful request replacement

---

## Dependencies

**Before this story:**
- Story 4.4: Design Base Custom PCB ✅

**After this story:**
- Story 4.6: Assemble and Test Base PCB

---

## References

- [Elecrow PCB Service](https://www.elecrow.com/pcb-manufacturing.html)
- [PCB Manufacturing Guide](https://docs.oshpark.com/design-tools/checklist/)

---

## Notes

- **Lead Time:** Plan ahead - PCB manufacturing adds 2-3 weeks to project timeline
- **Quantity:** Order 5+ boards for backups (mistakes during assembly)
- **Cost:** Very affordable ($4-5 per board for small quantities)
- **Alternatives:** OSH Park (US-based, faster but pricier), JLCPCB (similar to Elecrow), PCBWay
- **Future:** Consider assembly service (PCBA) where manufacturer solders components - adds cost but saves time

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
