# Story 1.3: Order and Receive Head+Ears PCB from Elecrow

**Epic:** Epic 1 - Head+Ears Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 2-3 weeks (lead time), 1 hour (ordering)

---

## User Story

**As a** builder,
**I want** the Head+Ears PCB ordered from Elecrow and delivered,
**so that** I can proceed with assembly.

---

## Acceptance Criteria

1. ✅ Gerber files uploaded to Elecrow website
2. ✅ PCB specifications confirmed (layer count, thickness, color, quantity)
3. ✅ Order placed and payment completed
4. ✅ PCB received and visually inspected for manufacturing defects
5. ✅ Continuity test confirms no shorts between power rails
6. ✅ PCB dimensions and mounting holes match design specifications

---

## Implementation Steps

### 1. Prepare Gerber Files for Upload

```bash
cd ~/olaf/modules/head-ears/hardware/gerbers/

# Verify all required files present
ls -la
# Expected:
# head-ears-pcb.GTL (top copper)
# head-ears-pcb.GBL (bottom copper)
# head-ears-pcb.GTS (top soldermask)
# head-ears-pcb.GBS (bottom soldermask)
# head-ears-pcb.GTO (top silkscreen)
# head-ears-pcb.GBO (bottom silkscreen)
# head-ears-pcb.TXT (drill file)
# head-ears-pcb.GML (board outline)

# Create ZIP file for upload
zip head-ears-pcb-gerbers.zip *.GTL *.GBL *.GTS *.GBS *.GTO *.GBO *.TXT *.GML
```

### 2. Create Elecrow Account and Navigate to PCB Order Page

1. Visit: https://www.elecrow.com/
2. Create account or log in
3. Navigate to: Services → PCB Manufacturing
4. Or direct link: https://www.elecrow.com/pcb-manufacturing.html

### 3. Configure PCB Specifications

**Order Form Settings:**

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Layer** | 2 Layer | Standard double-sided |
| **Dimensions** | 100mm × 80mm | Measure from Gerber viewer |
| **Quantity** | 5 or 10 | Small batch for prototyping |
| **Thickness** | 1.6mm | Standard |
| **Color** | Green | Cheapest option (or choose preference) |
| **Surface Finish** | HASL | Lead-free HASL recommended |
| **Copper Weight** | 1 oz | Standard for signal traces |
| **Gold Fingers** | No | Not needed |
| **Flying Probe Test** | No | Optional, adds cost |
| **Min. Track/Spacing** | 6/6 mil | Standard (design uses 10 mil) |

**Lead Time:**
- Standard: 7-10 business days + shipping (3-7 days) = **2-3 weeks total**
- Express: Available for extra cost (not necessary for prototype)

### 4. Upload Gerber Files

1. Click "Add Gerber File"
2. Upload: `head-ears-pcb-gerbers.zip`
3. Wait for online Gerber viewer to load
4. **CRITICAL:** Visually inspect the preview:
   - Check all layers display correctly
   - Verify board outline
   - Confirm component pads and traces visible
   - Check mounting hole positions

**If preview looks incorrect:**
- Re-export Gerbers from Fritzing
- Check file naming convention matches Elecrow requirements
- Contact Elecrow support for assistance

### 5. Review and Place Order

**Quote Review:**
- Typical cost for 5× 100×80mm boards: ~$20-30 USD
- Shipping (DHL/ePacket): ~$15-25 USD
- **Total: ~$35-55 USD**

**Add to Cart:**
1. Review specifications one final time
2. Add to cart
3. Proceed to checkout

**Payment:**
- PayPal or credit card accepted
- Keep confirmation email with order number

### 6. Track Order and Prepare for Receipt

**After Order Placement:**
1. Receive confirmation email with order number (e.g., ELE123456)
2. Elecrow will review Gerbers (1-2 business days)
3. If issues found, Elecrow will email for clarification
4. Once approved, manufacturing begins (5-7 days)
5. Shipping notification with tracking number
6. Delivery (3-7 days depending on shipping method)

**Document Order:**
Create `modules/head-ears/hardware/pcb-order-log.md`:

```markdown
# Head+Ears PCB Order Log

## Order 1 - Prototype
- **Date Ordered:** 2025-XX-XX
- **Supplier:** Elecrow
- **Order Number:** ELE123456
- **Quantity:** 5 boards
- **Cost:** $XX.XX (PCBs) + $XX.XX (shipping) = $XX.XX total
- **Tracking:** [tracking number]
- **Expected Delivery:** 2025-XX-XX
- **Actual Delivery:** [date received]
- **Quality:** [notes after inspection]
```

### 7. Receive and Inspect PCBs

**Upon Delivery:**

1. **Visual Inspection:**
   - Check for physical damage during shipping
   - Count boards (should match order quantity)
   - Inspect silkscreen (readable, aligned)
   - Check soldermask (no scratches, uniform color)
   - Verify mounting holes drilled correctly

2. **Dimensional Check:**
   ```bash
   # Measure with calipers:
   # - Board length: 100mm ± 0.2mm
   # - Board width: 80mm ± 0.2mm
   # - Mounting hole diameter: 3.2mm ± 0.1mm
   # - Mounting hole spacing: [measure and document]
   ```

3. **Electrical Continuity Test (CRITICAL):**
   ```bash
   # Use multimeter in continuity mode:
   # 1. Test for shorts between power rails:
   #    - 5V to GND: Should be open (no continuity)
   #    - 3.3V to GND: Should be open
   #    - 5V to 3.3V: Should be open
   # 2. Test expected connections:
   #    - GND pads: Should have continuity (ground plane)
   #    - Signal traces: Check a few critical paths
   ```

**Pass/Fail Criteria:**
- ✅ **PASS:** No shorts, all dimensions within tolerance, no physical defects
- ❌ **FAIL:** Shorts between power rails → Contact Elecrow for replacement
- ⚠️ **MINOR ISSUES:** Cosmetic defects (silkscreen misalignment) → Document but proceed

**Photo Documentation:**
Take photos of:
- Boards as received (packaging)
- Top side of PCB
- Bottom side of PCB
- Any defects found

Save photos to: `modules/head-ears/hardware/photos/pcb-v1-received/`

---

## Testing & Validation

**Test 1: Gerber Preview Before Order**
```bash
# Use online Gerber viewer
# Visit: https://www.pcbway.com/project/OnlineGerberViewer.html
# Upload ZIP file
# Verify: All layers, correct dimensions, no missing traces
```

**Test 2: Power Rail Isolation**
```bash
# With multimeter in continuity/resistance mode:
# Probe between 5V and GND on PCB → Should read >10MΩ (open)
# Probe between 3.3V and GND → Should read >10MΩ
# If readings <100Ω, likely short → Inspect visually under magnification
```

**Test 3: Ground Plane Continuity**
```bash
# Probe between any two GND pads → Should have continuity (0Ω)
# If no continuity, ground plane may not be connected properly
```

**Test 4: Mounting Hole Alignment**
```bash
# Place PCB on cardboard template of enclosure
# Or use CAD printout of enclosure design
# Verify holes align ± 0.5mm tolerance
```

---

## Troubleshooting

**Issue 1: Gerber Upload Failed**
- **Symptom:** Elecrow website rejects ZIP file
- **Solution:**
  - Check file size (<50MB)
  - Verify ZIP contains only Gerber files (no .fzz or other files)
  - Re-export from Fritzing with different naming convention
  - Try uploading individual files instead of ZIP

**Issue 2: Gerber Preview Shows Errors**
- **Symptom:** Missing traces, incorrect board outline, no drill holes
- **Solution:**
  - Check drill file format (Excellon, .TXT extension)
  - Verify board outline layer (GML or GKO file present)
  - Re-export Gerbers, ensure all layers selected in export dialog

**Issue 3: Elecrow Requests Design Clarification**
- **Symptom:** Email from Elecrow asking about trace width or spacing
- **Solution:**
  - Respond promptly to avoid delays
  - Confirm design meets their minimum specs (6/6 mil)
  - If issue critical, may need to redesign and re-order

**Issue 4: PCB Arrives with Short Between Power Rails**
- **Symptom:** Multimeter shows continuity between 5V and GND
- **Solution:**
  - Visual inspection under magnification (look for solder bridges, copper slivers)
  - Check for conductive debris (clean with isopropyl alcohol)
  - If manufacturing defect, contact Elecrow with photos for replacement
  - Keep defective board for comparison/learning

**Issue 5: Mounting Holes Don't Align with Enclosure**
- **Symptom:** Holes off by >1mm
- **Solution:**
  - Verify enclosure design is accurate (re-measure)
  - Check PCB dimensions with calipers (manufacturing tolerance ±0.2mm)
  - If Elecrow error (rare), request replacement
  - If design error, proceed with assembly, revise for v2.0

**Issue 6: Long Delivery Time (>4 weeks)**
- **Symptom:** Order stuck in customs or lost in shipping
- **Solution:**
  - Check tracking number status
  - Contact Elecrow support with order number
  - May need to file shipping claim or request re-ship
  - Plan ahead: order PCBs early to allow buffer time

---

## Dependencies

**Before this story:**
- Story 1.2: Design Head+Ears Custom PCB in Fritzing ✅
- Gerber files exported and verified
- Elecrow account created
- Payment method ready

**After this story:**
- Story 1.4: Assemble and Test Head+Ears PCB
- Order components from BOM while waiting for PCB delivery (parallel task)

---

## References

- [Elecrow PCB Manufacturing](https://www.elecrow.com/pcb-manufacturing.html)
- [Elecrow PCB Specifications](https://www.elecrow.com/download/PCB_Specification.pdf)
- [Online Gerber Viewer](https://www.pcbway.com/project/OnlineGerberViewer.html)
- [Gerber File Format Guide](https://www.ucamco.com/en/gerber)

---

## Notes

- **Lead Time Planning:** PCB manufacturing is the longest wait in the build process. Order early, work on other modules in parallel.
- **Quantity:** 5 boards minimum recommended (keep spares for mistakes, revisions, or sharing with community).
- **Cost Optimization:** Elecrow is cost-effective for small batches. Alternatives: JLCPCB, PCBWay, OSH Park (USA-based, faster but pricier).
- **Shipping Method:** ePacket (cheap, 2-3 weeks) vs DHL Express (expensive, 3-5 days). For prototypes, ePacket sufficient.
- **Customs:** Packages from China may incur import duties (varies by country). Budget extra 10-20% for potential fees.
- **Quality:** Elecrow has good reputation for quality. Defect rate typically <1%. Shorts between power rails are RARE but check anyway.
- **Design Iteration:** Expect to find issues during assembly (Story 1.4). Plan for v2.0 PCB order after testing, but v1.0 should be functional.
- **Parallel Work:** While waiting for PCB, complete Stories 1.5 (enclosure design), 1.6 (firmware), and order components from BOM.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
