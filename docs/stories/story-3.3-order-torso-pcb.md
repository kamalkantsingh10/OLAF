# Story 3.3: Order and Receive Torso PCB from Elecrow

**Epic:** Epic 3 - Torso Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 2-3 weeks (lead time), 1 hour (ordering)

---

## User Story

**As a** builder,
**I want** the Torso PCB ordered from Elecrow and delivered,
**so that** I can proceed with assembly.

---

## Acceptance Criteria

1. ✅ Gerber files uploaded to Elecrow
2. ✅ PCB specifications confirmed (2-layer, 1.6mm, 1oz copper, 5-10 boards)
3. ✅ Order placed and payment completed
4. ✅ PCB received and visually inspected
5. ✅ Continuity test confirms no shorts between power rails
6. ✅ Dimensions match design

---

## Implementation Steps

Follow same process as Story 1.3:
1. Prepare gerbers ZIP
2. Upload to Elecrow (80mm×60mm, 2-layer, green, HASL)
3. Review preview
4. Place order (~$20-30 + shipping)
5. Track delivery (2-3 weeks)
6. Inspect on receipt (shorts test, dimensions)

Document in `modules/torso/hardware/pcb-order-log.md`

---

## Testing & Validation

Power rail isolation test (5V to GND: >10MΩ, 3.3V to GND: >10MΩ)

---

## Dependencies

**Before:** Story 3.2 ✅
**After:** Story 3.4

---

**Created:** 2025-12-16
