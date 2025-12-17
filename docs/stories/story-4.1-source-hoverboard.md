# Story 4.1: Source and Disassemble Hoverboard for Parts

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** to source a donor hoverboard and disassemble it to extract the 36V battery and BLDC motors,
**so that** I have the core power and propulsion components for the Base module.

---

## Acceptance Criteria

1. ✅ Hoverboard sourced (used or broken OK, must have functional battery and motors)
2. ✅ Hoverboard safely disassembled using proper tools
3. ✅ 36V lithium battery pack extracted and tested (voltage, capacity)
4. ✅ 2× BLDC hub motors (6.5" or 8" wheels) removed and tested
5. ✅ Motor specifications documented: KV rating, resistance, hall sensor wiring
6. ✅ Battery BMS (Battery Management System) functional and intact
7. ✅ Battery connector and wiring harness preserved
8. ✅ Safety procedures followed: battery disconnected first, no short circuits
9. ✅ All parts organized and labeled for Base module assembly

---

## Implementation Steps

### 1. Source a Donor Hoverboard

**Where to buy:**
- eBay: Search "broken hoverboard" or "hoverboard parts" ($50-150)
- Craigslist/Facebook Marketplace: Local pickup, negotiate price
- Repair shops: Ask for non-repairable units
- Amazon/AliExpress: New hoverboards ($100-200) if used not available

**Recommended models:**
- Generic 6.5" or 8" wheel hoverboards (most common)
- 36V battery (standard voltage)
- 250W-350W motors per wheel
- Avoid fancy models with Bluetooth/LEDs (unnecessary cost)

**What to look for:**
- Battery: Must hold charge (even partially), no swelling/damage
- Motors: Should spin freely, hall sensors intact
- BMS: Visible circuit board on battery, no burn marks
- Condition: Broken mainboard OK, physical damage to battery/motors = AVOID

**Cost estimate:**
- Used broken hoverboard: $50-100
- Working hoverboard: $150-200

### 2. Prepare Disassembly Workspace

**Safety first:**
```bash
# Safety equipment required:
- Safety glasses (protect from springs/screws)
- Insulated gloves (for battery handling)
- Multimeter (test voltages)
- Fire extinguisher nearby (lithium batteries)
- Non-conductive work surface (rubber mat)
- Ventilated area (no flammable materials nearby)
```

**Tools required:**
- Phillips screwdriver (PH2)
- Flathead screwdriver (small)
- Allen key set (metric)
- Wire cutters (for cable ties)
- Multimeter (voltage measurement)
- Electrical tape
- Ziplock bags (for screws/small parts)
- Label maker or masking tape + marker

### 3. Disassemble Hoverboard Housing

**Step-by-step disassembly:**

```bash
# Step 1: Remove bottom cover screws
#   - Typically 10-15 Phillips screws
#   - Keep screws in labeled bag: "Bottom Cover"

# Step 2: Open case (two halves)
#   - Gently pry apart with flathead screwdriver
#   - Watch for wires connecting two halves
#   - Take photo of internal layout before disconnecting

# Step 3: Identify components:
#   - Battery pack (largest component, rectangular)
#   - Mainboard (circuit board with plugs)
#   - 2× Motor cables (thick 5-wire or 8-wire bundles)
#   - Power switch
#   - Charging port
```

### 4. Safely Disconnect and Extract Battery

**CRITICAL SAFETY STEP:**

```bash
# Step 1: Measure battery voltage BEFORE disconnecting
#   - Set multimeter to DC voltage (60V range)
#   - Touch red probe to battery + terminal
#   - Touch black probe to battery - terminal
#   - Expected: 36-42V (fully charged), 30-36V (discharged)
#   - If 0V: Battery dead, may not be usable

# Step 2: Disconnect battery from mainboard
#   - Locate battery connector (typically XT60 or XT90)
#   - Pull connector straight out (do NOT twist)
#   - Immediately cover connector terminals with electrical tape
#   - NEVER short battery terminals (causes fire/explosion)

# Step 3: Remove battery from case
#   - Unscrew mounting brackets (usually 4 screws)
#   - Lift battery out carefully (heavy: 2-3kg)
#   - Inspect for damage: swelling, punctures, burn marks
#   - If damaged: DO NOT USE, dispose at battery recycling center

# Step 4: Label battery
#   - Write voltage (36V) and capacity (e.g., 4.4Ah) on masking tape
#   - Attach to battery with tape
```

**Battery safety rules:**
- Never puncture or open battery case
- Never short terminals
- Store in cool, dry place (away from flammables)
- If battery smells bad or is hot: Move outside immediately
- Discharge to 30-40% for long-term storage (safer than full charge)

### 5. Remove BLDC Motors from Wheels

**Motor extraction:**

```bash
# Step 1: Disconnect motor cables from mainboard
#   - Each motor has 5-8 wires (3× phase + 5× hall sensors)
#   - Take photo of connector pinout
#   - Label wires with tape: "Left Motor" or "Right Motor"
#   - Carefully pull connectors from mainboard

# Step 2: Remove wheel assemblies
#   - Unscrew axle bolts (4-6 per wheel, M5 or M6)
#   - Keep bolts in labeled bag: "Motor Mounting Hardware"
#   - Lift wheel assembly out of frame

# Step 3: Remove tire from hub motor (optional)
#   - Tire is press-fit onto hub motor rim
#   - Use flathead screwdriver to pry tire off rim
#   - Work around circumference slowly
#   - Tire removal not required if using wheels as-is
```

### 6. Test and Document Motors

**Motor testing:**

```bash
# Test 1: Spin test
#   - Manually spin motor by hand
#   - Should spin smoothly with slight magnetic cogging
#   - Resistance from back-EMF (generates voltage when spinning)
#   - No grinding, clicking, or stuck spots

# Test 2: Hall sensor test
#   - Use multimeter in continuity mode
#   - Test hall sensor wires for continuity to ground
#   - Spin motor slowly, hall sensors should toggle on/off
#   - Expected: 5 wires (VCC, GND, Hall A, Hall B, Hall C)

# Test 3: Phase resistance measurement
#   - Set multimeter to resistance (Ω) mode
#   - Measure resistance between phase wires:
#     - Phase A ↔ Phase B: ~0.1-1Ω (low resistance)
#     - Phase B ↔ Phase C: ~0.1-1Ω
#     - Phase C ↔ Phase A: ~0.1-1Ω
#   - All three measurements should be similar
#   - If open circuit (infinite resistance): Motor damaged

# Test 4: Back-EMF test (optional)
#   - Connect multimeter to two phase wires (AC voltage mode)
#   - Spin motor by hand
#   - Should generate 1-10V AC (depends on spin speed)
#   - Confirms motor windings intact
```

**Motor specifications to document:**

Create `modules/base/motor_specs.md`:

```markdown
# Hoverboard Motor Specifications

## Motor Type
- BLDC hub motor (brushless DC)
- Wheel size: 6.5" or 8"
- Power rating: 250-350W per motor

## Electrical Specs
- Voltage: 36V nominal
- KV rating: ~50-70 RPM/V (estimated)
- Phase resistance: [measured value] Ω
- Hall sensors: 5-wire (VCC, GND, A, B, C)

## Pinout (Document from photo)
Left Motor:
- Phase A: [wire color]
- Phase B: [wire color]
- Phase C: [wire color]
- Hall VCC: [wire color, typically red]
- Hall GND: [wire color, typically black]
- Hall A: [wire color]
- Hall B: [wire color]
- Hall C: [wire color]

Right Motor:
[Same pinout]

## Mounting
- Hub diameter: [measure in mm]
- Bolt pattern: [number of bolts, spacing in mm]
- Axle diameter: [measure in mm]
```

### 7. Inspect and Test Battery BMS

**BMS verification:**

```bash
# The BMS (Battery Management System) is the circuit board attached to battery
# It protects battery from:
#   - Overcharge (>42V)
#   - Over-discharge (<30V)
#   - Overcurrent (>10-15A)
#   - Short circuit
#   - Cell imbalance

# Visual inspection:
#   - Look for burn marks, blown components
#   - Check all solder joints intact
#   - Verify no corrosion on terminals

# Functional test:
#   1. Measure battery voltage at main terminals
#   2. Connect a load (10Ω resistor, 5W) across terminals
#   3. Voltage should remain stable (not drop rapidly)
#   4. BMS should allow discharge (voltage present at terminals)
#   5. If voltage drops to 0V: BMS protection triggered or BMS faulty

# If BMS is dead:
#   - Replace with generic 36V 10S BMS (available on Amazon/eBay for $15-30)
#   - Match voltage (36V = 10S lithium)
#   - Match current rating (15-20A continuous minimum)
```

### 8. Salvage Additional Useful Components

**Optional components to save:**

```bash
# 1. Power switch
#   - Rocker switch (usually 10A rated)
#   - Can be reused for Base module master power

# 2. Charging port
#   - 36V barrel jack (2.1mm or 2.5mm)
#   - Reuse for robot charging

# 3. Wiring harness
#   - XT60/XT90 connectors (battery to ESC)
#   - High-current wire (12-14 AWG silicone)
#   - Save for Base module power distribution

# 4. Mainboard (optional)
#   - Even if broken, has useful connectors and power traces
#   - Can be used for reference or parts
```

### 9. Organize and Store Components

**Component storage:**

```bash
# Create labeled storage:
#   - Battery: Separate plastic bin, stored at 40% charge
#   - Motors: Wrap in bubble wrap, store upright
#   - Small parts: Ziplock bags with labels
#   - Wiring: Coil neatly, label with tape

# Storage location:
#   - Cool, dry area (not garage/attic if temperature extremes)
#   - Battery: Away from flammables
#   - Motors: Protected from moisture

# Label everything:
#   "OLAF Base Module Parts"
#   "36V Battery - Handle with Care"
#   "Left Motor - Tested OK"
#   "Right Motor - Tested OK"
```

### 10. Dispose of Remaining Parts Responsibly

**Waste disposal:**

```bash
# Recycle:
#   - Plastic shell: Curbside recycling (check local rules)
#   - Mainboard: E-waste recycling center
#   - Wires/connectors: Keep for future projects or e-waste

# Do NOT trash:
#   - Battery (if damaged): Take to battery recycling center
#   - Electronics: Contain heavy metals, require proper disposal
```

---

## Testing & Validation

**Test 1: Battery Voltage**
```bash
# Measure with multimeter
# Expected: 36-42V (healthy battery)
# If <30V: Battery over-discharged, may not recover
```

**Test 2: Battery Capacity (Approximate)**
```bash
# Discharge test (if battery charged):
#   - Connect 36V 10W LED bulb as load
#   - Measure current: ~0.3A
#   - Time how long until BMS cuts off (battery at ~30V)
#   - Capacity (Ah) ≈ Current × Time
#   - Example: 0.3A × 12 hours = 3.6Ah
```

**Test 3: Motor Spin Test**
```bash
# Both motors spin freely
# No grinding or resistance
```

**Test 4: Hall Sensor Functionality**
```bash
# Hall sensors toggle when motor spins
# All 3 sensors respond
```

---

## Troubleshooting

**Issue 1: Battery Voltage 0V**
- **Solution:** BMS protection active or battery dead. Try charging with 42V charger (if available). If still 0V, replace BMS or buy new battery.

**Issue 2: Motor Won't Spin Freely**
- **Solution:** Internal damage or rust. Try adding penetrating oil to bearings. If still stuck, motor may be unusable.

**Issue 3: Hall Sensors Not Responding**
- **Solution:** Check wiring continuity. Hall sensors may be damaged. ODrive can run motors without hall sensors (sensorless mode).

**Issue 4: Battery Swollen or Hot**
- **Solution:** DO NOT USE. Battery is damaged and dangerous. Dispose immediately at recycling center.

**Issue 5: Can't Find Affordable Hoverboard**
- **Solution:** Check multiple sources (eBay, Craigslist, FB Marketplace). Post "wanted" ad. Consider buying 36V battery ($80) and motors ($40 each) separately.

---

## Dependencies

**Before this story:**
- None (first story in Epic 4)

**After this story:**
- Story 4.2: Breadboard Base Components and Test Connectivity
- Story 4.3: Design Power Distribution System for 36V Battery

---

## References

- [Hoverboard Disassembly Guide](https://www.instructables.com/How-to-Disassemble-a-Hoverboard/)
- [Lithium Battery Safety](https://batteryuniversity.com/article/bu-304a-safety-concerns-with-li-ion)
- [BLDC Motor Basics](https://www.monolithicpower.com/en/brushless-dc-motor-fundamentals)

---

## Notes

- **Battery Safety:** Lithium batteries can catch fire if damaged or short-circuited. Always handle with care.
- **Motor Compatibility:** Most hoverboard motors are compatible with ODrive. 6.5" and 8" wheels both work.
- **Cost Savings:** Using hoverboard parts saves $200-300 vs buying battery and motors separately.
- **Wheel Size:** 6.5" wheels = more torque, 8" wheels = higher top speed. Both work for OLAF.
- **Battery Capacity:** Most hoverboards have 4-4.4Ah battery. Runtime for OLAF: ~30-60 minutes (depends on usage).
- **Future Upgrades:** Can replace battery with higher capacity (8Ah+) for longer runtime.

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
