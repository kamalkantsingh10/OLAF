# Epic 2: 3D Design & Physical Structure

**Epic Goal:** Design all mechanical components for Olaf in OnShape (cloud-based CAD), including head housing (OAK-D Pro camera mount, dual OLED eye sockets, mmWave sensor bay, microphone mounts), ear articulation mounts (2-DOF Feetech SCS0009 servo brackets), neck gimbal (3-DOF STS3215 servo assembly), body chassis (Raspberry Pi mount, battery compartment, buck converter housing, DLP projector angle mount), and base platform (hoverboard motor integration, ODrive mounting). Export printable STL files, 3D print all components, and assemble physical structure validating mechanical ranges (neck pan ±90°, tilt ±45°, roll ±30°, ear 2-DOF movement) and component fitment.

**Duration:** 2-3 weeks (Weeks 2-4, overlaps Epic 1 end)

**Prerequisites:** Epic 1 Story 1.1 (repository structure), Story 1.2 (power system for weight/sizing constraints)

**Value Delivered:** Physical robot structure ready for full personality system (Epic 3), provides tangible build-in-public content (photos/videos of assembly), validates mechanical design before full electronics integration, enables community to replicate physical build.

---

## Story 2.1: OnShape Setup & Component Research

**As a** mechanical designer,
**I want** OnShape configured and key component datasheets collected,
**so that** I can design accurate CAD models that fit actual hardware dimensions.

### Acceptance Criteria:

1. **OnShape Account:**
   - Free OnShape account created (sufficient for public projects)
   - Public workspace created: "Olaf - Open-Source AI Companion"
   - Workspace set to public visibility (aligns with open-source commitment)

2. **Component Datasheets Collected:**
   - **OAK-D Pro:** Dimensions, mounting holes, lens position (from Luxonis specs)
   - **OLED displays (2x):** Screen size (128x64 mm actual dimensions), mounting holes
   - **mmWave sensor:** PCB dimensions, antenna orientation requirements
   - **Microphone array:** Physical size, mounting requirements
   - **Feetech SCS0009 servos (4x):** Body dimensions, horn attachment, rotation range
   - **Feetech STS3215 servos (3x):** Larger body size, torque specs, horn type
   - **Raspberry Pi 8GB:** Board dimensions, mounting holes (M2.5), port locations
   - **Buck converters (2x):** PCB size, heat sink clearance
   - **DLP projector:** Dimensions, lens position, throw distance
   - **Hoverboard motors/wheels:** Wheel diameter, axle width, motor body size

3. **Reference Dimensions Document:**
   - Spreadsheet created: `hardware/component-dimensions.csv`
   - All components listed with length × width × height, weight (if available)
   - Mounting hole patterns documented
   - Committed to repository

4. **OnShape Tutorials Completed:**
   - Basic sketch tutorial (if new to OnShape)
   - Assembly tutorial (mating parts)
   - Export to STL tutorial
   - Estimated 2-4 hours for OnShape learning curve (if new user)

5. **Design Constraints Documented:**
   - Maximum head weight: Based on STS3215 neck servo torque (estimate 500-800g max for safe operation)
   - Minimum cable routing space: 10mm diameter channels for wire bundles
   - Ventilation requirements: Raspberry Pi + buck converters need airflow
   - Battery compartment size: Fit hoverboard battery (measure actual battery dimensions)

**Dependencies:** Epic 1 Story 1.1 (repo for documentation)

**Estimated Effort:** 4-6 hours (includes research, datasheet collection, OnShape setup)

---

## Story 2.2: Head Housing CAD Design

**As a** CAD designer,
**I want** a head housing that securely mounts all sensors and displays,
**so that** Olaf's "face" components are protected and properly aligned.

### Acceptance Criteria:

1. **OnShape Part Studio Created:**
   - Part studio name: "Head Housing"
   - Sketch-based design (parametric for easy adjustments)

2. **Component Mounting Features:**
   - **OAK-D Pro mount:**
     - Front-facing camera position (centered on "face")
     - Mounting holes match OAK-D Pro spec (M2 or M3 threaded inserts planned)
     - Lens unobstructed, IR emitters clear
   - **Dual OLED eye sockets:**
     - Side-by-side placement, 40-50mm spacing (human-like eye distance scaled down)
     - Recessed pockets to hold OLED displays flush or slightly inset
     - Viewing angle: Tilted slightly outward (5-10°) for wider expression visibility
   - **mmWave sensor bay:**
     - Top or forehead position (360° coverage, minimal obstruction)
     - Antenna clearance (no metal/conductive material within 20mm)
   - **Microphone array mount:**
     - Side or top positions (multiple mics for directional audio)
     - Acoustic ports/holes to allow sound entry

3. **Cable Routing:**
   - Internal channels for:
     - OAK-D Pro USB cable
     - OLED I2C wires (2x displays)
     - mmWave sensor wires
     - Microphone wires
   - Cable exit point at bottom (toward neck connection)
   - Minimum 10mm diameter channels

4. **Structural Features:**
   - Wall thickness: 2-3mm (balance strength vs. print time/weight)
   - Mounting boss for neck attachment (M4 or M5 threaded insert)
   - Split design option: Front face + back housing (easier assembly/cable routing)
   - Rounded edges per branding guide (no sharp corners, friendly aesthetic)

5. **Ventilation:**
   - Small vent holes for air circulation (prevent heat buildup from OAK-D Pro)
   - Positioned away from OLED viewing area

6. **Weight Estimate:**
   - OnShape mass properties: Head housing + components ≤ 600g (within STS3215 torque limit)

7. **Documentation:**
   - OnShape public link added to `hardware/3d-models/README.md`
   - Design notes committed: Component placement rationale, cable routing plan

**Dependencies:** Story 2.1 (component dimensions, OnShape setup)

**Estimated Effort:** 8-12 hours (CAD design iteration)

---

## Story 2.3: Ear Articulation Mounts CAD Design

**As a** CAD designer,
**I want** ear mounts with 2-DOF articulation for Feetech SCS0009 servos,
**so that** ears can tilt vertically and rotate horizontally for expressive movement.

### Acceptance Criteria:

1. **OnShape Part Studio Created:**
   - Part studio name: "Ear Mounts" (2x mirrors for left/right)

2. **Servo Integration:**
   - **Base servo (horizontal rotation):**
     - Mount for SCS0009 servo body (secure, no wobble)
     - Servo horn attachment point
     - Rotation range: 180-270° (more than needed, will limit in software)
   - **Top servo (vertical tilt):**
     - Mounted to base servo horn
     - Second SCS0009 servo body mount
     - Ear "fin" attachment to top servo horn

3. **Ear Fin Design:**
   - Chappie-inspired shape: Triangular or curved fin, 60-80mm height
   - Lightweight: Thin walls (1.5-2mm), possibly with lattice infill
   - Weight per ear assembly (2 servos + fin): ≤150g

4. **Mounting to Head:**
   - Attachment points on sides of head housing
   - M3 or M4 bolts, possibly heat-set inserts
   - Cable routing from servos into head housing

5. **Range of Motion:**
   - Horizontal rotation (base servo): ±90° from center (180° total)
   - Vertical tilt (top servo): ±45° from neutral (90° total)
   - No mechanical interference at extreme angles (simulate in OnShape assembly)

6. **Wiring:**
   - Cable channels for servo daisy-chain wiring
   - Exit point toward head housing base

7. **Aesthetic:**
   - Rounded, organic shapes (not angular/robotic)
   - Retro-futurism styling per branding guide

8. **Documentation:**
   - OnShape public link
   - Assembly instructions: Servo installation order, horn alignment

**Dependencies:** Story 2.2 (head housing for attachment points)

**Estimated Effort:** 6-10 hours

---

## Story 2.4: Neck Gimbal CAD Design (3-DOF)

**As a** CAD designer,
**I want** a 3-DOF neck gimbal using Feetech STS3215 servos,
**so that** the head can pan (±90°), tilt (±45°), and roll (±30°) for expressive gestures.

### Acceptance Criteria:

1. **OnShape Assembly Created:**
   - Assembly name: "Neck Gimbal - 3 DOF"
   - 3 separate part studios: Pan base, Tilt yoke, Roll cradle

2. **Pan Axis (Base Servo):**
   - **STS3215 servo mount:** Vertical orientation, servo body secured to body chassis
   - **Servo horn attachment:** Large circular or cross-pattern horn
   - **Yoke mount:** Tilt yoke attaches to pan servo horn
   - **Rotation range:** ±90° (180° total sweep, software-limited for cable safety)
   - **Cable pass-through:** Hollow center or side channel for wires from head/tilt/roll servos

3. **Tilt Axis (Middle Servo):**
   - **STS3215 servo mount:** Horizontal orientation, mounted to pan yoke
   - **Servo horn attachment:** Connects to roll cradle
   - **Rotation range:** ±45° (90° total, look up/down)
   - **Balance point:** Center of mass of head + roll assembly aligned with tilt axis (prevents servo strain)

4. **Roll Axis (Top Servo):**
   - **STS3215 servo mount:** Attached to tilt yoke
   - **Servo horn attachment:** Connects directly to head housing bottom mount
   - **Rotation range:** ±30° (60° total, head tilt sideways)
   - **Minimalist design:** Smallest/lightest components here (top of kinematic chain)

5. **Structural Strength:**
   - Wall thickness: 3-4mm for servo mounts (high stress areas)
   - Reinforcement ribs around servo body clamps
   - Metal hardware option: M4 bolts + heat-set inserts for servo attachment (stronger than plastic-to-plastic)

6. **Cable Management:**
   - Continuous path from head housing → roll → tilt → pan → body
   - Cable strain relief at each joint (loops or flexible conduit)
   - No cables crossing rotation axes (prevents tangling)

7. **Weight Budget:**
   - Total neck gimbal weight (3 servos + mounts): ≤400g
   - Head + ears + neck total: ≤1000g (conservative for base stability)

8. **Assembly Simulation:**
   - OnShape assembly shows full range of motion (pan ±90°, tilt ±45°, roll ±30°)
   - No part collisions at any combination of angles
   - Interference detection run in OnShape

9. **Documentation:**
   - OnShape public link
   - Assembly guide: Servo installation sequence (critical: which servo connects where)
   - Calibration notes: Servo zero positions (all servos centered = head looking forward, level)

**Dependencies:** Story 2.2 (head housing for top connection), body chassis size estimate (for base mounting)

**Estimated Effort:** 10-14 hours (most complex mechanical design)

---

## Story 2.5: Body Chassis CAD Design

**As a** CAD designer,
**I want** a modular body chassis that houses all electronics and supports the neck,
**so that** all components are organized, protected, and accessible for maintenance.

### Acceptance Criteria:

1. **OnShape Assembly Created:**
   - Assembly name: "Body Chassis"
   - Modular panels: Front, back, left, right, top, bottom (can be disassembled)

2. **Component Bays:**
   - **Raspberry Pi mount:**
     - M2.5 standoffs, 4x mounting holes
     - Access to all ports (USB, HDMI, GPIO, power)
     - Acrylic window option (show the tech per branding guide)
   - **Battery compartment:**
     - Sized for hoverboard battery (measure actual dimensions from Epic 1)
     - Secure retention (straps or clips, prevent shifting during movement)
     - Ventilation holes (battery heat dissipation)
   - **Buck converter mounts:**
     - 2x buck converters (36V→12V, 36V→5V)
     - Heat sink clearance (converters can get warm under load)
     - Access to output terminals for wiring
   - **DLP projector mount:**
     - Angled bracket: 30-45° downward tilt (toward floor)
     - Adjustable tilt mechanism (fine-tune projection angle)
     - Lens clearance at front of body (unobstructed view downward)

3. **Structural Features:**
   - **Neck gimbal base mount:** Top of chassis, reinforced plate for pan servo
   - **Base platform attachment:** Bottom mount points for hoverboard wheel platform
   - **LED status indicator cutouts:** Front panel windows for RGB LEDs
   - **Power switch access:** External toggle switch (emergency cutoff from Epic 1)
   - **Charging port access:** Hoverboard charge port accessible from exterior

4. **Cable Management:**
   - Internal cable routing channels
   - Velcro or zip-tie mounting points for wire harnesses
   - Separation: High-power cables (battery, motors) vs. low-power (servos, sensors)

5. **Ventilation:**
   - Perforated panels or vent slots for Raspberry Pi, buck converters
   - Airflow path: Bottom intake → top exhaust (passive convection)

6. **Modularity:**
   - Panels removable with M3 bolts or snap-fit clips
   - Easy access to components without full disassembly
   - Future expansion space: Room for additional modules (future sensors, etc.)

7. **Dimensions:**
   - Height: ~20-25cm (body only, excluding neck/head)
   - Width: ~15-20cm (proportional to head size)
   - Depth: ~15-20cm (accommodate battery depth)

8. **Weight:**
   - Body chassis (empty): ≤300g (3D printed PLA)
   - Total body weight with electronics: ~2-3kg (battery is heaviest component)

9. **Aesthetic:**
   - Rounded edges, retro-futurism styling
   - Panel lines visible but clean (not overly complex)
   - White/light gray color (per branding guide)

10. **Documentation:**
    - OnShape public link
    - Component placement diagram (top-down view showing internal layout)
    - Assembly instructions: Panel attachment order

**Dependencies:** Story 2.1 (component dimensions), Story 2.4 (neck gimbal base size)

**Estimated Effort:** 10-14 hours

---

## Story 2.6: Base Platform CAD Design (Hoverboard Integration)

**As a** CAD designer,
**I want** a base platform that integrates hoverboard wheels and ODrive controller,
**so that** Olaf has a stable, mobile foundation with differential drive capability.

### Acceptance Criteria:

1. **OnShape Assembly Created:**
   - Assembly name: "Base Platform - Hoverboard Wheels"

2. **Hoverboard Wheel Integration:**
   - **Wheel mounts:**
     - Brackets to secure hoverboard hub motors (measure actual motor dimensions from salvaged hoverboard)
     - Axle spacing: 25-35cm (differential drive, turning radius ~40-50cm)
     - Wheel diameter: ~6.5 inches (typical hoverboard wheel, measure actual)
   - **Motor body support:**
     - Clamps or brackets hold motor body stationary (prevent rotation)
     - Wiring exit points for motor phase wires (3-phase BLDC)

3. **ODrive Mount:**
   - Secure mounting plate for ODrive controller (v3.6 or S1 dimensions)
   - Access to ODrive connectors (motor phase wires, encoder wires, power, USB/UART)
   - Heat sink clearance (ODrive MOSFETs can get warm)
   - Vibration isolation optional (rubber standoffs) to protect electronics from motor vibration

4. **Structural Support:**
   - **Central spine or platform:**
     - Connects left and right wheel mounts
     - Attachment points for body chassis (top mounting bosses)
   - **Bumper/skirt:**
     - Protective ring around perimeter (prevents damage if Olaf bumps into furniture)
     - Ground clearance: 10-15mm (navigate over small obstacles, thresholds)
     - Smooth rounded shape (gentle contact if collision)

5. **Cable Routing:**
   - Motor phase wires (thick, high-current) → ODrive
   - Encoder wires (thin signal wires) → ODrive
   - ODrive power from buck converter (36V or 12V, check ODrive spec)
   - ODrive communication (UART/USB/CAN) → Raspberry Pi (routed up through chassis)

6. **Stability:**
   - Low center of gravity: Heavy battery in bottom of body chassis
   - Wheelbase: Wide enough to prevent tipping during turns
   - Castor wheel optional (3-wheel config) vs. two-wheel balance (requires active balancing—defer to two-wheel skid-steer)

7. **Weight:**
   - Base platform (without motors): ≤500g
   - Total base weight with motors + ODrive: ~3-4kg (motors are heavy)

8. **Aesthetic:**
   - Clean lines, integrated look (not just motors bolted to platform)
   - Concealed wiring where possible
   - White/light gray matching body chassis

9. **Documentation:**
   - OnShape public link
   - Wheel alignment guide (ensure wheels parallel for straight movement)
   - ODrive installation notes

**Dependencies:** Story 2.5 (body chassis for top attachment points), hoverboard dimensions from Epic 1

**Estimated Effort:** 8-12 hours

---

## Story 2.7: Complete Assembly & Interference Check

**As a** CAD designer,
**I want** a complete OnShape assembly with all modules integrated,
**so that** I can verify fitment, detect interferences, and finalize the design before printing.

### Acceptance Criteria:

1. **Master Assembly Created:**
   - OnShape assembly name: "Olaf V1 - Complete Robot"
   - All sub-assemblies inserted:
     - Head housing (Story 2.2)
     - Ears (2x, Story 2.3)
     - Neck gimbal (Story 2.4)
     - Body chassis (Story 2.5)
     - Base platform (Story 2.6)

2. **Component Population:**
   - Simplified models of all major components inserted (OAK-D Pro, Raspberry Pi, servos, battery, motors)
   - Components positioned in correct locations per design
   - Mates/constraints applied (parts move realistically)

3. **Kinematic Simulation:**
   - Neck movement simulated:
     - Pan servo rotates ±90°
     - Tilt servo rotates ±45°
     - Roll servo rotates ±30°
     - All combinations tested (e.g., pan +90°, tilt +45°, roll +30° simultaneously)
   - Ear movement simulated:
     - Both ears move independently through full 2-DOF range

4. **Interference Detection:**
   - OnShape interference detection tool run
   - No part collisions detected at any valid pose
   - If interference found: Redesign affected parts, re-test

5. **Cable Routing Verification:**
   - Visual check: Cable paths clear from head → neck → body
   - No cables crushed or pinched at any joint angle
   - Cable strain relief adequate (loops, service length)

6. **Weight & Balance:**
   - OnShape mass properties calculated:
     - Total weight: Target ≤6kg (battery + motors = ~4kg, rest = ~2kg)
     - Center of gravity: Should be low and centered over wheelbase
   - If top-heavy: Redesign or add ballast to base

7. **Dimensional Check:**
   - Overall dimensions measured:
     - Height: ~50-60cm (target from PRD)
     - Width: ~35-40cm (target from PRD)
     - Depth: ~30-40cm
   - Fits through standard doorway (80cm width with clearance)

8. **Aesthetic Review:**
   - Proportions look balanced (not too top-heavy visually)
   - Retro-futurism styling consistent across all parts
   - Rounded, friendly appearance (per branding guide)

9. **Public Sharing:**
   - OnShape assembly set to public view
   - Public link added to `hardware/3d-models/README.md`
   - Link shared in GitHub README

10. **Design Freeze:**
    - All parts marked as "ready for export"
    - Version tagged in OnShape (e.g., "V1.0 - Print Ready")

**Dependencies:** All previous CAD stories (2.2-2.6)

**Estimated Effort:** 4-6 hours (assembly, testing, iteration)

---

## Story 2.8: STL Export & Print Settings Documentation

**As a** 3D printing operator,
**I want** STL files with documented print settings for each part,
**so that** I (and community builders) can successfully print all components.

### Acceptance Criteria:

1. **STL Export:**
   - All parts exported from OnShape as STL files
   - File naming convention: `olaf_<module>_<part>.stl`
     - Examples: `olaf_head_housing_front.stl`, `olaf_ear_left_base.stl`, `olaf_neck_pan_mount.stl`
   - Files organized in directories:
     ```
     hardware/3d-models/stl/
     ├── head/
     ├── ears/
     ├── neck/
     ├── body/
     └── base/
     ```

2. **Print Settings Document:**
   - File created: `hardware/3d-models/PRINT_SETTINGS.md`
   - Settings for each part:
     - **Material:** PLA or PETG (default PLA, PETG for high-stress parts like neck mounts)
     - **Layer height:** 0.2mm (standard quality)
     - **Infill:** 20% (per branding guide)
     - **Wall thickness:** 3 perimeters
     - **Supports:** Yes/No, where to place
     - **Print orientation:** Optimal orientation for strength + minimal supports
     - **Estimated print time:** Hours per part (from slicer preview)
     - **Estimated material:** Grams of filament per part

3. **Special Instructions:**
   - **Heat-set inserts:** Parts requiring threaded inserts documented (size, quantity, insertion depth)
   - **Post-processing:** Sanding, drilling, reaming (if needed for fitment)
   - **Multi-part assemblies:** Print order recommendations (e.g., test-fit smaller parts first)

4. **Slicer Profiles:**
   - Optional: Cura or PrusaSlicer profile files committed
   - Configured with optimal settings (speeds, temperatures, retraction)

5. **BOM Update:**
   - Filament requirements added to BOM:
     - Total PLA needed: ~X kg (estimated from slicer material calculations)
     - Color: White or light gray (per branding guide)
     - Supplier links (Amazon, MatterHackers, local supplier)

6. **Test Print Validation:**
   - At least 1-2 small parts test-printed before committing all STLs
   - Verify: Dimensions accurate, no warping, supports remove cleanly
   - If issues found: Adjust OnShape model, re-export STL

**Dependencies:** Story 2.7 (design finalized)

**Estimated Effort:** 3-4 hours (export, documentation, test print)

---

## Story 2.9: 3D Printing All Components

**As a** maker,
**I want** all Olaf components 3D printed and ready for assembly,
**so that** I can build the physical robot structure.

### Acceptance Criteria:

1. **Print Queue:**
   - All STL files imported into slicer (Cura, PrusaSlicer, or similar)
   - Print queue organized by priority:
     - **Week 1:** Head housing, small test parts (ear base, neck mounts)
     - **Week 2:** Neck gimbal, body chassis panels
     - **Week 3:** Base platform, remaining parts

2. **Print Execution:**
   - Prints started according to queue
   - Failed prints documented (part name, failure reason, retry plan)
   - Successful prints checked for:
     - Dimensional accuracy (measure critical dimensions with calipers)
     - No warping or layer separation
     - Supports removed cleanly

3. **Quality Control:**
   - Parts fit together without forcing (test-fit before final assembly)
   - Screw holes sized correctly (M3, M4 bolts thread smoothly or heat-set inserts fit snugly)
   - No cracks or weak points (especially thin walls, servo mounts)

4. **Print Log:**
   - Spreadsheet or table: `hardware/3d-models/print-log.csv`
   - Columns: Part name, print date, material, print time, weight, pass/fail, notes
   - Track total material used, total print time

5. **Photos:**
   - At least 3 photos per major module (head, ears, neck, body, base)
   - Photos show: Printed parts on build plate, parts removed/cleaned, test-fit assembly
   - Uploaded to `hardware/3d-models/photos/`

6. **Completion:**
   - All parts printed successfully (or reprinted after failures)
   - Parts organized in labeled bins/boxes for assembly
   - Ready to proceed to Story 2.10 (assembly)

**Dependencies:** Story 2.8 (STL files, print settings)

**Estimated Effort:** 40-80 hours (actual print time, mostly unattended, 1-2 weeks calendar time)

---

## Story 2.10: Physical Assembly & Mechanical Validation

**As a** robot builder,
**I want** all 3D printed parts assembled with servos and hardware,
**so that** I can validate mechanical ranges and structural integrity before adding electronics.

### Acceptance Criteria:

1. **Hardware Acquired:**
   - M2, M2.5, M3, M4 bolts (various lengths: 6mm, 10mm, 16mm, 20mm)
   - Heat-set inserts (M3, M4, appropriate length)
   - Feetech SCS0009 servos (4x for ears)
   - Feetech STS3215 servos (3x for neck)
   - Hoverboard motors + wheels (salvaged from Epic 1)
   - ODrive controller

2. **Heat-Set Insert Installation:**
   - Inserts installed in all designated holes (head, neck, body, base)
   - Installation tool: Soldering iron with insert tip OR generic soldering iron + insert holder
   - Inserts seated flush with surface, threads clean

3. **Servo Installation - Ears:**
   - 4x SCS0009 servos mounted (2 per ear: base + top)
   - Servo horns attached (correct orientation, set screw tight)
   - Ear fins attached to top servo horns
   - Ears attached to head housing sides
   - **Range of motion test:**
     - Horizontal rotation: ±90° achieved without binding
     - Vertical tilt: ±45° achieved without binding
     - No mechanical interference (ear fin doesn't hit head housing)

4. **Servo Installation - Neck:**
   - 3x STS3215 servos mounted (pan, tilt, roll)
   - Assembly sequence: Pan base → Tilt yoke → Roll cradle → Head housing
   - Servo horns aligned to zero positions (all centered = head forward, level)
   - **Range of motion test:**
     - Pan: ±90° achieved (head rotates left/right smoothly)
     - Tilt: ±45° achieved (head looks up/down smoothly)
     - Roll: ±30° achieved (head tilts sideways smoothly)
     - Combined movements: Test all 3 axes simultaneously, no binding

5. **Body Assembly:**
   - Raspberry Pi mounted on standoffs
   - Buck converters secured
   - Battery compartment fitted with hoverboard battery (test-fit, confirm retention)
   - DLP projector bracket installed, angle adjustable

6. **Base Assembly:**
   - Hoverboard motors mounted to left/right brackets
   - Wheels attached (axle secured, no wobble)
   - ODrive mounted
   - Motor phase wires connected to ODrive (initial wiring, not powered yet)
   - Encoder wires connected (if using hall sensors)

7. **Full Robot Assembly:**
   - Base platform attached to body chassis
   - Neck gimbal attached to body top
   - Head housing attached to neck roll cradle
   - Ears attached to head housing
   - **Stand test:** Robot stands upright, balanced, doesn't tip over

8. **Mechanical Validation:**
   - **Weight measurement:** Total robot weight ≤6kg (use scale)
   - **Center of gravity:** Roughly centered over wheelbase (doesn't lean forward/back excessively)
   - **Manual movement test:**
     - Manually rotate neck through full range (check for binding, cable interference)
     - Manually move ears (smooth motion, no grinding)
     - Roll base on flat surface (wheels rotate freely, no scraping)

9. **Cable Routing (Dry Run):**
   - Temporarily route cables from head → neck → body (not connected, just path validation)
   - Verify: Cables don't get pinched at any neck angle
   - Service loops adequate at each joint

10. **Photo Documentation:**
    - At least 5 photos:
      - Assembled head with ears
      - Neck gimbal (3 DOF visible)
      - Body chassis interior (component placement)
      - Base platform with wheels
      - Full robot assembled
    - Photos uploaded to `hardware/photos/epic2-assembly/`

11. **Issues Log:**
    - Document any fitment issues (too tight, too loose)
    - Document any redesign needs (if found, create GitHub issues for future fixes)
    - Document any mechanical limitations (e.g., "Neck tilts only ±40° due to cable tension")

**Dependencies:** Story 2.9 (all parts printed), servos/hardware acquired

**Estimated Effort:** 10-16 hours (assembly, testing, iteration)

---

## Story 2.11: Epic 2 Documentation & Content Creation

**As a** community builder and content creator,
**I want** complete 3D design documentation and build-in-public content,
**so that** others can replicate the physical structure and follow the build journey.

### Acceptance Criteria:

1. **OnShape Documentation:**
   - `hardware/3d-models/README.md` updated with:
     - OnShape public assembly link
     - Description of each module (head, ears, neck, body, base)
     - Design rationale (why certain choices were made)
     - Known issues / future improvements

2. **Assembly Guide:**
   - `hardware/assembly-guide.md` created with:
     - Step-by-step instructions (1. Print parts, 2. Install heat-set inserts, 3. Assemble neck, etc.)
     - Photos for each major step
     - Tools required list (soldering iron, Allen keys, calipers, etc.)
     - Estimated time per module

3. **BOM Update:**
   - `hardware/bom/epic2_bom.csv` created with:
     - 3D printing filament (PLA, quantity, supplier)
     - Servos (SCS0009, STS3215, quantities, links)
     - Hardware (bolts, inserts, quantities, links)
     - Hoverboard salvage components
     - ODrive controller
     - Costs updated for Standard Olaf tier (~$700 target)

4. **Troubleshooting Section:**
   - `docs/troubleshooting.md` updated with Epic 2 issues:
     - "Heat-set inserts won't seat" → solution (lower temp, pilot hole size)
     - "Servo binding in neck" → solution (alignment, cable interference)
     - "Print warped" → solution (bed adhesion, temperature settings)

5. **GitHub Milestone:**
   - GitHub milestone "Epic 2: 3D Design" created
   - All Epic 2 stories closed, linked to milestone
   - Milestone marked complete

6. **Content Creation (LinkedIn):**
   - **Week 3 Post:** "Head Module CAD Complete" (see `.content/epic-content-plan.md`)
     - OnShape screenshot
     - Design challenges: Cable routing, sensor placement
     - Photo of first test print
   - **Week 4 Post:** "Olaf Takes Shape! Full Assembly Preview"
     - Photo of printed components laid out
     - Video/GIF of neck gimbal moving (manual test)
     - Reality check: Weight, servo torque

7. **Content Creation (YouTube):**
   - **Video #1 Script Drafted:** "Building Olaf Part 1: 3D Design & Assembly" (10-15 min)
     - Intro, OnShape walkthrough, printing time-lapse, assembly, mechanical test
   - Video filming notes documented (B-roll needed, scene list)
   - Video scheduled for filming/editing after Epic 2 complete

8. **Photos/Videos Organized:**
   - At least 10 photos total from Epic 2:
     - CAD screenshots (3-4)
     - Printing process (2-3)
     - Assembly steps (4-5)
   - 1-2 short videos:
     - Neck moving through 3-DOF range
     - Ears articulating

**Dependencies:** All Epic 2 stories complete (2.1-2.10)

**Estimated Effort:** 6-8 hours (documentation writing, content prep)

---

## Epic 2 Summary

**Total Stories:** 11
**Estimated Total Effort:** 90-130 hours (1-2 weeks calendar time for printing, 2-3 weeks total including design)

**Key Deliverables:**
- ✅ Complete OnShape CAD assembly (public, shareable)
- ✅ STL files for all components (head, ears, neck, body, base)
- ✅ All parts 3D printed (PLA/PETG, white/light gray)
- ✅ Physical robot assembled (structure only, servos installed but not yet wired)
- ✅ Mechanical ranges validated (neck 3-DOF, ears 2-DOF)
- ✅ Build documentation (assembly guide, BOM, troubleshooting)
- ✅ Build-in-public content (LinkedIn posts, YouTube video #1 prep)

**Success Criteria Met:**
- Physical structure exists and stands upright
- Mechanical ranges meet PRD requirements (pan ±90°, tilt ±45°, roll ±30°, ear 2-DOF)
- Total weight ≤6kg (within stability/torque budgets)
- Design is replicable (OnShape public, STLs available, assembly guide complete)
- Community can start printing along
- First YouTube video ready to film

**Next Epic:** Epic 3 - Complete Personality Expression System

---

**Related Documents:**
- [PRD](../prd.md)
- [Epic 1: Foundation](epic-01-foundation.md)
- [Epic 3: Complete Personality](epic-03-personality.md)
- [Epic Content Plan](../../.content/epic-content-plan.md)
