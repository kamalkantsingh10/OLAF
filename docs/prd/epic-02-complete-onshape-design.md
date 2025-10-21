# Epic 2: Complete OnShape Design & Community Feedback

**Epic Goal:** Design ALL mechanical components in OnShape in one complete pass: head housing (eyes, OAK-D Pro, mmWave, mics), ears (2-DOF articulation), neck gimbal (3-DOF), core torso (Pi mount, heart LCD integration, status indicators), power enclosure (battery, buck converters, charging), projector bay (DLP mounting), and base platform (hoverboard wheels, ODrive). Export production-ready STLs. Post complete design to Reddit/maker communities for feedback (3-7 day cycle). Incorporate feedback and mark design as "V1.0 Draft - Ready to Print" for use in all subsequent build epics.

**Duration:** 2-3 weeks (Weeks 1.5-4)

**Prerequisites:** Epic 1 Story 1.1 (repository structure), Epic 1 Story 1.10 (initial concept sketch provides starting aesthetic)

**Value Delivered:** Complete validated mechanical design before printing anything expensive, community feedback incorporated early, all STLs ready for Epics 3-7, design is replicable by community, eliminates costly redesign-reprint cycles, OnShape public link serves as project showcase.

**Design Philosophy:** This epic front-loads ALL mechanical design work to get community validation before committing to 3D printing. Feedback from Reddit/makers will inform final adjustments. Subsequent epics (3-7) will print and assemble these validated designs, applying any feedback-driven tweaks as needed.

---

## Story 2.1: OnShape Setup & Component Research

**As a** mechanical designer,
**I want** OnShape configured and all component datasheets/dimensions collected,
**so that** I can design accurate CAD models that fit actual hardware and avoid fitment issues.

### Acceptance Criteria:

1. **OnShape Account:**
   - Free OnShape account created (sufficient for public projects, no paid plan needed)
   - Public workspace created: "Olaf - Open-Source AI Companion Robot"
   - Workspace set to public visibility (aligns with open-source commitment, allows community viewing)

2. **Component Datasheets & Dimensions Collected:**
   - **Vision/Sensors:**
     - OAK-D Pro: PCB dimensions, mounting holes, lens position, USB-C port location (from Luxonis docs)
     - GC9A01 displays (3×): 1.28" round TFT dimensions, mounting tabs, screen bezel size (2× eyes + 1× heart)
     - mmWave sensor (DFRobot SEN0395 or similar): PCB size, antenna orientation, mounting holes
     - Microphone array (ReSpeaker 2-Mic HAT or similar): Board dimensions, mic positions, GPIO header
     - Speaker/beeper: Physical size, mounting method
   - **Servos:**
     - Feetech SCS0009 (4×): Body dimensions (L×W×H), horn attachment point, shaft diameter, rotation range
     - Feetech STS3215 (3×): Body dimensions, horn type, torque specs, mounting holes, cable exit
   - **Electronics:**
     - Raspberry Pi 5 (8GB): Board dimensions, mounting holes (M2.5 spacing), port locations (USB, HDMI, GPIO, power, cooling requirements)
     - ESP32-S3 modules (3×): Board size (varies by dev board), pin header spacing, USB port
     - Buck converters (2×): PCB size, heat sink clearance, terminal positions
     - ODrive controller: Board dimensions, connector positions, heat sink requirements
     - Hoverboard battery: Measure actual salvaged battery (L×W×H, weight, BMS dimensions)
   - **Motors/Wheels:**
     - Hoverboard hub motors (2×): Wheel diameter, axle width, motor body size, phase wire exit
   - **Projector:**
     - DLP projector (model TBD): Dimensions, lens position, throw distance, ventilation requirements, power button location

3. **Reference Dimensions Spreadsheet:**
   - File created: `hardware/component-dimensions.csv`
   - Columns: Component, Length (mm), Width (mm), Height (mm), Weight (g), Mounting Holes, Notes
   - All components listed with accurate measurements
   - Mounting hole patterns documented (e.g., "M2.5, 58mm × 49mm spacing")
   - Committed to repository

4. **OnShape Tutorials Completed (if new to OnShape):**
   - Basic sketch tutorial (constraints, dimensions)
   - Part modeling tutorial (extrude, revolve, fillet)
   - Assembly tutorial (mating parts, interference detection)
   - Export to STL tutorial
   - Estimated 2-4 hours learning curve for OnShape beginners

5. **Design Constraints Documented:**
   - **Weight limits:**
     - Head assembly (housing + eyes + OAK-D + mmWave + mics + speaker): ≤600g (STS3215 neck servo torque safety margin)
     - Ear assembly (2 servos + fin, per ear): ≤150g each
     - Total robot: Target ≤6kg (for stability and component stress)
   - **Cable routing:**
     - Minimum cable channel diameter: 10mm (accommodate wire bundles, flex sleeving)
     - Service loops at each neck joint: 50mm slack minimum
   - **Ventilation:**
     - Raspberry Pi: Passive airflow or small fan cutout (CPU can throttle without cooling)
     - Buck converters: Heat sink clearance + ventilation slots (can get warm under load)
     - DLP projector: Ventilation for projector intake/exhaust (projectors generate heat)
   - **Battery compartment:**
     - Sized for actual hoverboard battery dimensions (measure from Epic 1 if salvaged, or standard 36V 4-10Ah size)
     - Secure retention (straps, Velcro, or frame clamps to prevent shifting during movement)
   - **Aesthetic:**
     - Rounded edges, no sharp corners (friendly, retro-futurism styling from branding guide)
     - White/light gray base color (per branding guide)
     - Visible tech: Acrylic windows optional to showcase internal components

6. **Initial Concept Review:**
   - Review Epic 1 Story 1.10 initial concept sketch + Reddit feedback
   - Document key feedback themes to incorporate (e.g., "users suggested wider base for stability")
   - Create "design decisions" document: `hardware/3d-models/design-decisions.md`

**Dependencies:** Epic 1 Story 1.1 (repo for documentation), Epic 1 Story 1.10 (initial concept + feedback)

**Estimated Effort:** 6-8 hours (datasheet collection, measurements, OnShape setup, tutorials if needed)

---

## Story 2.2: Head Housing CAD Design

**As a** CAD designer,
**I want** a complete head housing design that mounts all sensors, displays, and structural features,
**so that** Olaf's "face" components are protected, properly aligned, and ready to print.

### Acceptance Criteria:

1. **OnShape Part Studio Created:**
   - Part studio name: "Head Housing V1"
   - Parametric sketch-based design (easy to adjust dimensions if feedback requires changes)

2. **Component Mounting Features:**
   - **OAK-D Pro camera mount:**
     - Front-facing position (centered on "face" for symmetrical look)
     - Mounting bosses/holes match OAK-D Pro spec (M2 or M3, verify from datasheet)
     - Lens unobstructed, IR emitters clear (no plastic blocking stereo vision)
     - USB-C cable exit to interior (routed down to neck)
   - **Dual OLED "eye" sockets:**
     - Side-by-side placement, 40-50mm spacing (human-like eye distance, scaled for robot size)
     - Recessed pockets hold GC9A01 displays flush or slightly inset (protects screens)
     - Viewing angle: Tilted slightly outward (5-10°) for wider expression visibility
     - Wire routing channels from eyes → interior (I2C + power)
   - **mmWave sensor bay:**
     - Top/forehead position (360° coverage, minimal obstruction)
     - Antenna clearance: No conductive material within 20mm (metal interferes with RF)
     - Sensor can be angled slightly forward (optimize human detection range)
   - **Microphone array mount:**
     - Side or top positions (multiple mics for directional audio, beamforming)
     - Acoustic ports/holes allow sound entry (small perforations, 2-3mm diameter)
     - Weatherproofing optional (mesh over holes to prevent dust/debris)
   - **Speaker/beeper mount:**
     - Internal cavity with sound exit port (front or side)
     - Secure mounting (vibrations from speaker require firm attachment)

3. **Structural Features:**
   - **Wall thickness:** 2-3mm (balance strength vs. print time/weight/material cost)
   - **Mounting boss for neck attachment:**
     - Bottom center of head (connects to neck roll servo cradle)
     - M4 or M5 threaded insert planned (heat-set insert for strong metal threads)
     - Reinforced area (extra material around mount point to handle neck torque)
   - **Split design (recommended):**
     - Front face plate + back housing shell (easier assembly, cable routing access)
     - Snap-fit or bolted assembly (M3 bolts around perimeter)
     - Alignment pins ensure consistent reassembly
   - **Rounded edges:** Per branding guide, all external corners filleted (3-5mm radius, friendly aesthetic)

4. **Cable Routing Channels:**
   - Internal channels for:
     - OAK-D Pro USB cable (thick, ~5mm diameter)
     - Eye displays I2C wires (2× thin bundles)
     - mmWave sensor wires
     - Microphone wires (I2C or SPI to Pi)
     - Speaker wires
   - Cable exit point at bottom (toward neck connection, single bundled exit)
   - Minimum 10mm diameter main channel (accommodate all wire bundles + some slack)

5. **Ventilation:**
   - Small vent holes (3-5mm diameter) for air circulation
   - Positioned away from OLED viewing areas (don't obscure "eyes")
   - Prevent heat buildup from OAK-D Pro (camera can get warm during operation)

6. **Ear Attachment Points:**
   - Mounting bosses on left/right sides of head (for ear base servo brackets)
   - M3 or M4 threaded inserts planned
   - Positioned to allow ear rotation clearance (ears won't hit head at extreme angles)

7. **Weight Estimate:**
   - OnShape mass properties calculated: Head housing (empty) + all mounted components ≤ 600g
   - If over budget: Reduce wall thickness, add lightening pockets, or use lattice infill in thick areas

8. **Design Review Checklist:**
   - [ ] All components from Story 2.1 dimensions fit correctly
   - [ ] No interference between components (use OnShape interference detection)
   - [ ] Cable paths don't cross moving parts
   - [ ] Printability: No overhangs >45° without supports (or acceptable support placement)
   - [ ] Aesthetic matches branding guide (rounded, friendly, retro-futurism)

9. **Documentation:**
   - OnShape public link added to `hardware/3d-models/README.md`
   - Design notes committed: Component placement rationale, cable routing plan, weight breakdown

**Dependencies:** Story 2.1 (component dimensions, OnShape setup)

**Estimated Effort:** 10-14 hours (CAD design, iteration, interference checks)

---

## Story 2.3: Ears (2-DOF) & Neck Gimbal (3-DOF) CAD Design

**As a** CAD designer,
**I want** complete ear articulation and neck gimbal designs,
**so that** Olaf has expressive ear movement and full 3-axis head articulation ready to print.

### Acceptance Criteria:

1. **OnShape Part Studios Created:**
   - Part studio: "Ear Mounts V1" (2× mirrored for left/right)
   - Part studio: "Neck Gimbal V1 - 3 DOF"

2. **Ear Design (2-DOF per ear):**
   - **Base servo mount (horizontal rotation):**
     - Bracket secures SCS0009 servo body (from Story 2.1 dimensions)
     - Servo horn attachment point (large circular horn recommended)
     - Rotation range: 180-270° mechanical (software will limit to safe range)
     - Attaches to head housing side bosses (from Story 2.2)
   - **Top servo mount (vertical tilt):**
     - Mounted to base servo horn output shaft
     - Second SCS0009 servo body bracket
     - Ear "fin" attaches to top servo horn
   - **Ear fin design:**
     - Chappie-inspired shape: Triangular or curved fin, 60-80mm height
     - Lightweight construction: Thin walls (1.5-2mm), lattice infill optional
     - Organic, rounded shape (not angular/sharp, friendly aesthetic)
     - Weight per ear assembly (2 servos + fin): Target ≤150g
   - **Cable routing:**
     - Wire channels for servo daisy-chain (SCS0009 bus protocol, single cable to both ear servos)
     - Cable exit toward head housing base (merges with neck cables)
   - **Range of motion validation:**
     - Horizontal: ±90° from center (180° total)
     - Vertical: ±45° from neutral (90° total)
     - No mechanical interference at extreme combinations (simulate in OnShape assembly)

3. **Neck Gimbal Design (3-DOF):**
   - **Pan Axis (Base Servo - STS3215):**
     - Servo body mount: Vertical orientation, secured to body chassis top
     - Large servo horn (cross or circular pattern for strength)
     - Tilt yoke mounts to pan horn output
     - Rotation range: ±90° (180° total sweep, software-limited for cable safety)
     - Hollow center or side channel: Pass cables from head/tilt/roll servos down to body
   - **Tilt Axis (Middle Servo - STS3215):**
     - Servo body mount: Horizontal orientation, attached to pan yoke
     - Servo horn connects to roll cradle
     - Rotation range: ±45° (90° total, look up/down)
     - **Balance point critical:** Center of mass of head + roll assembly should align with tilt axis (prevents servo strain, smoother motion)
   - **Roll Axis (Top Servo - STS3215):**
     - Servo body mount: Attached to tilt yoke
     - Servo horn connects directly to head housing bottom mount
     - Rotation range: ±30° (60° total, head tilt sideways "curious" gesture)
     - Minimalist design: Smallest/lightest components here (top of kinematic chain, weight impacts lower servos)

4. **Structural Strength:**
   - **Wall thickness:** 3-4mm for servo mounts (high stress areas, heavier loads than head housing)
   - **Reinforcement ribs:** Around servo body clamps, near mounting bosses
   - **Metal hardware:** M4 bolts + heat-set inserts for servo attachment (stronger than plastic-to-plastic friction fit)

5. **Cable Management:**
   - Continuous cable path: Head → Roll → Tilt → Pan → Body
   - Cable strain relief at each joint (loops or flexible conduit, prevent pulling on connectors)
   - No cables crossing rotation axes (prevents tangling, ensures full range of motion)

6. **Weight Budget:**
   - Neck gimbal assembly (3× STS3215 servos + mounts): Target ≤400g
   - Head + ears + neck total: ≤1000g (conservative for base stability, lower center of gravity better)

7. **Assembly Simulation in OnShape:**
   - Create assembly: Ears → Head → Roll → Tilt → Pan
   - Mate all servos, test full range of motion:
     - Pan ±90°, Tilt ±45°, Roll ±30°
     - Ears: Horizontal ±90°, Vertical ±45°
   - Run interference detection: No part collisions at any valid pose
   - If interference found: Redesign affected parts, retest

8. **Documentation:**
   - OnShape public links for ears + neck assemblies
   - Assembly guide notes: Servo installation sequence (critical: which servo connects where, horn alignment for zero positions)
   - Calibration notes: All servos centered = head looking forward level, ears neutral position

**Dependencies:** Story 2.1 (servo dimensions), Story 2.2 (head housing for ear attachment points and neck top mount)

**Estimated Effort:** 12-16 hours (most complex mechanical design, multiple DOF, interference testing)

---

## Story 2.4: Core Torso & Power Enclosure CAD Design

**As a** CAD designer,
**I want** a modular core torso housing all electronics and a secure power enclosure,
**so that** all components are organized, protected, accessible, and the power system is safe.

### Acceptance Criteria:

1. **OnShape Assembly Created:**
   - Assembly name: "Core Torso & Power V1"
   - Modular panels: Front, back, left, right, top, bottom (can be disassembled for maintenance)

2. **Component Bays - Core Torso:**
   - **Raspberry Pi mount:**
     - M2.5 standoffs (4× mounting holes, standard Pi spacing)
     - Access to all ports: USB, HDMI, GPIO header, Ethernet, power
     - Cooling: Passive ventilation slots OR small 30mm fan cutout (optional, depends on workload)
     - Optional acrylic window: Showcase the Pi (per branding guide "visible tech")
   - **Heart LCD display integration:**
     - GC9A01 round display from Epic 1, now properly housed in torso
     - Front-facing cutout (centered on "chest")
     - Display bezel or frame (protect screen edges)
     - ESP32 module bay nearby (controls heart via SPI, same module from Epic 1)
   - **Status indicator LEDs:**
     - RGB LEDs (3-5×): Power status, battery level, system state, emotion cues
     - Front panel windows/light pipes (allow LED light to shine through)
     - Wiring channels to LED strip or individual LEDs
   - **ESP32 module bays:**
     - Mounts for 2-3 ESP32 boards (head module, ears/neck module, body/projector module)
     - Access to USB ports for programming
     - I2C bus routing channels (all ESP32s on shared I2C bus to Pi)

3. **Power Enclosure (within or adjacent to torso):**
   - **Battery compartment:**
     - Sized for hoverboard battery dimensions (from Story 2.1 measurements)
     - Secure retention: Velcro straps, frame clamps, or snap-fit holder (prevent shifting during movement)
     - Ventilation holes: Battery heat dissipation (small perforations, not blocking airflow)
     - Access panel: Easily removable for battery replacement/charging
   - **Buck converter mounts:**
     - 2× buck converters (36V→12V for Pi, 36V→5V for ESP32s/servos)
     - Heat sink clearance (converters can get warm under load, 5-10mm clearance around heat sink fins)
     - Access to output terminals for wiring (screw terminals or solder pads)
     - Secure mounting (vibration from motors can loosen components)
   - **Charging port:**
     - External access to hoverboard BMS charge port
     - Panel-mount connector or pass-through for charge cable
     - Indicator LED cutout (charge status: red=charging, green=full)
   - **Power distribution area:**
     - Space for fuse holder (30-40A main fuse on battery output)
     - Emergency cutoff switch: Panel-mount toggle or push-button (accessible from exterior)
     - Wire harness routing (high-power: 18 AWG or thicker, low-power: 22-24 AWG)

4. **Structural Features:**
   - **Neck gimbal base mount:**
     - Top of chassis, reinforced plate for pan servo mounting (from Story 2.3)
     - M4 or M5 bolts, heat-set inserts
     - Load-bearing design (supports weight of head + neck assembly)
   - **Base platform attachment:**
     - Bottom mount points for hoverboard wheel platform (from Story 2.6)
     - 4-6× mounting bosses with M4/M5 inserts
   - **Panel attachment:**
     - Removable panels with M3 bolts or snap-fit clips
     - Alignment pins for consistent reassembly
     - Tool-less access to frequently serviced components (Pi, ESP32s for reprogramming)

5. **Cable Management:**
   - Internal cable routing channels (separate high-power from low-power/signal lines)
   - Velcro or zip-tie mounting points for wire harnesses
   - Cable exit points: Top (to neck/head), bottom (to base motors/sensors), sides (optional expansion)

6. **Ventilation:**
   - Perforated panels or vent slots (front/back/top)
   - Airflow path: Bottom intake → top exhaust (passive convection, hot air rises)
   - Keep vents away from sensitive electronics (prevent dust ingress into Pi GPIO)

7. **Dimensions:**
   - Height: 20-25cm (torso only, excluding neck/head/base)
   - Width: 15-20cm (proportional to head size, balanced look)
   - Depth: 15-20cm (accommodate battery depth, typically ~10-15cm for hoverboard batteries)

8. **Weight:**
   - Torso chassis (empty, 3D printed PLA): Target ≤300g
   - Total torso weight with electronics: ~2-3kg (battery is heaviest single component at ~2kg)

9. **Aesthetic:**
   - Rounded edges, retro-futurism styling (consistent with head design)
   - Panel lines visible but clean (not overly complex, modular look)
   - White/light gray color (per branding guide)

10. **Documentation:**
    - OnShape public link
    - Component placement diagram (top-down view showing internal layout: Pi here, battery here, converters here)
    - Assembly instructions: Panel attachment order, cable routing sequence

**Dependencies:** Story 2.1 (component dimensions, battery size), Story 2.3 (neck gimbal base size for top mount)

**Estimated Effort:** 12-16 hours (complex internal layout, cable routing planning, power safety features)

---

## Story 2.5: DLP Projector Bay CAD Design

**As a** CAD designer,
**I want** a projector mounting bay with optocoupler control integration,
**so that** the DLP projector is securely mounted, angled correctly, and controllable via ESP32.

### Acceptance Criteria:

1. **OnShape Part Studio Created:**
   - Part studio name: "Projector Bay V1"
   - Integrated into torso design (Story 2.4) or separate module that attaches to torso

2. **Projector Mounting:**
   - **Angled bracket:** 30-45° downward tilt (projects onto floor in front of robot)
   - **Adjustable tilt mechanism:**
     - Slotted holes or hinge design (fine-tune projection angle during assembly)
     - Locking mechanism (thumbscrew, friction fit, or bolt to hold angle)
   - **Secure projector retention:**
     - Cradle or clamp design fits projector body (measure actual DLP projector from Story 2.1)
     - Vibration isolation optional (rubber pads, prevent motor vibrations from shaking projected image)
   - **Lens clearance:**
     - Unobstructed view downward/forward
     - No torso panels blocking projection path

3. **Ventilation for Projector:**
   - DLP projectors generate heat (LED or lamp-based)
   - Intake vents aligned with projector air intake (typically side or rear)
   - Exhaust vents for hot air exit (prevent heat buildup in torso)
   - Ensure projector cooling fan has clear airflow (blocked vents = overheating = auto-shutdown)

4. **Cable Routing:**
   - **HDMI cable:** From Raspberry Pi to projector HDMI input (mini HDMI or full-size, check projector spec)
   - **Power cable:** From buck converter (12V or 5V, check projector power input) to projector power input
   - **Optocoupler control wires:** ESP32 GPIO → optocoupler circuit → projector power button terminals

5. **Optocoupler Control Integration:**
   - **Access to projector power button:**
     - Physical access to projector PCB power button terminals (may require opening projector case slightly)
     - Optocoupler circuit will simulate button press (short terminals momentarily to turn on/off)
   - **Mounting area for optocoupler circuit board:**
     - Small PCB or breadboard space (~30mm × 40mm) near projector
     - Secure mounting (vibration-proof)
   - **Wire routing:** ESP32 GPIO pins → optocoupler board → projector button terminals (short wires, <15cm)

6. **Lens Access:**
   - Focus adjustment access (manual or motorized focus, depends on projector model)
   - Lens cleaning access (front of projector easily reachable)

7. **Documentation:**
   - OnShape public link
   - Projector installation notes: Angle adjustment procedure, cable routing, optocoupler wiring diagram
   - Note: Actual optocoupler circuit schematic will be in Epic 8 firmware/hardware docs

**Dependencies:** Story 2.1 (projector dimensions), Story 2.4 (torso chassis for mounting points)

**Estimated Effort:** 4-6 hours (projector-specific design, ventilation planning, optocoupler integration planning)

---

## Story 2.6: Base Platform CAD Design (Hoverboard Wheels + ODrive)

**As a** CAD designer,
**I want** a stable base platform integrating hoverboard wheels and ODrive controller,
**so that** Olaf has a mobile foundation ready for balancing and navigation.

### Acceptance Criteria:

1. **OnShape Assembly Created:**
   - Assembly name: "Base Platform V1 - Hoverboard Wheels"

2. **Hoverboard Wheel Integration:**
   - **Wheel mounts:**
     - Brackets secure hoverboard hub motors (from Story 2.1 measurements: wheel diameter ~6.5", motor body size)
     - Axle spacing: 25-35cm (differential drive, turning radius ~40-50cm, wider = more stable)
     - Motor body support: Clamps or U-brackets hold motor stationary (prevent housing rotation)
   - **Motor alignment:**
     - Wheels parallel (ensures straight movement, no unintended turning)
     - Alignment jig or guide pins during assembly
   - **Wiring exit points:**
     - Motor phase wires (3-phase BLDC, thick cables ~12-14 AWG)
     - Hall sensor wires (if using hall-effect encoders, thin signal wires)
     - Cable routing toward ODrive mount

3. **ODrive Mount:**
   - Secure mounting plate for ODrive controller (v3.6 or S1, from Story 2.1 dimensions)
   - Access to ODrive connectors:
     - Motor phase terminals (M0, M1)
     - Encoder inputs (if using encoders)
     - Power input (from battery via buck converter or direct 36V, check ODrive spec)
     - Communication (UART/USB/CAN to Raspberry Pi)
   - Heat sink clearance: ODrive MOSFETs can get warm (5-10mm clearance around heat sinks)
   - Vibration isolation optional: Rubber standoffs (protect electronics from motor vibrations)

4. **Structural Support:**
   - **Central spine or platform:**
     - Connects left and right wheel mounts (structural rigidity)
     - Attachment points for torso (from Story 2.4 bottom mounts)
     - Top mounting bosses: 4-6× M4/M5 inserts for torso attachment
   - **Bumper/protective skirt:**
     - Perimeter ring or bumper guards (prevent damage if Olaf bumps furniture/walls)
     - Ground clearance: 10-15mm (navigate over small obstacles, thresholds, carpet transitions)
     - Smooth rounded shape (gentle contact, no sharp corners to damage floors)

5. **Cable Routing:**
   - Motor phase wires (thick, high-current) → ODrive motor terminals
   - Encoder/hall sensor wires (thin signal wires) → ODrive encoder inputs
   - ODrive power: From battery or buck converter (depends on ODrive input voltage range, 12-48V typical)
   - ODrive communication: UART/USB/CAN cable routed up through base platform → Raspberry Pi in torso

6. **Stability & Balance:**
   - **Low center of gravity:** Heavy battery in torso bottom (above base) helps lower CoG
   - **Wheelbase:** Wide enough to prevent tipping during turns (25-35cm spacing from above)
   - **Two-wheel skid-steer config:** No castor wheel (active balancing required, deferred to Epic 6)
   - **Kickstand mount point:** Small servo-actuated kickstand (deploys when not balancing, prevents tipping when stationary)

7. **Weight:**
   - Base platform (3D printed, no motors): Target ≤500g
   - Total base weight with motors + ODrive: ~3-4kg (hoverboard motors are heavy, ~1.5kg each)

8. **Aesthetic:**
   - Clean lines, integrated look (not just motors bolted to flat plate)
   - Concealed wiring where possible (channels or conduit)
   - White/light gray matching torso (consistent color scheme)

9. **Documentation:**
   - OnShape public link
   - Wheel alignment guide (ensure wheels parallel for straight movement)
   - ODrive installation notes (motor phase wire order, encoder wiring if applicable)

**Dependencies:** Story 2.1 (motor/wheel/ODrive dimensions), Story 2.4 (torso bottom mounting points)

**Estimated Effort:** 8-12 hours (motor mounting, alignment features, structural design)

---

## Story 2.7: Complete Assembly, Interference Check & Weight Validation

**As a** CAD designer,
**I want** a complete OnShape assembly with all modules integrated and validated,
**so that** I can verify fitment, detect interferences, confirm weight budgets, and finalize the design before exporting STLs.

### Acceptance Criteria:

1. **Master Assembly Created:**
   - OnShape assembly name: "Olaf V1 - Complete Robot Assembly"
   - All sub-assemblies inserted:
     - Head housing (Story 2.2)
     - Ears (2×, Story 2.3)
     - Neck gimbal (Story 2.3)
     - Core torso + power enclosure (Story 2.4)
     - Projector bay (Story 2.5)
     - Base platform (Story 2.6)

2. **Component Population:**
   - Simplified models of all major components inserted:
     - OAK-D Pro, Raspberry Pi, ESP32 modules (3×), servos (7× total), GC9A01 displays (3×)
     - Battery, buck converters, ODrive, hoverboard motors, DLP projector
   - Components positioned in correct locations per design
   - Mates/constraints applied (parts move realistically, servos rotate on correct axes)

3. **Kinematic Simulation:**
   - **Neck movement:**
     - Pan servo rotates ±90°
     - Tilt servo rotates ±45°
     - Roll servo rotates ±30°
     - Test all combinations (e.g., pan +90°, tilt +45°, roll +30° simultaneously)
   - **Ear movement:**
     - Both ears move independently through full 2-DOF range
     - Horizontal ±90°, vertical ±45° for each ear

4. **Interference Detection:**
   - OnShape interference detection tool run on complete assembly
   - Test at multiple poses:
     - All servos neutral (0°)
     - All servos at positive extremes (+90° pan, +45° tilt, +30° roll, +90° ear horizontal, +45° ear vertical)
     - All servos at negative extremes (opposite directions)
     - Mixed poses (random combinations)
   - **Pass criteria:** No part collisions detected at any valid pose
   - **If interference found:** Redesign affected parts (clearance increase, relocation), re-test

5. **Cable Routing Verification:**
   - Visual check: Cable paths clear from head → neck → torso → base
   - No cables crushed/pinched at any neck angle (especially at extreme tilt/roll)
   - Service loops adequate at each joint (50mm slack minimum to allow full range without pulling)

6. **Weight & Balance Validation:**
   - OnShape mass properties calculated for complete assembly:
     - **Total weight:** Target ≤6kg (battery + motors ~4kg, rest ~2kg)
     - **Center of gravity (CoG):** Should be low and centered over wheelbase
       - Ideal CoG height: <20cm above ground (lower = more stable)
       - CoG lateral position: Centered between wheels (±2cm tolerance)
   - **If top-heavy:** Redesign (move battery lower, reduce head weight, add ballast to base)
   - **If CoG off-center:** Adjust component placement (shift battery, relocate heavy items)

7. **Dimensional Check:**
   - Overall dimensions measured in OnShape:
     - **Height:** Target 50-60cm (from ground to top of head, ears neutral)
     - **Width:** Target 35-40cm (wheelbase + bumper/body width)
     - **Depth:** Target 30-40cm (front to back, includes projector if protruding)
   - **Doorway clearance:** Width <80cm (fits through standard doorway with margin)

8. **Aesthetic Review:**
   - Proportions look balanced (not too top-heavy visually, head size proportional to body)
   - Retro-futurism styling consistent across all modules (rounded edges, friendly curves)
   - Color scheme: White/light gray base, accent colors optional (LEDs provide color)

9. **Printability Check:**
   - Review all parts for 3D printing feasibility:
     - Overhangs: Minimize >45° overhangs, or ensure supports are acceptable
     - Wall thickness: All walls ≥1.5mm (thinner = weak, prone to failure)
     - Support removal: Ensure supports can be removed without damaging part
   - Large parts may need splitting (if >200mm on any axis, exceeds common printer build volumes)

10. **Design Freeze Checklist:**
    - [ ] All interferences resolved
    - [ ] Weight budget met (≤6kg)
    - [ ] CoG within acceptable range
    - [ ] Dimensions meet PRD targets (50-60cm height, 35-40cm width)
    - [ ] All cable paths validated
    - [ ] Aesthetic approved
    - [ ] Printability confirmed
    - [ ] All parts ready for STL export

**Dependencies:** All previous CAD stories (2.2-2.6)

**Estimated Effort:** 6-10 hours (assembly, testing, iteration, troubleshooting interferences)

---

## Story 2.8: STL Export & Print Settings Documentation

**As a** 3D printing operator,
**I want** all parts exported as STLs with comprehensive print settings documentation,
**so that** I (and community builders) can successfully print all components with minimal trial-and-error.

### Acceptance Criteria:

1. **STL Export from OnShape:**
   - All parts exported as STL files (binary format, more compact than ASCII)
   - File naming convention: `olaf_<module>_<part>_v1.stl`
     - Examples:
       - `olaf_head_housing_front_v1.stl`
       - `olaf_head_housing_back_v1.stl`
       - `olaf_ear_left_base_v1.stl`
       - `olaf_ear_left_fin_v1.stl`
       - `olaf_neck_pan_mount_v1.stl`
       - `olaf_torso_front_panel_v1.stl`
       - `olaf_base_platform_v1.stl`
   - Files organized in directories:
     ```
     hardware/3d-models/stl/
     ├── head/
     ├── ears/
     ├── neck/
     ├── torso/
     ├── projector/
     └── base/
     ```

2. **Print Settings Document:**
   - File created: `hardware/3d-models/PRINT_SETTINGS.md`
   - For each part, document:
     - **Material:** PLA (default, easiest to print), PETG (high-stress parts like neck mounts, more durable), or ABS (advanced, better strength)
     - **Layer height:** 0.2mm (standard quality), 0.15mm for detailed parts (ear fins, head housing face), 0.3mm for large structural parts (base platform)
     - **Infill:** 20% (per branding guide, good strength-to-weight ratio), increase to 30-40% for high-stress parts (neck mounts, servo brackets)
     - **Wall thickness:** 3-4 perimeters (standard, provides good strength)
     - **Supports:** Yes/No
       - If yes: Where to place (auto-generate or manual placement, note critical support areas)
     - **Print orientation:** Optimal orientation for:
       - **Strength:** Layer lines perpendicular to stress direction (e.g., servo mounts printed upright so layers resist pull-out)
       - **Minimal supports:** Orient to reduce overhang angles
       - **Surface finish:** Best surface facing up (no support marks on visible faces)
     - **Estimated print time:** Hours per part (from slicer preview, based on 0.2mm layer height, 50mm/s speed)
     - **Estimated material:** Grams of filament per part (from slicer material calculator)

3. **Part-Specific Recommendations:**
   - **Head housing:** PLA, 0.15mm layers for smooth face, supports for eye sockets, print face-down for best finish
   - **Ear fins:** PLA, 0.2mm layers, supports for curved sections, lightweight infill (15-20%)
   - **Neck mounts:** PETG (stronger than PLA), 0.2mm layers, 30% infill, print servo brackets upright for layer strength
   - **Torso panels:** PLA, 0.2mm or 0.3mm layers (large parts, speed up print), supports for internal bosses
   - **Base platform:** PLA or PETG, 0.3mm layers (large, structural), 25% infill, supports for motor mounts

4. **Special Instructions:**
   - **Heat-set inserts:**
     - List all parts requiring threaded inserts (part name, insert size, quantity, hole diameter)
     - Example: "Head housing back: M3 × 5mm inserts, 8× required, 4.2mm pilot holes"
     - Insertion guide: Soldering iron temp (200-220°C for PLA, 240-260°C for PETG)
   - **Post-processing:**
     - Sanding: Which surfaces need smoothing (visible faces, mating surfaces for tight fit)
     - Drilling/reaming: Holes that may need cleanup (servo shafts, bolt holes if too tight)
     - Acetone vapor smoothing (ABS only): Optional for aesthetic finish
   - **Multi-part assemblies:**
     - Print order recommendations (e.g., print test-fit parts first: small servo bracket to verify dimensions before printing full neck assembly)

5. **Slicer Profiles (Optional):**
   - Cura or PrusaSlicer profile files created:
     - `olaf_standard_pla_0.2mm.curaprofile` (or `.ini` for PrusaSlicer)
     - Configured with optimal settings: Print speed (50mm/s), temps (PLA 200-210°C nozzle, 60°C bed), retraction (5mm at 50mm/s)
   - Profiles committed to `hardware/3d-models/slicer-profiles/`

6. **BOM Update - Filament:**
   - Total filament requirements calculated (sum all part weights from slicer):
     - Total PLA needed: Estimate ~2-3kg (for all parts, with 10% waste margin)
     - Total PETG needed (if using for high-stress parts): ~0.5-1kg
   - Add to `hardware/bom/epic2_bom.csv`:
     - PLA filament, 3kg spools, white or light gray, supplier links (Amazon, MatterHackers)
     - PETG filament, 1kg spool, same color
     - Heat-set inserts: M2.5 (qty), M3 (qty), M4 (qty), M5 (qty), supplier links
     - Estimated cost: ~$60-80 for filament + $20-30 for inserts

7. **Test Print Validation:**
   - Print 2-3 small test parts before committing to full print queue:
     - Example: Ear base servo bracket, neck servo mount, torso panel corner section
   - Verify:
     - Dimensions accurate (measure with calipers, compare to OnShape model)
     - No warping (bed adhesion good, part flat)
     - Supports remove cleanly (no damage to part surface)
     - Heat-set insert holes correct size (insert fits snugly, no forcing)
   - **If issues found:** Adjust OnShape model (hole size, add chamfers, increase clearance), re-export STL, re-test

**Dependencies:** Story 2.7 (design finalized, ready for export)

**Estimated Effort:** 6-8 hours (STL export, print settings documentation, test prints, iteration)

---

## Story 2.9: Community Feedback - Reddit/Maker Forum Post

**As a** open-source project maintainer,
**I want** to post the complete design to Reddit and maker communities for feedback,
**so that** I can gather diverse perspectives, identify issues I missed, and build community engagement before printing.

### Acceptance Criteria:

1. **Feedback Post Preparation:**
   - **OnShape links ready:** All assemblies (head, ears, neck, torso, projector, base, complete robot) set to public view-only
   - **Renders/screenshots:**
     - Minimum 6 high-quality images (1920x1080 or higher):
       - Complete robot assembly (front, 3/4 view, side)
       - Head close-up (show sensor placement, eye sockets)
       - Neck gimbal exploded view (show 3-DOF mechanism)
       - Torso interior layout (component placement diagram)
       - Base platform (wheel mounting, ODrive location)
       - Projector bay (angled mounting, ventilation)
     - Optional: OnShape animation GIF (neck moving through range of motion, 5-10 seconds)
   - **Design summary document:**
     - 1-2 page PDF: Key specs, component list, design goals, weight budget, dimensions
     - Committed to `docs/design-summary-v1.pdf`

2. **Reddit Post Creation:**
   - **Target subreddit(s):**
     - Primary: r/robotics (most relevant, active community)
     - Secondary: r/3Dprinting (focus on printability feedback), r/raspberry_pi (Pi-centric builders)
     - Optional: r/DIY, r/maker
   - **Post title (engaging, specific):**
     - Example: "Olaf V1 Design Complete - Open-Source AI Companion Robot (OnShape CAD) - Feedback Requested!"
   - **Post content structure:**
     - **Introduction:**
       - Project overview: Self-balancing AI companion with personality expression
       - Open-source commitment: Full CAD public, replicable build
     - **Design highlights:**
       - 3-DOF neck gimbal (pan/tilt/roll)
       - 2-DOF articulated ears (Chappie-inspired)
       - Dual OLED eyes + OAK-D Pro stereo vision
       - 36V hoverboard power system
       - DLP floor projector with optocoupler control
       - Target weight: ≤6kg, target height: 50-60cm
     - **OnShape link:** "Full design here: [OnShape public link]"
     - **Specific feedback questions:**
       - "Mechanical engineers: Any concerns with the 3-DOF neck design? (weight distribution, servo torque)"
       - "3D printing experts: See any printability red flags? (overhangs, support issues)"
       - "Robotics builders: Is the wheelbase (30cm) wide enough for stability at 60cm height?"
       - "General feedback: Does the aesthetic feel friendly/approachable? Too bulky? Too fragile-looking?"
       - "Any suggestions for improvement before I start printing?"
   - **Images embedded:** 6+ images inline (Reddit supports multiple images in post)

3. **Additional Maker Forum Posts (Optional):**
   - Post to other communities (if time allows):
     - Hackaday.io project page (create project, upload design, link to OnShape)
     - Arduino/ESP32 forums (focus on I2C architecture, smart slave pattern)
     - ROS Discourse (ROS2-specific feedback on architecture)

4. **Feedback Collection Window:**
   - Monitor post for **3-7 days** (peak engagement typically first 48 hours, but allow time for thoughtful responses)
   - Engage with comments:
     - Respond to questions (clarify design choices, provide additional info)
     - Thank contributors for suggestions (acknowledge all feedback, even if not implemented)
     - Ask follow-up questions if feedback is unclear ("Can you elaborate on the servo torque concern?")
   - Set time boundary: **Max 6-8 hours total engagement** (respond to top 15-20 comments, don't get overwhelmed)

5. **Feedback Documentation:**
   - Create file: `docs/community-feedback/reddit-v1-design-feedback.md`
   - Document:
     - **Common themes** (recurring feedback across multiple comments):
       - Example: "5 users mentioned wheelbase seems narrow for height, suggested 35-40cm"
     - **Actionable suggestions** (specific, implementable ideas):
       - Example: "Add lightening pockets to neck mounts (reduce weight by ~20g)"
       - Example: "Include alignment pins on torso panels (easier reassembly)"
     - **Design validations** (positive feedback confirming choices):
       - Example: "3 users praised modular panel design for maintenance access"
     - **Questions raised** (areas needing clarification or further thought):
       - Example: "How will cables handle continuous neck rotation? (need to document cable strain relief)"

6. **Design Decision Log:**
   - For each actionable suggestion, document decision:
     - **Implement now:** Adjust OnShape model before STL export (critical issues, easy fixes)
     - **Implement during build:** Apply change when printing/assembling in Epics 3-7 (minor tweaks)
     - **Defer to V2:** Good idea but out of scope for V1 (document for future iteration)
     - **Won't implement:** Explain rationale (e.g., "Suggestion conflicts with weight budget")
   - Append decisions to `hardware/3d-models/design-decisions.md`

7. **OnShape Model Updates (if critical feedback):**
   - If major issues found (e.g., "Servo torque insufficient for head weight"):
     - Update OnShape model (reduce head weight, increase servo size, add counterweight)
     - Re-run Story 2.7 validation (interference check, weight check)
     - Re-export affected STLs
     - Update Reddit post with edit: "Updated design based on feedback: [changes made]"

**Dependencies:** Story 2.8 (STLs ready), Story 2.7 (complete assembly validated)

**Estimated Effort:** 6-10 hours (post creation, image prep, community engagement over 3-7 days, feedback documentation)

---

## Story 2.10: Design Finalization & Version Tagging

**As a** project maintainer,
**I want** the design marked as "V1.0 Draft - Ready to Print" with feedback incorporated,
**so that** Epics 3-7 have a stable, validated design baseline to work from.

### Acceptance Criteria:

1. **Incorporate Critical Feedback:**
   - Review `docs/community-feedback/reddit-v1-design-feedback.md`
   - Implement all "Implement now" changes from Story 2.9 decision log
   - Update OnShape models as needed
   - Re-export STLs if models changed (overwrite old STLs or version as `v1.1` if significant changes)

2. **OnShape Version Tagging:**
   - In OnShape, create version/release:
     - Version name: "V1.0 Draft - Ready to Print"
     - Description: "Complete mechanical design, community feedback incorporated, STLs exported for Epics 3-7"
     - Date: [Current date]
   - This locks the design state (can always revert or branch from this version)

3. **GitHub Release (Optional but Recommended):**
   - Create GitHub release tag: `v1.0-design-complete`
   - Release notes:
     - "Epic 2 Complete: OnShape Design & Community Feedback"
     - Summary of design: Modules, key specs, weight, dimensions
     - Link to OnShape assemblies
     - Link to community feedback summary
     - Note: "STLs in `hardware/3d-models/stl/`, ready for Epics 3-7 build phases"
   - Attach STL zip file (all STLs in one archive for easy download by community)

4. **Documentation Updates:**
   - Update `hardware/3d-models/README.md`:
     - OnShape links (all assemblies)
     - Design philosophy summary
     - Community feedback highlights
     - Known issues / future improvements (anything deferred to V2)
     - Print settings reference (`PRINT_SETTINGS.md`)
   - Update main `README.md`:
     - Epic 2 status: Complete ✅
     - Link to OnShape design
     - Link to STL downloads (GitHub release or `hardware/3d-models/stl/`)

5. **BOM Finalization:**
   - Verify `hardware/bom/epic2_bom.csv` complete:
     - Filament (PLA, PETG quantities and costs)
     - Heat-set inserts (all sizes and quantities)
     - Hardware (M2.5, M3, M4, M5 bolts, various lengths)
     - Tools (soldering iron for inserts, Allen keys, calipers)
   - Total Epic 2 cost estimate: ~$80-120 (filament + inserts + bolts)

6. **Epic 2 Milestone Closure:**
   - GitHub milestone "Epic 2: Complete OnShape Design" created (if not already)
   - All Epic 2 stories linked to milestone, marked complete
   - Milestone closed with summary:
     - "All mechanical components designed, validated, and exported as STLs"
     - "Community feedback incorporated, design ready for build phases"
     - "Next: Epic 3 (Head Module Build)"

7. **Build-in-Public Content (LinkedIn/Social):**
   - **Post content:**
     - Title: "Olaf's Design Complete! 🤖✨ Full CAD in OnShape (Open Source)"
     - Images: 3-4 best renders from Story 2.9 (complete robot, head close-up, neck gimbal)
     - Text:
       - "After 2-3 weeks of CAD work, Olaf's V1 mechanical design is done!"
       - "7 DOF articulation (3-axis neck, 2-axis ears per side), dual OLED eyes, stereo vision, floor projector"
       - "Posted to Reddit for feedback - community suggested X, Y, Z improvements (implemented!)"
       - "All CAD public on OnShape, STLs ready to print"
       - "Next: Printing and assembling head module (Epic 3)"
       - Link to OnShape, GitHub
   - Post published to LinkedIn, Twitter, or personal blog

**Dependencies:** Story 2.9 (feedback collected and documented)

**Estimated Effort:** 4-6 hours (model updates, version tagging, documentation, content creation)

---

## Epic 2 Summary

**Total Stories:** 10
**Estimated Total Effort:** 70-98 hours (2-3 weeks for solo builder with CAD experience, 3-4 weeks if learning OnShape)

**Key Deliverables:**
- ✅ Complete OnShape CAD design (all modules: head, ears, neck, torso, power, projector, base)
- ✅ Component dimensions spreadsheet (all hardware measured and documented)
- ✅ STL files exported for all parts (organized by module, ready to print)
- ✅ Comprehensive print settings documentation (material, orientation, supports, time/cost estimates)
- ✅ Community feedback gathered and incorporated (Reddit/maker forums, 3-7 day cycle)
- ✅ Design validated (interference checks, weight budget, CoG analysis, printability review)
- ✅ Public OnShape links (V1.0 tagged version, community can view/fork)
- ✅ BOM updated (filament, inserts, hardware costs)
- ✅ Build-in-public content (LinkedIn post, project showcase)

**Success Criteria Met:**
- All mechanical components designed to fit real hardware dimensions (no "will it fit?" surprises)
- Weight budget: ≤6kg confirmed via OnShape mass properties
- No interferences at full range of motion (7 DOF tested in simulation)
- Community feedback incorporated (design validated by external builders/engineers)
- STLs production-ready (no redesign-reprint cycles, saves time and material costs)
- Design is fully open-source and replicable (OnShape public, STLs downloadable)

**What This Epic Enables:**
- Epic 3: Head module can be printed and assembled with confidence (no design unknowns)
- Epic 4: Ears + neck can be built knowing servo mounts will fit correctly
- Epic 5: Torso + power enclosure dimensions finalized (battery, converters, Pi all fit)
- Epic 6: Base platform ready to print (wheel mounts, ODrive bay validated)
- Epic 7: Integration phase has complete design to work from (no improvisation needed)
- Community: Followers can start printing along, replicate build, suggest improvements

**Next Epic:** Epic 3: Head Module - Complete Build (print head parts, assemble eyes + sensors + OAK-D Pro)

---

**Related Documents:**
- [Epic List](epic-list.md) - Overall project roadmap
- [Epic 1: Foundation](epic-01-foundation.md) - I2C architecture + heart display (provides initial concept)
- [Epic 3: Head Module Build](epic-03-head-build.md) - First assembly using Epic 2 designs
- Component dimensions: `hardware/component-dimensions.csv`
- Print settings: `hardware/3d-models/PRINT_SETTINGS.md`
- Community feedback: `docs/community-feedback/reddit-v1-design-feedback.md`
