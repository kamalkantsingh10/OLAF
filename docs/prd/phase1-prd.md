# OLAF Phase 1: Hardware Build & ROS2 Foundation - Product Requirements Document (PRD)

## Goals and Background Context

### Goals

Phase 1 establishes the physical and software foundation for OLAF, delivering:

- ✅ **Hardware Foundation**: All 4 modules (Head+Ears, Neck, Torso, Base) physically assembled, PCB-integrated, and mounted on robot frame
- ✅ **ROS2 Control Layer**: Basic pub/sub ROS2 driver nodes enabling Pi to command each module independently
- ✅ **Module Independence**: Each module testable in isolation with component-level validation scripts
- ✅ **Self-Balancing Capability**: Base module operational with 200Hz PID control running on ESP32
- ✅ **End-to-End Validation**: Simple demo scripts proving coordinated movement across all modules via ROS2
- ✅ **Build-Ready Documentation**: Wiring diagrams, assembly instructions, and troubleshooting guides for replication

### Background Context

OLAF's architecture requires **physical-first development**: the personality coordination, AI integration, and SLAM navigation (Phase 2) cannot begin until the hardware modules respond reliably to software commands. Phase 1 eliminates hardware risk before investing in complex middleware.

This phase follows a **module-by-module build pattern**: breadboard → PCB design → order → test → 3D print → firmware → ROS2 integration → mount. Each module completes this cycle independently, enabling incremental validation and reducing debugging complexity when integrating the full system.

The **ROS2 foundation** setup happens first, establishing the orchestration layer workspace and tooling. Then each module gets its own ROS2 driver node with basic pub/sub topics (no services/actions yet—those come in Phase 2 for coordinated expressions).

**Architecture Updates for Phase 1**:
- Kickstand servo moved from Base module to Neck module (I2C 0x09) for easier daisy-chaining with neck servos
- Base module focuses exclusively on balancing, mobility, and power distribution
- 36V hoverboard battery and motors provide proven platform for self-balancing
- Power distribution system converts 36V to 12V/5V/3.3V rails for all modules

### Change Log

| Date | Version | Description | Author |
|------|---------|-------------|--------|
| 2025-12-16 | v1.0 | Initial Phase 1 PRD | Kamal Singh |

---

## Requirements

### Functional Requirements

**FR1**: Each of the 4 modules (Head+Ears, Neck, Torso, Base) must be breadboarded with all components connected and individually tested for functionality before PCB design begins.

**FR2**: Custom PCBs integrating ESP32 + all sensors/actuators must be designed in **Fritzing** for all 4 modules, following the module-first architecture pattern.

**FR3**: PCBs must be ordered from **Elecrow (China)** for cost optimization, received, and assembled with all components soldered and tested.

**FR4**: 3D printed enclosures, brackets, and mounting hardware must be designed in OnShape and printed for all 4 modules. Kickstand mechanism will include metal components (aluminum/steel rod, metal ground pad) for load-bearing strength.

**FR5**: All 4 modules must be physically mounted to the robot frame with secure connections and proper cable management.

**FR6**: Raspberry Pi 5 must be installed in the Torso module with speakerphone connected (USB), **OAK-D-Pro RGBD camera** connected (USB to Pi, camera mounted in Head+Ears module), and floor projector connected (HDMI to Head+Ears module).

**FR7**: ESP32 firmware must be developed for each module implementing I2C slave interface (addresses 0x08-0x0B), hardware drivers, and basic command handlers.

**FR8**: ROS2 driver nodes must be created for each module with basic pub/sub topics enabling Pi to send commands and receive sensor data. **As each module is built, it must be discoverable as ROS2 topics** (e.g., `/head_ears/command`, `/neck/position`) for incremental validation.

**FR9**: Component-level testing scripts must exist for each module validating electrical connectivity, sensor readings, actuator movement, and display output.

**FR10**: ROS2 Humble must be installed and configured on Raspberry Pi with workspace structure following the module-first architecture.

**FR11**: I2C communication must be validated between Pi (master) and all 4 ESP32 modules (slaves) with reliable read/write operations at 400kHz bus speed.

**FR12**: Simple end-to-end demo script must demonstrate coordinated movement: **deploy kickstand, blink eyes, move ears, tilt head, animate heart, retract kickstand, and balance base**—all triggered by ROS2 topic publishes.

**FR13**: Base module self-balancing must be operational with **basic 200Hz PID control loop fine-tuned for stable balancing**, running on ESP32, MPU6050 IMU integration, and ODrive motor control via UART. Base uses **36V hoverboard battery and BLDC motors**. Advanced balancing features (adaptive gains, disturbance rejection) are deferred to Phase 2.

**FR14**: Neck module must include kickstand servo (Feetech STS3215) daisy-chained with 3-DOF neck servos for stationary mode, controlled by Neck ESP32 firmware. Kickstand mechanism designed with metal components for structural strength.

**FR15**: Head+Ears module ESP32 must control floor projector power (GPIO → optocoupler) and focus (linear servo via PWM/UART) in response to I2C commands, while Pi sends HDMI video content.

**FR16**: Base module houses **36V hoverboard battery pack** and **power distribution system** converting 36V to 12V/5V/3.3V rails with buck converters, distributing power to all 4 modules (Head+Ears, Neck, Torso, Base ESP32).

### Non-Functional Requirements

**NFR1**: Each module must be independently testable without requiring other modules to be connected or operational.

**NFR2**: Comprehensive documentation must be created including per-module wiring diagrams (pin assignments, circuits), assembly instructions (step-by-step with photos), and troubleshooting guides (common issues and solutions).

**NFR3**: Phase 1 must NOT include AI integration, personality coordination, or SLAM navigation—those are explicitly scoped for Phase 2.

**NFR4**: All code must follow the module-first repository structure with firmware in `modules/{module}/firmware/`, ROS2 drivers in `ros2/src/olaf_drivers/{module}_driver/`, and tests colocated with implementations.

**NFR5**: All code, PCB designs, and 3D models must be committed to Git with descriptive commit messages and tagged at Phase 1 completion milestone (`v1.0-phase1`).

**NFR6**: PCB designs must use standard component footprints and avoid exotic parts to ensure manufacturability and future repairability.

**NFR7**: All 3D printed parts must be printable on a standard FDM printer (e.g., Prusa, Ender 3) without support structures where feasible.

---

## Technical Assumptions

### Repository Structure
**Structure**: Monorepo
**Tool**: Git (standard)
**Organization**: Module-first architecture where each physical module (Head+Ears, Neck, Torso, Base) is a complete subsystem with firmware, hardware, tests, diagnostics, and documentation colocated.
**Rationale**: Matches distributed smart peripheral architecture; enables independent module development; simplifies navigation (everything for a module lives in `modules/{module}/`).

### Service Architecture
**Architecture**: Distributed Embedded System
**Pattern**: Smart I2C Peripherals (ESP32s) + Central Orchestrator (Raspberry Pi 5)
**Communication**: I2C bus (400kHz) for Pi ↔ ESP32 modules; ROS2 Humble pub/sub on Pi
**Rationale**: I2C provides 5-20ms latency vs 80-200ms WiFi; ROS2 industry-standard for robotics; ESP32 offloads real-time tasks from Linux Pi.

### Testing Requirements
**Phase 1 Scope**:
- ✅ **Component-level tests**: Individual hardware validation (servos move, OLEDs display, motors spin)
- ✅ **I2C communication tests**: Pi can read/write each ESP32 module
- ✅ **Module integration tests**: Basic multi-module coordination (blink eyes + move head)
- ✅ **Performance validation**: 200Hz IMU loop on Base, 30 FPS OLED animation on Head
- ❌ **No unit tests for firmware**: Deferred to Phase 2 (focus on hardware validation first)
- ❌ **No automated CI/CD**: Manual testing sufficient for hardware prototyping phase

**Rationale**: Hardware integration risk is highest in Phase 1; focus testing effort on physical validation before investing in test automation infrastructure.

### Hardware Platforms

**Compute Platform**:
- **Main**: Raspberry Pi 5 16GB + Hailo AI Kit (26 TOPS, PCIe) - orchestration layer
- **Modules**: ESP32-WROOM-32 (or equivalent) per module - smart I2C slaves

**Camera**:
- **Model**: Luxonis OAK-D-Pro (RGBD camera with IMU over USB)
- **Usage**: SLAM input, obstacle detection, person tracking (Phase 2 integration)

**Base Module - Hoverboard Components**:
- **Battery**: 36V hoverboard battery pack (10S Li-ion, typically 4-5Ah)
- **Motors**: 2× hoverboard BLDC motors (typically 350W each, 6.5" wheels)
- **Motor Controller**: ODrive S1 (or equivalent) handling 36V input, dual motor control via UART
- **IMU**: MPU6050 (I2C) for 200Hz balancing loop

**Power Distribution** (Base Module):
- **Input**: 36V from hoverboard battery
- **Buck Converters**: 36V → 12V (servos), 36V → 5V (Pi, 6A+), 36V → 3.3V (ESP32s, 2A+)
- **Protection**: Fuse/circuit breaker on 36V line, low-voltage cutoff, emergency stop switch

**PCB Design Tool**:
- **Software**: Fritzing (schematic capture + PCB layout)
- **Manufacturer**: Elecrow (China) for cost-effective fabrication
- **Copper Weight**: 2oz copper for high-current power traces on Base module PCB

**3D Modeling**:
- **Software**: OnShape (cloud-based CAD, version controlled)
- **Printer**: Standard FDM (Prusa, Ender 3, or equivalent)

**Rationale**: Tools chosen for maker accessibility (Fritzing easier than KiCad for beginners), cost optimization (Elecrow competitive pricing), proven hoverboard components for self-balancing, and build-in-public workflow (OnShape cloud collaboration).

### Software Stack

**Orchestration Layer (Raspberry Pi)**:
- **OS**: Raspberry Pi OS (Debian 12 Bookworm, 64-bit)
- **ROS2**: Humble Hawksbill (LTS until May 2027)
- **Language**: Python 3.11+ for ROS2 nodes
- **Dependencies**: `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `smbus2` (I2C), `odrive` (motor control)

**Module Firmware (ESP32)**:
- **Framework**: Arduino/ESP-IDF (PlatformIO build system)
- **Language**: C/C++
- **Libraries**: Wire (I2C), Adafruit GFX (OLED), ODriveArduino (motor control), MPU6050 (IMU)

**Version Control**: Git (GitHub repository)

**Rationale**: ROS2 Humble LTS ensures 5+ year support; Python for rapid prototyping; PlatformIO for multi-target ESP32 firmware management.

### Additional Technical Assumptions

**I2C Bus Configuration**:
- **Addresses**: Head+Ears (0x08), Neck (0x09), Torso (0x0A), Base (0x0B)
- **Speed**: 400kHz (standard mode) for Phase 1, upgrade to 1MHz (fast mode) if needed in Phase 2
- **Pull-ups**: 4.7kΩ resistors on Pi side

**Servo Communication**:
- Feetech STS3215 servos use **TTL serial (half-duplex UART)** for control
- Neck module: 4 servos daisy-chained (3 DOF + kickstand) on single UART bus
- Ear modules: 2 servos each (2 DOF per ear) on separate UART buses

**ODrive Configuration**:
- **Model**: ODrive S1 (or equivalent) with dual motor control
- **Input Voltage**: 36V from hoverboard battery
- **Communication**: UART (115200 baud) from Base ESP32
- **Control Mode**: Velocity control for differential drive, position control for balancing

**ROS2 Message Types**:
- **Phase 1**: Standard messages only (`std_msgs/String`, `std_msgs/Int32`, `geometry_msgs/Twist`)
- **Phase 2**: Custom messages (.msg), services (.srv), actions (.action)

**Rationale**: Keep Phase 1 simple with standard ROS2 messages; defer custom interfaces until personality coordination requirements are fully understood.

---

## Epic List

**Epic 0: ROS2 Foundation Setup**
Establish the orchestration layer workspace on Raspberry Pi with ROS2 Humble installed, workspace structure created following module-first architecture, and I2C communication tools configured. This enables incremental module integration as each is built.

**Epic 1: Head+Ears Module Build**
Complete Head+Ears module (I2C 0x08) from breadboard prototype through PCB fabrication, 3D printed enclosure, ESP32 firmware development, ROS2 driver node creation, and physical mounting to robot frame. Module controls 2× OLED eyes (SPI), 2× articulated ears (2-DOF UART servos), floor projector power/focus (GPIO/PWM), and integrates OAK-D-Pro camera (USB to Pi).

**Epic 2: Neck Module Build**
Complete Neck module (I2C 0x09) with 3-DOF servo array (pan/tilt/roll), kickstand servo (daisy-chained STS3215 with metal structural components), 2× presence sensors, ESP32 firmware, ROS2 driver node, and mounting to robot frame. Kickstand enables stationary mode vs balancing mode.

**Epic 3: Torso Module Build**
Complete Torso module (I2C 0x0A) housing Raspberry Pi 5 + Hailo AI Kit, 2.8" square heart display (SPI), thermal printer, ESP32 firmware, ROS2 driver node, and mounting to robot frame. This module is the compute hub, receiving power from Base module.

**Epic 4: Base Module Build**
Complete Base module (I2C 0x0B) with self-balancing two-wheel platform using hoverboard components (36V battery, BLDC motors), ODrive motor controller, MPU6050 IMU, 200Hz PID control loop, power distribution system (36V → 12V/5V/3.3V), ESP32 firmware, ROS2 driver node, and mounting to robot frame. Achieves basic fine-tuned balancing capability and powers all modules.

**Epic 5: End-to-End Demo Script**
Create simple ROS2 Python script demonstrating coordinated multi-module operation: **deploy kickstand, blink eyes, move ears, tilt head, animate heart, retract kickstand, and balance base**—all triggered via ROS2 topic publishes. Validates Phase 1 completion: all modules respond to software commands.

---

## Epic 0: ROS2 Foundation Setup

**Goal**: Establish the ROS2 Humble development environment on Raspberry Pi 5 with proper workspace structure, I2C tools, and module discovery capability, enabling incremental testing as each module is built.

### Story 0.1: Install ROS2 Humble on Raspberry Pi

**As a** builder,
**I want** ROS2 Humble installed on Raspberry Pi 5 with all dependencies,
**so that** I can develop ROS2 driver nodes for OLAF modules.

**Acceptance Criteria:**
1. Raspberry Pi OS (Debian 12 Bookworm, 64-bit) is installed and updated
2. ROS2 Humble Hawksbill is installed following official Pi installation guide
3. Core ROS2 packages are verified (`ros-humble-desktop` or `ros-humble-base`)
4. `rosdep` is initialized and dependencies are installed
5. ROS2 environment is sourced in `.bashrc` for automatic activation
6. Basic ROS2 commands work: `ros2 topic list`, `ros2 node list`

### Story 0.2: Create ROS2 Workspace with Module-First Structure

**As a** builder,
**I want** a ROS2 workspace structured according to the module-first architecture,
**so that** each module's ROS2 driver node is organized and discoverable.

**Acceptance Criteria:**
1. Workspace directory created at `~/olaf/ros2/` with `src/` subdirectory
2. Following packages created under `src/`: `olaf_bringup`, `olaf_description`, `olaf_drivers`
3. Under `olaf_drivers/`, create placeholder packages: `head_ears_driver`, `neck_driver`, `torso_driver`, `base_driver`
4. Each driver package has proper `package.xml` and `setup.py` (Python) or `CMakeLists.txt` (C++)
5. Workspace builds successfully: `colcon build` completes without errors
6. Workspace can be sourced: `source install/setup.bash` works

### Story 0.3: Configure I2C Communication Tools

**As a** builder,
**I want** I2C communication configured and testable on Raspberry Pi,
**so that** I can validate module connectivity as each ESP32 is built.

**Acceptance Criteria:**
1. I2C interface enabled in Raspberry Pi configuration (`raspi-config`)
2. Python `smbus2` library installed: `pip install smbus2`
3. I2C tools installed: `sudo apt install i2c-tools`
4. I2C bus scan works: `i2cdetect -y 1` displays bus addresses
5. Simple Python test script can read/write I2C register to a test device (or loopback)
6. Module I2C addresses documented (0x08=Head+Ears, 0x09=Neck, 0x0A=Torso, 0x0B=Base)

### Story 0.4: Set Up Development Tools and Dependencies

**As a** builder,
**I want** all necessary development tools and Python dependencies installed,
**so that** I can efficiently develop and test ROS2 nodes.

**Acceptance Criteria:**
1. Python 3.11+ is installed and set as default
2. PlatformIO is installed for ESP32 firmware development: `pip install platformio`
3. Git is configured with user name and email
4. Required Python packages installed: `rclpy`, `odrive`, `numpy`
5. VS Code (or preferred IDE) is installed with ROS2 and PlatformIO extensions
6. USB permissions configured for OAK-D-Pro camera and ESP32 flashing

---

## Epic 1: Head+Ears Module Build

**Goal**: Complete the Head+Ears module (I2C 0x08) with full hardware integration, ESP32 firmware, ROS2 driver node, and physical mounting. Module will control 2× OLED eyes (128×64 SPI), 2× articulated ears (2-DOF UART servos each), floor projector power/focus control, and integrate OAK-D-Pro RGBD camera.

### Story 1.1: Breadboard Head+Ears Components and Test Connectivity

**As a** builder,
**I want** all Head+Ears components breadboarded and individually tested,
**so that** I can verify functionality before committing to PCB design.

**Acceptance Criteria:**
1. ESP32 development board connected to breadboard with power supply
2. 2× OLED displays (128×64, SPI) wired and displaying test images
3. 2× ear servo buses wired (4 servos total: 2-DOF per ear, Feetech UART)
4. Floor projector power circuit breadboarded (GPIO → optocoupler → relay/MOSFET)
5. Floor projector focus servo wired (linear servo via PWM/UART)
6. OAK-D-Pro camera connected to separate test system (Pi) and verified working
7. Component-level test script confirms: OLEDs display animations, servos move to commanded positions, projector power switches on/off, focus servo adjusts

### Story 1.2: Design Head+Ears Custom PCB in Fritzing

**As a** builder,
**I want** a custom PCB designed in Fritzing integrating ESP32 and all Head+Ears components,
**so that** I can replace the breadboard with a compact, reliable circuit board.

**Acceptance Criteria:**
1. Fritzing schematic created with ESP32 module, OLED connections (SPI), servo UART buses, optocoupler circuit, focus servo connection
2. PCB layout designed with proper component placement, trace routing, and mounting holes
3. Power distribution traces sized appropriately (5V, 3.3V rails)
4. Pin assignments documented in wiring diagram (saved as `modules/head-ears/wiring.md`)
5. Design reviewed for electrical correctness (no shorts, proper pull-ups/pull-downs)
6. Gerber files exported and ready for manufacturing

### Story 1.3: Order and Receive Head+Ears PCB from Elecrow

**As a** builder,
**I want** the Head+Ears PCB ordered from Elecrow and delivered,
**so that** I can proceed with assembly.

**Acceptance Criteria:**
1. Gerber files uploaded to Elecrow website
2. PCB specifications confirmed (layer count, thickness, color, quantity)
3. Order placed and payment completed
4. PCB received and visually inspected for manufacturing defects
5. Continuity test confirms no shorts between power rails
6. PCB dimensions and mounting holes match design specifications

### Story 1.4: Assemble and Test Head+Ears PCB

**As a** builder,
**I want** all components soldered to Head+Ears PCB and tested for functionality,
**so that** I have a working circuit board ready for firmware development.

**Acceptance Criteria:**
1. All components soldered: ESP32 module, connectors for OLEDs/servos/projector, optocoupler, passive components
2. Visual inspection confirms proper solder joints (no cold joints, bridges)
3. Power-on test confirms correct voltage rails (5V, 3.3V) with no excessive current draw
4. I2C address configured as 0x08 in firmware (or hardware jumpers if applicable)
5. Component-level tests pass: OLEDs display test pattern, servos respond to commands, projector power circuit switches, focus servo moves
6. PCB mounted on temporary test fixture for firmware development

### Story 1.5: Design and 3D Print Head+Ears Enclosure

**As a** builder,
**I want** 3D printed enclosure and mounting brackets designed in OnShape and printed,
**so that** I can protect the PCB and mount OLEDs/servos/projector to the robot.

**Acceptance Criteria:**
1. OnShape CAD model created for Head+Ears enclosure with space for PCB, OLED mounts, ear servo brackets, projector mount, camera mount
2. Design includes cable routing channels and mounting holes for robot frame attachment
3. STL files exported for 3D printing
4. Parts printed on FDM printer with appropriate settings (layer height, infill, supports)
5. Printed parts fit together with PCB, OLEDs, servos, projector, and camera
6. Assembly instructions drafted with photos (saved as `modules/head-ears/assembly.md`)

### Story 1.6: Develop Head+Ears ESP32 Firmware

**As a** builder,
**I want** ESP32 firmware that implements I2C slave interface and hardware drivers for all Head+Ears components,
**so that** the Pi can send commands to control eyes, ears, and projector.

**Acceptance Criteria:**
1. PlatformIO project created in `modules/head-ears/firmware/`
2. I2C slave interface implemented at address 0x08 with register map defined
3. OLED driver implemented: can display images, animations at 30+ FPS
4. Ear servo driver implemented: can command 4 servos (2 per ear) to positions
5. Projector control implemented: GPIO commands turn power on/off, PWM/UART controls focus servo
6. Basic I2C commands work: `SET_EYES`, `SET_EAR_POSITION`, `PROJECTOR_ON/OFF`, `FOCUS_NEAR/FAR`
7. Firmware flashed to ESP32 and responds to I2C commands from Pi

### Story 1.7: Create Head+Ears ROS2 Driver Node

**As a** builder,
**I want** a ROS2 driver node that translates ROS2 topics into I2C commands for Head+Ears module,
**so that** I can control the module using standard ROS2 pub/sub.

**Acceptance Criteria:**
1. Python ROS2 node created in `ros2/src/olaf_drivers/head_ears_driver/`
2. Node subscribes to topics: `/head_ears/eyes` (display command), `/head_ears/ears` (position command), `/head_ears/projector` (power/focus command)
3. Node publishes sensor data (if applicable, e.g., camera feed as separate topic)
4. I2C communication implemented using `smbus2` library
5. ROS2 messages translated to I2C register writes to address 0x08
6. Node launches successfully and appears in `ros2 node list`
7. Manual topic publish triggers expected hardware response: `ros2 topic pub /head_ears/eyes ...` makes eyes blink

### Story 1.8: Mount Head+Ears Module to Robot Frame

**As a** builder,
**I want** Head+Ears module physically mounted to robot frame with all cables connected,
**so that** the module is integrated into the robot structure.

**Acceptance Criteria:**
1. Module mounted securely to robot frame using 3D printed brackets and hardware
2. OLED displays positioned correctly for "eyes" appearance
3. Ear servos mounted with full range of motion (no mechanical interference)
4. Floor projector mounted and aimed at floor surface
5. OAK-D-Pro camera mounted with clear field of view
6. I2C cable connected to Pi (appropriate length, strain relief)
7. Power cables connected to Base module power distribution
8. Cable management ensures no loose wires interfering with movement
9. Module responds to ROS2 commands when powered: `ros2 topic pub` tests pass

---

## Epic 2: Neck Module Build

**Goal**: Complete the Neck module (I2C 0x09) with 3-DOF servo array (pan/tilt/roll), kickstand servo (daisy-chained via UART), 2× presence sensors for 360° human detection, ESP32 firmware, ROS2 driver node, and physical mounting. This module enables expressive head gestures and stationary/balancing mode transitions.

### Story 2.1: Breadboard Neck Components and Test Connectivity

**As a** builder,
**I want** all Neck components breadboarded and individually tested,
**so that** I can verify functionality before committing to PCB design.

**Acceptance Criteria:**
1. ESP32 development board connected to breadboard with power supply
2. 4× Feetech STS3215 servos wired on single UART bus (daisy-chained): 3 for pan/tilt/roll + 1 for kickstand
3. 2× presence sensors (mmWave or PIR) wired and providing detection signals
4. Servo daisy-chain communication tested: all 4 servos respond to unique IDs
5. Presence sensors tested: can detect human presence in their coverage zones
6. Component-level test script confirms: all servos move to commanded positions, kickstand deploys/retracts, presence sensors report detection state
7. Servo ID configuration documented (e.g., Pan=ID1, Tilt=ID2, Roll=ID3, Kickstand=ID4)

### Story 2.2: Design Neck Custom PCB in Fritzing

**As a** builder,
**I want** a custom PCB designed in Fritzing integrating ESP32 and all Neck components,
**so that** I can replace the breadboard with a compact, reliable circuit board.

**Acceptance Criteria:**
1. Fritzing schematic created with ESP32 module, servo UART bus (daisy-chain topology), presence sensor connections
2. PCB layout designed with proper component placement, trace routing, and mounting holes
3. Power distribution traces sized appropriately for servo current draw (5V rail, beefy traces)
4. Pin assignments documented in wiring diagram (saved as `modules/neck/wiring.md`)
5. Design reviewed for electrical correctness (no shorts, proper pull-ups for UART)
6. Gerber files exported and ready for manufacturing

### Story 2.3: Order and Receive Neck PCB from Elecrow

**As a** builder,
**I want** the Neck PCB ordered from Elecrow and delivered,
**so that** I can proceed with assembly.

**Acceptance Criteria:**
1. Gerber files uploaded to Elecrow website
2. PCB specifications confirmed (layer count, thickness, color, quantity)
3. Order placed and payment completed
4. PCB received and visually inspected for manufacturing defects
5. Continuity test confirms no shorts between power rails
6. PCB dimensions and mounting holes match design specifications

### Story 2.4: Assemble and Test Neck PCB

**As a** builder,
**I want** all components soldered to Neck PCB and tested for functionality,
**so that** I have a working circuit board ready for firmware development.

**Acceptance Criteria:**
1. All components soldered: ESP32 module, connectors for servos/presence sensors, passive components
2. Visual inspection confirms proper solder joints (no cold joints, bridges)
3. Power-on test confirms correct voltage rails (5V, 3.3V) with no excessive current draw
4. I2C address configured as 0x09 in firmware (or hardware jumpers if applicable)
5. Component-level tests pass: all 4 servos respond on UART bus, presence sensors provide readings
6. PCB mounted on temporary test fixture for firmware development

### Story 2.5: Design and 3D Print Neck Enclosure and Servo Mounts

**As a** builder,
**I want** 3D printed enclosure, servo mounting brackets, and kickstand mechanism designed in OnShape and printed,
**so that** I can protect the PCB and mount servos/sensors to the robot.

**Acceptance Criteria:**
1. OnShape CAD model created for Neck enclosure with space for PCB, 3-DOF servo mounting, kickstand servo mount, presence sensor mounts
2. Design includes mechanical linkages for pan/tilt/roll motion and kickstand deployment mechanism
3. Design ensures full range of motion for all servos without mechanical interference
4. STL files exported for 3D printing
5. Parts printed on FDM printer with appropriate settings (layer height, infill, supports)
6. Printed parts fit together with PCB, servos, sensors, and allow smooth motion
7. Assembly instructions drafted with photos (saved as `modules/neck/assembly.md`)

### Story 2.6: Design Kickstand Mechanism with Single Servo

**As a** builder,
**I want** a kickstand mechanism designed that deploys/retracts using a single STS3215 servo,
**so that** the robot can transition between stationary mode (kickstand down) and balancing mode (kickstand up).

**Acceptance Criteria:**
1. OnShape CAD model created for kickstand mechanism including servo mount, linkage/lever arm, and ground contact pad
2. Mechanism design allows single servo rotation to deploy kickstand downward (stationary mode) and retract upward (balancing mode)
3. Deployed kickstand position is stable and can support robot weight (load calculation documented)
4. Retracted kickstand position clears ground and doesn't interfere with base module wheels
5. Servo mounting position integrated into Neck module mechanical design
6. Mechanical range of motion matches servo rotation limits (no over-travel)
7. **Bill of materials includes both 3D printed parts and metal components (e.g., aluminum or steel rod for kickstand leg, metal ground pad, fasteners, bearings if needed)**
8. STL files exported for 3D printed kickstand components (servo bracket, mounting plate, linkage parts)
9. **Metal parts specified with dimensions for fabrication or sourcing (e.g., "10mm aluminum rod, 150mm length" or "steel foot plate, 50mm × 50mm × 3mm")**
10. Parts printed and metal components fabricated/sourced, then mechanically tested with servo: smooth deploy/retract motion, stable when deployed under load
11. Kickstand design documented with photos, BOM, servo angle specifications (deploy angle, retract angle), and load test results in `modules/neck/assembly.md`

### Story 2.7: Develop Neck ESP32 Firmware

**As a** builder,
**I want** ESP32 firmware that implements I2C slave interface and hardware drivers for all Neck components,
**so that** the Pi can send commands to control neck position, kickstand, and read presence sensors.

**Acceptance Criteria:**
1. PlatformIO project created in `modules/neck/firmware/`
2. I2C slave interface implemented at address 0x09 with register map defined
3. Servo driver implemented: can command 4 servos on daisy-chain UART bus (pan/tilt/roll angles + kickstand deploy/retract)
4. Presence sensor driver implemented: reads 2 sensors and reports detection state
5. Basic I2C commands work: `SET_NECK_POSITION` (pan/tilt/roll angles), `DEPLOY_KICKSTAND`, `RETRACT_KICKSTAND`, `GET_PRESENCE_STATE`
6. Smooth motion curves implemented (optional but recommended for organic movement)
7. Firmware flashed to ESP32 and responds to I2C commands from Pi

### Story 2.8: Create Neck ROS2 Driver Node

**As a** builder,
**I want** a ROS2 driver node that translates ROS2 topics into I2C commands for Neck module,
**so that** I can control the module using standard ROS2 pub/sub.

**Acceptance Criteria:**
1. Python ROS2 node created in `ros2/src/olaf_drivers/neck_driver/`
2. Node subscribes to topics: `/neck/position` (pan/tilt/roll command), `/neck/kickstand` (deploy/retract command)
3. Node publishes sensor data: `/neck/presence` (detection state from 2 sensors)
4. I2C communication implemented using `smbus2` library
5. ROS2 messages translated to I2C register writes to address 0x09
6. Node launches successfully and appears in `ros2 node list`
7. Manual topic publish triggers expected hardware response: `ros2 topic pub /neck/position ...` moves head, `ros2 topic pub /neck/kickstand ...` deploys/retracts kickstand

### Story 2.9: Mount Neck Module to Robot Frame

**As a** builder,
**I want** Neck module physically mounted to robot frame with all cables connected,
**so that** the module is integrated into the robot structure and can support the head.

**Acceptance Criteria:**
1. Module mounted securely to robot frame (likely connecting Torso to Head+Ears module)
2. Servos positioned to allow full pan/tilt/roll range of motion
3. Kickstand mechanism can deploy and retract without obstruction
4. Presence sensors positioned for 360° coverage (front + back or left + right)
5. I2C cable connected to Pi (appropriate length, strain relief)
6. Power cables connected to Base module power distribution
7. Cable management ensures no loose wires interfering with neck movement
8. Module responds to ROS2 commands when powered: `ros2 topic pub` tests pass for position and kickstand
9. Mechanical load test: neck can support weight of Head+Ears module without sagging

---

## Epic 3: Torso Module Build

**Goal**: Complete the Torso module (I2C 0x0A) housing Raspberry Pi 5 + Hailo AI Kit, 2.8" square heart display (SPI), and thermal printer. This module is the compute hub for the robot, hosting the ROS2 orchestration layer and personality display elements. Power is received from Base module via cable.

### Story 3.1: Breadboard Torso Components and Test Connectivity

**As a** builder,
**I want** all Torso components breadboarded and individually tested,
**so that** I can verify functionality before committing to PCB design.

**Acceptance Criteria:**
1. ESP32 development board connected to breadboard with power supply
2. 2.8" square display (SPI) wired and displaying test animations (beating heart pattern)
3. Thermal printer wired (UART or parallel interface) and printing test output
4. Component-level test script confirms: display animates smoothly (heart beat patterns at various rhythms), printer outputs text and simple graphics

### Story 3.2: Design Torso Custom PCB in Fritzing

**As a** builder,
**I want** a custom PCB designed in Fritzing integrating ESP32 and all Torso components,
**so that** I can replace the breadboard with a compact, reliable circuit board.

**Acceptance Criteria:**
1. Fritzing schematic created with ESP32 module, 2.8" display connection (SPI), thermal printer connection (UART/parallel)
2. PCB layout designed with proper component placement, trace routing, and mounting holes
3. Power input connector included (receives 5V and/or 3.3V from Base module power distribution)
4. I2C connector included (SDA, SCL, GND for connection to Pi I2C bus)
5. Pin assignments documented in wiring diagram (saved as `modules/torso/wiring.md`)
6. Design reviewed for electrical correctness (no shorts, proper decoupling capacitors)
7. Gerber files exported and ready for manufacturing

### Story 3.3: Order and Receive Torso PCB from Elecrow

**As a** builder,
**I want** the Torso PCB ordered from Elecrow and delivered,
**so that** I can proceed with assembly.

**Acceptance Criteria:**
1. Gerber files uploaded to Elecrow website
2. PCB specifications confirmed (layer count, thickness, color, quantity)
3. Order placed and payment completed
4. PCB received and visually inspected for manufacturing defects
5. Continuity test confirms no shorts between power rails
6. PCB dimensions and mounting holes match design specifications

### Story 3.4: Assemble and Test Torso PCB

**As a** builder,
**I want** all components soldered to Torso PCB and tested for functionality,
**so that** I have a working circuit board ready for firmware development.

**Acceptance Criteria:**
1. All components soldered: ESP32 module, connectors for display/printer/power/I2C, passive components
2. Visual inspection confirms proper solder joints (no cold joints, bridges)
3. Power-on test: use bench power supply (5V, 3.3V) to verify proper operation with no excessive current draw
4. I2C address configured as 0x0A in firmware (or hardware jumpers if applicable)
5. Component-level tests pass: display shows animations, printer outputs test page
6. PCB mounted on temporary test fixture for firmware development

### Story 3.5: Design and 3D Print Torso Enclosure with Pi Housing

**As a** builder,
**I want** 3D printed enclosure designed in OnShape to house Raspberry Pi 5, Torso PCB, display, and printer,
**so that** all components are protected and organized in the robot's torso.

**Acceptance Criteria:**
1. OnShape CAD model created for Torso enclosure with compartments for: Raspberry Pi 5 + Hailo Kit, Torso PCB, 2.8" display (front-facing), thermal printer (accessible output slot)
2. Design includes ventilation for Pi and Hailo Kit thermal management (heat sinks, airflow channels)
3. Design includes cable routing channels for internal wiring and external connections (power from Base, I2C bus, USB peripherals, HDMI to Head+Ears)
4. Design includes mounting points for robot frame attachment (likely top connection to Neck, bottom to Base)
5. Display is front-facing and visible; printer output slot allows paper to exit
6. Power input connector is accessible from Base module
7. STL files exported for 3D printing
8. Parts printed on FDM printer with appropriate settings (layer height, infill, supports)
9. Printed parts fit together with all components (Pi, PCB, display, printer) and provide adequate cooling
10. Assembly instructions drafted with photos (saved as `modules/torso/assembly.md`)

### Story 3.6: Develop Torso ESP32 Firmware

**As a** builder,
**I want** ESP32 firmware that implements I2C slave interface and hardware drivers for all Torso components,
**so that** the Pi can send commands to control heart display and thermal printer.

**Acceptance Criteria:**
1. PlatformIO project created in `modules/torso/firmware/`
2. I2C slave interface implemented at address 0x0A with register map defined
3. Heart display driver implemented: can display animated beating heart with variable rhythm (speed, pattern, intensity)
4. Thermal printer driver implemented: can print text, simple graphics sent via I2C commands
5. Basic I2C commands work: `SET_HEART_RHYTHM`, `PRINT_TEXT`, `PRINT_IMAGE`
6. Firmware flashed to ESP32 and responds to I2C commands from Pi

### Story 3.7: Create Torso ROS2 Driver Node

**As a** builder,
**I want** a ROS2 driver node that translates ROS2 topics into I2C commands for Torso module,
**so that** I can control the module using standard ROS2 pub/sub.

**Acceptance Criteria:**
1. Python ROS2 node created in `ros2/src/olaf_drivers/torso_driver/`
2. Node subscribes to topics: `/torso/heart` (rhythm command), `/torso/print` (print command with text/image data)
3. I2C communication implemented using `smbus2` library
4. ROS2 messages translated to I2C register writes to address 0x0A
5. Node launches successfully and appears in `ros2 node list`
6. Manual topic publish triggers expected hardware response: `ros2 topic pub /torso/heart ...` changes heart animation, `ros2 topic pub /torso/print ...` prints output

### Story 3.8: Install and Configure Raspberry Pi 5 in Torso Module

**As a** builder,
**I want** Raspberry Pi 5 + Hailo AI Kit installed in Torso enclosure with all peripherals connected,
**so that** the orchestration layer has its compute platform integrated into the robot.

**Acceptance Criteria:**
1. Raspberry Pi 5 mounted securely in Torso enclosure with proper standoffs/brackets
2. Hailo AI Kit installed on PCIe slot and secured
3. Power cable routed from Base module to Pi power input (5V via GPIO or USB-C)
4. OAK-D-Pro camera USB cable routed to Pi (cable runs to Head+Ears module)
5. Speakerphone USB cable connected to Pi
6. HDMI cable connected from Pi to Head+Ears module (for floor projector video)
7. I2C bus connected: Pi GPIO to Torso PCB I2C connector (Pi acts as I2C master)
8. SD card with ROS2 workspace installed in Pi
9. Pi boots successfully and all USB devices recognized (`lsusb`)
10. I2C bus scan from Pi shows Torso ESP32 at address 0x0A

### Story 3.9: Mount Torso Module to Robot Frame

**As a** builder,
**I want** Torso module physically mounted to robot frame with all cables connected,
**so that** the module is integrated into the robot structure as the compute hub.

**Acceptance Criteria:**
1. Module mounted securely to robot frame (connects Neck above and Base below)
2. Heart display is front-facing and visible
3. Thermal printer output slot is accessible
4. Power cable from Base module connected and delivering stable voltage
5. I2C bus cable connected to Base module I2C distribution (once Base is built)
6. Cable management ensures no loose wires interfering with movement or airflow
7. Module responds to ROS2 commands when powered: `ros2 topic pub` tests pass for heart display and printer
8. Pi remains cool under normal operation (thermal monitoring via software)

---

## Epic 4: Base Module Build

**Goal**: Complete the Base module (I2C 0x0B) with self-balancing two-wheel platform using hoverboard components (36V battery, BLDC motors), ODrive motor controller, MPU6050 IMU, 200Hz PID control loop, power distribution system (36V → 12V/5V/3.3V), ESP32 firmware, ROS2 driver node, and mounting to robot frame. This module provides mobility, balancing, and powers all other modules.

### Story 4.1: Source and Disassemble Hoverboard for Parts

**As a** builder,
**I want** a hoverboard sourced and disassembled to extract motors, wheels, and battery,
**so that** I have proven components for the self-balancing base.

**Acceptance Criteria:**
1. Hoverboard purchased (used/new) or equivalent parts sourced separately
2. 2× hoverboard BLDC motors extracted (typically 350W each, 6.5" wheels attached)
3. 36V battery pack extracted (typically 10S Li-ion, 4-5Ah) and tested for capacity/health
4. Motor specifications documented: voltage rating, KV rating, hall sensor wiring, phase wire colors
5. Battery specifications documented: voltage (36V nominal, 42V fully charged), capacity (Ah), discharge rating (C rating), connector type, BMS included
6. Wheels inspected for wear and replaced if necessary
7. All extracted parts tested: motors spin freely, battery charges and holds voltage
8. Parts inventory documented in `modules/base/README.md` with photos

### Story 4.2: Breadboard Base Components and Test Connectivity

**As a** builder,
**I want** all Base components breadboarded and individually tested,
**so that** I can verify functionality before committing to PCB design.

**Acceptance Criteria:**
1. ESP32 development board connected to breadboard with power supply
2. MPU6050 IMU wired (I2C) and providing gyro + accelerometer readings at 200Hz
3. ODrive S1 (or equivalent) wired to 36V battery via power switch and fuse
4. 1× hoverboard motor connected to ODrive (motor phase wires, hall sensors)
5. UART connection established between ESP32 and ODrive (115200 baud)
6. ODrive can command motor velocity and position via UART from ESP32
7. IMU data readable from ESP32 at 200Hz (verify timing with oscilloscope or logic analyzer)
8. Component-level test script confirms: IMU provides stable readings, motor spins smoothly under ODrive control, ESP32 can read/write ODrive parameters

### Story 4.3: Design Power Distribution System for 36V Battery

**As a** builder,
**I want** a power distribution system designed to convert 36V battery to 12V/5V/3.3V rails and distribute to all modules,
**so that** the robot has reliable power for 2-4 hour runtime.

**Acceptance Criteria:**
1. Power budget calculated for all modules: Pi 5 + Hailo (25W), 4× ESP32s (2W each), servos peak/continuous (calculate per servo type), displays/sensors (5W), ODrive + motors (100W+ peak)
2. Buck converter selected for 36V → 12V rail (current capacity for servos and any 12V loads)
3. Buck converter selected for 36V → 5V rail (current capacity 6A+ for Pi + USB peripherals)
4. Buck converter selected for 36V → 3.3V rail (current capacity 2A+ for 4× ESP32s)
5. Battery protection included: fuse/circuit breaker on 36V line, voltage monitoring, low-voltage cutoff
6. Power distribution topology designed: 36V battery → main switch → fuse → buck converters → distribution board → power connectors for 4 modules
7. Wiring gauge selected based on current and cable length (36V line needs heavy gauge)
8. Charge port location and type determined (compatible with hoverboard battery BMS)
9. Emergency stop switch included in design
10. Design documented in `modules/base/wiring.md` with schematic and power budget table

### Story 4.4: Design Base Custom PCB in Fritzing

**As a** builder,
**I want** a custom PCB designed in Fritzing integrating ESP32, power distribution, and all Base components,
**so that** I can replace the breadboard with a compact, reliable circuit board.

**Acceptance Criteria:**
1. Fritzing schematic created with ESP32 module, MPU6050 (I2C), ODrive UART connection, buck converter inputs/outputs, power distribution connectors
2. PCB layout designed with proper component placement, HEAVY power traces for 36V/12V/5V rails, and mounting holes
3. Power traces sized for maximum current draw (use trace width calculators, consider 2oz copper for high-current traces)
4. Connectors included for: ODrive (UART + power), 2× motors (phase wires, hall sensors), IMU, 4× module power outputs, battery input, charge port, main switch, I2C bus distribution
5. Fuse holder footprint included on 36V line
6. Pin assignments documented in wiring diagram (saved as `modules/base/wiring.md`)
7. Design reviewed for electrical correctness (no shorts, proper decoupling, thermal relief for buck converters, EMI considerations for motor phase wires)
8. Gerber files exported and ready for manufacturing

### Story 4.5: Order and Receive Base PCB from Elecrow

**As a** builder,
**I want** the Base PCB ordered from Elecrow and delivered,
**so that** I can proceed with assembly.

**Acceptance Criteria:**
1. Gerber files uploaded to Elecrow website
2. PCB specifications confirmed (layer count, thickness, color, quantity, **2oz copper for power traces**)
3. Order placed and payment completed
4. PCB received and visually inspected for manufacturing defects
5. Continuity test confirms no shorts between power rails
6. PCB dimensions and mounting holes match design specifications

### Story 4.6: Assemble and Test Base PCB

**As a** builder,
**I want** all components soldered to Base PCB and tested for functionality,
**so that** I have a working circuit board ready for firmware development.

**Acceptance Criteria:**
1. All components soldered: ESP32 module, buck converters, connectors for ODrive/motors/IMU/battery/modules, fuse holder, passive components
2. Visual inspection confirms proper solder joints (no cold joints, bridges)
3. **CRITICAL SAFETY TEST**: Power-on WITHOUT battery first - use bench power supply set to 36V with current limit (1A) to verify no shorts
4. Power-on test WITH battery: confirm stable 12V, 5V, 3.3V rails with no excessive current draw
5. I2C address configured as 0x0B in firmware
6. Component-level tests pass: IMU provides readings, ODrive responds to UART commands, motors spin under control, all power rails stable under load
7. PCB mounted on temporary test fixture for firmware development

### Story 4.7: Design and 3D Print Base Platform and Motor Mounts

**As a** builder,
**I want** 3D printed platform, motor mounts, and battery enclosure designed in OnShape,
**so that** I can mount motors, battery, PCB, and ODrive to create the mobile base.

**Acceptance Criteria:**
1. OnShape CAD model created for Base platform with space for: Base PCB, ODrive controller, 36V battery pack, motor mounts (left/right), caster wheel or kickstand support area
2. Motor mounts designed to securely hold hoverboard motors with proper alignment (parallel axles, correct wheel spacing)
3. Battery enclosure or mounting bracket designed with secure retention and access for charging
4. Platform includes mounting points for Torso module connection (top surface)
5. Design ensures proper weight distribution (battery low, centered)
6. Cable routing channels included for motor phase wires, power distribution, I2C bus
7. STL files exported for 3D printing
8. Parts printed on FDM printer with appropriate settings (consider high infill for structural parts)
9. Printed parts fit together with PCB, ODrive, battery, motors and provide stable base
10. Assembly instructions drafted with photos (saved as `modules/base/assembly.md`)

### Story 4.8: Configure ODrive for Hoverboard Motors

**As a** builder,
**I want** ODrive configured and calibrated for hoverboard BLDC motors,
**so that** I have reliable closed-loop motor control for balancing.

**Acceptance Criteria:**
1. ODrive firmware updated to latest stable version
2. Motor parameters configured: pole pairs, resistance, inductance (use ODrive auto-calibration)
3. Encoder configuration: hall sensor mode enabled, CPR (counts per revolution) set correctly
4. Current limits configured: continuous and peak current limits set based on motor specs and battery capacity
5. Velocity limits configured: maximum RPM set to safe value
6. Control mode tested: velocity control and position control both functional
7. Motor calibration completed: ODrive successfully calibrates both motors and enters CLOSED_LOOP_CONTROL state
8. UART communication protocol documented: commands for velocity control, position read, error checking
9. Configuration saved to ODrive non-volatile memory
10. Configuration parameters documented in `modules/base/diagnostics/odrive_config.md`

### Story 4.9: Develop Base ESP32 Firmware with 200Hz Balancing PID

**As a** builder,
**I want** ESP32 firmware that implements I2C slave interface, 200Hz PID balancing loop, and ODrive motor control,
**so that** the robot can self-balance and respond to navigation commands from Pi.

**Acceptance Criteria:**
1. PlatformIO project created in `modules/base/firmware/`
2. I2C slave interface implemented at address 0x0B with register map defined
3. MPU6050 driver implemented: reads gyro + accelerometer at 200Hz using hardware timer interrupt
4. Complementary filter or Kalman filter implemented for angle estimation from IMU data
5. PID controller implemented: takes tilt angle error, outputs motor velocity commands
6. ODrive UART driver implemented: sends velocity commands to ODrive at 200Hz
7. Basic PID tuning completed: robot can balance for at least 10 seconds without falling
8. I2C command interface: `SET_TARGET_VELOCITY` (for navigation), `ENABLE_BALANCING`, `DISABLE_BALANCING`, `GET_IMU_DATA`, `GET_ODOMETRY`
9. Safety features: automatic shutdown if tilt angle exceeds threshold, watchdog timer for ODrive communication
10. Firmware flashed to ESP32 and robot balances when powered on

### Story 4.10: Fine-Tune Self-Balancing PID Parameters

**As a** builder,
**I want** PID parameters tuned for stable, responsive balancing,
**so that** the robot can maintain balance under various conditions.

**Acceptance Criteria:**
1. PID tuning methodology documented (Ziegler-Nichols, manual tuning, or other)
2. Proportional gain (Kp) tuned: robot responds to tilt without excessive oscillation
3. Integral gain (Ki) tuned: eliminates steady-state error (robot doesn't drift slowly)
4. Derivative gain (Kd) tuned: dampens oscillations, provides stability
5. Robot can balance on flat surface for 60+ seconds consistently
6. Robot recovers from small external disturbances (gentle push)
7. Battery voltage compensation (optional): PID accounts for voltage drop as battery depletes
8. Final PID parameters documented in firmware and `modules/base/README.md`
9. Balancing behavior video recorded for documentation

### Story 4.11: Create Base ROS2 Driver Node

**As a** builder,
**I want** a ROS2 driver node that translates ROS2 topics into I2C commands for Base module,
**so that** I can control balancing and navigation using standard ROS2 pub/sub.

**Acceptance Criteria:**
1. Python ROS2 node created in `ros2/src/olaf_drivers/base_driver/`
2. Node subscribes to topics: `/base/cmd_vel` (velocity command for navigation, standard `geometry_msgs/Twist`)
3. Node publishes sensor data: `/base/imu` (IMU data), `/base/odom` (wheel odometry for SLAM)
4. I2C communication implemented using `smbus2` library
5. ROS2 messages translated to I2C register writes to address 0x0B
6. Node launches successfully and appears in `ros2 node list`
7. Manual topic publish triggers expected hardware response: `ros2 topic pub /base/cmd_vel ...` causes robot to move forward/backward/turn while balancing

### Story 4.12: Mount Base Module and Connect All Modules

**As a** builder,
**I want** Base module completed with all motors, battery, and PCB mounted, and power/I2C cables connected to all other modules,
**so that** the robot has a functional mobile platform powering the entire system.

**Acceptance Criteria:**
1. Both hoverboard motors mounted securely to Base platform with wheels attached
2. 36V battery pack mounted securely with proper retention and access to charge port
3. Base PCB and ODrive controller mounted to platform
4. All motor phase wires and hall sensor cables connected to ODrive
5. Power cables routed from Base power distribution to all 4 modules (Head+Ears, Neck, Torso, Base ESP32)
6. I2C bus cables connected from Pi (in Torso) to all 4 modules including Base ESP32
7. Main power switch and emergency stop accessible
8. Cable management ensures no wires near wheels or moving parts
9. Base module powers up and enters balancing mode automatically
10. All modules powered and responding: `i2cdetect -y 1` shows addresses 0x08, 0x09, 0x0A, 0x0B
11. Robot can balance autonomously when placed upright
12. Battery runtime test: measure actual runtime under typical load vs. target 2-4 hours

---

## Epic 5: End-to-End Demo Script

**Goal**: Create a simple ROS2 Python script demonstrating coordinated multi-module operation to validate Phase 1 completion. The demo will sequentially activate all modules via ROS2 topic publishes: deploy kickstand, perform expressive movements (eyes, ears, head), animate heart, retract kickstand, and demonstrate balancing capability.

### Story 5.1: Create End-to-End Demo Script

**As a** builder,
**I want** a ROS2 Python script that sends commands to all modules in sequence,
**so that** I can validate that all hardware modules respond correctly to software commands.

**Acceptance Criteria:**
1. Python script created in `ros2/src/olaf_bringup/scripts/phase1_demo.py`
2. Script uses `rclpy` to create ROS2 node and publish to module topics
3. Demo sequence implemented:
   - **Step 1**: Deploy kickstand via `/neck/kickstand` topic (stationary mode)
   - **Step 2**: Blink eyes via `/head_ears/eyes` topic (2-3 blink cycles)
   - **Step 3**: Move ears via `/head_ears/ears` topic (perked up, then relaxed positions)
   - **Step 4**: Tilt head via `/neck/position` topic (pan left/right, tilt up/down, roll left/right)
   - **Step 5**: Animate heart via `/torso/heart` topic (slow rhythm → fast rhythm → slow rhythm)
   - **Step 6**: Print demo message via `/torso/print` topic ("Phase 1 Complete!")
   - **Step 7**: Retract kickstand via `/neck/kickstand` topic (prepare for balancing mode)
   - **Step 8**: Enable balancing via `/base/cmd_vel` topic (robot balances in place)
   - **Step 9**: Small movement test via `/base/cmd_vel` (forward 0.5m, backward 0.5m, stop)
4. Each step includes appropriate delays for movement completion (e.g., 2s for eyes, 3s for neck movement)
5. Script includes logging output describing each step as it executes
6. Script handles graceful shutdown on Ctrl+C
7. Script can be launched via `ros2 run olaf_bringup phase1_demo`

### Story 5.2: Create Demo Launch File

**As a** builder,
**I want** a ROS2 launch file that starts all driver nodes and the demo script,
**so that** I can run the complete Phase 1 demo with a single command.

**Acceptance Criteria:**
1. Launch file created in `ros2/src/olaf_bringup/launch/phase1_demo.launch.py`
2. Launch file starts all 4 driver nodes: `head_ears_driver`, `neck_driver`, `torso_driver`, `base_driver`
3. Launch file waits for all drivers to initialize (2-3 second delay)
4. Launch file starts the demo script `phase1_demo.py`
5. All nodes output to console with namespace prefixes for easy identification
6. Launch file can be executed with: `ros2 launch olaf_bringup phase1_demo.launch.py`
7. All nodes shut down cleanly when launch is terminated (Ctrl+C)

### Story 5.3: Test and Record End-to-End Demo

**As a** builder,
**I want** the end-to-end demo tested and recorded on video,
**so that** I can document Phase 1 completion and share progress publicly.

**Acceptance Criteria:**
1. All 4 modules powered on and connected (Head+Ears, Neck, Torso, Base)
2. ROS2 environment sourced and all driver nodes verified as functional
3. Demo launch file executed: `ros2 launch olaf_bringup phase1_demo.launch.py`
4. All demo steps execute successfully:
   - Kickstand deploys
   - Eyes blink visibly
   - Ears move through commanded positions
   - Head pans, tilts, and rolls
   - Heart display shows rhythm changes
   - Thermal printer outputs "Phase 1 Complete!" message
   - Kickstand retracts
   - Robot enters balancing mode and maintains balance
   - Robot moves forward/backward as commanded
5. Any failures documented with troubleshooting steps in `docs/guides/troubleshooting.md`
6. Video recorded showing full demo sequence from start to finish
7. Video uploaded for build-in-public documentation (YouTube, LinkedIn, README)
8. Phase 1 milestone tagged in Git: `git tag -a v1.0-phase1 -m "Phase 1: Hardware Build & ROS2 Foundation Complete"`

### Story 5.4: Document Phase 1 Completion and Lessons Learned

**As a** builder,
**I want** Phase 1 completion documented with build summary, lessons learned, and next steps,
**so that** I can share knowledge with the community and plan Phase 2.

**Acceptance Criteria:**
1. Phase 1 summary document created in `docs/guides/phase1-summary.md` including:
   - Total build time (weeks/hours)
   - Total cost breakdown (modules, PCBs, 3D printing, components)
   - Challenges encountered and solutions
   - Lessons learned (what worked, what didn't, what would be done differently)
   - Photos of completed modules and full robot
2. Wiring diagrams finalized for all 4 modules with actual pin assignments
3. BOM (Bill of Materials) complete and accurate for all modules with supplier links
4. Build instructions reviewed and updated based on actual build experience
5. Known issues documented in `docs/guides/known-issues.md` with workarounds
6. Phase 2 planning initiated: identify middleware features needed (personality coordination, SLAM, AI integration)
7. README.md updated with Phase 1 completion status and demo video link
8. Build-in-public post shared on social media with demo video and lessons learned

---

## Next Steps

### Phase 2 Preview: Middleware & Intelligence

Once Phase 1 is complete with all hardware modules operational and responding to ROS2 commands, Phase 2 will focus on:

**Orchestration & Coordination:**
- Personality coordinator: synchronized expressions across eyes/ears/neck/heart/beeps
- State machine: manage robot states (idle, listening, thinking, expressing, navigating)
- Emotion engine: generate coordinated multi-channel emotional expressions

**Navigation & Mobility:**
- Google Cartographer SLAM integration with OAK-D-Pro
- Nav2 stack configuration for autonomous navigation
- Follow-me mode: vision-based person tracking
- Obstacle avoidance with depth sensing

**AI Integration:**
- Hailo Whisper STT: <200ms local speech recognition
- Cloud AI agents (Claude/GPT-4): reasoning, tool use, personality generation
- Context management: SQLite conversation history, user preferences
- Function routing: AI decides which modules to activate

**Advanced Features:**
- Floor projector content generation and display
- Thermal printer output formatting
- Presence sensor fusion for 360° human detection
- Advanced balancing: disturbance rejection, adaptive gains

Phase 2 PRD will be developed after Phase 1 completion and lessons learned integration.

---

**End of Phase 1 PRD**
