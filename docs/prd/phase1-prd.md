# OLAF Phase 1: Hardware Build & ROS2 Foundation - Product Requirements Document (PRD)

## Goals and Background Context

### Goals

Phase 1 establishes the physical and software foundation for OLAF, delivering:

- **Hardware Foundation**: All modules physically assembled and mounted — 2 ESP32 real-time controllers (Head, Base) plus Pi-controlled peripherals (Neck servos, Ear servos, Indicator LEDs, Heart display, Kickstand)
- **ROS2 Control Layer**: Basic pub/sub ROS2 Jazzy driver nodes enabling Pi to command each module independently
- **Module Independence**: Each module testable in isolation with component-level validation scripts
- **Self-Balancing Capability**: Base module operational with 200Hz PID control running on ESP32
- **End-to-End Validation**: Simple demo scripts proving coordinated movement across all modules via ROS2
- **Build-Ready Documentation**: Wiring diagrams, assembly instructions, and troubleshooting guides for replication

### Background Context

OLAF uses a **hybrid-control architecture** where the Raspberry Pi 5 directly controls most peripherals via the Sunfounder Fusion HAT+ and USB adapters, while only two ESP32 modules handle tasks requiring dedicated real-time control (eye animations at 60 FPS, self-balancing at 200Hz).

**Key Architecture Decisions:**

1. **Reduced ESP32 count (4→2)** — Eliminates soldering complexity by leveraging Fusion HAT's pre-built interfaces and USB servo adapters
2. **Sunfounder Fusion HAT+** — Provides WS2812 LED control, PWM for kickstand servos, I2C bridge, and I2S audio without custom PCBs
3. **USB Serial for servos** — Waveshare Bus Servo Adapters control neck (3-DOF) and ears (4 servos) via plug-and-play USB
4. **4" Pi display as heart** — Replaces dedicated Torso ESP32; Pi renders heart animations directly
5. **Dual battery architecture** — 36V hoverboard battery (motors, servos, ESP32s) isolated from 7.4V logic battery (Pi, HAT, display, LEDs)
6. **Kickstand at base** — Model plane landing gear servos (4.8-7.4V) controlled via long PWM wires from Fusion HAT

This architecture achieves <3s AI response latency, supports 2-4 hour battery runtime, and minimizes soldering through use of pre-built adapter boards.

### Change Log

| Date | Version | Description | Author |
|------|---------|-------------|--------|
| 2025-12-16 | v1.0 | Initial Phase 1 PRD | Kamal Singh |
| 2026-01-31 | v2.0 | Complete rewrite for Fusion HAT architecture (4 ESP32 → 2 ESP32) | Kamal Singh |

---

## Requirements

### Functional Requirements

**FR1**: Head module (ESP32, I2C 0x10) must drive 2× GC9A01 round OLED displays (240×240, SPI) at 30-60 FPS for expressive eye animations.

**FR2**: Base module (ESP32, I2C 0x11) must implement 200Hz PID self-balancing loop using MPU6050 IMU and ODrive motor control via UART.

**FR3**: Neck module must be controllable via USB serial (Waveshare Bus Servo Adapter A) with 3× Feetech STS3215 servos providing pan/tilt/roll articulation.

**FR4**: Ears module must be controllable via USB serial (second Waveshare Bus Servo Adapter A) with 4× Feetech SCS0009 servos providing 2-DOF per ear.

**FR5**: Indicator module must control 24× WS2812 LEDs (3×8 daisy-chained strips) via Fusion HAT WS2812 port for interaction feedback, status display, and PID visualization.

**FR6**: Heart display (4" Raspberry Pi Display) must render animated heart graphics directly from Pi via SPI/DSI.

**FR7**: Kickstand mechanism must use 2× model plane landing gear servos (4.8-7.4V PWM) controlled via Fusion HAT PWM channels, with long signal wires routed to base platform.

**FR8**: ROS2 Jazzy must be installed on Ubuntu 24.04 LTS with 5 driver nodes: `olaf_head`, `olaf_base`, `olaf_neck`, `olaf_ears`, `olaf_indicator`.

**FR9**: I2C communication must be validated between Pi (master) and both ESP32 modules (Head 0x10, Base 0x11) at 400kHz-1MHz bus speed with 5-20ms latency.

**FR10**: Power distribution must provide: 36V direct to ODrive/motors, 36V→12V buck for servos (via Waveshare adapters), 36V→5V buck for ESP32s and OLEDs.

**FR11**: Logic power must be isolated on 2000mAh 7.4V battery (managed by Fusion HAT) powering Pi 5, Fusion HAT+, AI HAT, 4" display, WS2812 LEDs, and kickstand servos.

**FR12**: Component-level testing scripts must exist for each module validating electrical connectivity, sensor readings, actuator movement, and display output.

**FR13**: Simple end-to-end demo script must demonstrate coordinated movement: deploy kickstand, blink eyes, move ears, tilt head, animate heart, flash indicators, retract kickstand, and balance.

### Non-Functional Requirements

**NFR1**: Each module must be independently testable without requiring other modules to be connected or operational.

**NFR2**: Comprehensive documentation must be created including per-module wiring diagrams (pin assignments, circuits), assembly instructions (step-by-step with photos), and troubleshooting guides.

**NFR3**: Phase 1 must NOT include AI integration, personality coordination, or SLAM navigation—those are explicitly scoped for Phase 2.

**NFR4**: All code must follow the module-first repository structure with firmware in `modules/{module}/firmware/`, ROS2 drivers in `ros2/src/olaf_{module}/`.

**NFR5**: All code, PCB designs, and 3D models must be committed to Git with descriptive commit messages and tagged at Phase 1 completion milestone (`v1.0-phase1`).

**NFR6**: Build must minimize soldering — use pre-built adapter boards (Fusion HAT, Waveshare adapters) wherever possible.

---

## Technical Assumptions

### Hardware Architecture

**Orchestration Layer (Raspberry Pi 5 16GB):**
- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy Jalisco (LTS until 2029)
- **HATs**: Sunfounder Fusion HAT+ (bottom), Hailo-8L AI Kit 26 TOPS (top)
- **Display**: 4" Raspberry Pi Display (heart animation)

**Module Layer (2× ESP32 only):**
- **Head ESP32** (I2C 0x10): Controls 2× GC9A01 OLED eyes via SPI
- **Base ESP32** (I2C 0x11): 200Hz balancing PID, MPU6050 IMU, ODrive UART

**Direct Pi Control (via HAT/USB):**
- **Neck**: 3× STS3215 servos via Waveshare USB adapter
- **Ears**: 4× SCS0009 servos via Waveshare USB adapter
- **Indicators**: 24× WS2812 LEDs via Fusion HAT
- **Kickstand**: 2× landing gear servos via Fusion HAT PWM
- **Audio**: I2S speaker via Fusion HAT

### Communication Protocols

| Interface | Connection | Speed | Latency |
|-----------|-----------|-------|---------|
| I2C | Pi ↔ Head ESP32 (0x10) | 400kHz-1MHz | 5-20ms |
| I2C | Pi ↔ Base ESP32 (0x11) | 400kHz-1MHz | 5-20ms |
| USB Serial | Pi ↔ Waveshare Neck | UART | <10ms |
| USB Serial | Pi ↔ Waveshare Ears | UART | <10ms |
| SPI | Head ESP32 → 2× GC9A01 | 10-20MHz | <1ms |
| UART | Base ESP32 → ODrive | 115200 | <5ms |
| WS2812 | Fusion HAT → LEDs | 800kHz | <1ms |
| PWM | Fusion HAT → Kickstand | 50Hz | <20ms |

### Power Architecture

**36V System (Hoverboard Battery, Charging Port A):**
- ODrive + BLDC motors: 36V direct
- Neck/Ear servos: 36V→12V buck → Waveshare adapters
- ESP32s + OLEDs: 36V→5V buck

**7.4V Logic System (2000mAh Battery, Charging Port B, Fusion HAT managed):**
- Raspberry Pi 5: 5V regulated
- Fusion HAT+: 5V
- AI HAT (Hailo): 5V
- 4" Heart Display: 5V
- WS2812 LEDs (24): 5V
- Kickstand servos (2): 4.8-7.4V

### Software Stack

**Orchestration Layer:**
- **OS**: Ubuntu 24.04 LTS (64-bit)
- **ROS2**: Jazzy Jalisco
- **Language**: Python 3.11+
- **Libraries**: `rclpy`, `smbus2` (I2C), `pyserial` (USB), `rpi_ws281x` or Fusion HAT SDK

**Module Firmware:**
- **Framework**: Arduino/ESP-IDF (PlatformIO)
- **Language**: C++17
- **Libraries**: Wire (I2C slave), TFT_eSPI or GC9A01 driver, MPU6050, ODriveArduino

### ROS2 Node Topology

```
/olaf/head      → I2C 0x10 → /head/expression, /head/blink, /head/look
/olaf/base      → I2C 0x11 → /cmd_vel, /odom, /imu
/olaf/neck      → USB Serial → /neck/pose, /neck/lookat
/olaf/ears      → USB Serial → /ears/emote, /ears/perk
/olaf/indicator → HAT WS2812 → /indicator/interact, /indicator/status, /indicator/pid
```

---

## Epic List

**Epic 0: ROS2 Foundation Setup**
Establish ROS2 Jazzy workspace on Raspberry Pi 5 with Ubuntu 24.04, Fusion HAT configuration, I2C tools, and USB serial setup for Waveshare adapters.

**Epic 1: Head Module Build**
Complete Head module (ESP32, I2C 0x10) with 2× GC9A01 round OLED eyes, ESP32 firmware for 60 FPS animations, and ROS2 driver node.

**Epic 2: Base Module Build**
Complete Base module (ESP32, I2C 0x11) with hoverboard platform (36V battery, BLDC motors), ODrive, MPU6050 IMU, 200Hz PID balancing, power distribution, and ROS2 driver node.

**Epic 3: Neck Module Build**
Complete Neck module with 3× STS3215 servos (pan/tilt/roll) via Waveshare USB adapter, 3D printed mounts, and ROS2 driver node.

**Epic 4: Ears Module Build**
Complete Ears module with 4× SCS0009 servos (2-DOF per ear) via Waveshare USB adapter, 3D printed mounts, and ROS2 driver node.

**Epic 5: Indicators, Heart & Kickstand Build**
Complete Indicator module (24× WS2812 via HAT), Heart display (4" Pi display), and Kickstand mechanism (2× landing gear servos via HAT PWM).

**Epic 6: End-to-End Demo Script**
Create and execute demo script validating all modules via ROS2 with coordinated multi-module operation.

---

## Epic 0: ROS2 Foundation Setup

**Goal**: Establish the ROS2 Jazzy development environment on Raspberry Pi 5 with Ubuntu 24.04, Fusion HAT configuration, and communication tools configured for both I2C (ESP32s) and USB serial (Waveshare adapters).

### Story 0.1: Install Ubuntu 24.04 and ROS2 Jazzy on Raspberry Pi

**As a** builder,
**I want** Ubuntu 24.04 LTS and ROS2 Jazzy installed on Raspberry Pi 5,
**so that** I can develop ROS2 driver nodes for OLAF modules.

**Acceptance Criteria:**
1. Ubuntu 24.04 LTS (64-bit) is installed on Pi 5 and boots successfully
2. ROS2 Jazzy Jalisco is installed following official installation guide
3. Core ROS2 packages are verified (`ros-jazzy-desktop` or `ros-jazzy-base`)
4. `rosdep` is initialized and dependencies are installed
5. ROS2 environment is sourced in `.bashrc` for automatic activation
6. Basic ROS2 commands work: `ros2 topic list`, `ros2 node list`

### Story 0.2: Create ROS2 Workspace with Module-First Structure

**As a** builder,
**I want** a ROS2 workspace structured according to the module-first architecture,
**so that** each module's ROS2 driver node is organized and discoverable.

**Acceptance Criteria:**
1. Workspace directory created at `~/olaf_ws/` with `src/` subdirectory
2. Following packages created: `olaf_bringup`, `olaf_interfaces`, `olaf_head`, `olaf_base`, `olaf_neck`, `olaf_ears`, `olaf_indicator`
3. Each package has proper `package.xml` and `setup.py` (Python)
4. Workspace builds successfully: `colcon build` completes without errors
5. Workspace can be sourced: `source install/setup.bash` works

### Story 0.3: Configure Fusion HAT and I2C Communication

**As a** builder,
**I want** Fusion HAT+ configured and I2C communication testable,
**so that** I can validate ESP32 module connectivity.

**Acceptance Criteria:**
1. Fusion HAT+ physically installed on Pi 5 GPIO header
2. I2C interface enabled via `raspi-config` or `/boot/firmware/config.txt`
3. Python `smbus2` library installed
4. I2C tools installed: `sudo apt install i2c-tools`
5. I2C bus scan works: `i2cdetect -y 1` displays bus addresses
6. Fusion HAT SDK/libraries installed for WS2812 and PWM control
7. Module I2C addresses documented (0x10=Head, 0x11=Base)

### Story 0.4: Configure USB Serial for Waveshare Adapters

**As a** builder,
**I want** USB serial communication configured for Waveshare Bus Servo Adapters,
**so that** I can control neck and ear servos from Pi.

**Acceptance Criteria:**
1. Both Waveshare Bus Servo Adapter (A) units connected via USB
2. USB devices appear as `/dev/ttyUSB*` or `/dev/ttyACM*`
3. Python `pyserial` library installed
4. udev rules created for persistent device naming (e.g., `/dev/waveshare_neck`, `/dev/waveshare_ears`)
5. Test script can communicate with Feetech servos (ping, read position)
6. USB permissions configured for non-root access

### Story 0.5: Set Up Development Tools and Dependencies

**As a** builder,
**I want** all necessary development tools and Python dependencies installed,
**so that** I can efficiently develop and test ROS2 nodes and ESP32 firmware.

**Acceptance Criteria:**
1. Python 3.11+ is available (ships with Ubuntu 24.04)
2. PlatformIO installed for ESP32 firmware development
3. Git configured with user name and email
4. Required Python packages installed: `smbus2`, `pyserial`, `numpy`
5. VS Code (or preferred IDE) installed with ROS2 and PlatformIO extensions
6. USB permissions configured for ESP32 flashing

---

## Epic 1: Head Module Build

**Goal**: Complete the Head module (ESP32, I2C 0x10) with 2× GC9A01 round OLED displays (240×240, 1.28") for expressive eye animations at 30-60 FPS.

### Story 1.1: Breadboard Head Components and Test Connectivity

**As a** builder,
**I want** Head module components breadboarded and individually tested,
**so that** I can verify functionality before final assembly.

**Acceptance Criteria:**
1. ESP32-S3-DevKitC-1 connected to breadboard with USB power
2. 2× GC9A01 round displays (240×240, SPI) wired and displaying test images
3. Both displays driven simultaneously via SPI (separate CS pins)
4. Display refresh rate measured at 30+ FPS
5. Component-level test script confirms: both eyes display animations smoothly

### Story 1.2: Design Head Module Wiring and Assembly

**As a** builder,
**I want** Head module wiring documented and assembly planned,
**so that** I can create a clean, maintainable build.

**Acceptance Criteria:**
1. Wiring diagram created showing ESP32 → 2× GC9A01 connections (SPI, CS, DC, RST, power)
2. I2C slave connection documented (SDA, SCL to Pi via cable)
3. Power input documented (5V from 36V→5V buck converter)
4. Pin assignments saved in `modules/head/wiring.md`
5. Mounting strategy defined for eyes in head enclosure
6. Cable routing planned for I2C and power from torso

### Story 1.3: Develop Head ESP32 Firmware

**As a** builder,
**I want** ESP32 firmware that implements I2C slave interface and drives both eye displays,
**so that** the Pi can send expression commands via I2C.

**Acceptance Criteria:**
1. PlatformIO project created in `modules/head/firmware/`
2. I2C slave interface implemented at address 0x10 with register map defined
3. GC9A01 driver implemented using TFT_eSPI or equivalent library
4. Eye animation system: supports multiple expressions (neutral, happy, sad, surprised, angry, sleepy, wink)
5. Smooth transitions between expressions (interpolated animation)
6. Basic I2C commands work: `SET_EXPRESSION`, `SET_BLINK`, `SET_LOOK_DIRECTION`
7. Frame rate maintains 30+ FPS during animations
8. Firmware flashed to ESP32 and responds to I2C commands from Pi

### Story 1.4: Create Head ROS2 Driver Node

**As a** builder,
**I want** a ROS2 driver node that translates ROS2 topics into I2C commands for Head module,
**so that** I can control the eyes using standard ROS2 pub/sub.

**Acceptance Criteria:**
1. Python ROS2 node created in `ros2/src/olaf_head/`
2. Node subscribes to topics: `/head/expression`, `/head/blink`, `/head/look`
3. I2C communication implemented using `smbus2` library
4. ROS2 messages translated to I2C register writes to address 0x10
5. Node launches successfully and appears in `ros2 node list`
6. Manual topic publish triggers expected hardware response

### Story 1.5: Design and 3D Print Head Enclosure

**As a** builder,
**I want** 3D printed head enclosure with eye display mounts,
**so that** the displays are properly positioned as robot "eyes".

**Acceptance Criteria:**
1. OnShape CAD model created for head enclosure with circular cutouts for round displays
2. Display mounts position eyes at appropriate spacing and angle
3. Space allocated for ESP32, wiring, and cable routing
4. STL files exported for 3D printing
5. Parts printed and displays fit correctly
6. Assembly instructions documented in `modules/head/assembly.md`

---

## Epic 2: Base Module Build

**Goal**: Complete the Base module (ESP32, I2C 0x11) with self-balancing two-wheel platform using hoverboard components (36V battery, BLDC motors), ODrive motor controller, MPU6050 IMU, 200Hz PID control loop, and power distribution system.

### Story 2.1: Source and Disassemble Hoverboard for Parts

**As a** builder,
**I want** a hoverboard sourced and disassembled to extract motors, wheels, and battery,
**so that** I have proven components for the self-balancing base.

**Acceptance Criteria:**
1. Hoverboard purchased or equivalent parts sourced separately
2. 2× hoverboard BLDC motors extracted (typically 350W each, 6.5" wheels)
3. 36V battery pack extracted and tested for capacity/health
4. Motor and battery specifications documented
5. Parts inventory documented in `modules/base/README.md`

### Story 2.2: Breadboard Base Components and Test Connectivity

**As a** builder,
**I want** Base module components breadboarded and individually tested,
**so that** I can verify functionality before final assembly.

**Acceptance Criteria:**
1. ESP32-S3-DevKitC-1 connected to breadboard with power
2. MPU6050 IMU wired (I2C) and providing readings at 200Hz
3. ODrive connected to 36V battery via power switch and fuse
4. Motor connected to ODrive and responding to commands
5. UART connection established between ESP32 and ODrive
6. Component-level test confirms: IMU reads stable, motor spins under control

### Story 2.3: Design Power Distribution System

**As a** builder,
**I want** power distribution system designed for 36V battery,
**so that** all modules receive appropriate voltage rails.

**Acceptance Criteria:**
1. Power budget calculated for all loads
2. Buck converters selected: 36V→12V (servos), 36V→5V (ESP32s, OLEDs)
3. Battery protection: fuse, low-voltage cutoff, emergency stop
4. Distribution topology documented in `modules/base/wiring.md`
5. Charge port location and wiring specified

### Story 2.4: Assemble Base Platform

**As a** builder,
**I want** base platform assembled with motors, battery, and electronics,
**so that** I have a functional mobile base.

**Acceptance Criteria:**
1. Skateboard suspension/trucks mounted with hoverboard wheels
2. Battery mounted securely with access to charge port
3. ODrive and ESP32 mounted to platform
4. Power distribution wired and tested
5. All connections secure with proper cable management
6. Assembly documented in `modules/base/assembly.md`

### Story 2.5: Configure ODrive for Hoverboard Motors

**As a** builder,
**I want** ODrive configured and calibrated for hoverboard BLDC motors,
**so that** I have reliable closed-loop motor control.

**Acceptance Criteria:**
1. ODrive firmware updated to latest stable version
2. Motor parameters configured via auto-calibration
3. Hall sensor mode enabled and configured
4. Current and velocity limits set appropriately
5. Both motors calibrate successfully
6. Configuration documented in `modules/base/odrive-config.md`

### Story 2.6: Develop Base ESP32 Firmware with 200Hz Balancing PID

**As a** builder,
**I want** ESP32 firmware implementing 200Hz PID balancing and ODrive control,
**so that** the robot can self-balance.

**Acceptance Criteria:**
1. PlatformIO project created in `modules/base/firmware/`
2. I2C slave interface implemented at address 0x11
3. MPU6050 driver reads at 200Hz using hardware timer
4. Complementary filter for angle estimation
5. PID controller outputs motor velocity commands
6. ODrive UART driver sends commands at 200Hz
7. Safety: auto-shutdown if tilt exceeds threshold
8. Basic balancing achieved (10+ seconds stable)

### Story 2.7: Fine-Tune Self-Balancing PID Parameters

**As a** builder,
**I want** PID parameters tuned for stable balancing,
**so that** the robot maintains balance reliably.

**Acceptance Criteria:**
1. Kp, Ki, Kd tuned systematically
2. Robot balances on flat surface for 60+ seconds
3. Robot recovers from small disturbances
4. Final parameters documented in firmware and README
5. Balancing video recorded

### Story 2.8: Create Base ROS2 Driver Node

**As a** builder,
**I want** a ROS2 driver node for Base module,
**so that** I can control balancing and navigation via ROS2.

**Acceptance Criteria:**
1. Python ROS2 node created in `ros2/src/olaf_base/`
2. Node subscribes to `/cmd_vel` (geometry_msgs/Twist)
3. Node publishes `/odom` and `/imu`
4. I2C communication to address 0x11
5. Node launches and responds to velocity commands

---

## Epic 3: Neck Module Build

**Goal**: Complete Neck module with 3× Feetech STS3215 servos (pan/tilt/roll) controlled via Waveshare Bus Servo Adapter over USB serial.

### Story 3.1: Test Waveshare Adapter with STS3215 Servos

**As a** builder,
**I want** Waveshare adapter controlling STS3215 servos,
**so that** I can verify USB serial communication.

**Acceptance Criteria:**
1. Waveshare Bus Servo Adapter (A) connected via USB
2. 12V power connected to adapter (from 36V→12V buck)
3. 3× STS3215 servos daisy-chained on bus
4. Each servo assigned unique ID (1, 2, 3)
5. Python test script moves all servos to commanded positions
6. Servo feedback (position, load) readable

### Story 3.2: Design Neck Mechanical Assembly

**As a** builder,
**I want** neck mechanical design with 3-DOF articulation,
**so that** the head can pan, tilt, and roll expressively.

**Acceptance Criteria:**
1. OnShape CAD model for neck servo mounts and linkages
2. Pan servo at base, tilt in middle, roll at top
3. Full range of motion: pan ±90°, tilt ±45°, roll ±30°
4. Head mounting point at top
5. Cable routing for head I2C and power
6. STL files exported and parts printed

### Story 3.3: Assemble and Test Neck Module

**As a** builder,
**I want** neck module assembled and tested,
**so that** head articulation works smoothly.

**Acceptance Criteria:**
1. Servos mounted in 3D printed brackets
2. Linkages assembled with smooth motion
3. No mechanical interference through full range
4. Head mount supports weight of head module
5. Cables routed cleanly
6. Assembly documented in `modules/neck/assembly.md`

### Story 3.4: Create Neck ROS2 Driver Node

**As a** builder,
**I want** ROS2 driver node for Neck module via USB serial,
**so that** I can control head articulation via ROS2.

**Acceptance Criteria:**
1. Python ROS2 node created in `ros2/src/olaf_neck/`
2. Node subscribes to `/neck/pose` (pan, tilt, roll angles)
3. USB serial communication via pyserial to Waveshare adapter
4. Feetech servo protocol implemented
5. Smooth motion interpolation (optional)
6. Node launches and moves servos on topic publish

---

## Epic 4: Ears Module Build

**Goal**: Complete Ears module with 4× Feetech SCS0009 servos (2-DOF per ear) controlled via second Waveshare Bus Servo Adapter over USB serial.

### Story 4.1: Test Waveshare Adapter with SCS0009 Servos

**As a** builder,
**I want** Waveshare adapter controlling SCS0009 ear servos,
**so that** I can verify communication with smaller servos.

**Acceptance Criteria:**
1. Second Waveshare Bus Servo Adapter (A) connected via USB
2. Power connected (12V or appropriate for SCS0009)
3. 4× SCS0009 servos daisy-chained (IDs 1-4)
4. Python test script moves all servos
5. Servo feedback readable

### Story 4.2: Design Ear Mechanical Assembly

**As a** builder,
**I want** ear mechanical design with 2-DOF per ear,
**so that** ears can express emotions through movement.

**Acceptance Criteria:**
1. OnShape CAD model for ear mounts and ear shapes
2. 2 servos per ear: base rotation + ear angle
3. Ear shapes suitable for robot aesthetic
4. Mounting points on head enclosure
5. STL files exported and parts printed

### Story 4.3: Assemble and Test Ears Module

**As a** builder,
**I want** ears assembled and tested,
**so that** ear articulation works expressively.

**Acceptance Criteria:**
1. Servos mounted in ear brackets
2. Ear shapes attached to servo horns
3. Full range of motion without interference
4. Both ears move symmetrically
5. Assembly documented in `modules/ears/assembly.md`

### Story 4.4: Create Ears ROS2 Driver Node

**As a** builder,
**I want** ROS2 driver node for Ears module,
**so that** I can control ear expressions via ROS2.

**Acceptance Criteria:**
1. Python ROS2 node created in `ros2/src/olaf_ears/`
2. Node subscribes to `/ears/emote` (preset emotions) and `/ears/pose` (direct angles)
3. USB serial to second Waveshare adapter
4. Preset emotions: perked, relaxed, alert, sad, happy
5. Node launches and ears respond to topics

---

## Epic 5: Indicators, Heart & Kickstand Build

**Goal**: Complete Indicator module (24× WS2812 LEDs via Fusion HAT), Heart display (4" Pi display), and Kickstand mechanism (2× landing gear servos via HAT PWM).

### Story 5.1: Configure and Test WS2812 LED Strips

**As a** builder,
**I want** WS2812 LED strips controlled via Fusion HAT,
**so that** I can provide visual feedback and status indication.

**Acceptance Criteria:**
1. 24× WS2812 LEDs wired in 3×8 daisy chain configuration
2. Connected to Fusion HAT WS2812 data port
3. Fusion HAT SDK or rpi_ws281x library configured
4. Test script can control individual LEDs and animations
5. Three logical strips defined: Interaction (8), Status (8), PID (8)

### Story 5.2: Create Indicator ROS2 Driver Node

**As a** builder,
**I want** ROS2 driver node for Indicator LEDs,
**so that** I can trigger LED patterns via ROS2.

**Acceptance Criteria:**
1. Python ROS2 node created in `ros2/src/olaf_indicator/`
2. Node subscribes to `/indicator/interact`, `/indicator/status`, `/indicator/pid`
3. Preset patterns: listening, thinking, speaking, error, success
4. PID visualization shows balance state
5. Node launches and LEDs respond to topics

### Story 5.3: Configure Heart Display

**As a** builder,
**I want** 4" Pi display showing animated heart,
**so that** the robot has an expressive "heart" element.

**Acceptance Criteria:**
1. 4" Raspberry Pi Display connected via SPI/DSI
2. Display configured in Ubuntu (framebuffer or desktop)
3. Python script renders animated heart graphics
4. Heart animation varies with "emotion" (fast beat, slow beat, flutter)
5. Display integrated into torso enclosure design

### Story 5.4: Configure and Test Kickstand Servos

**As a** builder,
**I want** kickstand servos controlled via Fusion HAT PWM,
**so that** the robot can transition between stationary and balancing modes.

**Acceptance Criteria:**
1. 2× model plane landing gear servos (4.8-7.4V) wired
2. Long PWM signal wires routed from HAT to base platform
3. Power from 7.4V logic battery via HAT
4. Fusion HAT PWM channels configured (P0, P1)
5. Test script deploys and retracts kickstand
6. Deployed position stable, retracted clears ground

### Story 5.5: Create Kickstand Control Integration

**As a** builder,
**I want** kickstand controllable via ROS2,
**so that** mode transitions are software-controlled.

**Acceptance Criteria:**
1. Kickstand control added to `olaf_indicator` node or separate node
2. Topic `/kickstand/deploy` and `/kickstand/retract`
3. Service (optional): `/kickstand/set_state`
4. Safety interlock with base balancing (disable balance before deploy)
5. Node responds to kickstand commands

---

## Epic 6: End-to-End Demo Script

**Goal**: Create and execute demo script validating all modules via ROS2 with coordinated multi-module operation.

### Story 6.1: Create End-to-End Demo Script

**As a** builder,
**I want** a ROS2 Python script demonstrating all modules,
**so that** I can validate Phase 1 completion.

**Acceptance Criteria:**
1. Script created in `ros2/src/olaf_bringup/scripts/phase1_demo.py`
2. Demo sequence:
   - Deploy kickstand
   - Blink eyes (multiple expressions)
   - Move ears (perked, relaxed)
   - Tilt head (pan/tilt/roll demo)
   - Animate heart (rhythm changes)
   - Flash indicator LEDs (patterns)
   - Retract kickstand
   - Enable balancing
   - Small movement test (forward/backward)
3. Appropriate delays between steps
4. Logging output for each step
5. Graceful shutdown on Ctrl+C

### Story 6.2: Create Demo Launch File

**As a** builder,
**I want** a launch file starting all nodes and demo,
**so that** I can run complete demo with single command.

**Acceptance Criteria:**
1. Launch file in `ros2/src/olaf_bringup/launch/phase1_demo.launch.py`
2. Starts all driver nodes: head, base, neck, ears, indicator
3. Waits for initialization, then starts demo script
4. Clean shutdown on termination
5. Executable via `ros2 launch olaf_bringup phase1_demo.launch.py`

### Story 6.3: Test and Record End-to-End Demo

**As a** builder,
**I want** demo tested and recorded,
**so that** I can document Phase 1 completion.

**Acceptance Criteria:**
1. All modules powered and connected
2. Demo executes successfully through all steps
3. Any failures documented with troubleshooting
4. Video recorded of full demo
5. Git tag created: `v1.0-phase1`

### Story 6.4: Document Phase 1 Completion

**As a** builder,
**I want** Phase 1 documented with summary and lessons learned,
**so that** knowledge is captured for community and Phase 2 planning.

**Acceptance Criteria:**
1. Summary document in `docs/guides/phase1-summary.md`
2. Total build time and cost breakdown
3. Challenges and solutions documented
4. Photos of completed modules
5. Known issues documented
6. README updated with completion status and demo video
7. Build-in-public post shared

---

## Next Steps

### Phase 2 Preview: Middleware & Intelligence

Once Phase 1 is complete, Phase 2 will focus on:

**Orchestration & Coordination:**
- Personality coordinator: synchronized expressions across all modules
- State machine: idle, listening, thinking, expressing, navigating
- Emotion engine: multi-channel emotional expressions

**Navigation & Mobility:**
- Cartographer SLAM integration
- Nav2 stack configuration
- Follow-me mode with person tracking
- Obstacle avoidance

**AI Integration:**
- Hailo Whisper STT: <200ms local speech recognition
- Cloud AI agents (Claude/GPT-4): reasoning and personality
- Context management with SQLite
- Function routing to modules

---

**End of Phase 1 PRD v2.0**
