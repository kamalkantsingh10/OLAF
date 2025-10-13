# Epic 1: Foundation & I2C Communication (Heart Display)

**Epic Goal:** Establish the complete development foundation including repository structure, ROS2 environment on Raspberry Pi, I2C communication protocol, and power distribution. Deliver a minimal working personality—a heart LCD display showing emotion-driven beating animation via I2C commands from the orchestrator—proving the three-layer architecture (Orchestrator → I2C → Smart ESP32 Peripheral) works end-to-end with I2C + SPI communication validated.

**Duration:** 2 weeks (Weeks 1-2)

**Prerequisites:** None (first epic)

**Value Delivered:** Quick win proving I2C + SPI architecture viability, establishes shared register protocol, enables all future module development, provides "Olaf's heart beats!" moment within first 2 weekends. Heart display demonstrates emotion-driven animation (60 FPS) without requiring full head assembly.

**Architecture Focus:** This epic validates the core architectural decision that ROS2 runs ONLY on Pi, with ESP32 modules acting as smart I2C slaves containing full hardware drivers and animation engines. Heart LCD proves SPI display performance before investing in dual-eye head assembly in Epic 2.

---

## Story 1.1: Repository Setup & Project Structure

**As a** developer building Olaf,
**I want** a well-organized monorepo with clear structure and documentation,
**so that** I can clone the project, understand the layout, and start contributing immediately.

### Acceptance Criteria:

1. Repository created on GitHub with name `olaf` under appropriate account
2. Directory structure matches architecture specification:
   ```
   olaf/
   ├── docs/                       # PRD, architecture, epics
   │   ├── prd/
   │   ├── architecture/
   │   └── stories/
   ├── hardware/                   # 3D models, wiring, BOM
   │   ├── 3d-models/
   │   ├── wiring/
   │   └── bom/
   ├── modules/                    # ESP32 firmware (C++)
   │   ├── head/
   │   ├── ears_neck/
   │   ├── body/
   │   └── base/
   ├── orchestrator/               # Raspberry Pi Python + ROS2
   │   ├── ros2_nodes/
   │   │   └── hardware_drivers/
   │   ├── launch/
   │   └── config/
   ├── shared/                     # Common headers/definitions
   │   └── i2c_registers.h
   ├── tests/
   ├── tools/
   │   ├── setup/
   │   └── diagnostics/
   ├── .gitignore
   ├── README.md
   └── LICENSE
   ```
3. README.md contains:
   - Project overview (self-balancing AI companion)
   - Architecture diagram showing I2C-based communication
   - Quick start instructions
   - Links to PRD, architecture docs, epics
4. `.gitignore` configured to exclude:
   - Python: `__pycache__/`, `*.pyc`, `venv/`
   - ROS2: `build/`, `install/`, `log/`
   - PlatformIO: `.pio/`, `.vscode/`
5. `shared/i2c_registers.h` created with common register definitions:
   ```cpp
   // Common registers (all modules)
   #define REG_MODULE_ID           0x00
   #define REG_FIRMWARE_VERSION    0x01
   #define REG_STATUS              0x02
   #define REG_COMMAND             0x04

   // Status flags
   #define STATUS_READY            0x01
   #define STATUS_BUSY             0x02
   #define STATUS_ERROR            0x04
   ```
6. MIT License file added
7. Initial commit pushed

**Dependencies:** None

**Estimated Effort:** 2-3 hours

---

## Story 1.2: Power System Assembly & Validation

**As a** robot builder,
**I want** a reliable 36V hoverboard power system with buck converters,
**so that** all modules receive stable 5V and 12V power for operation.

### Acceptance Criteria:

1. **Battery Selection:**
   - Hoverboard battery salvaged (36V nominal, 4-10Ah capacity)
   - Voltage measured: 36-42V (confirms healthy cells)
   - BMS retained for safe charging

2. **Buck Converter Installation:**
   - 36V → 12V buck converter (10A) installed for Raspberry Pi power
   - 36V → 5V buck converter (10A) installed for ESP32s + servos
   - Output voltages measured under no-load: 12V ±0.2V, 5V ±0.1V

3. **Load Testing:**
   - 12V rail tested with 3A load (simulates Pi): voltage stable
   - 5V rail tested with 2A load (simulates ESP32): voltage stable
   - Both converters loaded simultaneously: no voltage sag >5%

4. **Safety:**
   - Fuse/circuit breaker on battery main output (30-40A)
   - Emergency power switch accessible
   - Proper wire gauge: 18 AWG minimum for power rails
   - Secure connectors (XT60, JST-XH, or screw terminals)

5. **Physical Mounting:**
   - Battery, converters, and connections mounted securely
   - System can be moved without cables disconnecting
   - No risk of shorts (insulated terminals)

6. **Documentation:**
   - Wiring diagram saved: `hardware/wiring/power-system-v1.jpg`
   - BOM updated with battery source, converters, connectors, costs

**Dependencies:** Story 1.1 (repo for documentation)

**Estimated Effort:** 6-8 hours (includes salvaging hoverboard)

---

## Story 1.3: Raspberry Pi ROS2 Setup (I2C Master)

**As a** robotics developer,
**I want** ROS2 Humble installed on Raspberry Pi 5 with I2C enabled,
**so that** I can run orchestrator nodes and communicate with ESP32 modules via I2C.

### Acceptance Criteria:

1. **OS Installation:**
   - Raspberry Pi OS (64-bit, Debian 12 Bookworm) flashed to microSD (64GB+)
   - Pi boots successfully, SSH enabled
   - WiFi configured for cloud API access
   - Static IP optional but recommended

2. **ROS2 Installation:**
   - ROS2 Humble installed: `sudo apt install ros-humble-desktop`
   - Environment sourced: `source /opt/ros/humble/setup.bash` added to `.bashrc`
   - Verification: `ros2 --version` shows "humble"

3. **I2C Configuration:**
   - I2C enabled via `raspi-config`: Interface Options → I2C → Enable
   - I2C device visible: `ls /dev/i2c-*` shows `/dev/i2c-1`
   - I2C tools installed: `sudo apt install i2c-tools`
   - I2C permissions: User added to `i2c` group

4. **Python I2C Library:**
   - `smbus2` installed: `pip3 install smbus2`
   - Test script can open I2C bus successfully

5. **Development Tools:**
   - Python 3.11+ (comes with Pi OS)
   - `colcon` installed: `sudo apt install python3-colcon-common-extensions`
   - Git installed and configured

6. **Repository Clone:**
   - Repo cloned to Pi: `git clone <repo_url> ~/olaf`
   - Directory structure visible

7. **Documentation:**
   - Setup script: `tools/setup/install_ros2_humble_pi.sh` automates ROS2 + I2C setup
   - README updated with Pi setup instructions

**Dependencies:** Story 1.1 (repo), Story 1.2 (power to boot Pi)

**Estimated Effort:** 3-4 hours

---

## Story 1.4: Body Module - Heart LCD Hardware Assembly

**As a** hardware builder,
**I want** a body module with heart LCD display proving I2C + SPI communication,
**so that** I can validate the architecture with emotion-driven heartbeat animation.

### Acceptance Criteria:

1. **Components Acquired:**
   - 1× ESP32-S3-WROOM-2 (N8R8) or ESP32-WROOM-32
   - 1× GC9A01 Round TFT Display (1.28", 240×240, SPI interface)
   - Jumper wires for SPI + I2C connections
   - Breadboard for prototyping

2. **SPI Heart Display Wiring (ESP32 → GC9A01):**
   - VCC → 3.3V (ESP32)
   - GND → GND
   - SCL (CLK) → GPIO18 (ESP32 SPI SCK)
   - SDA (MOSI) → GPIO23 (ESP32 SPI MOSI)
   - RES (RST) → GPIO4 (ESP32 reset pin)
   - DC → GPIO2 (ESP32 data/command pin)
   - CS → GPIO5 (ESP32 chip select)

3. **I2C Wiring (Pi → ESP32):**
   - Pi GPIO2 (SDA) → ESP32 GPIO21 (I2C SDA)
   - Pi GPIO3 (SCL) → ESP32 GPIO22 (I2C SCL)
   - Pi GND → ESP32 GND (common ground)
   - Pi 5V → ESP32 VIN (power from buck converter)

4. **I2C Address Assignment:**
   - ESP32 Body module configured as I2C slave at address **0x0A**

5. **Communication Tests:**
   - SPI test: Upload GC9A01 library example, heart displays at 60 FPS
   - I2C test: `sudo i2cdetect -y 1` shows device at address `0x0A`
   - Verify SPI speed: 10-20 MHz clock (smooth animation, no tearing)

6. **Physical Mounting:**
   - Module secured to breadboard
   - Heart display positioned for visibility
   - Stable wiring, no loose connections

7. **Documentation:**
   - Wiring photos: `hardware/wiring/body-heart-lcd-assembly.jpg`
   - Pin mapping documented in `modules/body/README.md`
   - BOM updated

**Note:** Projector control (optocoupler) and LED strips will be added in later epics (Epic 6 for LEDs, Epic 12 for projector).

**Dependencies:** Story 1.2 (5V power), Story 1.3 (I2C enabled on Pi)

**Estimated Effort:** 2-3 hours

---

## Story 1.5: Body Module ESP32 Firmware - Heart Animation

**As a** firmware developer,
**I want** ESP32 firmware rendering emotion-driven heart animation,
**so that** I can validate I2C + SPI communication with a beating heart display.

### Acceptance Criteria:

1. **Development Environment:**
   - PlatformIO project: `modules/body/firmware/`
   - Libraries: `TFT_eSPI` or `Adafruit_GC9A01A`, `Wire.h`

2. **I2C Slave Implementation:**
   - ESP32 configured as I2C slave at address **0x0A**
   - I2C receive/request handlers implemented

3. **I2C Register Map (Body Module 0x0A):**
   - `0x00`: Module ID (returns 0x0A)
   - `0x02`: Status byte (READY/BUSY/ERROR)
   - `0x10`: Emotion type (0=neutral, 1=happy, 2=excited, 3=sad, 4=curious, 5=thinking, 6=confused)
   - `0x11`: Emotion intensity (1-5, affects beat amplitude)
   - `0x12`: Heart rate override (BPM, 0=auto from emotion)
   - `0x13-0x15`: Heart color (RGB, 0-255 each)

4. **Heart Animation Engine:**
   - Heart sprite (240×240 display, centered)
   - Beating animation: Scale/pulse effect synchronized to BPM
   - **Emotion-to-BPM mapping:**
     - Neutral: 60-70 BPM | Happy: 80-90 BPM | Excited: 100-120 BPM
     - Sad: 50-60 BPM | Curious: 70-80 BPM | Thinking: 65-75 BPM | Confused: 75-85 BPM
   - Intensity: 1=subtle (10% scale), 5=dramatic (50% scale)
   - Color variations: red=default, blue=sad, yellow=happy
   - 60 FPS smooth animation

5. **Testing:**
   - Firmware compiled and uploaded
   - Serial: "I2C Slave initialized at 0x0A"
   - Heart beats at 70 BPM (neutral) on startup
   - Responsive to I2C commands via `i2cset`

6. **Performance:**
   - 60 FPS maintained
   - I2C latency <10ms
   - Memory stable over 10 minutes

7. **Code Quality:**
   - Files: `main.cpp`, `heart_animation.cpp`, `gc9a01_driver_spi.cpp`, `i2c_slave.cpp`
   - Comments explain I2C protocol
   - Code committed to `modules/body/firmware/`

**Dependencies:** Story 1.4 (hardware assembled)

**Estimated Effort:** 4-6 hours

---

## Story 1.6: ROS2 Body Driver Node - I2C Bridge

**As a** software developer,
**I want** a ROS2 node that translates ROS2 topics into I2C register writes,
**so that** the orchestrator can control the heart display without knowing I2C details.

### Acceptance Criteria:

1. **Node Structure:**
   - Python ROS2 node: `orchestrator/ros2_nodes/hardware_drivers/body_driver.py`
   - Node name: `/olaf/body_driver`
   - Uses `rclpy` and `smbus2`

2. **I2C Communication:**
   - Opens I2C bus 1: `bus = SMBus(1)`
   - Helper functions for register read/write to address 0x0A

3. **ROS2 Subscriptions:**
   - `/olaf/body/emotion` (custom msg: emotion_type, intensity)
   - Callback writes to registers 0x10-0x11
   - `/olaf/body/heart_color` (std_msgs/ColorRGBA)
   - Callback writes to registers 0x13-0x15

4. **ROS2 Publications:**
   - `/olaf/body/status` (std_msgs/String) at 10Hz
   - Reads status register 0x02, publishes "READY", "BUSY", or "ERROR"

5. **Module Health Check:**
   - On startup: Read register 0x00, verify response is 0x0A
   - If not responding: Log error, publish ERROR status

6. **Error Handling:**
   - I2C timeout handling
   - Retry logic: 3 attempts with 100ms delay
   - Persistent failure: Publish ERROR

7. **Testing:**
   - Node launches: `ros2 run olaf_drivers body_driver`
   - Topics appear: `/olaf/body/emotion`, `/olaf/body/status`
   - Manual test: Publish emotion → heart changes BPM

**Dependencies:** Story 1.3 (ROS2 + I2C), Story 1.5 (firmware)

**Estimated Effort:** 4-6 hours

---

## Story 1.7: Minimal Orchestrator - Heartbeat Coordinator

**As a** software developer,
**I want** a minimal orchestrator node that sends emotion commands via ROS2,
**so that** I can prove the full pipeline (Orchestrator → ROS2 → I2C → ESP32 → SPI heart display) works.

### Acceptance Criteria:

1. **Orchestrator Node:**
   - Python ROS2 node: `orchestrator/ros2_nodes/minimal_coordinator.py`
   - Node name: `/olaf/minimal_coordinator`

2. **Publisher:**
   - Publishes to `/olaf/body/emotion`

3. **Emotion Cycle:**
   - Timer cycles through emotions every 10 seconds:
     - Neutral → Happy → Excited → Sad → Curious → Thinking → Confused → repeat
   - Intensity randomly varies between 2-4

4. **Execution:**
   - Launch: `ros2 run olaf_orchestrator minimal_coordinator`
   - Every 10 seconds: Logs emotion change, heart display updates

5. **End-to-End Validation:**
   - Full chain: Coordinator → Body driver → I2C → ESP32 → SPI heart
   - Latency: Publish to heart update <100ms
   - Reliability: 50 emotion changes without failure

6. **Code Quality:**
   - Clean, commented code
   - Committed to `orchestrator/ros2_nodes/minimal_coordinator.py`

**Dependencies:** Story 1.6 (body driver)

**Estimated Effort:** 3-4 hours

---

## Story 1.8: Test Harness CLI - I2C Direct Testing

**As a** developer or tester,
**I want** a command-line tool to send I2C commands directly to body module,
**so that** I can debug without running full ROS2 stack.

### Acceptance Criteria:

1. **CLI Tool:**
   - Python script: `tools/diagnostics/olaf-test`
   - Executable: `chmod +x tools/diagnostics/olaf-test`
   - Uses `smbus2` for direct I2C access

2. **Commands:**
   - `olaf-test body emotion <type> <intensity>` → Writes registers 0x10-0x11 to address 0x0A
   - `olaf-test body status` → Reads register 0x02, prints status
   - `olaf-test body id` → Reads register 0x00, prints module ID
   - `olaf-test scan` → Runs `i2cdetect -y 1`, shows all I2C devices

3. **Implementation:**
   ```python
   def set_emotion(emotion_type, intensity):
       bus = SMBus(1)
       bus.write_byte_data(0x0A, 0x10, emotion_type)
       bus.write_byte_data(0x0A, 0x11, intensity)
       print(f"Emotion set: type={emotion_type}, intensity={intensity}")
   ```

4. **Usage:**
   - `./tools/diagnostics/olaf-test body emotion 1 3` → Sets happy emotion, intensity 3
   - `--help` shows all commands

5. **Error Handling:**
   - "I2C bus not found" if I2C not enabled
   - "No response from module 0x0A" if ESP32 not responding

6. **Documentation:**
   - Usage guide: `tools/diagnostics/README.md`

**Dependencies:** Story 1.5 (firmware), Story 1.3 (I2C enabled)

**Estimated Effort:** 2-3 hours

---

## Story 1.9: Launch File - Automated Startup

**As a** system operator,
**I want** a ROS2 launch file that starts all Epic 1 nodes,
**so that** I can run the full system with one command.

### Acceptance Criteria:

1. **Launch File Created:**
   - File: `orchestrator/launch/minimal_system.launch.py`
   - Starts: Body driver node + Minimal coordinator

2. **Launch Configuration:**
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='olaf_drivers',
               executable='body_driver',
               name='body_driver'
           ),
           Node(
               package='olaf_orchestrator',
               executable='minimal_coordinator',
               name='minimal_coordinator'
           ),
       ])
   ```

3. **Execution:**
   - Command: `ros2 launch orchestrator minimal_system.launch.py`
   - Both nodes start automatically
   - Heart cycles through emotions every 10 seconds

4. **Logging:**
   - Launch output shows both nodes starting
   - Logs visible in terminal

5. **Documentation:**
   - Launch file documented in README
   - "Quick Start" section updated

**Dependencies:** Story 1.6 (driver), Story 1.7 (coordinator)

**Estimated Effort:** 2 hours

---

## Story 1.10: 30-Minute Continuous Operation Test

**As a** quality assurance tester,
**I want** to run the minimal system for 30 minutes without failures,
**so that** I can validate I2C communication stability before Epic 2.

### Acceptance Criteria:

1. **Test Setup:**
   - Launch: `ros2 launch orchestrator minimal_system.launch.py`
   - Battery at full charge
   - Timer set for 30 minutes

2. **Test Execution:**
   - Heart cycles through emotions every 10 seconds (180 cycles total)
   - Monitor: Terminal logs, heart animation, I2C communication

3. **Success Criteria:**
   - No crashes: All nodes run 30 minutes
   - No missed updates: ~180 emotion changes complete (±5% tolerance)
   - Stable latency: Emotion change → heart update <100ms throughout
   - Memory stable: ESP32 free heap doesn't decrease

4. **Metrics Collected:**
   - Total runtime: 30:00 minutes
   - Total emotion changes: ~180
   - I2C errors: 0
   - Average latency: <100ms (sampled 10 times)

5. **Pass/Fail:**
   - PASS: All criteria met
   - FAIL: Any crash, >10% missed updates, latency >100ms

6. **Documentation:**
   - Results: `tests/integration/epic1_30min_test_results.md`
   - Screenshot of terminal after 30 min

**Dependencies:** All Epic 1 stories (1.1-1.9)

**Estimated Effort:** 1 hour test + 2-4 hours fixes if needed

---

## Story 1.11: Epic 1 Documentation & Content

**As a** future contributor or follower,
**I want** comprehensive Epic 1 documentation and build-in-public content,
**so that** I can replicate the setup and follow the journey.

### Acceptance Criteria:

1. **Technical Documentation:**
   - `modules/body/README.md`:
     - Hardware connections (SPI + I2C pinout)
     - Firmware build/upload process
     - I2C register map reference
   - `orchestrator/ros2_nodes/README.md`:
     - How to run driver nodes
     - Topic reference
     - I2C communication flow

2. **Hardware Documentation:**
   - BOM: `hardware/bom/epic1_bom.csv`
     - Battery, buck converters, ESP32, GC9A01 display, wires
     - Supplier links, costs
   - Wiring diagrams in `hardware/wiring/`

3. **Setup Scripts:**
   - `tools/setup/install_ros2_humble_pi.sh` tested on fresh Pi
   - README "Quick Start" section complete

4. **Photos/Videos:**
   - Photos: Power system, heart display beating, full breadboard setup
   - Optional: 15-second video of beating heart

5. **GitHub Milestone:**
   - Milestone "Epic 1: Foundation" created
   - All stories closed and linked
   - Milestone marked complete

6. **Build-in-Public Content:**
   - LinkedIn post: "Olaf's Heart Beats! ❤️🤖"
     - Photo of beating heart display
     - Explanation: I2C + SPI architecture, 60 FPS emotion-driven animation
     - Technical depth: Smart I2C slave pattern
   - Post published

7. **Troubleshooting Guide:**
   - `docs/troubleshooting-epic1.md`:
     - "ESP32 not detected on I2C" → solution
     - "GC9A01 shows garbage" → wiring check
     - "ROS2 node can't open I2C" → permissions fix

**Dependencies:** All Epic 1 stories complete

**Estimated Effort:** 6-8 hours (docs + photos + content)

---

## Epic 1 Summary

**Total Stories:** 11
**Estimated Total Effort:** 40-55 hours (2 weeks for solo builder)

**Key Deliverables:**
- ✅ Repository structure with shared I2C register definitions
- ✅ Power system (36V hoverboard battery + buck converters)
- ✅ ROS2 Humble on Raspberry Pi with I2C enabled
- ✅ **Body Module:** Heart LCD (GC9A01, 60 FPS emotion BPM) at I2C address 0x0A
- ✅ ROS2 body driver node translating topics → I2C registers
- ✅ Orchestrator sending coordinated emotion commands
- ✅ Test harness for direct I2C module testing
- ✅ Launch file for automated system startup
- ✅ 30-minute continuous operation validated
- ✅ Complete documentation + build-in-public content

**Architecture Validated:**
- ✅ I2C-only communication (no WiFi on ESP32s)
- ✅ SPI display for 60 FPS smooth animation
- ✅ Smart I2C slave pattern (ESP32 has animation engine)
- ✅ ROS2 only on Pi (driver nodes as I2C bridges)
- ✅ Shared register protocol (foundation for all modules)

**Success Criteria Met:**
- Three-layer architecture proven (Orchestrator → I2C Driver → Smart ESP32)
- Power system stable and safe
- "Quick win" achieved: Olaf's heart beats with emotion-driven animation!
- Foundation ready for Epic 2 (complete head module with dual eyes)

**Next Epic:** Epic 2: Head Module - Eyes & Sensors

---

**Related Documents:**
- [PRD](../prd/index.md) - FR1-FR7 validated
- [Architecture](../architecture/index.md) - I2C + SPI pattern proven
- [Epic List](epic-list.md) - Overall project roadmap
