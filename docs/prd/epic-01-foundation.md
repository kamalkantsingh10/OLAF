# Epic 1: Foundation & I2C Communication (Eye Displays)

**Epic Goal:** Establish the complete development foundation including repository structure, ROS2 environment on Raspberry Pi, and I2C communication protocol. Deliver a minimal working personality—dual eye LCD displays showing emotion-driven expressions via I2C commands from the orchestrator—proving the three-layer architecture (Orchestrator → I2C → Smart ESP32 Peripheral) works end-to-end with I2C + SPI communication validated. Use temporary bench power supply (full power system deferred to Epic 5: Core Torso & Power System).

**Duration:** 1.5 weeks (Weeks 1-1.5)

**Prerequisites:** None (first epic)

**Value Delivered:** Quick win proving I2C + SPI architecture viability, establishes shared register protocol, enables all future module development, provides "Olaf's eyes come alive!" moment within first 2 weekends. Eye displays demonstrate emotion-driven expressions (60 FPS) validating Head Module architecture before full head assembly in Epic 3. Initial OnShape design concept validated with community feedback.

**Architecture Focus:** This epic validates the core architectural decision that ROS2 runs ONLY on Pi, with ESP32 modules acting as smart I2C slaves containing full hardware drivers and animation engines. Dual eye LCDs prove Head Module (I2C 0x08) SPI display performance and expression engine before physical head assembly in Epic 3.

**Power Note:** For Epic 1 development, use a temporary bench power supply (e.g., lab power supply set to 5V for ESP32 + displays, or USB power banks). The complete power system (36V hoverboard battery, BMS, buck converters, charging circuit, safety features) will be designed and built in Epic 5: Core Torso & Power System.

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
   ├── firmware/                    # ESP32 firmware (C++)
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

## Story 1.2: Raspberry Pi ROS2 Setup (I2C Master)

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

**Dependencies:** Story 1.1 (repo)

**Estimated Effort:** 3-4 hours

**Note:** For development, power the Raspberry Pi via USB-C power supply or official Pi power adapter.

---

## Story 1.3: Head Module - Eye LCD Hardware Assembly

**As a** hardware builder,
**I want** a head module with dual eye LCD displays proving I2C + SPI communication,
**so that** I can validate the architecture with emotion-driven eye expressions.

### Acceptance Criteria:

1. **Components Acquired:**
   - 1× ESP32-S3-WROOM-2 (N16R8) or ESP32-WROOM-32
   - 2× GC9A01 Round TFT Display (1.28", 240×240, SPI interface)
   - Jumper wires for SPI + I2C connections
   - Breadboard for prototyping

2. **SPI Eye Display Wiring (ESP32 → 2× GC9A01):**
   - **Left Eye Display:**
     - VCC → 3.3V (ESP32)
     - GND → GND
     - SCL (CLK) → GPIO18 (ESP32 SPI SCK - shared)
     - SDA (MOSI) → GPIO23 (ESP32 SPI MOSI - shared)
     - RES (RST) → GPIO4 (ESP32 reset pin)
     - DC → GPIO2 (ESP32 data/command pin)
     - CS → GPIO5 (ESP32 chip select - Left Eye)
   - **Right Eye Display:**
     - VCC → 3.3V (ESP32 - shared)
     - GND → GND (shared)
     - SCL (CLK) → GPIO18 (ESP32 SPI SCK - shared)
     - SDA (MOSI) → GPIO23 (ESP32 SPI MOSI - shared)
     - RES (RST) → GPIO4 (ESP32 reset pin - shared)
     - DC → GPIO2 (ESP32 data/command pin - shared)
     - CS → GPIO15 (ESP32 chip select - Right Eye)

3. **I2C Wiring (Pi → ESP32):**
   - Pi GPIO2 (SDA) → ESP32 GPIO21 (I2C SDA)
   - Pi GPIO3 (SCL) → ESP32 GPIO22 (I2C SCL)
   - Pi GND → ESP32 GND (common ground)
   - Pi 5V → ESP32 VIN (power from buck converter)

4. **I2C Address Assignment:**
   - ESP32 Head module configured as I2C slave at address **0x08**

5. **Communication Tests:**
   - SPI test: Upload GC9A01 library example, both eyes display at 60 FPS
   - I2C test: `sudo i2cdetect -y 1` shows device at address `0x08`
   - Verify SPI speed: 10-20 MHz clock (smooth animation, no tearing)
   - Test independent chip select: Each eye can be addressed separately

6. **Physical Mounting:**
   - Module secured to breadboard
   - Eye displays positioned side-by-side for visibility
   - Stable wiring, no loose connections

7. **Documentation:**
   - Wiring photos: `hardware/wiring/head-eye-lcd-assembly.jpg`
   - Pin mapping documented in `firmware/head/README.md`
   - BOM updated with ESP32, 2× GC9A01 displays, jumper wires

**Note:** This breadboard prototype validates the Head Module that will be integrated into the physical head housing in Epic 3.

**Dependencies:** Story 1.2 (I2C enabled on Pi)

**Estimated Effort:** 2-3 hours

**Power Note:** Use USB power bank or bench power supply (5V) to power the ESP32 and both GC9A01 displays during development.

---

## Story 1.4: Head Module ESP32 Firmware - Eye Expressions

**As a** firmware developer,
**I want** ESP32 firmware rendering emotion-driven eye expressions on dual displays,
**so that** I can validate I2C + SPI communication with expressive eyes.

### Acceptance Criteria:

1. **Development Environment:**
   - PlatformIO project: `firmware/head/firmware/`
   - Libraries: `TFT_eSPI` or `Adafruit_GC9A01A`, `Wire.h`

2. **I2C Slave Implementation:**
   - ESP32 configured as I2C slave at address **0x08**
   - I2C receive/request handlers implemented

3. **I2C Register Map (Head Module 0x08):**
   - `0x00`: Module ID (returns 0x08)
   - `0x02`: Status byte (READY/BUSY/ERROR)
   - `0x10`: Expression type (0=neutral, 1=happy, 2=curious, 3=thinking, 4=confused, 5=sad, 6=excited)
   - `0x11`: Expression intensity (1-5, affects animation intensity)
   - `0x12`: Blink trigger (write any value to trigger blink)

4. **Eye Expression Engine:**
   - Dual display rendering (left eye GPIO5 CS, right eye GPIO15 CS)
   - **Expression animations:**
     - Neutral: Circular pupils, steady gaze
     - Happy: Wide pupils, slight upward curve
     - Curious: Large pupils, slight offset
     - Thinking: Pupils looking up-right, occasional blink
     - Confused: Pupils wandering, asymmetric
     - Sad: Small pupils, downward gaze
     - Excited: Very wide pupils, rapid micro-movements
   - Intensity: 1=subtle changes, 5=exaggerated expressions
   - 60 FPS smooth animation on both displays
   - Synchronized rendering (both eyes update together)

5. **Testing:**
   - Firmware compiled and uploaded
   - Serial: "I2C Slave initialized at 0x08 - Head Module"
   - Both eyes display neutral expression on startup
   - Responsive to I2C commands via `i2cset`
   - Blink command triggers synchronized blink

6. **Performance:**
   - 60 FPS maintained on both displays
   - I2C latency <10ms
   - Memory stable over 10 minutes
   - Both eyes remain synchronized (±1 frame tolerance)

7. **Code Quality:**
   - Files: `main.cpp`, `eye_expression.cpp`, `gc9a01_driver_spi.cpp`, `i2c_slave.cpp`
   - Comments explain I2C protocol and dual-display rendering
   - Code committed to `firmware/head/firmware/`

**Dependencies:** Story 1.3 (hardware assembled)

**Estimated Effort:** 4-6 hours

---

## Story 1.5: ROS2 Head Driver Node - I2C Bridge

**As a** software developer,
**I want** a ROS2 node that translates ROS2 topics into I2C register writes,
**so that** the orchestrator can control the eye displays without knowing I2C details.

### Acceptance Criteria:

1. **Node Structure:**
   - Python ROS2 node: `ros2/src/orchestrator/ros2_nodes/hardware_drivers/head_driver.py`
   - Node name: `/olaf/head_driver`
   - Uses `rclpy` and `smbus2`

2. **I2C Communication:**
   - Opens I2C bus 1: `bus = SMBus(1)`
   - Helper functions for register read/write to address 0x08

3. **ROS2 Subscriptions:**
   - `/olaf/head/expression` (custom msg: expression_type, intensity)
   - Callback writes to registers 0x10-0x11
   - `/olaf/head/blink` (std_msgs/Empty)
   - Callback writes to register 0x12

4. **ROS2 Publications:**
   - `/olaf/head/status` (std_msgs/String) at 10Hz
   - Reads status register 0x02, publishes "READY", "BUSY", or "ERROR"

5. **Module Health Check:**
   - On startup: Read register 0x00, verify response is 0x08
   - If not responding: Log error, publish ERROR status

6. **Error Handling:**
   - I2C timeout handling
   - Retry logic: 3 attempts with 100ms delay
   - Persistent failure: Publish ERROR

7. **Testing:**
   - Node launches: `ros2 run orchestrator head_driver`
   - Topics appear: `/olaf/head/expression`, `/olaf/head/status`
   - Manual test: Publish expression → eyes change expression

**Dependencies:** Story 1.2 (ROS2 + I2C), Story 1.4 (firmware)

**Estimated Effort:** 4-6 hours

---

## Story 1.6: Minimal Orchestrator - Expression Coordinator

**As a** software developer,
**I want** a minimal orchestrator node that sends expression commands via ROS2,
**so that** I can prove the full pipeline (Orchestrator → ROS2 → I2C → ESP32 → SPI eye displays) works.

### Acceptance Criteria:

1. **Orchestrator Node:**
   - Python ROS2 node: `ros2/src/orchestrator/ros2_nodes/minimal_coordinator.py`
   - Node name: `/olaf/minimal_coordinator`

2. **Publisher:**
   - Publishes to `/olaf/head/expression`

3. **Expression Cycle:**
   - Timer cycles through expressions every 10 seconds:
     - Neutral → Happy → Curious → Thinking → Confused → Sad → Excited → repeat
   - Intensity randomly varies between 2-4
   - Trigger random blink every 3-8 seconds

4. **Execution:**
   - Launch: `ros2 run orchestrator minimal_coordinator`
   - Every 10 seconds: Logs expression change, eye displays update
   - Periodic blinks occur independently

5. **End-to-End Validation:**
   - Full chain: Coordinator → Head driver → I2C → ESP32 → SPI eyes
   - Latency: Publish to eye update <100ms
   - Reliability: 50 expression changes without failure
   - Blinks execute smoothly without interrupting expressions

6. **Code Quality:**
   - Clean, commented code
   - Committed to `ros2/src/orchestrator/ros2_nodes/minimal_coordinator.py`

**Dependencies:** Story 1.5 (body driver)

**Estimated Effort:** 3-4 hours

---

## Story 1.7: Test Harness CLI - I2C Direct Testing

**As a** developer or tester,
**I want** a command-line tool to send I2C commands directly to body module,
**so that** I can debug without running full ROS2 stack.

### Acceptance Criteria:

1. **CLI Tool:**
   - Python script: `tools/diagnostics/olaf-test`
   - Executable: `chmod +x tools/diagnostics/olaf-test`
   - Uses `smbus2` for direct I2C access

2. **Commands:**
   - `olaf-test head expression <type> <intensity>` → Writes registers 0x10-0x11 to address 0x08
   - `olaf-test head status` → Reads register 0x02, prints status
   - `olaf-test head id` → Reads register 0x00, prints module ID
   - `olaf-test head blink` → Writes to register 0x12, triggers blink
   - `olaf-test scan` → Runs `i2cdetect -y 1`, shows all I2C devices

3. **Implementation:**
   ```python
   def set_expression(expression_type, intensity):
       bus = SMBus(1)
       bus.write_byte_data(0x08, 0x10, expression_type)
       bus.write_byte_data(0x08, 0x11, intensity)
       print(f"Expression set: type={expression_type}, intensity={intensity}")
   ```

4. **Usage:**
   - `./tools/diagnostics/olaf-test head expression 1 3` → Sets happy expression, intensity 3
   - `./tools/diagnostics/olaf-test head blink` → Triggers eye blink
   - `--help` shows all commands

5. **Error Handling:**
   - "I2C bus not found" if I2C not enabled
   - "No response from module 0x08" if ESP32 not responding

6. **Documentation:**
   - Usage guide: `tools/diagnostics/README.md`

**Dependencies:** Story 1.4 (firmware), Story 1.2 (I2C enabled)

**Estimated Effort:** 2-3 hours

---

## Story 1.8: Launch File - Automated Startup

**As a** system operator,
**I want** a ROS2 launch file that starts all Epic 1 nodes,
**so that** I can run the full system with one command.

### Acceptance Criteria:

1. **Launch File Created:**
   - File: `ros2/src/orchestrator/launch/minimal_system.launch.py`
   - Starts: Head driver node + Minimal coordinator

2. **Launch Configuration:**
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='orchestrator',
               executable='head_driver',
               name='head_driver'
           ),
           Node(
               package='orchestrator',
               executable='minimal_coordinator',
               name='minimal_coordinator'
           ),
       ])
   ```

3. **Execution:**
   - Command: `ros2 launch orchestrator minimal_system.launch.py`
   - Both nodes start automatically
   - Eyes cycle through expressions every 10 seconds with periodic blinks

4. **Logging:**
   - Launch output shows both nodes starting
   - Logs visible in terminal

5. **Documentation:**
   - Launch file documented in README
   - "Quick Start" section updated

**Dependencies:** Story 1.5 (driver), Story 1.6 (coordinator)

**Estimated Effort:** 2 hours

---

## Story 1.9: 30-Minute Continuous Operation Test

**As a** quality assurance tester,
**I want** to run the minimal system for 30 minutes without failures,
**so that** I can validate I2C communication stability before Epic 2.

### Acceptance Criteria:

1. **Test Setup:**
   - Launch: `ros2 launch orchestrator minimal_system.launch.py`
   - Battery at full charge
   - Timer set for 30 minutes

2. **Test Execution:**
   - Eyes cycle through expressions every 10 seconds (180 cycles total)
   - Monitor: Terminal logs, eye expressions, I2C communication

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

**Dependencies:** All Epic 1 stories (1.1-1.8)

**Estimated Effort:** 1 hour test + 2-4 hours fixes if needed

---

## Story 1.10: Initial Design Concept & Community Feedback

**As a** maker and open-source contributor,
**I want** to create an initial visual concept in OnShape and gather Reddit community feedback,
**so that** I can validate the aesthetic direction and incorporate community suggestions before committing to the full Epic 2 design work.

### Story Context

**Existing System Integration:**
- Integrates with: Epic 2 3D Design workflow (Story 2.1 OnShape setup)
- Technology: OnShape cloud CAD, Reddit community engagement
- Follows pattern: Build-in-public content strategy from Epic 1 Story 1.11
- Touch points: Creates foundation for Epic 2 CAD work, validates aesthetic before major design investment

### Acceptance Criteria

**Functional Requirements:**

1. **OnShape Concept Sketch:**
   - Free OnShape account created (if not already from Epic 2 prep)
   - Simple sketch or concept model created showing Olaf's overall form:
     - Head shape (rounded, friendly aesthetic per branding guide)
     - Body proportions (target 50-60cm height, 35-40cm width)
     - Basic ear placement (Chappie-inspired triangular/curved fins)
     - Overall stance on hoverboard base
   - Design captures retro-futurism styling (rounded edges, no sharp corners, white/light gray color scheme)
   - OnShape link made public for community viewing

2. **Visual Presentation:**
   - Screenshots or rendered views from OnShape (minimum 3 angles: front, side, 3/4 view)
   - Optional: Simple annotation overlays showing key features (eyes, ears, projector, heart display)
   - Images exported at high resolution (1920x1080 minimum for clarity)

3. **Reddit Post Creation:**
   - Post drafted for relevant subreddit(s):
     - Primary: r/robotics or r/DIY
     - Secondary consideration: r/3Dprinting, r/raspberry_pi
   - Post title: Engaging, specific (e.g., "Designing Olaf - Open-Source AI Companion Robot with Personality - Feedback Welcome!")
   - Post content includes:
     - Project overview (self-balancing AI companion, emotion-driven personality)
     - Design goals (friendly aesthetic, retro-futurism, Chappie-inspired)
     - OnShape public link
     - Specific questions for feedback:
       - "What do you think of the proportions?"
       - "Does the aesthetic feel friendly/approachable?"
       - "Any suggestions for the ear design?"
       - "Concerns about balance/stability with this form factor?"
   - Images embedded in post (Reddit supports multiple images)

**Integration Requirements:**

4. Feedback from Reddit informs Epic 2 design decisions (captured in notes for Story 2.2-2.6)
5. OnShape concept serves as starting point for Story 2.1 (OnShape setup) and Story 2.2 (head housing CAD)
6. Community engagement builds early follower base for build-in-public strategy

**Quality Requirements:**

7. OnShape model is view-only public (no edit access needed)
8. Reddit post follows subreddit rules (checked before posting: self-promotion policies, image requirements)
9. Responses to community feedback are professional and appreciative (acknowledge suggestions, explain design rationale when needed)
10. Feedback summary documented in `docs/community-feedback/reddit-initial-design-feedback.md`

### Technical Notes

- **Integration Approach:**
  - OnShape concept is exploratory/low-fidelity - doesn't need full detail, just enough to communicate visual direction
  - Reddit post timing: After Epic 1 heart display working (have content showing progress), before committing to Epic 2 full design
  - Feedback collection window: 3-7 days to gather responses

- **Existing Pattern Reference:**
  - Similar to Epic 1 Story 1.11 build-in-public content (LinkedIn post with photo/explanation)
  - Extends strategy to include community input, not just one-way sharing

- **Key Constraints:**
  - Keep OnShape model simple (2-4 hours max design time) - this is concept validation, not final design
  - Avoid over-promising in Reddit post - be clear this is early-stage, feedback will be considered but not all suggestions implemented
  - Reddit engagement can be time-consuming - set boundary (e.g., respond to top 10-15 comments, summarize rest)

### Definition of Done

- [ ] OnShape concept model created and set to public
- [ ] Minimum 3 screenshots/renders exported
- [ ] Reddit post drafted and posted to appropriate subreddit(s)
- [ ] Post includes OnShape link, images, and specific feedback questions
- [ ] Community feedback monitored for 3-7 days
- [ ] Feedback summary documented in `docs/community-feedback/reddit-initial-design-feedback.md` including:
  - Common themes (e.g., "5 people mentioned head looks too large")
  - Actionable suggestions (e.g., "Consider wider base for stability")
  - Design decisions made (e.g., "Keeping current proportions because X, but will adjust ear angle based on feedback")
- [ ] Key insights incorporated into Epic 2 planning notes

### Risk and Compatibility Check

**Minimal Risk Assessment:**
- **Primary Risk:** Negative community feedback could be demotivating or suggest major design pivot
- **Mitigation:** Frame post as "early concept, iterative process" - manage expectations that this is exploratory. Remember: not all feedback must be implemented, use judgment to filter signal from noise.
- **Rollback:** If Reddit response is overwhelmingly negative or post doesn't gain traction, simply document feedback and proceed with Epic 2 based on original vision. Community input is valuable but not mandatory for project success.

**Compatibility Verification:**
- [ ] Reddit post doesn't conflict with existing design constraints (component dimensions from Epic 1)
- [ ] Feedback timeline doesn't block Epic 2 start (can proceed with Story 2.1 while gathering feedback)
- [ ] OnShape concept doesn't create unrealistic expectations (keep it simple, clearly communicate "concept" not "final")
- [ ] Community engagement time budget is reasonable (don't let Reddit consume more than 4-6 hours total including post creation and responses)

**Dependencies:** Epic 1 Story 1.11 (build-in-public content strategy established), optionally Story 1.9 (30-min test complete for credibility/progress photos)

**Estimated Effort:** 4-6 hours (2-3 hours OnShape concept, 1-2 hours Reddit post creation/image prep, 1-2 hours community engagement over following days)

---

## Story 1.11: Epic 1 Documentation & Content

**As a** future contributor or follower,
**I want** comprehensive Epic 1 documentation and build-in-public content,
**so that** I can replicate the setup and follow the journey.

### Acceptance Criteria:

1. **Technical Documentation:**
   - `firmware/head/README.md`:
     - Hardware connections (dual SPI displays + I2C pinout)
     - Firmware build/upload process
     - I2C register map reference
   - `ros2/src/orchestrator/ros2_nodes/README.md`:
     - How to run driver nodes
     - Topic reference
     - I2C communication flow

2. **Hardware Documentation:**
   - BOM: `hardware/bom/epic1_bom.csv`
     - ESP32-S3 or ESP32-WROOM, 2× GC9A01 displays, jumper wires, breadboard
     - Temporary power supplies (USB power banks or bench supply)
     - Supplier links, costs
   - Wiring diagrams in `hardware/wiring/`
   - Note: Full power system BOM will be in Epic 5 documentation

3. **Setup Scripts:**
   - `tools/setup/install_ros2_humble_pi.sh` tested on fresh Pi
   - README "Quick Start" section complete

4. **Photos/Videos:**
   - Photos: Power system, dual eye displays with expressions, full breadboard setup
   - Optional: 15-second video of eye expressions changing

5. **GitHub Milestone:**
   - Milestone "Epic 1: Foundation" created
   - All stories closed and linked
   - Milestone marked complete

6. **Build-in-Public Content:**
   - LinkedIn post: "Olaf's Eyes Come Alive! 👀🤖"
     - Photo of dual eye displays showing expressions
     - Explanation: I2C + SPI architecture, 60 FPS emotion-driven expressions on dual displays
     - Technical depth: Smart I2C slave pattern, Head Module validation
   - Post published

7. **Troubleshooting Guide:**
   - `docs/troubleshooting-epic1.md`:
     - "ESP32 not detected on I2C at 0x08" → solution
     - "One or both GC9A01 displays show garbage" → wiring check, verify chip select pins
     - "Eyes not synchronized" → check SPI timing, shared signal integrity
     - "ROS2 node can't open I2C" → permissions fix

**Dependencies:** All Epic 1 stories complete

**Estimated Effort:** 6-8 hours (docs + photos + content)

---

## Epic 1 Summary

**Total Stories:** 11
**Estimated Total Effort:** 32-47 hours (1.5 weeks for solo builder)

**Key Deliverables:**
- ✅ Repository structure with shared I2C register definitions
- ✅ ROS2 Humble on Raspberry Pi with I2C enabled
- ✅ **Head Module:** Dual Eye LCDs (2× GC9A01, 60 FPS expressions) at I2C address 0x08
- ✅ ROS2 head driver node translating topics → I2C registers
- ✅ Orchestrator sending coordinated expression commands
- ✅ Test harness for direct I2C module testing
- ✅ Launch file for automated system startup
- ✅ 30-minute continuous operation validated
- ✅ Complete documentation + build-in-public content
- ✅ Initial OnShape design concept + Reddit community feedback

**Power System Note:**
- Epic 1 uses temporary bench power (USB power banks, lab power supply, or Pi power adapter)
- Full power system (36V battery, BMS, buck converters, charging, safety) deferred to **Epic 5: Core Torso & Power System**

**Architecture Validated:**
- ✅ I2C-only communication (no WiFi on ESP32s)
- ✅ Dual SPI displays for 60 FPS smooth animation
- ✅ Smart I2C slave pattern (ESP32 has expression engine)
- ✅ ROS2 only on Pi (driver nodes as I2C bridges)
- ✅ Shared register protocol (foundation for all modules)
- ✅ Head Module (0x08) validated for Epic 3 integration

**Success Criteria Met:**
- Three-layer architecture proven (Orchestrator → I2C Driver → Smart ESP32)
- "Quick win" achieved: Olaf's eyes come alive with emotion-driven expressions!
- Head Module architecture validated before physical assembly in Epic 3
- Initial design concept validated with community feedback
- Foundation ready for Epic 2 (Complete OnShape Design & Community Feedback)

**Next Epic:** Epic 2: Complete OnShape Design & Community Feedback

---

**Related Documents:**
- [PRD](../prd/index.md) - FR1-FR7 validated
- [Architecture](../architecture/index.md) - I2C + SPI pattern proven
- [Epic List](epic-list.md) - Overall project roadmap
