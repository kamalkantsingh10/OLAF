# Epic 1: Foundation, Infrastructure & Minimal Personality

**Epic Goal:** Establish the complete development foundation including repository structure, ROS2 environment, power distribution system (hoverboard battery + buck converters), and basic ESP32-to-Raspberry Pi communication. Deliver a minimal working personality—a single OLED eye that can blink and display one simple emotion (neutral), plus a test beep—proving the three-layer architecture (Intelligence → Orchestration → Module) works before building complex features or mechanical structures.

**Duration:** 2-3 weeks (Weeks 1-2)

**Prerequisites:** None (first epic)

**Value Delivered:** Quick win proving architecture viability, enables all future development, provides "Olaf blinks at you!" moment within first 2-3 weekends.

---

## Story 1.1: Repository Setup & Project Structure

**As a** developer building Olaf,
**I want** a well-organized monorepo with clear structure and documentation,
**so that** I can clone the project, understand the layout, and start contributing immediately.

### Acceptance Criteria:

1. Repository created on GitHub with name `olaf` under appropriate account
2. Directory structure matches Technical Assumptions specification:
   ```
   olaf/
   ├── docs/
   ├── hardware/
   ├── modules/
   ├── orchestrator/
   ├── tests/
   ├── tools/
   ├── config/
   └── README.md
   ```
3. README.md contains:
   - Project overview (personality-first AI companion)
   - Architecture diagram (Intelligence → Orchestration → Module layers)
   - Quick start instructions (clone, dependencies, setup)
   - Link to PRD (`docs/prd.md`)
   - Link to branding guide (`docs/branding-guide.md`)
   - MIT License badge
4. `.gitignore` configured to exclude:
   - Python `__pycache__/`, `*.pyc`
   - ROS2 build artifacts (`build/`, `install/`, `log/`)
   - Environment files (`.env`)
   - IDE configs (`.vscode/`, `.idea/`)
5. `config/api-keys.template.env` created with placeholder variables:
   ```
   ANTHROPIC_API_KEY=your_claude_api_key_here
   WEATHER_API_KEY=your_weather_api_key_here
   ```
6. Initial commit pushed with message: "Initial repository structure and project documentation"
7. GitHub repository description set: "Open-source modular AI companion robot with personality-first design"
8. Topics tagged: `robotics`, `ros2`, `ai`, `open-source`, `esp32`, `physical-ai`

**Dependencies:** None

**Estimated Effort:** 2-4 hours

---

## Story 1.2: Power System Assembly & Validation

**As a** robot builder,
**I want** a reliable power distribution system using salvaged hoverboard components,
**so that** all modules receive stable voltage and I can safely charge the battery.

### Acceptance Criteria:

1. **Battery Selection:**
   - Hoverboard battery salvaged (36V nominal, 4-10Ah capacity documented)
   - Battery voltage measured: 36-42V (confirms healthy cells)
   - BMS (Battery Management System) circuit board identified and retained

2. **Buck Converter Installation:**
   - 36V → 12V buck converter (10A) installed and secured
   - 36V → 5V buck converter (10A) installed and secured
   - Converter output voltages measured under no-load: 12V ±0.2V, 5V ±0.1V

3. **Load Testing:**
   - 12V rail tested with 3A load (Raspberry Pi equivalent): voltage stable 12V ±0.3V
   - 5V rail tested with 2A load (servos + ESP32s): voltage stable 5V ±0.2V
   - Both converters loaded simultaneously: no voltage sag >5%

4. **Battery Monitoring:**
   - Voltage divider circuit connected to ADC input (Raspberry Pi GPIO or ESP32)
   - ADC reads battery voltage accurately (±0.5V of multimeter measurement)
   - Low-battery threshold set at 32V (≈20% capacity)

5. **Charging Circuit:**
   - Hoverboard charging port retained and accessible
   - Standard 42V hoverboard charger connects successfully
   - Charging tested: Battery voltage rises from 36V → 42V over 2-4 hours
   - BMS balances cells (verified by no error LEDs on BMS board)

6. **Safety:**
   - Fuse or circuit breaker installed on battery main output (30-40A rating)
   - Power switch (physical toggle) installed for emergency cutoff
   - Wiring uses appropriate gauge: 18-20 AWG for power rails, 22-24 AWG for signals
   - Connectors secure (JST-XH or screw terminals, no loose Dupont on power)

7. **Documentation:**
   - Wiring diagram created (Fritzing or hand-drawn, photographed)
   - BOM updated with battery source, buck converters, connectors
   - Power system photo uploaded to `hardware/wiring/power-system.jpg`

**Dependencies:** Story 1.1 (repo to commit documentation)

**Estimated Effort:** 6-10 hours (includes salvaging hoverboard, wiring, testing)

---

## Story 1.3: Raspberry Pi ROS2 Setup

**As a** robotics developer,
**I want** ROS2 Humble installed and configured on Raspberry Pi 8GB,
**so that** I can run the orchestration layer and communicate with ESP32 modules via micro-ROS.

### Acceptance Criteria:

1. **OS Installation:**
   - Raspberry Pi OS (64-bit, Debian Bookworm-based) flashed to SD card (minimum 32GB)
   - Pi boots successfully, SSH enabled (or direct monitor/keyboard access)
   - WiFi configured (2.4GHz network for ESP32 compatibility)
   - Static IP assigned (optional but recommended for development)

2. **ROS2 Installation:**
   - ROS2 Humble installed following official docs (apt packages)
   - Environment sourced: `source /opt/ros/humble/setup.bash` in `.bashrc`
   - Verification: `ros2 --version` returns "ros2 doctor: humble"

3. **micro-ROS Agent:**
   - `micro_ros_agent` package installed via apt or built from source
   - Agent launches successfully: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
   - No errors in agent terminal output

4. **Development Tools:**
   - Python 3.10+ installed (comes with Raspberry Pi OS)
   - `pip` installed, virtual environment created: `python3 -m venv ~/olaf_env`
   - ROS2 Python bindings available: `import rclpy` works in Python

5. **Network Configuration:**
   - Raspberry Pi and development laptop on same network
   - Ping successful between Pi and laptop
   - ROS2 discovery working: `ros2 topic list` shows topics from laptop

6. **Repository Clone:**
   - Repository cloned to Raspberry Pi: `git clone <repo_url> ~/olaf`
   - Directory structure visible: `ls ~/olaf` shows all folders

7. **Documentation:**
   - Setup script created: `tools/setup/install_ros2_humble.sh`
   - README updated with ROS2 setup instructions
   - Troubleshooting notes added for common issues (WiFi config, permissions)

**Dependencies:** Story 1.1 (repo), Story 1.2 (power to boot Pi)

**Estimated Effort:** 4-6 hours (includes learning ROS2 if new)

---

## Story 1.4: Minimal Head Module - Hardware Assembly

**As a** hardware builder,
**I want** a minimal head module with one OLED eye and ESP32,
**so that** I can test visual feedback before building the complete dual-eye system.

### Acceptance Criteria:

1. **Components Acquired:**
   - 1x ESP32 development board (DevKit v1 or similar)
   - 1x OLED display (128x64 pixels, I2C interface, SSD1306 driver)
   - Jumper wires (I2C: SDA, SCL, VCC, GND)
   - Breadboard or prototype PCB for temporary assembly

2. **Wiring:**
   - OLED connected to ESP32 via I2C:
     - OLED VCC → ESP32 3.3V (or 5V rail from buck converter if OLED supports it)
     - OLED GND → ESP32 GND
     - OLED SDA → ESP32 GPIO21 (default I2C SDA)
     - OLED SCL → ESP32 GPIO22 (default I2C SCL)
   - Power connection: ESP32 VIN ← 5V from buck converter (or USB for initial testing)

3. **I2C Communication Test:**
   - Upload I2C scanner sketch to ESP32
   - OLED detected at address 0x3C or 0x3D (confirms wiring)
   - Serial monitor shows "I2C device found at address 0x3C"

4. **Basic Display Test:**
   - Upload Adafruit SSD1306 example sketch (Hello World)
   - OLED displays text "Hello Olaf!"
   - Screen updates without flickering

5. **Physical Mounting (Temporary):**
   - Components secured to breadboard or cardboard base (prevents shorts)
   - Cables strain-relieved (won't pull out during testing)
   - Module can sit on desk without tipping over

6. **Documentation:**
   - Wiring photo uploaded: `hardware/wiring/head-module-minimal.jpg`
   - Component list added to BOM: ESP32, OLED, wires
   - Wiring notes documented in `modules/head/README.md`

**Dependencies:** Story 1.2 (5V power available)

**Estimated Effort:** 2-4 hours

---

## Story 1.5: Minimal Head Module - Firmware (Eye Blink)

**As a** firmware developer,
**I want** ESP32 firmware that displays a neutral expression and blinks on command,
**so that** I can validate the orchestrator → module communication pipeline.

### Acceptance Criteria:

1. **Development Environment:**
   - PlatformIO or Arduino IDE configured for ESP32
   - Libraries installed:
     - `Adafruit_SSD1306` (OLED driver)
     - `Adafruit_GFX` (graphics primitives)
     - `micro_ros_arduino` (ROS2 node on ESP32)

2. **Neutral Expression Graphics:**
   - Neutral eye sprite created (simple circle with dot pupil, or peaceful closed curve)
   - Sprite stored as bitmap array in firmware: `const uint8_t neutral_eye[]`
   - Sprite renders correctly on OLED (centered, appropriate size)

3. **Blink Animation:**
   - Blink sequence defined:
     - Frame 1: Full open (neutral eye)
     - Frame 2: Half closed (top eyelid lowered)
     - Frame 3: Fully closed (horizontal line)
     - Frame 4: Half closed (opening)
     - Frame 5: Full open
   - Blink duration: 300-500ms total (fast enough to feel natural)
   - Animation plays smoothly (no visible tearing)

4. **micro-ROS Integration:**
   - ESP32 connects to micro-ROS agent running on Raspberry Pi (UDP port 8888)
   - ROS2 node created: `/olaf/head` with namespace
   - Subscribes to topic: `/olaf/head/eye_cmd` (message type: `std_msgs/String` or custom)
   - On message "blink" received → triggers blink animation

5. **Standalone Testing:**
   - Firmware uploaded to ESP32
   - ESP32 connects to WiFi (hardcoded SSID/password for now)
   - micro-ROS agent running on Pi shows: "Node '/olaf/head' connected"
   - Command line test: `ros2 topic pub /olaf/head/eye_cmd std_msgs/String "data: 'blink'"` → eye blinks

6. **Continuous Operation:**
   - Module runs continuously for 30 minutes without crashing
   - Can blink on command multiple times (tested 10x)
   - Memory leak check: ESP32 free heap stable over time

7. **Code Quality:**
   - Code committed to `modules/head/firmware/`
   - README documents build/upload process
   - Pin definitions clearly commented (I2C pins, power)

**Dependencies:** Story 1.3 (ROS2 + micro-ROS agent), Story 1.4 (hardware assembled)

**Estimated Effort:** 6-8 hours (includes micro-ROS learning curve)

---

## Story 1.6: Minimal Audio - Beeper Hardware & Test Tone

**As a** hardware builder,
**I want** a simple beeper/speaker that can play test tones,
**so that** I can validate audio output before implementing the full sound library.

### Acceptance Criteria:

1. **Component Selection:**
   - Passive piezo buzzer (simplest, works with PWM) OR
   - Small speaker (3W, 8Ω) + audio amplifier module (PAM8403 or similar)
   - Choice documented with rationale (piezo = simpler, speaker = better quality)

2. **Wiring:**
   - If piezo: Connect to ESP32 GPIO pin (e.g., GPIO25) + GND
   - If speaker: Amplifier powered from 5V rail, audio input from ESP32 DAC or PWM pin
   - Wiring diagram updated in `hardware/wiring/audio-system.jpg`

3. **Test Tone Firmware:**
   - Simple Arduino sketch uploaded to ESP32 (can reuse head module ESP32 or separate one)
   - Generates 440Hz tone (A4 note) via PWM or DAC
   - Plays tone for 500ms when triggered

4. **ROS2 Integration:**
   - ESP32 subscribes to `/olaf/audio/beep_cmd` topic
   - Message format: `std_msgs/Int16` (frequency in Hz) or `std_msgs/String` ("beep")
   - On message received → plays tone at specified frequency (or default 440Hz)

5. **Testing:**
   - Command: `ros2 topic pub /olaf/audio/beep_cmd std_msgs/String "data: 'beep'"` → beep plays
   - Adjustable volume (if speaker): potentiometer on amplifier or PWM duty cycle
   - No distortion or crackling at moderate volume

6. **Integration with Head Module (Optional for Story, Required Later):**
   - If same ESP32 as head: Both eye and beep commands work independently
   - If separate ESP32: Both nodes visible in `ros2 node list`

7. **Documentation:**
   - Component added to BOM
   - Audio wiring documented
   - Firmware committed to `modules/body/firmware/` or `modules/head/firmware/` (depending on location)

**Dependencies:** Story 1.2 (power), Story 1.3 (ROS2)

**Estimated Effort:** 3-5 hours

---

## Story 1.7: Basic Orchestrator - Blink & Beep Coordinator

**As a** software developer,
**I want** a minimal orchestrator node that sends coordinated blink + beep commands,
**so that** I can prove the three-layer architecture (orchestrator → modules) works.

### Acceptance Criteria:

1. **Orchestrator Setup:**
   - Python ROS2 node created: `orchestrator/minimal_demo.py`
   - Node name: `/olaf/orchestrator`
   - Imports: `rclpy`, `std_msgs.msg`

2. **Publishers Created:**
   - Publisher to `/olaf/head/eye_cmd` (for eye blink)
   - Publisher to `/olaf/audio/beep_cmd` (for beep)

3. **Coordinated Action Function:**
   - Function `blink_and_beep()` implemented:
     ```python
     def blink_and_beep(self):
         # Publish eye blink command
         self.eye_pub.publish(String(data='blink'))
         # Publish beep command (simultaneously or with small delay)
         self.beep_pub.publish(String(data='beep'))
     ```

4. **Test Interface:**
   - Timer-based trigger: Every 5 seconds, call `blink_and_beep()` (for continuous demo)
   - OR service call: Create ROS2 service `/olaf/test_blink_beep` that triggers action on request

5. **Execution:**
   - Launch orchestrator: `ros2 run olaf_orchestrator minimal_demo`
   - Verify in `ros2 node list`: `/olaf/orchestrator` appears
   - Verify publishers: `ros2 topic info /olaf/head/eye_cmd` shows orchestrator as publisher

6. **End-to-End Test:**
   - All three nodes running: micro-ROS agent + head module + audio module + orchestrator
   - Orchestrator triggers blink_and_beep
   - **Expected behavior:** Eye blinks AND beep plays within <500ms of each other
   - Latency measured (rough estimate via stopwatch or logs): <500ms between commands

7. **Logging:**
   - Orchestrator logs actions: `self.get_logger().info("Triggered blink and beep")`
   - Logs visible in terminal during execution

8. **Code Quality:**
   - Code committed to `orchestrator/minimal_demo.py`
   - Launch file created: `orchestrator/launch/minimal_demo.launch.py` (optional but nice)
   - README updated with orchestrator run instructions

**Dependencies:** Story 1.5 (eye blink firmware), Story 1.6 (beep firmware)

**Estimated Effort:** 4-6 hours

---

## Story 1.8: Test Harness CLI - Manual Module Testing

**As a** developer or tester,
**I want** a command-line tool to manually test individual modules,
**so that** I can debug issues without running the full orchestrator.

### Acceptance Criteria:

1. **CLI Tool Created:**
   - Python script: `tools/diagnostics/olaf-test`
   - Shebang line: `#!/usr/bin/env python3`
   - Executable permission: `chmod +x tools/diagnostics/olaf-test`

2. **Commands Implemented:**
   - `olaf-test blink` → Publishes blink command to `/olaf/head/eye_cmd`
   - `olaf-test beep` → Publishes beep command to `/olaf/audio/beep_cmd`
   - `olaf-test list` → Lists all active ROS2 nodes (shows module connectivity)

3. **ROS2 Integration:**
   - Script uses `rclpy` to publish messages
   - Creates temporary node: `/olaf/test_harness`
   - Publishes message, waits 1 second, shuts down cleanly

4. **Usage:**
   - From terminal: `./tools/diagnostics/olaf-test blink` → eye blinks
   - From terminal: `./tools/diagnostics/olaf-test beep` → beep plays
   - Help message: `./tools/diagnostics/olaf-test --help` shows available commands

5. **Error Handling:**
   - If ROS2 not running: Clear error message "ROS2 not initialized. Is ros2 daemon running?"
   - If invalid command: "Unknown command. Use --help for usage."

6. **Documentation:**
   - Tool usage documented in `tools/diagnostics/README.md`
   - Main README links to test harness documentation

**Dependencies:** Story 1.5 (modules to test), Story 1.7 (orchestrator as reference)

**Estimated Effort:** 3-4 hours

---

## Story 1.9: Battery Monitoring & Low-Battery Warning

**As a** robot operator,
**I want** visual and audio alerts when battery is low,
**so that** I can recharge before unexpected shutdown damages the battery or loses work.

### Acceptance Criteria:

1. **Voltage Sensing:**
   - ADC input reads battery voltage via voltage divider (36V → 3.3V range)
   - Voltage reading published to ROS2 topic: `/olaf/battery/voltage` (Float32, in volts)
   - Reading accuracy: ±0.5V compared to multimeter

2. **Battery Percentage Calculation:**
   - Formula: `percentage = (V_current - 30) / (42 - 30) * 100`
   - 42V = 100%, 36V ≈ 50%, 30V = 0%
   - Percentage published to `/olaf/battery/percentage` (Int16, 0-100)

3. **Low-Battery Threshold:**
   - Threshold: 32V (≈20% capacity)
   - When voltage drops below 32V: Low-battery warning triggered

4. **Warning Indicators:**
   - **Visual:** LED on body (if available) blinks red, OR eye displays warning symbol
   - **Audio:** Three beeps (440Hz, 200ms each, 200ms gaps) played immediately
   - **Log:** ROS2 logger warning: "Low battery! 20% remaining. Please recharge."

5. **Persistent Warning:**
   - Warning repeats every 60 seconds until battery recharged or system shutdown
   - Prevents battery over-discharge (30V cutoff protection via BMS)

6. **Testing:**
   - Simulate low battery: Manually lower voltage (adjust voltage divider temporarily) or drain battery to 32V
   - Verify warning triggers at correct voltage
   - Verify warning repeats until voltage rises above threshold

7. **Integration:**
   - Battery monitoring node runs automatically on orchestrator (or dedicated ESP32)
   - Added to system startup (launch file or systemd service for future)

8. **Documentation:**
   - Battery monitoring code committed to `orchestrator/battery_monitor.py` or `modules/body/firmware/`
   - Voltage divider calculation documented with schematic

**Dependencies:** Story 1.2 (battery + monitoring circuit), Story 1.5 or 1.6 (LED or beep for warning)

**Estimated Effort:** 4-5 hours

---

## Story 1.10: 30-Minute Continuous Operation Test

**As a** quality assurance tester,
**I want** to run Olaf continuously for 30 minutes without failures,
**so that** I can validate system stability before moving to Epic 2.

### Acceptance Criteria:

1. **Test Setup:**
   - All modules running: Raspberry Pi orchestrator + head module (eye) + audio module (beep)
   - Battery fully charged (42V, 100%)
   - Test environment: Room temperature, stable WiFi

2. **Test Procedure:**
   - Start orchestrator with continuous blink + beep (every 5 seconds)
   - Monitor for 30 minutes (set timer)
   - Observe: terminal logs, eye behavior, beep sounds, battery voltage

3. **Success Criteria:**
   - **No crashes:** All ROS2 nodes remain active for full 30 minutes
   - **No missed commands:** Eye blinks every 5 seconds (360 blinks total), beeps every 5 seconds (360 beeps)
   - **Stable latency:** Blink + beep coordination remains <500ms throughout test
   - **Memory stable:** ESP32 free heap doesn't decrease (check at start, 15min, 30min)
   - **Battery drain:** Voltage drops from 42V → 40-41V (≈2-5% drain expected for minimal load)

4. **Failure Handling:**
   - If crash occurs: Log error, investigate cause, fix bug, restart test
   - If commands missed: Check ROS2 topic echo, verify network stability
   - If latency degrades: Profile code, optimize bottlenecks

5. **Metrics Collected:**
   - Total runtime: 30:00 minutes
   - Total blinks: ~360 (verify via log count)
   - Total beeps: ~360 (verify via log count)
   - Battery voltage at start: X.XX V
   - Battery voltage at end: X.XX V
   - Average latency: <500ms (sampled 10 times during test)

6. **Pass/Fail:**
   - **PASS:** All success criteria met, no crashes
   - **FAIL:** Any crash, >5% missed commands, latency >500ms for >10% of samples

7. **Documentation:**
   - Test results logged in `tests/integration/epic1_30min_test_results.md`
   - Pass/fail status, metrics, any issues encountered
   - Screenshot or video of final terminal output (proof of 30min runtime)

**Dependencies:** All Epic 1 stories (1.1-1.9)

**Estimated Effort:** 1 hour (test execution) + 2-4 hours (bug fixes if needed)

---

## Story 1.11: Epic 1 Documentation & Handoff

**As a** future contributor or builder,
**I want** comprehensive documentation of Epic 1 work,
**so that** I can understand the foundation and replicate the setup.

### Acceptance Criteria:

1. **README Updates:**
   - Main `README.md` updated with:
     - Quick start guide (clone → setup → run minimal demo)
     - Link to Epic 1 completion status
     - Photos of working system (eye blinking, breadboard setup)

2. **Hardware Documentation:**
   - `hardware/bom/epic1_bom.csv` created with all components:
     - Hoverboard battery, buck converters, ESP32, OLED, wires, connectors
     - Supplier links (AliExpress, Amazon)
     - Costs (actual prices paid)
   - `hardware/wiring/` contains:
     - Power system diagram
     - Head module wiring photo
     - Audio wiring photo

3. **Software Documentation:**
   - `modules/head/README.md` explains:
     - Hardware connections
     - Firmware build/upload process
     - How to test standalone
   - `orchestrator/README.md` explains:
     - How to run orchestrator
     - ROS2 topics used
     - Coordination logic

4. **Setup Scripts:**
   - `tools/setup/install_ros2_humble.sh` tested on fresh Raspberry Pi
   - `tools/setup/configure_wifi.sh` (optional) for easy network setup

5. **Troubleshooting Guide:**
   - `docs/troubleshooting.md` created with common issues:
     - "ESP32 won't connect to WiFi" → solution
     - "OLED shows garbage" → I2C address check
     - "Beep doesn't play" → wiring/pin check
     - "ROS2 topics not visible" → network discovery fix

6. **Photos/Videos:**
   - At least 3 photos:
     - Power system assembled
     - Head module (ESP32 + OLED) working
     - Full Epic 1 setup on desk
   - Optional: 30-second video of blink + beep demo

7. **GitHub Milestone:**
   - GitHub milestone "Epic 1: Foundation" created
   - All Epic 1 stories closed, linked to milestone
   - Milestone marked complete

8. **Content Creation:**
   - LinkedIn post drafted (see `.content/epic-content-plan.md` Week 2 Post #3)
   - "Olaf's First Blink!" with photo/video, explanation of power system
   - Post published or scheduled

**Dependencies:** All Epic 1 stories complete (1.1-1.10)

**Estimated Effort:** 4-6 hours (documentation writing, photo editing, content creation)

---

## Epic 1 Summary

**Total Stories:** 11
**Estimated Total Effort:** 45-60 hours (2-3 weeks for solo builder working weekends)

**Key Deliverables:**
- ✅ Repository structure established
- ✅ Power system operational (hoverboard battery + buck converters)
- ✅ ROS2 Humble running on Raspberry Pi
- ✅ Minimal head module: 1 eye blinks on command
- ✅ Audio module: Beeper plays test tone
- ✅ Orchestrator coordinates eye + beep
- ✅ Test harness for manual module testing
- ✅ Battery monitoring with low-battery warnings
- ✅ 30-minute continuous operation validated
- ✅ Complete documentation for replication

**Success Criteria Met:**
- Three-layer architecture proven (Intelligence → Orchestration → Module)
- ROS2 + micro-ROS communication working
- Power system stable and safe
- "Quick win" achieved: Olaf blinks at you!
- Foundation ready for Epic 2 (3D design) and Epic 3 (full personality)

**Next Epic:** Epic 2 - 3D Design & Physical Structure

---

**Related Documents:**
- [PRD](../prd.md)
- [Epic Content Plan](../../.content/epic-content-plan.md)
- [Epic 2: 3D Design & Physical Structure](epic-02-3d-design.md)
