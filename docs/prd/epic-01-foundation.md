# Epic 1: Foundation, Infrastructure & Minimal Personality

**Epic Goal:** Establish the complete development foundation including repository structure, ROS2 environment on Raspberry Pi, I2C communication protocol, and power distribution. Deliver a minimal working personality—a single SPI-driven OLED eye that can display animations and blink via I2C commands from the orchestrator—proving the three-layer architecture (Orchestrator → I2C → Smart ESP32 Peripheral) works end-to-end.

**Duration:** 2-3 weeks (Weeks 1-3)

**Prerequisites:** None (first epic)

**Value Delivered:** Quick win proving I2C + SPI architecture viability, establishes shared register protocol, enables all future module development, provides "Olaf blinks at you!" moment within first 2-3 weekends.

**Architecture Focus:** This epic validates the core architectural decision that ROS2 runs ONLY on Pi, with ESP32 modules acting as smart I2C slaves containing full hardware drivers and animation engines.

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
   │   ├── prd.md
   │   ├── architecture.md
   │   ├── brief.md
   │   └── epics/
   ├── hardware/                   # 3D models, wiring, BOM
   │   ├── 3d-models/
   │   ├── wiring/
   │   └── bom/
   ├── modules/                    # ESP32 firmware (C++)
   │   ├── head/
   │   │   └── firmware/
   │   ├── ears/
   │   ├── neck/
   │   ├── projector/
   │   └── base/
   ├── orchestrator/               # Raspberry Pi Python + ROS2
   │   ├── ros2_nodes/
   │   │   └── hardware_drivers/
   │   ├── launch/
   │   └── config/
   ├── shared/                     # Common headers/definitions
   │   └── i2c_registers.h         # Shared I2C register map
   ├── tests/
   ├── tools/
   │   ├── setup/
   │   └── diagnostics/
   ├── config/
   │   └── api-keys.template.env
   ├── .gitignore
   ├── README.md
   └── LICENSE
   ```
3. README.md contains (from updated README):
   - Project overview (self-balancing AI companion)
   - Architecture diagram showing I2C-based communication
   - Quick start instructions
   - Links to PRD, architecture docs, epics
4. `.gitignore` configured to exclude:
   - Python: `__pycache__/`, `*.pyc`, `venv/`
   - ROS2: `build/`, `install/`, `log/`
   - PlatformIO: `.pio/`, `.vscode/`
   - Environment: `.env`
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
6. `config/api-keys.template.env` created with placeholder for Claude API (future use)
7. MIT License file added
8. Initial commit pushed: "Initial repository structure with I2C architecture foundation"

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
   - 5V rail tested with 2A load (simulates ESP32 + servo): voltage stable
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
   - Power distribution schematic documented

**Note:** ODrive will handle battery monitoring in future epics, so no ADC voltage sensing needed in Epic 1.

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
   - WiFi configured for cloud API access (Pi only, NOT for ESP32 communication)
   - Static IP optional but recommended

2. **ROS2 Installation:**
   - ROS2 Humble installed: `sudo apt install ros-humble-desktop`
   - Environment sourced: `source /opt/ros/humble/setup.bash` added to `.bashrc`
   - Verification: `ros2 --version` shows "humble"

3. **I2C Configuration:**
   - I2C enabled via `raspi-config`: Interface Options → I2C → Enable
   - I2C device visible: `ls /dev/i2c-*` shows `/dev/i2c-1`
   - I2C tools installed: `sudo apt install i2c-tools`
   - I2C permissions: User added to `i2c` group: `sudo usermod -a -G i2c $USER`

4. **Python I2C Library:**
   - `smbus2` installed: `pip3 install smbus2`
   - Test script can open I2C bus:
     ```python
     from smbus2 import SMBus
     bus = SMBus(1)  # I2C bus 1
     print("I2C bus opened successfully")
     ```

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
   - I2C troubleshooting notes added

**Dependencies:** Story 1.1 (repo), Story 1.2 (power to boot Pi)

**Estimated Effort:** 3-4 hours

---

## Story 1.4: Minimal Head Module - Hardware Assembly (SPI OLED)

**As a** hardware builder,
**I want** a minimal head module with ONE SPI-driven OLED and ESP32,
**so that** I can achieve 30-60 FPS animation performance before building dual-eye system.

### Acceptance Criteria:

1. **Components Acquired:**
   - 1× ESP32-DevKitC (ESP32-WROOM-32)
   - 1× OLED display (128×64, SSD1306, **SPI interface** - NOT I2C)
   - Jumper wires for SPI + I2C connections
   - Breadboard for prototyping

2. **SPI OLED Wiring (Pi → ESP32 → OLED):**
   - **OLED to ESP32 (SPI):**
     - OLED VCC → 3.3V (ESP32)
     - OLED GND → GND
     - OLED SCK (CLK) → GPIO18 (ESP32 SPI SCK)
     - OLED MOSI (SDA) → GPIO23 (ESP32 SPI MOSI)
     - OLED RES (RST) → GPIO4 (ESP32 reset pin)
     - OLED DC → GPIO2 (ESP32 data/command pin)
     - OLED CS → GPIO5 (ESP32 chip select)

3. **I2C Wiring (Pi → ESP32 for commands):**
   - **Pi to ESP32 (I2C):**
     - Pi GPIO2 (SDA) → ESP32 GPIO21 (I2C SDA)
     - Pi GPIO3 (SCL) → ESP32 GPIO22 (I2C SCL)
     - Pi GND → ESP32 GND (common ground critical!)
     - Pi 5V → ESP32 VIN (power from buck converter)

4. **I2C Address Assignment:**
   - ESP32 Head module configured as I2C slave at address **0x08** (per architecture)

5. **SPI Communication Test:**
   - Upload Adafruit SSD1306 SPI example to ESP32
   - OLED displays "Hello Olaf!" at 30+ FPS (smooth, no flicker)
   - Verify SPI speed: 10-20 MHz clock (measure with oscilloscope or trust library default)

6. **I2C Detection Test:**
   - From Pi: `sudo i2cdetect -y 1` shows device at address `0x08`
   - Confirms ESP32 is visible as I2C slave

7. **Physical Mounting:**
   - Components secured to breadboard (no loose wiring)
   - Module stable on desk during testing

8. **Documentation:**
   - Wiring photos: `hardware/wiring/head-module-minimal-spi.jpg`
   - Pin mapping documented in `modules/head/README.md`
   - BOM updated with ESP32, SPI OLED, wires

**Dependencies:** Story 1.2 (5V power), Story 1.3 (I2C enabled on Pi)

**Estimated Effort:** 3-4 hours

---

## Story 1.5: Body Module - Hardware Assembly (Heart LCD + Projector Control + LEDs)

**As a** hardware builder,
**I want** a body module with heart LCD display, projector power control, and status LEDs,
**so that** Olaf can express emotions through heartbeat animation and control body-mounted indicators.

### Acceptance Criteria:

1. **Components Acquired:**
   - 1× ESP32-S3-WROOM-2 (N8R8) or ESP32-WROOM-32
   - 1× GC9A01 Round TFT Display (1.28", 240×240, SPI interface) - Heart display
   - 1× Relay or MOSFET module (for 12V projector power switching)
   - 1× WS2812B RGB LED strip (addressable, 5-10 LEDs for status indicators)
   - Jumper wires for SPI + I2C + GPIO connections
   - Breadboard for prototyping

2. **SPI Heart Display Wiring (ESP32 → GC9A01):**
   - **Heart LCD to ESP32 (SPI):**
     - VCC → 3.3V (ESP32)
     - GND → GND
     - SCL (CLK) → GPIO18 (ESP32 SPI SCK)
     - SDA (MOSI) → GPIO23 (ESP32 SPI MOSI)
     - RES (RST) → GPIO4 (ESP32 reset pin)
     - DC → GPIO2 (ESP32 data/command pin)
     - CS → GPIO5 (ESP32 chip select)

3. **I2C Wiring (Pi → ESP32 for commands):**
   - **Pi to ESP32 (I2C):**
     - Pi GPIO2 (SDA) → ESP32 GPIO21 (I2C SDA)
     - Pi GPIO3 (SCL) → ESP32 GPIO22 (I2C SCL)
     - Pi GND → ESP32 GND (common ground)
     - Pi 5V → ESP32 VIN (power from buck converter)

4. **Projector Power Control Wiring:**
   - **Relay/MOSFET:**
     - ESP32 GPIO (e.g., GPIO15) → Relay signal pin
     - Relay COM → 12V power supply (from buck converter)
     - Relay NO (normally open) → Projector 12V input
     - Projector GND → Power supply GND

5. **LED Strip Wiring:**
   - **WS2812B:**
     - 5V → 5V power rail (buck converter)
     - GND → Common ground
     - DIN (data in) → ESP32 GPIO (e.g., GPIO16)

6. **I2C Address Assignment:**
   - ESP32 Body module configured as I2C slave at address **0x0A**

7. **Communication Tests:**
   - **SPI test:** Upload GC9A01 library example, heart shape displays at 60 FPS
   - **I2C test:** `sudo i2cdetect -y 1` shows device at address `0x0A`
   - **Relay test:** GPIO high/low toggles projector power (measure with multimeter)
   - **LED test:** WS2812B strip displays test pattern (rainbow cycle)

8. **Physical Mounting:**
   - Module secured to breadboard or temporary body mock-up
   - Heart display positioned on "chest" area (front center of body)
   - LED strip positioned as status indicator ring or accent lighting

9. **Documentation:**
   - Wiring photos: `hardware/wiring/body-module-assembly.jpg`
   - Pin mapping documented in `modules/body/README.md`
   - BOM updated with all body module components

**Dependencies:** Story 1.2 (5V power), Story 1.3 (I2C enabled on Pi)

**Estimated Effort:** 4-5 hours

---

## Story 1.6: Body Module ESP32 Firmware - Heart Animation + Projector + LEDs

**As a** firmware developer,
**I want** ESP32 firmware managing heart animation, projector power control, and LED patterns,
**so that** Olaf's body indicators respond to orchestrator commands and personality states.

### Acceptance Criteria:

1. **Development Environment:**
   - PlatformIO project: `modules/body/firmware/`
   - Libraries installed:
     - `TFT_eSPI` or `Adafruit_GC9A01A` (heart display)
     - `FastLED` or `Adafruit_NeoPixel` (WS2812B LEDs)
     - `Wire.h` (built-in, I2C slave)

2. **I2C Slave Implementation:**
   - ESP32 configured as I2C slave at address **0x0A**
   - I2C receive/request handlers implemented (same pattern as head module)

3. **I2C Register Map (Body Module 0x0A):**
   - **Common Registers:**
     - `0x00`: Module ID (returns 0x0A)
     - `0x02`: Status byte (READY/BUSY/ERROR)
   - **Heart Display (0x10-0x1F):**
     - `0x10`: Emotion type (maps to heart rate: 0=neutral, 1=happy, etc.)
     - `0x11`: Emotion intensity (1-5, affects beat amplitude)
     - `0x12`: Heart rate override (BPM, 0=auto from emotion)
     - `0x13`: Heart color (R component, 0-255)
     - `0x14`: Heart color (G component, 0-255)
     - `0x15`: Heart color (B component, 0-255)
   - **Projector Control (0x20-0x2F):**
     - `0x20`: Projector power (0=OFF, 1=ON)
     - `0x21`: Projector status (read-only: 0=OFF, 1=ON, 2=ERROR)
   - **LED Control (0x30-0x4F):**
     - `0x30`: LED mode (0=OFF, 1=solid, 2=breathing, 3=rainbow, 4=pulse)
     - `0x31`: LED brightness (0-255)
     - `0x32`: LED color (R component)
     - `0x33`: LED color (G component)
     - `0x34`: LED color (B component)

4. **Heart Animation Engine:**
   - Heart sprite (240×240 circular display, centered heart shape)
   - Beating animation: Scale/pulse effect synchronized to heart rate
   - **Emotion-to-BPM mapping:**
     - Neutral: 60-70 BPM (calm, steady)
     - Happy: 80-90 BPM (upbeat)
     - Excited: 100-120 BPM (rapid)
     - Sad: 50-60 BPM (slow, heavy)
     - Curious: 70-80 BPM (alert)
     - Thinking: 65-75 BPM (focused)
     - Confused: 75-85 BPM (uncertain)
   - Intensity affects beat amplitude: 1=subtle (10% scale), 5=dramatic (50% scale)
   - Color variations based on emotion (red=default, blue=sad, yellow=happy, etc.)
   - 60 FPS smooth animation, no tearing or flicker

5. **Projector Power Control:**
   - GPIO control for relay/MOSFET
   - Write to register `0x20`: 0=turn off, 1=turn on
   - Status monitoring: Read register `0x21` returns current state
   - Graceful power sequencing (optional: delay before shutdown)

6. **LED Pattern Engine:**
   - Solid color mode: All LEDs same color
   - Breathing mode: Fade in/out (synchronized with heart if emotion active)
   - Rainbow mode: Color cycle animation
   - Pulse mode: Quick flash (notification indicator)
   - Pattern updates at 30+ Hz for smooth animations

7. **Testing:**
   - Firmware compiled and uploaded via USB
   - Serial monitor shows: "I2C Slave initialized at 0x0A"
   - Heart beats at default 70 BPM on startup (neutral emotion)
   - Module responsive to I2C commands (tested via `i2cset` from Pi)
   - Projector relay toggles correctly
   - LED strip displays all test patterns

8. **Performance:**
   - Heart animation: 60 FPS maintained
   - LED update rate: 30+ Hz
   - I2C response latency: <10ms
   - Stable memory: Free heap stable over 10 minutes

9. **Code Quality:**
   - Organized structure:
     - `main.cpp`: Setup, loop, I2C handlers
     - `heart_animation.cpp`: Heart rendering logic
     - `projector_control.cpp`: Relay control
     - `led_patterns.cpp`: WS2812B pattern engine
     - `i2c_slave.cpp`: Register map handling
   - Comments explain I2C protocol and register map
   - Code committed to `modules/body/firmware/`

**Dependencies:** Story 1.5 (body module hardware assembled)

**Estimated Effort:** 8-10 hours

---

## Story 1.7: Head Module ESP32 Firmware - I2C Slave + SPI Animation Engine

**As a** firmware developer,
**I want** ESP32 firmware that acts as I2C slave and renders animations on SPI OLED,
**so that** I can receive high-level commands from Pi and execute smooth eye animations locally.

**Note:** Original Story 1.5 - renumbered due to body module addition (Stories 1.5-1.6)

### Acceptance Criteria:

1. **Development Environment:**
   - PlatformIO installed in VS Code
   - Project created: `modules/head/firmware/` with `platformio.ini`
   - Libraries installed:
     - `Adafruit SSD1306` (SPI mode)
     - `Adafruit GFX Library`
     - `Wire.h` (built-in, I2C slave)

2. **I2C Slave Implementation:**
   - ESP32 configured as I2C slave at address **0x08**
   - I2C receive handler implemented:
     ```cpp
     void onReceive(int numBytes) {
       if (Wire.available()) {
         uint8_t reg = Wire.read();  // Register address
         uint8_t value = Wire.read(); // Value
         handleCommand(reg, value);
       }
     }
     ```
   - I2C request handler for status reads:
     ```cpp
     void onRequest() {
       Wire.write(module_status);  // Send status byte
     }
     ```

3. **I2C Register Map (Head Module 0x08):**
   - Implemented registers:
     - `0x00`: Module ID (returns 0x08)
     - `0x02`: Status byte (READY/BUSY/ERROR flags)
     - `0x04`: Command register (write to trigger action)
     - `0x10`: Expression type (0x00=neutral, 0x01=happy, etc.)
     - `0x11`: Expression intensity (1-5)
     - `0x12`: Blink trigger (write 1 to blink)

4. **Animation Engine:**
   - Neutral eye sprite created (simple circle or peaceful eye shape)
   - Blink animation frames stored in PROGMEM:
     - Frame 0: Open (100%)
     - Frame 1: 75% closed
     - Frame 2: 50% closed (mid-blink)
     - Frame 3: 75% closed (opening)
     - Frame 4: Open (100%)
   - Animation renders at 30+ FPS on SPI OLED
   - Smooth transitions (no tearing or flicker)

5. **Command Handling:**
   - Write to `REG_BLINK_TRIGGER` (0x12): Triggers 300ms blink animation
   - Animation plays asynchronously (doesn't block I2C)
   - After animation completes, status returns to READY

6. **Testing:**
   - Firmware compiled and uploaded via USB
   - Serial monitor shows: "I2C Slave initialized at 0x08"
   - Eye displays neutral expression on startup
   - Module responsive to I2C commands (tested via `i2cset` from Pi)

7. **Performance:**
   - Animation frame rate: 30-60 FPS (measured via frame counter)
   - I2C response latency: <10ms (command received → action started)
   - No memory leaks: Free heap stable over 10 minutes

8. **Code Quality:**
   - Code organized:
     - `main.cpp`: Setup, loop, I2C handlers
     - `animation_engine.cpp`: Frame rendering logic
     - `i2c_slave.cpp`: Register map handling
   - Comments explain I2C protocol and register map
   - Code committed to `modules/head/firmware/`

**Dependencies:** Story 1.4 (hardware assembled)

**Estimated Effort:** 8-10 hours (includes I2C slave learning curve)

---

## Story 1.8: ROS2 Head Driver Node - I2C Bridge

**As a** software developer,
**I want** a ROS2 node that translates ROS2 topics into I2C register writes,
**so that** the orchestrator can control the head module without knowing I2C details.

### Acceptance Criteria:

1. **Node Structure:**
   - Python ROS2 node created: `orchestrator/ros2_nodes/hardware_drivers/head_driver.py`
   - Node name: `/olaf/head_driver`
   - Uses `rclpy` and `smbus2`

2. **I2C Communication:**
   - Opens I2C bus 1: `bus = SMBus(1)`
   - Helper functions:
     ```python
     def write_register(self, reg, value):
         self.bus.write_byte_data(0x08, reg, value)

     def read_register(self, reg):
         return self.bus.read_byte_data(0x08, reg)
     ```

3. **ROS2 Subscriptions:**
   - Subscribes to `/olaf/head/blink` (std_msgs/Empty)
   - Callback writes to I2C register 0x12 (blink trigger)
   - Subscribes to `/olaf/head/expression` (custom msg or String)
   - Callback writes expression type to register 0x10

4. **ROS2 Publications:**
   - Publishes `/olaf/head/status` (std_msgs/String) at 10Hz
   - Reads status register 0x02 via I2C
   - Publishes "READY", "BUSY", or "ERROR" based on status byte

5. **Module Health Check:**
   - On startup: Reads register 0x00 (Module ID)
   - Verifies response is 0x08 (Head module)
   - If module not responding: Logs error, publishes ERROR status

6. **Error Handling:**
   - I2C timeout handling (if ESP32 not responding)
   - Retry logic: 3 attempts with 100ms delay
   - If persistent failure: Publishes ERROR and logs to ROS2 logger

7. **Testing:**
   - Node launches: `ros2 run olaf_drivers head_driver`
   - Verify in topic list: `/olaf/head/blink`, `/olaf/head/status` appear
   - Manual test: `ros2 topic pub /olaf/head/blink std_msgs/Empty` → eye blinks
   - Status topic shows "READY"

8. **Code Quality:**
   - Type hints for all functions
   - Docstrings explain I2C protocol
   - Code committed to `orchestrator/ros2_nodes/hardware_drivers/head_driver.py`

**Dependencies:** Story 1.3 (ROS2 + I2C on Pi), Story 1.5 (ESP32 firmware)

**Estimated Effort:** 6-8 hours

---

## Story 1.9: Minimal Orchestrator - Blink Coordinator

**As a** software developer,
**I want** a minimal orchestrator node that sends blink commands via ROS2,
**so that** I can prove the full pipeline (Orchestrator → ROS2 → I2C → ESP32 → SPI OLED) works.

### Acceptance Criteria:

1. **Orchestrator Node:**
   - Python ROS2 node: `orchestrator/ros2_nodes/minimal_coordinator.py`
   - Node name: `/olaf/minimal_coordinator`

2. **Publisher:**
   - Publishes to `/olaf/head/blink` (std_msgs/Empty)

3. **Blink Loop:**
   - Timer triggers blink every 5 seconds:
     ```python
     def timer_callback(self):
         self.get_logger().info("Triggering blink")
         self.blink_pub.publish(Empty())
     ```

4. **Execution:**
   - Launch: `ros2 run olaf_orchestrator minimal_coordinator`
   - Every 5 seconds: Logs "Triggering blink" and eye blinks

5. **End-to-End Validation:**
   - **Full chain running:**
     1. Minimal coordinator publishes to `/olaf/head/blink`
     2. Head driver node receives message
     3. Head driver writes I2C register 0x12 to ESP32
     4. ESP32 receives I2C command
     5. ESP32 renders blink animation on SPI OLED
   - **Latency:** From publish to eye blink < 100ms
   - **Reliability:** 100 consecutive blinks without failure

6. **Code Quality:**
   - Clean, commented code
   - Committed to `orchestrator/ros2_nodes/minimal_coordinator.py`

**Dependencies:** Story 1.8 (head driver node)

**Estimated Effort:** 3-4 hours

---

## Story 1.10: Test Harness CLI - I2C Direct Testing

**As a** developer or tester,
**I want** a command-line tool to send I2C commands directly to modules,
**so that** I can debug without running full ROS2 stack.

### Acceptance Criteria:

1. **CLI Tool:**
   - Python script: `tools/diagnostics/olaf-test`
   - Executable: `chmod +x tools/diagnostics/olaf-test`
   - Uses `smbus2` for direct I2C access

2. **Commands:**
   - `olaf-test head blink` → Writes I2C register 0x12 (blink) to address 0x08
   - `olaf-test head status` → Reads register 0x02, prints status byte
   - `olaf-test head id` → Reads register 0x00, prints module ID
   - `olaf-test scan` → Runs `i2cdetect -y 1`, shows all I2C devices

3. **Implementation:**
   ```python
   def blink_head():
       bus = SMBus(1)
       bus.write_byte_data(0x08, 0x12, 0x01)  # Write to blink register
       print("Blink command sent to Head (0x08)")
   ```

4. **Usage:**
   - From terminal: `./tools/diagnostics/olaf-test head blink` → eye blinks
   - Help: `./tools/diagnostics/olaf-test --help` shows commands

5. **Error Handling:**
   - If I2C not available: "I2C bus not found. Is I2C enabled?"
   - If module doesn't respond: "No response from module 0x08"

6. **Documentation:**
   - Usage guide: `tools/diagnostics/README.md`
   - Examples for each command

**Dependencies:** Story 1.7 (ESP32 firmware), Story 1.3 (I2C enabled)

**Estimated Effort:** 3-4 hours

---

## Story 1.11: Launch File - Automated Startup

**As a** system operator,
**I want** a ROS2 launch file that starts all Epic 1 nodes,
**so that** I can run the full system with one command.

### Acceptance Criteria:

1. **Launch File Created:**
   - File: `orchestrator/launch/minimal_system.launch.py`
   - Starts:
     - Head driver node (`head_driver.py`)
     - Minimal coordinator (`minimal_coordinator.py`)

2. **Launch Configuration:**
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='olaf_drivers',
               executable='head_driver',
               name='head_driver'
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
   - Eye blinks every 5 seconds

4. **Logging:**
   - Launch output shows both nodes starting
   - Logs visible in terminal

5. **Documentation:**
   - Launch file documented in README
   - "Quick Start" section updated

**Dependencies:** Story 1.8 (driver), Story 1.9 (coordinator)

**Estimated Effort:** 2-3 hours

---

## Story 1.12: 30-Minute Continuous Operation Test

**As a** quality assurance tester,
**I want** to run the minimal system for 30 minutes without failures,
**so that** I can validate I2C communication stability before Epic 2.

### Acceptance Criteria:

1. **Test Setup:**
   - Launch: `ros2 launch orchestrator minimal_system.launch.py`
   - Battery at full charge (ODrive will monitor in future)
   - Timer set for 30 minutes

2. **Test Execution:**
   - Eye blinks every 5 seconds (360 blinks total)
   - Monitor:
     - Terminal logs (no errors)
     - Eye behavior (smooth animations)
     - I2C communication (no timeouts)

3. **Success Criteria:**
   - **No crashes:** All nodes run 30 minutes
   - **No missed blinks:** ~360 blinks complete (±5% tolerance)
   - **Stable latency:** Blink command → animation < 100ms throughout
   - **Memory stable:** ESP32 free heap doesn't decrease

4. **Metrics Collected:**
   - Total runtime: 30:00 minutes
   - Total blinks: ~360
   - I2C errors: 0
   - Average latency: <100ms (sampled 10 times)

5. **Pass/Fail:**
   - **PASS:** All criteria met
   - **FAIL:** Any crash, >10% missed blinks, latency >100ms

6. **Documentation:**
   - Results: `tests/integration/epic1_30min_test_results.md`
   - Screenshot of terminal after 30 min

**Dependencies:** All Epic 1 stories (1.1-1.11)

**Estimated Effort:** 1 hour test + 2-4 hours fixes if needed

---

## Story 1.13: Epic 1 Documentation & Content

**As a** future contributor or follower,
**I want** comprehensive Epic 1 documentation and build-in-public content,
**so that** I can replicate the setup and follow the journey.

### Acceptance Criteria:

1. **Technical Documentation:**
   - `modules/head/README.md`:
     - Hardware connections (SPI + I2C pinout)
     - Firmware build/upload process
     - I2C register map reference
   - `orchestrator/ros2_nodes/README.md`:
     - How to run driver nodes
     - Topic reference
     - I2C communication flow

2. **Hardware Documentation:**
   - BOM: `hardware/bom/epic1_bom.csv`
     - Battery, buck converters, ESP32, SPI OLED, wires
     - Supplier links, costs
   - Wiring diagrams in `hardware/wiring/`

3. **Setup Scripts:**
   - `tools/setup/install_ros2_humble_pi.sh` tested on fresh Pi
   - README "Quick Start" section complete

4. **Photos/Videos:**
   - Photos:
     - Power system assembled
     - Head module (ESP32 + SPI OLED) blinking
     - Full Epic 1 breadboard setup
   - Optional: 15-second video of blinking eye

5. **GitHub Milestone:**
   - Milestone "Epic 1: Foundation" created
   - All stories closed and linked
   - Milestone marked complete

6. **Build-in-Public Content:**
   - LinkedIn post: "Olaf's First Blink! 🤖"
     - Photo of blinking eye
     - Explanation: I2C + SPI architecture, 30-60 FPS animation
     - Technical depth: Smart I2C slave pattern
   - Post published

7. **Troubleshooting Guide:**
   - `docs/troubleshooting-epic1.md`:
     - "ESP32 not detected on I2C" → solution
     - "SPI OLED shows garbage" → wiring check
     - "ROS2 node can't open I2C" → permissions fix

**Dependencies:** All Epic 1 stories complete

**Estimated Effort:** 6-8 hours (docs + photos + content)

---

## Epic 1 Summary

**Total Stories:** 13 (includes body module: Stories 1.5-1.6)
**Estimated Total Effort:** 62-80 hours (3-4 weeks for solo builder)

**Key Deliverables:**
- ✅ Repository structure with shared I2C register definitions
- ✅ Power system (36V hoverboard battery + buck converters)
- ✅ ROS2 Humble on Raspberry Pi with I2C enabled
- ✅ **Head Module:** ESP32 firmware as I2C slave (address 0x08), SPI OLED eyes at 30-60 FPS
- ✅ **Body Module:** Heart LCD (GC9A01, 60 FPS emotion BPM), projector power control, WS2812B LEDs (address 0x0A)
- ✅ ROS2 driver nodes translating topics → I2C registers (head + body)
- ✅ Orchestrator sending coordinated commands (blink + heartbeat)
- ✅ Test harness for direct I2C module testing
- ✅ Launch file for automated system startup
- ✅ 30-minute continuous operation validated
- ✅ Complete documentation + build-in-public content

**Architecture Validated:**
- ✅ I2C-only communication (no WiFi on ESP32s)
- ✅ SPI OLED for 30-60 FPS smooth animation
- ✅ Smart I2C slave pattern (ESP32 has animation engine)
- ✅ ROS2 only on Pi (driver nodes as I2C bridges)
- ✅ Shared register protocol (foundation for all modules)

**Success Criteria Met:**
- Three-layer architecture proven (Orchestrator → I2C Driver → Smart ESP32)
- Power system stable and safe
- "Quick win" achieved: Olaf blinks at you with smooth SPI animation!
- Foundation ready for Epic 2 (complete Head module with dual eyes)

**Next Epic:** Epic 2: Complete Head Module (Dual Eyes, Presence Sensor, Audio)

---

**Related Documents:**
- [PRD](../prd.md) - FR1-FR7 validated
- [Architecture](../architecture.md) - I2C + SPI pattern proven
- [Epic 2: Complete Head Module](epic-02-head-module.md) (next)
