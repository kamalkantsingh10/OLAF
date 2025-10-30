# Technical Assumptions

## Repository Structure

**Type: Monorepo**

The project will use a single repository containing all modules, orchestration code, documentation, and hardware designs. This simplifies dependency management and ensures all components version together.

```
olaf/
├── docs/                    # Documentation, PRD, architecture, build guides
│   ├── prd.md
│   ├── branding-guide.md
│   ├── architecture.md
│   └── build-guides/
├── hardware/               # 3D models, wiring diagrams, BOMs
│   ├── 3d-models/         # STL files organized by module
│   ├── wiring/            # Fritzing diagrams, connection tables
│   └── bom/               # Bills of materials with supplier links
├── firmware/                # Module-specific firmware and configs
│   ├── head/              # ESP32 firmware: OAK-D Pro, eyes, mic, mmWave
│   ├── ears/              # ESP32 firmware: 2-DOF servo control (Feetech SCS0009)
│   ├── neck/              # ESP32 firmware: 3-DOF servo control (STS3215)
│   ├── projector/         # ESP32 firmware: DLP control
│   ├── body/              # ESP32 firmware: LED indicators, status
│   └── base/              # ESP32 firmware: ODrive motor control interface
├── ros2/src/orchestrator/           # Raspberry Pi Python application
│   ├── personality/       # Expression coordination engine
│   ├── ai_integration/    # Cloud AI (Claude API) interface
│   ├── navigation/        # SLAM, path planning (ROS2)
│   ├── state_management/  # Conversation context, system state
│   └── ros2_nodes/        # ROS2 node implementations
├── tests/                  # Module and integration tests
│   ├── unit/              # Per-module unit tests
│   ├── integration/       # Cross-module integration tests
│   └── fixtures/          # Test harnesses, mock data
├── tools/                  # Setup scripts, utilities
│   ├── setup/             # First-time environment setup
│   ├── calibration/       # Servo calibration, sensor tuning, ODrive config
│   └── diagnostics/       # Module health checks
├── config/                 # Configuration templates
│   ├── api-keys.template.env
│   ├── module-config.yaml
│   └── ros2-params/
└── README.md
```

**Rationale:**
- Single source of truth for all project components
- Simplifies cloning and getting started (one `git clone`)
- Coordinated versioning (hardware + software + docs evolve together)
- Easier dependency management (Python packages, ROS2 workspaces in one place)
- Aligns with educational goal (newcomers see complete project structure)

## Service Architecture

**Type: Modular Microservices within Monorepo**

Each hardware module functions as an independent microservice (ESP32 + firmware) communicating via ROS2 topics. The orchestration layer (Raspberry Pi) acts as coordinator without tight coupling to individual modules.

**Key Architectural Decisions:**

**1. Module Layer (ESP32 Smart Peripherals):**
- Each module runs standalone firmware (C/C++ using Arduino framework or ESP-IDF)
- **Smart I2C Slave Architecture:** Each ESP32 acts as intelligent I2C slave (addresses 0x08-0x0C)
  - Contains full hardware drivers (servo control, display rendering, sensor polling)
  - Executes high-level semantic commands from Pi (e.g., "express happy level 3")
  - Manages real-time control loops locally (Base: 200Hz PID, Head: 200Hz animation)
  - ROS2 nodes run ONLY on Pi; ESP32s are smart peripherals, NOT ROS2 nodes
- **I2C Communication Protocol:**
  - Standard register map (0x00-0x0F: status, command, error codes)
  - Module-specific registers (0x10+: expressions, velocity, odometry)
  - 400kHz-1MHz bus speed achieves 5-20ms command latency
  - Each module unique I2C address for orchestrator communication
- **Local behavior execution:** Pre-programmed patterns reduce orchestrator overhead
  - Example: "Express happy level 3" → ESP32 renders cached animation frames
  - Orchestrator sends intent, module handles microsecond-precision timing

**2. Orchestration Layer (Raspberry Pi 5 8GB + Hailo-8L AI Kit):**
- Python 3.11+ application running on Raspberry Pi OS (Debian 12 Bookworm, 64-bit)
- **ROS2 Humble (LTS until 2027):** Communication backbone for orchestrator nodes
  - ALL ROS2 nodes run on Pi (personality, navigation, AI, 5× hardware driver nodes)
  - Driver nodes act as I2C bridges translating ROS2 topics to I2C register writes
  - Bridges cloud AI decisions to high-level I2C commands sent to ESP32 modules
- **Personality Coordination Engine:**
  - Receives emotion type + intensity from AI layer
  - Translates to synchronized I2C commands sent to Head (eyes), Ears, Neck modules
  - Ensures <500ms coordination (FR13) via concurrent I2C writes
- **Local AI Acceleration:** Hailo-8L AI Kit (13 TOPS)
  - Whisper STT inference: <200ms latency (saves 1-1.5s vs cloud STT)
  - Future: Vision model inference, gesture recognition
- **SLAM Navigation:** Using **Google Cartographer** with RGBD camera depth data
  - Real-time mapping and localization
  - Path planning for autonomous movement
  - Obstacle avoidance using depth sensing
- **SQLite for local storage:**
  - Conversation history and context
  - Interaction logs (FR18)
  - Module configuration and calibration data

**3. Intelligence Layer (Hybrid: Local Hailo + Cloud AI):**
- **Local AI (Hailo-accelerated):**
  - Whisper STT: Speech-to-text with <200ms latency
  - Runs on Hailo-8L AI accelerator (PCIe interface to Pi)
  - Enables fast interaction loop without cloud dependency for audio input
- **Cloud AI (Anthropic Claude API):** RESTful API integration over HTTPS
  - **Claude 3.5 Sonnet** for V1 (standardize on single provider)
  - Natural language understanding
  - Intent classification and function routing
  - Sentiment analysis → emotion type + intensity mapping
- **Conversation context management:**
  - Orchestrator maintains message history locally
  - Sends rolling context window to API (last N messages + system prompt)
  - Persists to SQLite across power cycles (FR15)
- **Retry logic:** Exponential backoff (1s, 2s, 4s, max 10s, 3 attempts) per FR17
- **Graceful degradation:** On API failure, fall back to "thinking" expression + cached responses

**Communication Flow Example:**

```
User speaks: "Hey Olaf, what's the weather?"
    ↓
1. HEAD ESP32: mmWave detects presence, mic captures audio
    ↓ I2C read by /olaf/head_driver node: REG_AUDIO_READY=1
2. Orchestrator (/olaf/ai_agent node):
   - Retrieves audio buffer via I2C
   - Runs Whisper STT on Hailo accelerator (<200ms)
   - Sends transcript to Claude API via HTTPS
    ↓
3. Claude API responds: Intent=weather_query, Emotion=curious, Intensity=2
    ↓
4. Orchestrator (/olaf/personality_coordinator node):
   - Queries weather API (local fetch)
   - Publishes ROS2 topic: /olaf/personality/express
   - Driver nodes translate to I2C commands:
       HEAD (0x08): REG_EXPRESSION_TYPE=CURIOUS, REG_INTENSITY=2
       EARS (0x09): REG_POSITION=FORWARD_ALERT
       NECK (0x0A): REG_TILT=SLIGHT (10°)
       PROJECTOR (0x0B): REG_DISPLAY_CMD + weather data
    ↓
5. ESP32 modules execute locally stored behaviors (<500ms sync)
    ↓
6. User sees: Olaf tilts head curiously, ears forward, eyes animate,
              projects weather on floor, beeps in curious pattern
```

**Rationale:**
- **Modularity:** Individual modules testable/replaceable without affecting others (FR4, FR5)
- **Responsiveness:** I2C + local firmware execution achieves 5-20ms latency (NFR2: <100ms)
- **Scalability:** Easy to add new modules (new I2C address + driver node)
- **Fault tolerance:** Module failures don't crash orchestrator (graceful degradation per FR6)
- **Real-time Guarantee:** ESP32 FreeRTOS handles microsecond-precision control (200Hz balancing) that Linux cannot
- **Hybrid AI:** Local Hailo STT (<200ms) + cloud reasoning achieves NFR1: <3s total response
- **Power Efficiency:** No WiFi on ESP32s saves ~1000mA, extends battery runtime (NFR4: 2-4 hours)

## Testing Requirements

**Type: Hybrid Testing Strategy (Unit + Integration + Manual)**

**Unit Testing:**
- **Module firmware:** Each ESP32 module has standalone test suite
  - Servo range validation (Feetech SCS0009 ears: 2-DOF, STS3215 neck: 3-DOF with pan ±90°, tilt ±45°, roll ±30°)
  - Sensor data integrity (RGBD camera depth accuracy, mmWave presence detection)
  - Expression rendering (OLED eyes via SPI display correct animations at 30+ FPS)
  - Balancing PID tuning (Base module: 200Hz control loop stability)
- **Orchestrator components:** Python unit tests using `pytest`
  - Personality engine logic (emotion + intensity → module commands)
  - AI integration (mocked API responses, intent parsing)
  - Navigation algorithms (path planning on simulated maps)
- **Coverage target:** 70%+ for critical paths (personality, navigation, AI)

**Integration Testing:**
- **Module-to-orchestrator:** I2C communication validation
  - Latency tests (NFR2: <100ms module communication via I2C)
  - Command-response verification (write I2C REG_COMMAND to 0x0A, confirm STS3215 servo movement)
  - Graceful degradation tests (disconnect module mid-operation, verify FR6 behavior)
- **End-to-end workflows:** Simulated user interactions
  - Voice command → AI processing → expression coordination → projection display
  - Navigation scenarios (move to point, avoid obstacle, follow person) with ODrive motor control
  - Multi-turn conversations with context persistence

**Manual/Physical Testing:**
- **Personality validation:** User testing per FR7 (>80% emotion ID, >70% intensity assessment)
  - Show 20 random expressions to 5+ testers
  - Record identification accuracy, adjust animations based on feedback
- **Navigation accuracy:** SLAM precision testing per NFR5 (±10cm)
  - Mark waypoints in apartment, measure Olaf's arrival accuracy
  - Test on different floor surfaces (hardwood, tile, carpet)
- **Battery runtime:** Continuous operation test per NFR4 (2-4 hours)
  - Full charge → idle/conversation/moderate navigation → measure time to shutdown
  - Optimize power consumption based on results

**Test Harness (FR5):**
- **Module standalone test tool:** CLI utility for manual testing
  - `olaf-test neck --pan 45` → Writes I2C register to 0x0A, commands STS3215 neck servo to pan 45°
  - `olaf-test ears --tilt 30` → Writes I2C register to 0x09, commands SCS0009 ear servos to tilt 30°
  - `olaf-test eyes --emotion happy --intensity 3` → Writes I2C to 0x08, renders expression
  - `olaf-test base --velocity 0.5` → Writes I2C to 0x0C, commands ODrive motors at 0.5 m/s
  - `olaf-test base --balance` → Triggers balancing mode, monitors 200Hz PID loop stability
  - Logs module I2C responses, validates register reads
- **CI/CD Integration (V2):** Automated testing on GitHub Actions
  - V1: Manual test execution before major commits
  - V2: Full CI pipeline with hardware-in-the-loop testing (if feasible)

**Rationale:**
- **Unit tests:** Catch regressions early in development (especially personality engine complexity)
- **Integration tests:** Validate ROS2 communication and latency requirements
- **Manual tests:** Physical robot behavior (expression recognition, navigation) requires human assessment
- **Hybrid approach:** Balances automated validation with real-world performance verification

## Additional Technical Assumptions and Requests

**Power Management:**
- **Battery:** Hoverboard battery pack (36V, typically 4-10Ah depending on hoverboard model)
  - **Voltage:** 36V nominal (10S Li-ion, 42V fully charged, 30V cutoff)
  - **Capacity:** Estimated 4000-8000mAh (depends on salvaged hoverboard model)
  - **Expected runtime:** 2-4 hours continuous operation (NFR4) depending on capacity and usage
- **Power Distribution:**
  - **36V → 12V Buck Converter (10A):** Powers Raspberry Pi 8GB via secondary 12V→5V converter, ODrive motor controller, DLP projector
  - **36V → 5V Buck Converter (10A):** Powers ESP32 modules, servos (Feetech SCS0009, STS3215), OAK-D Pro camera, sensors
  - Both converters rated for 10A continuous to handle peak loads
- **Charging:** Hoverboard's original charging circuit board (BMS - Battery Management System)
  - Retains hoverboard main circuit board for integrated charging functionality
  - Standard hoverboard charger (42V, 2A typical) used for recharging
  - BMS handles cell balancing, over-charge/discharge protection
- **Battery Monitoring:**
  - Voltage monitoring via ADC (ESP32 or Raspberry Pi GPIO)
  - Low-battery warning at 32V (≈20% capacity): LED indicator + beep alert
  - Auto-shutdown at 30V (battery cutoff) to prevent over-discharge damage
  - Display remaining % via calculation: (V_current - 30) / (42 - 30) × 100

**Servo Specifications:**
- **Ears (2x modules, 2-DOF each):** Feetech SCS0009 serial bus servos
  - **Protocol:** Serial communication (likely half-duplex TTL UART)
  - **DOF per ear:** Vertical tilt + horizontal rotation = 2 servos per ear module
  - **Total servos:** 4x SCS0009 (2 ears × 2 DOF)
  - **Torque:** Check spec sheet (typically 8-12 kg·cm for this series)
  - **Speed:** Adjustable via protocol (intensity mapping: level 1=slow, level 5=fast)
  - **Voltage:** 5V (powered by 36V→5V buck converter)
- **Neck (1x module, 3-DOF):** Feetech STS3215 serial bus servos
  - **Protocol:** Serial communication (ST series, likely compatible with SCS protocol)
  - **DOF:** Pan, tilt, roll = 3 servos total
  - **Torque:** Higher than SCS0009 (typically 15-20 kg·cm for neck support)
  - **Range:** Pan ±90°, tilt ±45°, roll ±30° per FR9
  - **Voltage:** 5V or 6V (check spec, use 5V from buck converter)
  - **Position feedback:** Built-in position sensing for closed-loop control

**Motor Control (Base Module):**
- **Motors:** Hoverboard brushless DC (BLDC) hub motors (2x, differential drive)
  - **Type:** Typically 350W per wheel (700W total peak power)
  - **Voltage:** 36V nominal
  - **Encoder:** Hall sensors built into motors (for position/velocity feedback)
- **Controller:** ODrive motor controller
  - **Model:** ODrive v3.6 or ODrive S1 (V1: likely v3.6 for cost, V2: S1 for compactness)
  - **Channels:** Dual-channel (controls both left and right motors)
  - **Interface:** UART, USB, or CAN bus communication with Raspberry Pi
  - **Power:** Powered directly from 36V hoverboard battery
  - **Functionality:**
    - Closed-loop velocity control (SLAM navigation requires precise speed control)
    - Encoder feedback for odometry (wheel rotation → distance traveled)
    - Current limiting to prevent battery over-draw
  - **ROS2 Integration:** ESP32 base module or Raspberry Pi directly communicates with ODrive via serial
    - Publishes `/olaf/base/odometry` (wheel encoder data for SLAM)
    - Subscribes to `/olaf/base/velocity_cmd` (target linear/angular velocity from navigation)

**Development Environment:**
- **Primary OS:** Linux (Ubuntu 22.04 LTS or Raspberry Pi OS for orchestrator dev)
  - ROS2 Humble natively supported on Ubuntu 22.04
  - Cross-compilation for ESP32 via PlatformIO or Arduino CLI
- **IDE:** VS Code with extensions:
  - PlatformIO (ESP32 firmware development)
  - Python (orchestrator development)
  - ROS extension (topic monitoring, launch files)
- **Version Control:** Git with GitHub for hosting
  - Branch strategy: `main` (stable), `develop` (active work), feature branches
  - Semantic versioning for releases (v1.0.0, v1.1.0, etc.)

**Networking:**
- **WiFi:** 2.4GHz preferred (longer range, better obstacle penetration than 5GHz)
  - Raspberry Pi 8GB has dual-band WiFi (802.11ac)
  - ESP32 modules: 2.4GHz only (sufficient for local ROS2 communication)
- **Local network:** Orchestrator and modules communicate over local WiFi (no internet required for ROS2)
- **Cloud connectivity:** Orchestrator requires internet for Claude API calls (NFR9)
- **Fallback:** If WiFi drops, Olaf enters "offline mode" (stationary, no AI, pre-programmed expressions only)

**Security Considerations:**
- **API Key Management:**
  - Store in `.env` file (excluded via `.gitignore`)
  - Template provided: `config/api-keys.template.env`
  - Load via Python `dotenv` library at runtime
  - **Never commit keys to repository** (use pre-commit hooks to prevent)
- **Conversation Privacy:**
  - All conversation data stored locally on Raspberry Pi (SQLite database)
  - Cloud AI API calls: Send only necessary context (last 5-10 messages, no full history)
  - Option to disable logging (privacy mode for sensitive conversations)
  - HTTPS encryption for all API calls (NFR12)
- **Network Security:**
  - Raspberry Pi firewall: Allow only outbound HTTPS, ROS2 ports on local network
  - No remote access by default (SSH disabled or key-only auth)
  - Optional: VPN for remote debugging (not required for V1)

**Third-Party Libraries & APIs:**
- **ROS2 Packages:**
  - `micro_ros_agent` (ESP32 to ROS2 bridge)
  - `rtabmap_ros` or `cartographer_ros` (SLAM)
  - `navigation2` (path planning, obstacle avoidance)
  - `cv_bridge` (OAK-D Pro image processing)
  - `odrive_ros2` (ODrive motor controller integration, community package or custom node)
- **Python Libraries:**
  - `anthropic` (Claude API client)
  - `rclpy` (ROS2 Python bindings)
  - `opencv-python` (computer vision, if needed beyond OAK-D Pro SDK)
  - `pygame` or `pyaudio` (sound generation for beeps)
  - `python-dotenv` (environment variable management)
  - `pytest` (unit testing)
- **ESP32 Libraries:**
  - `micro_ros_arduino` (ROS2 node on ESP32)
  - `Adafruit_SSD1306` (OLED eye display driver)
  - `FeetechServo` or custom library (SCS0009, STS3215 serial servo control)
  - Custom libraries for DLP projector control (vendor-specific)

**Hardware Sourcing:**
- **Primary Vendors:**
  - AliExpress (servos, sensors, ESP32 boards) - budget-friendly
  - Adafruit (quality sensors, well-documented modules) - reliability
  - Amazon (Raspberry Pi, buck converters, quick shipping) - convenience
- **Specific Components:**
  - **OAK-D Pro:** Luxonis official site or authorized distributors (~$300)
  - **DLP Projector:** TI DLP LightCrafter or compatible mini projector module (~$150-250)
  - **mmWave Sensor:** 24GHz radar module (e.g., DFRobot SEN0395 or similar, ~$20-40)
  - **Hoverboard (salvage):** Used hoverboards (eBay, local classifieds, ~$50-100 for broken units)
    - Extract: Battery pack, motors with wheels, BMS/charging circuit, motor controller (if not using ODrive)
  - **ODrive:** ODrive Robotics official site (~$120-200 depending on version)
  - **Feetech SCS0009:** AliExpress, RobotShop (~$15-25 each, need 4x)
  - **Feetech STS3215:** AliExpress, RobotShop (~$25-40 each, need 3x)
  - **Buck Converters (36V→12V, 36V→5V, 10A):** AliExpress, Amazon (~$10-20 each)
- **BOM Maintenance:** Keep supplier links updated in `hardware/bom/` (NFR8)

**AI-Assisted Development:**
- **Expectation:** 100% of code and documentation leverages AI assistance (Claude, GPT-4, GitHub Copilot)
  - Code generation: Boilerplate, module templates, ROS2 nodes, ODrive integration
  - Documentation: Build guides, API docs, servo calibration procedures, troubleshooting (auto-generated with human review)
  - Debugging: AI-assisted error analysis, solution suggestions
  - Optimization: Performance tuning, code refactoring recommendations
- **Workflow:**
  - Describe desired functionality to AI
  - Review generated code/docs for correctness
  - Iterate with AI on refinements
  - Commit final human-reviewed version
- **Rationale:** Aligns with brief's emphasis on AI-assisted workflow, accelerates development, educational for learners

**Deployment Strategy:**
- **V1 Target:** Self-hosted on Raspberry Pi (no external servers, no containers)
  - Direct Python execution via systemd service (auto-start on boot)
  - ROS2 launch files for module orchestration
  - Manual updates: SSH into Pi, `git pull`, restart services
- **V2+ Considerations:**
  - Docker containers for orchestrator (easier deployment, dependency isolation)
  - Over-the-air (OTA) updates for ESP32 firmware
  - Web dashboard for monitoring/configuration (Flask or FastAPI backend)

---

**Rationale & Assumptions:**

**Key Technical Decisions:**

1. **Monorepo:** Simplifies onboarding (one clone), ensures versioning consistency, educational clarity
2. **ROS2 on Pi only (no micro-ROS):** Simpler architecture, ESP32s are smart I2C slaves not ROS2 nodes
3. **I2C-only module communication:** 5-20ms latency vs 80-200ms WiFi, saves 1000mA power, deterministic timing
4. **SPI for OLED displays:** 30-60 FPS animation vs 10-15 FPS (I2C), critical for personality expression
5. **Hailo-8L AI accelerator:** Eliminates 1-1.5s cloud STT latency, <200ms Whisper inference, $70 investment
6. **Claude API (single provider for V1):** Reduces complexity vs. multi-provider support, can add GPT-4 in V2
7. **Self-balancing base (200Hz PID on ESP32):** Linux cannot guarantee real-time, FreeRTOS provides microsecond precision
8. **Cartographer for SLAM:** Google-maintained, lighter than RTAB-Map, proven 2D/3D mapping
9. **ODrive motor controller:** Closed-loop velocity control, encoder odometry for ±10cm SLAM accuracy
10. **Hoverboard salvage:** Reuses battery + motors + BMS = cost-effective, eco-friendly, proven components
11. **Feetech serial servos:** Daisy-chainable (reduces wiring), position feedback, programmable speed (intensity mapping)
12. **Dual buck converters (36V→12V, 36V→5V):** Clean power distribution, isolates high-power (motors) from low-power (logic) circuits
13. **OTA updates in V1:** Rapid iteration without disassembly, HTTP server on Pi serves firmware binaries
14. **Hybrid testing:** Automated tests catch regressions, manual tests validate physical robot behavior
15. **Local-first data:** Privacy-conscious, reduces cloud dependency, faster response for non-AI tasks
16. **AI-assisted development:** Dogfooding modern AI tools, accelerates development, teaches learners AI workflows

**Assumptions Made:**
- Hoverboard battery (36V, 4-8Ah) provides 2-4 hours runtime with efficient power management (WiFi disabled on ESP32s)
- ODrive interfaces with Base ESP32 via UART for real-time motor control commands
- Feetech servos (SCS0009, STS3215) provide sufficient torque for ear/neck articulation under head weight
- 10A buck converters handle peak current draw (Pi: ~3A, servos: ~2A each × 7 servos peak = ~14A total, staggered usage keeps average <10A per rail)
- Raspberry Pi 5 8GB + Hailo-8L sufficient for orchestrator workload (ROS2 + Python + Cartographer SLAM + Hailo inference + SQLite)
- ESP32 240MHz + 320KB RAM sufficient for I2C slave + animation engine + 200Hz PID control (Base module)
- I2C bus (400kHz-1MHz) handles 5 modules at 10-100Hz polling rate without bus contention
- Hailo-8L accelerator achieves <200ms Whisper STT latency + Claude API <2s = total <3s (P90) per NFR1
- RGBD camera depth accuracy meets ±10cm SLAM requirement with Cartographer (NFR5)
- Hoverboard BMS safely handles charging without modifications (standard 42V charger compatible)
- SPI-driven OLEDs (10-20 MHz) achieve 30-60 FPS animation rendering on ESP32 without framebuffer overflow

**Architectural Decisions Made (See architecture.md):**
- ✅ ROS2 Humble (LTS until 2027) selected for long-term support
- ✅ SLAM library: Cartographer (Google-maintained, lighter footprint)
- ✅ ODrive communication: UART from Base ESP32 (modular, isolates real-time control)
- ✅ Communication: I2C-only for modules (no WiFi on ESP32s), WiFi on Pi for cloud API only
- ✅ Display interface: SPI for OLEDs (30-60 FPS), not I2C (10-15 FPS)
- ✅ Self-balancing: 200Hz PID on Base ESP32 with MPU6050 IMU
- ✅ OTA updates: V1 scope with HTTP Flask server on Pi
- ✅ Servo control: Feetech serial bus servos (SCS0009, STS3215) with I2C interface
- ✅ Buck converter placement: Centralized in body (simpler wiring for V1)
- ✅ Projector: HDMI from Pi OR ESP32-controlled DLP (TBD based on projector model selected)

---
