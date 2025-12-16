# High Level Architecture

## Technical Summary

OLAF is a **distributed embedded robotics system** with hybrid AI intelligence, built on a three-layer architecture. The **Orchestration Layer** (Raspberry Pi 5 16GB + Hailo AI Kit running ROS2 Humble) coordinates four independent **Module Layer** smart peripherals (ESP32-based: Head+Ears, Neck, Torso, Base) via I2C bus communication, plus the Pi Module itself which houses the orchestrator hardware. The **Intelligence Layer** combines local AI acceleration (Hailo-accelerated Whisper STT for <200ms speech recognition) with cloud-based AI agents (Claude/GPT-4 APIs for reasoning and personality generation).

Key architectural decisions include: (1) I2C-only module communication eliminates WiFi complexity and achieves 5-20ms latency vs 80-200ms WiFi/ROS2, (2) ROS2 nodes run exclusively on Pi with ESP32s as smart I2C slaves containing full hardware drivers, animation engines, and real-time control loops, (3) OLED displays (128x64, SPI) for eyes with 30-60 FPS animated expressions, (4) Five independent modules via MECE principle: Head+Ears (eyes + articulated ears + floor projector at 0x08), Neck (3-DOF pan/tilt/roll + 2 presence sensors at 0x09), Torso (heart display + thermal printer at 0x0A), Base (self-balancing motors at 0x0B), Pi Module (speakerphone + Hailo STT orchestrator), (5) Head+Ears ESP32 controls floor projector power (optocoupler-based switching) and focus (linear servo), receiving HDMI video from Pi, (6) Torso ESP32 controls 2.8" square heart display and thermal printer for physical outputs, (7) Neck ESP32 manages two presence sensors for 360° human detection, (8) Self-balancing two-wheel base with 200Hz PID control and kickstand deployment (both managed by Base ESP32), and (9) OTA firmware updates in V1 enable rapid iteration without robot disassembly. This architecture achieves <3s AI response latency, supports 2-4 hour battery runtime, and enables weekend-sprint modular development per MECE principles.

## Platform and Infrastructure Choice

**Platform: Self-Hosted Embedded Robotics System**

**Core Infrastructure:**
- **Compute Platform**: Raspberry Pi 5 16GB (quad-core ARM Cortex-A76 @ 2.4GHz)
- **AI Accelerator**: Hailo AI Kit (26 TOPS, PCIe interface)
- **Operating System**: Raspberry Pi OS (Debian 12 Bookworm-based, 64-bit)
- **Deployment Model**: Self-contained mobile robot, no external servers
- **Network Requirements**: Home WiFi (2.4/5GHz) for cloud API access only

**Key Infrastructure Services:**
- **Local AI Inference**: Hailo-accelerated Whisper STT (~150-200ms latency)
- **Cloud AI Services**: Anthropic Claude API (V1), OpenAI GPT-4 API (future)
- **Database**: SQLite (embedded, conversation history + config)
- **OTA Server**: HTTP server on Pi (Flask/FastAPI) serving firmware binaries
- **Real-Time Communication**: I2C bus (400kHz-1MHz) for module coordination

**Deployment Host and Regions**: Local (Raspberry Pi), no cloud hosting. WiFi for API calls only.

## Repository Structure

**Structure:** Monorepo

**Monorepo Tool:** Git (no specialized monorepo tool needed for this scale)

**Organization Principle:** Module-first architecture where each physical module (Head+Ears, Neck, Torso, Base) is a complete, self-contained subsystem with its own firmware, hardware, tests, diagnostics, and documentation. This matches the distributed smart peripheral architecture pattern.

```
olaf/
├── modules/                    # All 4 modules (complete subsystems)
│   ├── head-ears/              # Head+Ears Module (I2C 0x08)
│   │   ├── firmware/           # ESP32 firmware
│   │   │   ├── src/            # Source files (main.cpp, drivers/)
│   │   │   ├── include/        # Header files
│   │   │   ├── test/           # Unit tests
│   │   │   └── platformio.ini  # Build configuration
│   │   ├── hardware/           # Physical designs
│   │   │   ├── pcb/            # KiCad PCB designs
│   │   │   ├── mechanical/     # 3D models (STEP, STL)
│   │   │   └── bom.csv         # Bill of materials
│   │   ├── tests/              # Module-specific integration tests
│   │   ├── diagnostics/        # Diagnostic tools (projector, eyes, ears)
│   │   ├── scripts/            # Build, flash, calibrate scripts
│   │   ├── wiring.md           # Pin assignments, circuits
│   │   ├── assembly.md         # Assembly instructions
│   │   └── README.md           # Module overview
│   │
│   ├── neck/                   # Neck Module (I2C 0x09)
│   │   ├── firmware/
│   │   ├── hardware/
│   │   ├── tests/
│   │   ├── diagnostics/
│   │   ├── scripts/
│   │   ├── wiring.md
│   │   ├── assembly.md
│   │   └── README.md
│   │
│   ├── torso/                  # Torso Module (I2C 0x0A)
│   │   ├── firmware/
│   │   ├── hardware/
│   │   ├── tests/
│   │   ├── diagnostics/
│   │   ├── scripts/
│   │   ├── wiring.md
│   │   ├── assembly.md
│   │   └── README.md
│   │
│   ├── base/                   # Base Module (I2C 0x0B)
│   │   ├── firmware/
│   │   ├── hardware/
│   │   ├── tests/
│   │   ├── diagnostics/
│   │   ├── scripts/
│   │   ├── wiring.md
│   │   ├── assembly.md
│   │   └── README.md
│   │
│   └── shared/                 # Shared across modules
│       ├── firmware/           # Shared firmware libraries
│       │   ├── i2c-protocol/   # Common I2C register definitions
│       │   ├── ota/            # OTA update handlers
│       │   └── utils/          # Math, filters, utilities
│       └── hardware/           # Shared hardware components
│           ├── symbols/        # KiCad symbols library
│           ├── footprints/     # KiCad footprints
│           └── parts/          # Common 3D parts
│
├── ros2/                       # ROS2 workspace (Orchestration Layer)
│   └── src/                    # ROS2 packages source
│       ├── olaf_bringup/       # Launch files, configs
│       ├── olaf_description/   # URDF, robot models
│       ├── olaf_drivers/       # Hardware driver nodes (I2C ↔ ROS2)
│       │   ├── head_ears_driver/
│       │   ├── neck_driver/
│       │   ├── torso_driver/
│       │   └── base_driver/
│       ├── olaf_personality/   # Personality coordination
│       ├── olaf_ai/            # AI integration (Whisper, agents)
│       └── olaf_navigation/    # SLAM, navigation stack
│
├── scripts/                    # System-wide automation
│   ├── setup/                  # Environment setup
│   ├── build/                  # Build all modules
│   ├── flash/                  # Flash all modules
│   ├── test/                   # Test all modules
│   └── deploy/                 # OTA deployment
│
├── tools/                      # System-wide tools
│   ├── diagnostics/            # System diagnostics (i2c_scanner, system_health)
│   ├── calibration/            # Cross-module calibration (camera)
│   ├── simulators/             # I2C module simulators
│   └── utils/                  # General utilities (log analyzer)
│
├── tests/                      # System-wide integration tests
│   ├── integration/            # Cross-module tests (personality sync, navigation)
│   └── fixtures/               # Test fixtures, mock data
│
├── config/                     # System configuration
│   ├── i2c/                    # I2C bus configuration
│   ├── ros2/                   # ROS2 parameters
│   └── firmware/               # Firmware configs (WiFi, OTA)
│
├── docs/                       # System-level documentation
│   ├── architecture.md         # System architecture (this document)
│   ├── brief.md                # Project brief
│   ├── prd/                    # Product requirements (sharded)
│   ├── guides/                 # Build guides, tutorials
│   ├── api/                    # API references (I2C, ROS2)
│   └── media/                  # Images, diagrams, videos
│
├── hardware/                   # System-level hardware only
│   └── wiring/                 # Inter-module wiring (I2C bus, power distribution)
│
├── archive/                    # Previous iterations (gitignored)
├── .bmad-core/                 # BMAD agent configuration (gitignored)
├── .claude/                    # Claude Code settings (gitignored)
│
├── Makefile                    # Top-level build orchestration
├── pyproject.toml              # Python dependencies (ROS2)
├── poetry.lock                 # Poetry lockfile (gitignored)
├── .gitignore                  # Git ignore rules
├── LICENSE                     # MIT License
└── README.md                   # Main project README
```

**Key Organizational Benefits:**
- **True Modularity:** Each module directory is a complete, deployable unit (firmware + hardware + tests + diagnostics)
- **Co-Located Development:** Everything for a module lives in `modules/{module}/` (code, CAD, docs, tests, tools)
- **Independent Testing:** Module-specific tests in `modules/{module}/tests/`, system tests in top-level `tests/`
- **Clear Ownership:** One person can own an entire module with clear boundaries
- **Simplified Navigation:** Working on head-ears? `cd modules/head-ears` - everything is there
- **Matches Architecture:** Code structure mirrors the physical 4-module smart peripheral architecture

## High Level Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│              INTELLIGENCE LAYER (Hybrid AI)                     │
│  Local: Whisper STT (Hailo) | Cloud: Agent (Claude/GPT-4)      │
│  • Speech Recognition (Hailo-accelerated, local <200ms)         │
│  • Agent Reasoning & Tool Use (cloud, WiFi)                     │
│  • Multi-step Planning & Context Management                     │
└──────────────────────┬──────────────────────────────────────────┘
                       │
                       │ HTTPS/REST (WiFi - Cloud Only)
                       │
┌──────────────────────▼──────────────────────────────────────────┐
│          ORCHESTRATION LAYER (Raspberry Pi 5 16GB + Hailo)      │
│  • Personality Coordination  • SLAM Navigation                  │
│  • Local AI (Whisper, Vision) • Sensor Fusion                  │
│  • State Management         • Tool Execution                    │
│  • I2C Master Controller    • OTA Server (HTTP)                 │
└──┬──────┬──────┬──────┬────────────────────────────────────────┘
   │      │      │      │
   │ 0x08 │ 0x09 │ 0x0A │ 0x0B  I2C (Wired Only)
   │      │      │      │
┌──▼────────────────┐┌─▼────┐┌─▼────┐┌▼──────────┐
│ HEAD+EARS         ││ NECK ││TORSO ││   BASE    │
│   ESP32           ││ESP32 ││ESP32 ││   ESP32   │
└┬──┬───┬────┬─────┘└──┬───┘└──┬───┘└┬─────┬────┘
 │  │   │    │HDMI     │UART   │SPI  │UART │UART
 │  │   │    │(Pi)     │Neck   │Heart│1:   │2:
 │  │   │    │Proj     │3-DOF  │2.8" │ODrv │Stand
 │  │   │    │+Focus   │+Sens× │Prnt │+IMU │Servo
 │  │   │    │         │  2    │     │     │
 │  │   │UART
 │  │   │Ears
 │  │   │2-DOF
 │  │   │(2×)
 │  │SPI
 │  │Eyes
 │  │OLED
 │GPIO
 │Proj Pwr
┌──▼───────────────────────────────────────────────────────────┐
│          MODULE LAYER (Physical Hardware)                    │
│  • HEAD+EARS MODULE (0x08):                                  │
│    - 2× OLED Eyes (128×64, SPI, 30-60 FPS animations)       │
│    - 2× Articulated Ears (2-DOF each, Feetech servos)      │
│    - Floor Projector (HDMI from Pi, controlled by ESP32):   │
│      • Power: Optocoupler switching (GPIO-controlled)       │
│      • Focus: Linear servo (PWM/UART, auto-focus)           │
│    - RGBD Camera + IMU (USB to Pi)                          │
│  • NECK MODULE (0x09):                                       │
│    - 3-DOF Servo Array (pan/tilt/roll, Feetech STS3215)    │
│    - 2× Presence Sensors (360° human detection)             │
│    - Smooth organic motion curves                           │
│  • TORSO MODULE (0x0A):                                      │
│    - 2.8" Square Display (animated beating heart, SPI)      │
│    - Thermal Printer (lists, reminders, recipes)            │
│    - Raspberry Pi Housing, Battery Pack, LED indicators     │
│  • BASE MODULE (0x0B):                                       │
│    - 2× Hoverboard BLDC Motors + ODrive (UART1)             │
│    - MPU6050 IMU (200Hz self-balancing PID)                 │
│    - Servo Kickstand (Feetech STS3215, UART2)              │
│  • PI MODULE (Orchestrator, no I2C address):                │
│    - Raspberry Pi 5 16GB + Hailo AI Kit (26 TOPS, PCIe)    │
│    - Speakerphone (USB, for voice I/O)                      │
│    - HDMI Output (video to Head+Ears projector)             │
│    - All ROS2 nodes, personality coordination               │
└──────────────────────────────────────────────────────────────┘

Communication Protocols:
━━━━━━━━ I2C: Pi ↔ 4× ESP32 modules (5-20ms latency, commands/sensor data)
- - - - - WiFi: Pi → Cloud AI APIs only (Claude/GPT-4)
━ ━ ━ ━ ━ SPI: ESP32 → OLED displays, heart display (high-speed graphics)
━·━·━·━·━ UART: ESP32 modules → servo controllers, motor controllers
─ ─ ─ ─ ─ USB: RGBD camera, Hailo AI Kit, Speakerphone → Raspberry Pi
━━━━━━━━ HDMI: Raspberry Pi → Head+Ears Module (Floor Projector video)
─·─·─·─·─ GPIO: Head+Ears ESP32 → Optocoupler → Projector Power
━·━·━·━·━ PWM/UART: Head+Ears ESP32 → Linear Servo (Projector Focus)
```

## Architectural Patterns

- **Layered Architecture (Intelligence → Orchestration → Module)**: Separates high-level reasoning (AI agents) from real-time control (ESP32 firmware), enabling independent development and testing. _Rationale: Matches robotics best practices, clear responsibility boundaries._

- **Smart Peripheral Pattern (I2C Slaves with Embedded Intelligence)**: ESP32 modules act as smart controllers containing full hardware drivers, animation engines, sensor processing, and real-time control loops (e.g., 200Hz self-balancing on Base module, projector power/focus control on Head+Ears module), receiving only high-level semantic commands from Pi. Pi sends HDMI video to projector but Head+Ears ESP32 controls power (GPIO → optocoupler) and focus (linear servo via PWM/UART), responding to I2C commands like `PROJECTOR_ON/OFF` and `FOCUS_NEAR/FAR`. _Rationale: Offloads real-time tasks from Linux-based Pi, reduces I2C traffic, enables autonomous execution, and follows separation of concerns (Pi=content, ESP32=hardware control)._

- **Hardware Abstraction via Driver Nodes**: ROS2 driver nodes on Pi translate between ROS2 topics (semantic) and I2C register writes (hardware-specific), isolating hardware details from application logic. _Rationale: Modules replaceable without changing high-level code, standard robotics pattern._

- **Hybrid AI Processing (Local Fast-Path + Cloud Reasoning)**: Hailo accelerator handles time-sensitive inference (Whisper STT) locally, while complex reasoning offloads to cloud LLMs. _Rationale: Balances latency (<3s requirement) with AI capability (GPT-4 quality personality)._

- **Event-Driven Communication (ROS2 Pub/Sub + I2C Interrupts)**: Asynchronous message passing between nodes, interrupt-driven I2C on ESP32s eliminates polling overhead. _Rationale: Responsive (<500ms expression sync), power-efficient, decouples producers/consumers._

- **Repository Pattern (Conversation & Config Storage)**: SQLite abstraction layer for data persistence, encapsulates queries. _Rationale: Testable without database, future migration to cloud storage possible._

- **Modular Firmware Pattern (Per-Module ESP32 Applications)**: Each module self-contained firmware project with OTA partition support. _Rationale: Independent development cycles, gradual rollout of updates, MECE principle enforcement._

- **Autonomous Real-Time Control Pattern (Base Self-Balancing)**: Critical timing loops (200Hz PID balancing) run entirely on ESP32 with hardware timers, Pi sends only abstract navigation commands. _Rationale: Linux cannot guarantee real-time deadlines, ESP32 FreeRTOS provides hard real-time guarantees essential for stability._

---
