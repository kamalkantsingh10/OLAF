# High Level Architecture

## Technical Summary

OLAF is a **hybrid-control robotics system** with AI intelligence, built on a three-layer architecture. The **Orchestration Layer** (Raspberry Pi 5 16GB + Sunfounder Fusion HAT+ + Hailo AI Kit running ROS2 Jazzy on Ubuntu 24.04) provides both central coordination and direct hardware control, while two **Module Layer** ESP32 controllers handle real-time tasks (Head for eye animations, Base for self-balancing). The **Intelligence Layer** combines local AI acceleration (Hailo-accelerated Whisper STT for <200ms speech recognition) with cloud-based AI agents (Claude/GPT-4 APIs for reasoning and personality generation).

Key architectural decisions include: (1) **Hybrid control topology** — Pi directly controls servos via USB (Waveshare Bus Servo Adapters for neck/ears), LEDs via Fusion HAT (WS2812 indicators), and kickstand via HAT PWM, while ESP32s handle real-time tasks requiring dedicated timing, (2) **Reduced ESP32 count** (4→2) eliminates soldering complexity by leveraging Fusion HAT's pre-built interfaces, (3) **I2C for ESP32 communication** (Head 0x10, Base 0x11) achieves 5-20ms latency for eye expressions and motor commands, (4) **Five ROS2 modules** via MECE principle: Head (OLED eyes via ESP32), Base (self-balancing via ESP32), Neck (3-DOF servos via USB), Ears (2-DOF articulated ears via USB), Indicator (24 WS2812 LEDs for interaction/status/PID visualization), (5) **4" Pi display as heart** replaces dedicated Torso ESP32, (6) **Dual battery architecture** — 36V hoverboard battery (motors, servos, ESP32s) with buck converters (36V→12V, 36V→5V), plus 2000mAh 7.4V battery (Pi, HAT, AI HAT, display, WS2812, kickstand) for isolated logic power, (7) **Self-balancing** with 200Hz PID on Base ESP32 using MPU6050 IMU and ODrive, (8) **Kickstand** via model plane landing gear servos (4.8-7.4V) controlled by Pi HAT PWM with long signal wires to base, and (9) OTA firmware updates for the 2 ESP32s enable rapid iteration. This architecture achieves <3s AI response latency, supports 2-4 hour battery runtime, and minimizes soldering through use of pre-built adapter boards.

## Platform and Infrastructure Choice

**Platform: Self-Hosted Embedded Robotics System**

**Core Infrastructure:**
- **Compute Platform**: Raspberry Pi 5 16GB (quad-core ARM Cortex-A76 @ 2.4GHz)
- **Operating System**: Ubuntu 24.04 LTS (64-bit)
- **Expansion HAT**: Sunfounder Fusion HAT+ (PWM, WS2812, I2C, I2S audio)
- **AI Accelerator**: Hailo AI Kit (26 TOPS, PCIe interface)
- **Deployment Model**: Self-contained mobile robot, no external servers
- **Network Requirements**: Home WiFi (2.4/5GHz) for cloud API access only

**Key Infrastructure Services:**
- **Local AI Inference**: Hailo-accelerated Whisper STT (~150-200ms latency)
- **Cloud AI Services**: Anthropic Claude API (V1), OpenAI GPT-4 API (future)
- **Database**: SQLite (embedded, conversation history + config)
- **OTA Server**: HTTP server on Pi (Flask/FastAPI) serving firmware binaries
- **Real-Time Communication**: I2C bus (400kHz-1MHz) for ESP32 coordination
- **Servo Communication**: USB serial via Waveshare Bus Servo Adapters

**Deployment Host and Regions**: Local (Raspberry Pi), no cloud hosting. WiFi for API calls only.

## Repository Structure

**Structure:** Monorepo

**Monorepo Tool:** Git (no specialized monorepo tool needed for this scale)

**Organization Principle:** Module-first architecture with hybrid control topology. ESP32 modules (Head, Base) contain firmware for real-time tasks, while Pi-controlled modules (Neck, Ears, Indicator) are driver-only packages managed directly via Fusion HAT interfaces.

```
olaf/
├── modules/                    # All 5 modules (physical subsystems)
│   ├── head/                   # Head Module (I2C 0x10) - ESP32
│   │   ├── firmware/           # ESP32 firmware
│   │   │   ├── src/            # Source files (main.cpp, drivers/)
│   │   │   ├── include/        # Header files
│   │   │   ├── test/           # Unit tests
│   │   │   └── platformio.ini  # Build configuration
│   │   ├── hardware/           # Physical designs
│   │   │   ├── mechanical/     # 3D models (STEP, STL)
│   │   │   └── bom.csv         # Bill of materials
│   │   ├── tests/              # Module-specific integration tests
│   │   ├── diagnostics/        # Diagnostic tools (eyes)
│   │   ├── scripts/            # Build, flash, calibrate scripts
│   │   ├── wiring.md           # Pin assignments, circuits
│   │   ├── assembly.md         # Assembly instructions
│   │   └── README.md           # Module overview
│   │
│   ├── base/                   # Base Module (I2C 0x11) - ESP32
│   │   ├── firmware/           # ESP32 firmware
│   │   │   ├── src/            # main.cpp, imu.cpp, odrive.cpp
│   │   │   ├── include/        # Header files
│   │   │   ├── test/           # Unit tests
│   │   │   └── platformio.ini  # Build configuration
│   │   ├── hardware/           # Physical designs
│   │   │   ├── mechanical/     # 3D models (wheels, frame)
│   │   │   └── bom.csv         # Bill of materials
│   │   ├── tests/              # Module-specific integration tests
│   │   ├── diagnostics/        # Diagnostic tools (balance, motors)
│   │   ├── scripts/            # Build, flash, calibrate scripts
│   │   ├── wiring.md           # Pin assignments, circuits
│   │   ├── assembly.md         # Assembly instructions
│   │   └── README.md           # Module overview
│   │
│   ├── neck/                   # Neck Module - Pi-controlled (USB serial)
│   │   ├── hardware/           # Physical designs
│   │   │   ├── mechanical/     # 3D models (neck mechanism)
│   │   │   └── bom.csv         # Bill of materials (3× STS3215)
│   │   ├── tests/              # Module-specific integration tests
│   │   ├── diagnostics/        # Diagnostic tools (servo test)
│   │   ├── scripts/            # Calibration scripts
│   │   ├── assembly.md         # Assembly instructions
│   │   └── README.md           # Module overview
│   │
│   ├── ears/                   # Ears Module - Pi-controlled (USB serial)
│   │   ├── hardware/           # Physical designs
│   │   │   ├── mechanical/     # 3D models (ear mechanisms)
│   │   │   └── bom.csv         # Bill of materials (4× SCS0009)
│   │   ├── tests/              # Module-specific integration tests
│   │   ├── diagnostics/        # Diagnostic tools (servo test)
│   │   ├── scripts/            # Calibration scripts
│   │   ├── assembly.md         # Assembly instructions
│   │   └── README.md           # Module overview
│   │
│   ├── indicator/              # Indicator Module - Pi-controlled (WS2812)
│   │   ├── hardware/           # Physical designs
│   │   │   ├── mechanical/     # 3D models (LED strip mounts)
│   │   │   └── bom.csv         # Bill of materials (3× 8-LED strips)
│   │   ├── tests/              # Module-specific tests
│   │   ├── diagnostics/        # LED test patterns
│   │   └── README.md           # Module overview
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
│       ├── olaf_drivers/       # Hardware driver nodes
│       │   ├── olaf_head/      # I2C driver → Head ESP32 (eyes)
│       │   ├── olaf_base/      # I2C driver → Base ESP32 (motors, IMU)
│       │   ├── olaf_neck/      # USB serial → Waveshare (neck servos)
│       │   ├── olaf_ears/      # USB serial → Waveshare (ear servos)
│       │   └── olaf_indicator/ # Fusion HAT → WS2812 (LED strips)
│       ├── olaf_personality/   # Personality coordination
│       ├── olaf_ai/            # AI integration (Whisper, agents)
│       └── olaf_navigation/    # SLAM, navigation stack
│
├── scripts/                    # System-wide automation
│   ├── setup/                  # Environment setup
│   ├── build/                  # Build all modules
│   ├── flash/                  # Flash ESP32 modules
│   ├── test/                   # Test all modules
│   └── deploy/                 # OTA deployment
│
├── tools/                      # System-wide tools
│   ├── diagnostics/            # System diagnostics (i2c_scanner, system_health)
│   ├── calibration/            # Cross-module calibration
│   ├── simulators/             # I2C module simulators
│   └── utils/                  # General utilities (log analyzer)
│
├── tests/                      # System-wide integration tests
│   ├── integration/            # Cross-module tests (personality sync, navigation)
│   └── fixtures/               # Test fixtures, mock data
│
├── config/                     # System configuration
│   ├── i2c/                    # I2C bus configuration (Head 0x10, Base 0x11)
│   ├── ros2/                   # ROS2 parameters
│   ├── servo/                  # Servo calibration (neck, ears)
│   └── firmware/               # Firmware configs (WiFi, OTA)
│
├── docs/                       # System-level documentation
│   ├── architecture.md         # System architecture (this document)
│   ├── brief.md                # Project brief
│   ├── prd/                    # Product requirements (sharded)
│   ├── guides/                 # Build guides, tutorials
│   ├── api/                    # API references (I2C, ROS2, USB serial)
│   └── media/                  # Images, diagrams, videos
│
├── hardware/                   # System-level hardware only
│   ├── power/                  # Power distribution (buck converters)
│   └── enclosure/              # Main body/torso housing Pi + HATs
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

**Module Responsibilities:**

| Module | Controller | Interface | Hardware |
|--------|------------|-----------|----------|
| **Head (0x10)** | ESP32 | I2C | 2× OLED eyes (GC9A01, SPI) |
| **Base (0x11)** | ESP32 | I2C | ODrive (UART), MPU6050 (I2C), self-balancing PID |
| **Neck** | Pi direct | USB Serial (Waveshare) | 3× STS3215 servos (pan/tilt/roll) |
| **Ears** | Pi direct | USB Serial (Waveshare) | 4× SCS0009 servos (2-DOF × 2 ears) |
| **Indicator** | Pi direct | Fusion HAT WS2812 | 24 LEDs (3× 8-LED strips) |

**Additional Pi-Controlled Hardware:**
- **Heart Display:** 4" Pi display (SPI) — animated heart, status
- **Kickstand:** 2× landing gear servos (4.8-7.4V) via Fusion HAT PWM
- **Speaker:** I2S via Fusion HAT audio
- **AI:** Hailo AI HAT (26 TOPS) for Whisper STT

**Key Organizational Benefits:**
- **Reduced Complexity:** Only 2 ESP32 modules to flash/debug (Head, Base)
- **No Soldering Required:** Fusion HAT provides pre-built interfaces for servos, LEDs, PWM
- **True Modularity:** ESP32 modules self-contained (firmware + hardware + tests)
- **Clear Ownership:** Pi-controlled modules are ROS2 packages only (no firmware)
- **Simplified Wiring:** USB cables for servos, single data line for WS2812
- **Matches Architecture:** Code structure mirrors the 5-module hybrid control topology

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
│       ORCHESTRATION LAYER (Pi 5 + Fusion HAT+ + AI HAT)         │
│                   Powered by 2000mAh 7.4V Battery               │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ RASPBERRY PI 5 16GB (Ubuntu 24.04)                         │ │
│  │  • ROS2 Jazzy (5 driver nodes)                             │ │
│  │  • Personality Coordination  • State Management            │ │
│  │  • SLAM Navigation          • AI Integration               │ │
│  │  • I2C Master (GPIO2/3)     • OTA Server                   │ │
│  └────────────────────────────────────────────────────────────┘ │
│  ┌─────────────────┐ ┌─────────────────┐ ┌──────────────────┐   │
│  │ FUSION HAT+     │ │ AI HAT (Hailo)  │ │ 4" HEART DISPLAY │   │
│  │ • WS2812 port   │ │ • 26 TOPS       │ │ • Animated heart │   │
│  │ • PWM (P0-P3)   │ │ • Whisper STT   │ │ • Status info    │   │
│  │ • I2C bridge    │ │ • Vision models │ │ • SPI from Pi    │   │
│  │ • I2S speaker   │ └─────────────────┘ └──────────────────┘   │
│  └────────┬────────┘                                            │
│           │                                                      │
└───┬───────┼───────┬─────────────────┬───────────────────────────┘
    │       │       │                 │
    │I2C    │PWM    │USB ×2           │WS2812
    │       │       │                 │Data
┌───▼───┐ ┌─▼─────┐ │           ┌─────▼─────────────────────────┐
│ HEAD  │ │KICK-  │ │           │     WS2812 INDICATOR STRIP    │
│ ESP32 │ │STAND  │ │           │     (24 LEDs, 3×8 daisy)      │
│ 0x10  │ │2×Servo│ │           │  ┌────────┬────────┬────────┐ │
└───┬───┘ │4.8-7.4V│ │          │  │Strip 1 │Strip 2 │Strip 3 │ │
    │     │(at base)│ │          │  │Interact│Status  │PID     │ │
┌───▼───┐ └────────┘ │           │  │8 LEDs  │8 LEDs  │8 LEDs  │ │
│2×OLED │             │           └──┴────────┴────────┴────────┘ │
│Eyes   │             │
│SPI    │       ┌─────┴─────────────────────────────┐
└───────┘       │                                   │
                ▼                                   ▼
    ┌───────────────────────┐         ┌───────────────────────┐
    │ WAVESHARE BUS SERVO   │         │ WAVESHARE BUS SERVO   │
    │ ADAPTER (A) - NECK    │         │ ADAPTER (A) - EARS    │
    │ USB Serial            │         │ USB Serial            │
    └───────────┬───────────┘         └───────────┬───────────┘
                │                                 │
    ┌───────────▼───────────┐         ┌───────────▼───────────┐
    │ NECK SERVOS           │         │ EAR SERVOS            │
    │ 3× Feetech STS3215    │         │ 4× Feetech SCS0009    │
    │ Pan / Tilt / Roll     │         │ 2-DOF × 2 ears        │
    │ 30 kg·cm torque       │         │ Daisy-chained         │
    └───────────────────────┘         └───────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    I2C BUS (GPIO2 SDA, GPIO3 SCL)               │
└────────────────────────┬────────────────────────────────────────┘
                         │
         ┌───────────────┴───────────────┐
         │                               │
    ┌────▼────┐                     ┌────▼────┐
    │  HEAD   │                     │  BASE   │
    │  ESP32  │                     │  ESP32  │
    │  0x10   │                     │  0x11   │
    └────┬────┘                     └────┬────┘
         │                               │
    ┌────▼────────────┐         ┌────────▼────────────────────┐
    │ 2× OLED EYES    │         │ MOTOR & BALANCE SYSTEM      │
    │ 128×64, SPI     │         │ ┌─────────┐ ┌─────────────┐ │
    │ 30-60 FPS       │         │ │ MPU6050 │ │ ODrive v3.6 │ │
    │ GC9A01 driver   │         │ │ IMU     │ │ UART        │ │
    │ Eye animations  │         │ │ 200Hz   │ │ 2× BLDC     │ │
    └─────────────────┘         │ └─────────┘ └─────────────┘ │
                                └─────────────────────────────┘
```

### Power Distribution

```
┌─────────────────────────────────────────────────────────────────┐
│              36V HOVERBOARD BATTERY (Charging Port A)           │
└────────┬────────────────────┬────────────────────┬──────────────┘
         │                    │                    │
         │ 36V Direct         │ 36V→12V Buck       │ 36V→5V Buck
         │                    │                    │
    ┌────▼────┐         ┌─────▼─────┐        ┌─────▼─────────────┐
    │ ODrive  │         │ SERVOS    │        │ ESP32s + OLED     │
    │ + BLDC  │         │ Neck ×3   │        │ • Head ESP32      │
    │ Motors  │         │ Ears ×4   │        │ • Base ESP32      │
    │         │         │ (12V in)  │        │ • 2× OLED eyes    │
    └─────────┘         └───────────┘        └───────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│          2000mAh 7.4V BATTERY (Fusion HAT, Charging Port B)     │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                    ┌─────────▼─────────┐
                    │ LOGIC SYSTEM      │
                    │ • Raspberry Pi 5  │
                    │ • Fusion HAT+     │
                    │ • AI HAT (Hailo)  │
                    │ • 4" Heart Display│
                    │ • WS2812 (24 LED) │
                    │ • Kickstand ×2    │
                    └───────────────────┘
```

### Communication Protocols

```
━━━━━━━━ I2C: Pi ↔ 2× ESP32 (Head 0x10, Base 0x11) - 5-20ms latency
─ ─ ─ ─  USB Serial: Pi ↔ 2× Waveshare Adapters (Neck, Ears servos)
- - - -  WiFi: Pi → Cloud AI APIs only (Claude/GPT-4)
━ ━ ━ ━  SPI: Head ESP32 → OLED displays (high-speed graphics)
━·━·━·━  UART: Base ESP32 → ODrive (motor commands, odometry)
────────  WS2812: Pi Fusion HAT → 24 LED strip (single data line)
─·─·─·─  PWM: Pi Fusion HAT → Kickstand servos (long wire to base)
```

### ROS2 Node Topology

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS2 JAZZY (on Pi 5 / Ubuntu 24.04)          │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────────┐  │
│  │ olaf_head    │  │ olaf_base    │  │ olaf_indicator        │  │
│  │ I2C → 0x10   │  │ I2C → 0x11   │  │ HAT WS2812            │  │
│  │              │  │              │  │                       │  │
│  │ /head/expr   │  │ /cmd_vel     │  │ /indicator/interact   │  │
│  │ /head/blink  │  │ /odom        │  │ /indicator/status     │  │
│  │ /head/look   │  │ /imu         │  │ /indicator/pid        │  │
│  └──────────────┘  └──────────────┘  └───────────────────────┘  │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────────┐  │
│  │ olaf_neck    │  │ olaf_ears    │  │ olaf_personality      │  │
│  │ USB Serial   │  │ USB Serial   │  │ AI Integration        │  │
│  │              │  │              │  │                       │  │
│  │ /neck/pose   │  │ /ears/emote  │  │ /speech/text          │  │
│  │ /neck/lookat │  │ /ears/perk   │  │ /ai/response          │  │
│  └──────────────┘  └──────────────┘  └───────────────────────┘  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Architectural Patterns

- **Layered Architecture (Intelligence → Orchestration → Module)**: Separates high-level reasoning (AI agents) from real-time control (ESP32 firmware) and direct hardware control (Pi via Fusion HAT), enabling independent development and testing. _Rationale: Matches robotics best practices, clear responsibility boundaries._

- **Hybrid Control Topology (ESP32 + Pi Direct)**: Real-time critical tasks (eye animations at 60 FPS, self-balancing at 200Hz) run on dedicated ESP32 controllers, while latency-tolerant hardware (servos, LEDs, kickstand) is controlled directly by Pi via Fusion HAT and USB adapters. _Rationale: Minimizes ESP32 count (4→2), eliminates soldering, leverages pre-built interfaces while preserving real-time guarantees where needed._

- **Smart Peripheral Pattern (I2C Slaves with Embedded Intelligence)**: Head and Base ESP32 modules act as smart controllers containing full hardware drivers, animation engines, and real-time control loops (200Hz self-balancing on Base, 60 FPS eye rendering on Head), receiving only high-level semantic commands from Pi via I2C. _Rationale: Offloads real-time tasks from Linux-based Pi, reduces I2C traffic, enables autonomous execution._

- **Direct HAT Control Pattern (Pi-Controlled Peripherals)**: Neck servos, ear servos, indicator LEDs, and kickstand are controlled directly by Pi through Fusion HAT interfaces (USB serial for Waveshare adapters, WS2812 data line for LEDs, PWM for kickstand servos). ROS2 nodes manage these peripherals without intermediate microcontrollers. _Rationale: Eliminates unnecessary ESP32s, simplifies wiring (USB cables vs. custom PCBs), reduces firmware maintenance burden._

- **Hardware Abstraction via Driver Nodes**: ROS2 driver nodes on Pi translate between ROS2 topics (semantic) and hardware interfaces (I2C registers for ESP32s, USB serial for Waveshare adapters, HAT APIs for LEDs/PWM), isolating hardware details from application logic. _Rationale: Modules replaceable without changing high-level code, standard robotics pattern._

- **Hybrid AI Processing (Local Fast-Path + Cloud Reasoning)**: Hailo accelerator handles time-sensitive inference (Whisper STT) locally, while complex reasoning offloads to cloud LLMs. _Rationale: Balances latency (<3s requirement) with AI capability (GPT-4 quality personality)._

- **Event-Driven Communication (ROS2 Pub/Sub)**: Asynchronous message passing between ROS2 nodes enables loose coupling. ESP32 modules use interrupt-driven I2C to minimize latency. _Rationale: Responsive (<500ms expression sync), power-efficient, decouples producers/consumers._

- **Repository Pattern (Conversation & Config Storage)**: SQLite abstraction layer for data persistence, encapsulates queries. _Rationale: Testable without database, future migration to cloud storage possible._

- **Reduced Firmware Footprint (2 ESP32 Modules Only)**: Only Head (eyes) and Base (balancing) require dedicated ESP32 firmware. All other hardware managed by Pi ROS2 nodes. _Rationale: Fewer firmware projects to maintain, faster iteration, reduced OTA complexity._

- **Autonomous Real-Time Control Pattern (Base Self-Balancing)**: Critical timing loops (200Hz PID balancing) run entirely on Base ESP32 with hardware timers, Pi sends only abstract navigation commands. _Rationale: Linux cannot guarantee real-time deadlines, ESP32 FreeRTOS provides hard real-time guarantees essential for stability._

- **Dual Battery Isolation (Power + Logic)**: 36V hoverboard battery powers high-current systems (motors, servos, ESP32s) via buck converters, while dedicated 2000mAh 7.4V battery powers logic systems (Pi, HATs, display, LEDs) via Fusion HAT. _Rationale: Isolates sensitive electronics from motor noise, enables independent charging, prevents brownouts during motor startup._

---
