# High Level Architecture

## Technical Summary

OLAF is a **distributed embedded robotics system** with hybrid AI intelligence, built on a three-layer architecture. The **Orchestration Layer** (Raspberry Pi 5 8GB + Hailo-8L AI Kit running ROS2 Humble) coordinates five independent **Module Layer** smart peripherals (ESP32-based: Head, Ears, Neck, Projector, Base) via I2C bus communication. The **Intelligence Layer** combines local AI acceleration (Hailo-accelerated Whisper STT for <200ms speech recognition) with cloud-based AI agents (Claude/GPT-4 APIs for reasoning and personality generation).

Key architectural decisions include: (1) I2C-only module communication eliminates WiFi complexity and achieves 5-20ms latency vs 80-200ms WiFi/ROS2, (2) ROS2 nodes run exclusively on Pi with ESP32-S3s as smart I2C slaves containing full hardware drivers, animation engines, and real-time control loops, (3) SPI-connected GC9A01 round TFT displays (240×240) enable 60 FPS full-color personality expression, (4) Bus servo controllers manage 4× ear servos (dedicated) and 4× neck+kickstand servos (shared) via serial bus, (5) ODrive motor controller provides closed-loop SLAM-grade odometry, (6) Self-balancing two-wheel base with 30 kg·cm kickstand managed via Neck module's servo controller (200Hz PID control runs on Base ESP32-S3), and (7) OTA firmware updates in V1 enable rapid iteration without robot disassembly. This architecture achieves <3s AI response latency (NFR1), supports 2-4 hour battery runtime (NFR4), and enables weekend-sprint modular development per MECE principles.

## Platform and Infrastructure Choice

**Platform: Self-Hosted Embedded Robotics System**

**Core Infrastructure:**
- **Compute Platform**: Raspberry Pi 5 8GB (quad-core ARM Cortex-A76 @ 2.4GHz)
- **AI Accelerator**: Hailo-8L AI Kit (13 TOPS, PCIe interface)
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

**Package Organization:**
- Firmware: Arduino/PlatformIO projects per module (modules/)
- Orchestrator: Python package with ROS2 nodes (orchestrator/)
- Hardware: 3D models, wiring, BOM (hardware/)
- Documentation: PRD, architecture, guides (docs/)

```
olaf/
├── .github/                    # CI/CD workflows (future)
├── docs/                       # Documentation
│   ├── prd.md
│   ├── architecture.md         # This document
│   ├── brief.md
│   └── epics/
├── hardware/                   # Physical design files
│   ├── 3d-models/             # STL files for 3D printing
│   ├── wiring/                # Fritzing diagrams, schematics
│   └── bom/                   # Bills of materials
├── modules/                    # ESP32 firmware (C/C++)
│   ├── head/
│   │   ├── firmware/
│   │   │   ├── head_controller.ino
│   │   │   ├── i2c_slave.cpp
│   │   │   ├── oled_driver_spi.cpp
│   │   │   ├── animation_engine.cpp
│   │   │   └── animations/
│   │   ├── platformio.ini
│   │   └── README.md
│   ├── ears/
│   ├── neck/
│   ├── projector/
│   └── base/
│       └── firmware/
│           ├── base_controller.ino
│           ├── balancing_controller.cpp
│           ├── odrive_uart.cpp
│           └── kickstand_control.cpp
├── orchestrator/              # Raspberry Pi software (Python)
│   ├── ros2_nodes/
│   │   ├── hardware_drivers/
│   │   ├── personality/
│   │   ├── ai_integration/
│   │   └── navigation/
│   ├── launch/
│   ├── config/
│   ├── ota_server/
│   └── requirements.txt
├── tests/
├── tools/
├── .gitignore
├── README.md
└── LICENSE
```

## High Level Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│           INTELLIGENCE LAYER (Hybrid AI)                    │
│  Local: Whisper STT (Hailo) | Cloud: Agent (Claude/GPT-4)  │
│  • Speech Recognition (Hailo-accelerated, local)            │
│  • Agent Reasoning & Tool Use (cloud, WiFi)                 │
│  • Multi-step Planning & Context Management                 │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       │ HTTPS/REST (WiFi - Cloud Only)
                       │
┌──────────────────────▼──────────────────────────────────────┐
│          ORCHESTRATION LAYER (Raspberry Pi 5 + Hailo)       │
│  • Personality Coordination  • SLAM Navigation              │
│  • Local AI (Whisper, Vision) • Sensor Fusion              │
│  • State Management         • Tool Execution                │
│  • I2C Master Controller    • OTA Server (HTTP)             │
└──┬─────────────┬─────────────┬─────────────┬────────────────────┘
   │             │             │             │
   │  I2C 0x08   │  I2C 0x09   │  I2C 0x0A   │  I2C 0x0B  (Wired Only)
   │             │             │             │
┌──▼──────┐  ┌──▼──────────┐ ┌▼──────────┐ ┌▼─────────┐
│  HEAD   │  │ EARS + NECK │ │   BODY    │ │   BASE   │
│ ESP32-S3│  │  ESP32-S3   │ │ ESP32-S3  │ │ ESP32-S3 │
│  +OTA   │  │   +OTA      │ │  +OTA     │ │  +OTA    │
└──┬──────┘  └─┬──────┬────┘ └┬────┬─────┘ └┬────┬────┘
   │           │      │       │    │        │    │
   │ SPI       │Servo │Servo  │SPI │Relay   │UART│Servo
   │ Eyes      │Bus A │Bus B  │LCD │+LEDs   │ODrv│Stand
   │(GC9A01)   │(4×)  │(3×)   │Hrt │WS2812B │+IMU│STS15
   │           │Ears  │Neck   │    │        │    │
┌──▼──────────────────────────────────────────────────────┐
│          MODULE LAYER (Physical Hardware)               │
│  • HEAD MODULE (0x08):                                  │
│    - 2× GC9A01 Round TFT Eyes (1.28", 240×240, SPI)    │
│    - mmWave Presence Sensor, Mic Array, Speaker        │
│  • EARS + NECK MODULE (0x09):                           │
│    - 4× Feetech SCS0009 (ears, Bus Servo Ctrl A)       │
│    - 3× Feetech STS3215 (neck, Bus Servo Ctrl B)       │
│  • BODY MODULE (0x0A):                                  │
│    - 1× GC9A01 Heart Display (1.28", 240×240, SPI)     │
│    - DLP Projector (HDMI to Pi, power via relay)       │
│    - WS2812B RGB LEDs (status/mood indicators)         │
│  • BASE MODULE (0x0B):                                  │
│    - 2× Hoverboard Motors + ODrive (UART)              │
│    - MPU6050 IMU (200Hz balancing)                     │
│    - 1× Feetech STS3215 Kickstand (30 kg·cm)           │
│  • ORCHESTRATOR (Pi 5):                                 │
│    - OAK-D Pro RGBD Camera (USB)                       │
│    - Hailo-8L AI Accelerator (PCIe)                    │
└─────────────────────────────────────────────────────────┘

Communication Protocols:
━━━━━━━━ I2C: Pi ↔ All ESP32-S3 modules (commands, sensor data)
- - - - - WiFi: Pi → Cloud AI APIs only (Claude/GPT-4)
━ ━ ━ ━ ━ SPI: ESP32-S3 → GC9A01 TFT displays (high-speed color graphics)
━·━·━·━·━ UART: Base ESP32-S3 → ODrive motor controller
─ ─ ─ ─ ─ Serial Bus: ESP32-S3 → Bus Servo Controllers (STSC series)
```

## Architectural Patterns

- **Layered Architecture (Intelligence → Orchestration → Module)**: Separates high-level reasoning (AI agents) from real-time control (ESP32 firmware), enabling independent development and testing. _Rationale: Matches robotics best practices, clear responsibility boundaries._

- **Smart Peripheral Pattern (I2C Slaves with Embedded Intelligence)**: ESP32 modules act as smart controllers containing full hardware drivers, animation engines, sensor processing, and real-time control loops (e.g., 200Hz self-balancing on Base module), receiving only high-level semantic commands from Pi. _Rationale: Offloads real-time tasks from Linux-based Pi, reduces I2C traffic, enables autonomous execution._

- **Hardware Abstraction via Driver Nodes**: ROS2 driver nodes on Pi translate between ROS2 topics (semantic) and I2C register writes (hardware-specific), isolating hardware details from application logic. _Rationale: Modules replaceable without changing high-level code, standard robotics pattern._

- **Hybrid AI Processing (Local Fast-Path + Cloud Reasoning)**: Hailo accelerator handles time-sensitive inference (Whisper STT) locally, while complex reasoning offloads to cloud LLMs. _Rationale: Balances latency (<3s requirement) with AI capability (GPT-4 quality personality)._

- **Event-Driven Communication (ROS2 Pub/Sub + I2C Interrupts)**: Asynchronous message passing between nodes, interrupt-driven I2C on ESP32s eliminates polling overhead. _Rationale: Responsive (<500ms expression sync), power-efficient, decouples producers/consumers._

- **Repository Pattern (Conversation & Config Storage)**: SQLite abstraction layer for data persistence, encapsulates queries. _Rationale: Testable without database, future migration to cloud storage possible._

- **Modular Firmware Pattern (Per-Module ESP32 Applications)**: Each module self-contained firmware project with OTA partition support. _Rationale: Independent development cycles, gradual rollout of updates, MECE principle enforcement._

- **Autonomous Real-Time Control Pattern (Base Self-Balancing)**: Critical timing loops (200Hz PID balancing) run entirely on ESP32 with hardware timers, Pi sends only abstract navigation commands. _Rationale: Linux cannot guarantee real-time deadlines, ESP32 FreeRTOS provides hard real-time guarantees essential for stability._

---
