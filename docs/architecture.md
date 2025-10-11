# OLAF Fullstack Architecture Document

## Introduction

This document outlines the complete fullstack architecture for **OLAF (Open Loveable AI Friend)**, an open-source modular AI companion robot. Unlike traditional software applications, OLAF is a **physical embodied AI system** that integrates embedded hardware (ESP32 modules), real-time robotic control (ROS2), local AI acceleration (Hailo AI Kit), and cloud AI reasoning (Claude/GPT-4 agents).

This architecture document serves as the single source of truth for implementing OLAF's three-layer system:
1. **Module Layer**: Independent hardware modules (Head, Ears, Neck, Projector, Base) each powered by ESP32
2. **Orchestration Layer**: Raspberry Pi 5 + Hailo AI Kit coordinating behaviors, AI, and navigation
3. **Intelligence Layer**: Hybrid local AI (Whisper STT) + cloud AI agents for reasoning and personality

The unified approach treats the embedded firmware, orchestration software, and AI integration as a cohesive fullstack system designed for AI-assisted development and maker replicability.

### Starter Template or Existing Project

**Status**: N/A - Greenfield project with pre-defined architectural constraints from PRD.

**Existing Decisions:**
- **Repository structure**: Monorepo confirmed in Technical Assumptions (already documented with clear folder structure)
- **Core technology commitments**:
  - ROS2 Humble for orchestration layer communication
  - Raspberry Pi 5 8GB + Hailo AI Kit for orchestration
  - ESP32 microcontrollers for module layer
  - Python 3.10+ for orchestrator
  - C/C++ (Arduino/ESP-IDF) for module firmware
  - I2C communication between Pi and ESP32 modules (WiFi for cloud API only)
  - SPI for OLED displays (high-speed graphics)
  - ODrive motor controller for base mobility

**Architecture Type**: This is a **hybrid physical robotics + AI orchestration system**:
- **Physical layer**: Embedded systems (ESP32 smart peripherals)
- **Orchestration layer**: Raspberry Pi running ROS2 + Python
- **Intelligence layer**: Hybrid local AI (Hailo) + cloud AI agents (Claude/GPT-4 API)

### Change Log

| Date | Version | Description | Author |
|------|---------|-------------|--------|
| 2025-10-11 | v1.0 | Initial architecture document | Winston (Architect Agent) |

---

## High Level Architecture

### Technical Summary

OLAF is a **distributed embedded robotics system** with hybrid AI intelligence, built on a three-layer architecture. The **Orchestration Layer** (Raspberry Pi 5 8GB + Hailo-8L AI Kit running ROS2 Humble) coordinates five independent **Module Layer** smart peripherals (ESP32-based: Head, Ears, Neck, Projector, Base) via I2C bus communication. The **Intelligence Layer** combines local AI acceleration (Hailo-accelerated Whisper STT for <200ms speech recognition) with cloud-based AI agents (Claude/GPT-4 APIs for reasoning and personality generation).

Key architectural decisions include: (1) I2C-only module communication eliminates WiFi complexity and achieves 5-20ms latency vs 80-200ms WiFi/ROS2, (2) ROS2 nodes run exclusively on Pi with ESP32s as smart I2C slaves containing full hardware drivers, animation engines, and real-time control loops, (3) SPI-connected OLEDs enable 30-60 FPS smooth personality expression, (4) ODrive motor controller provides closed-loop SLAM-grade odometry, (5) Self-balancing two-wheel base with kickstand managed entirely by Base ESP32 (200Hz PID control), and (6) OTA firmware updates in V1 enable rapid iteration without robot disassembly. This architecture achieves <3s AI response latency (NFR1), supports 2-4 hour battery runtime (NFR4), and enables weekend-sprint modular development per MECE principles.

### Platform and Infrastructure Choice

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

### Repository Structure

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

### High Level Architecture Diagram

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
└──┬────────┬────────┬────────┬────────┬──────────────────────┘
   │        │        │        │        │
   │  I2C   │  I2C   │  I2C   │  I2C   │  I2C (Wired Only)
   │ 0x08   │ 0x09   │ 0x0A   │ 0x0B   │ 0x0C
   │        │        │        │        │
┌──▼───┐ ┌─▼────┐ ┌─▼────┐ ┌─▼──────┐ ┌▼─────┐
│ HEAD │ │ EARS │ │ NECK │ │PROJECT-│ │ BASE │
│ESP32 │ │ESP32 │ │ESP32 │ │OR ESP32│ │ESP32 │
│+OTA  │ │+OTA  │ │+OTA  │ │ +OTA   │ │+OTA  │
└──┬───┘ └──┬───┘ └──┬───┘ └───┬────┘ └──┬───┘
   │        │        │         │         │
   │ SPI    │ Servo  │ Servo   │ DLP     │ UART
   │ OLEDs  │ I2C    │ I2C     │ Control │ ODrive
   │        │        │         │         │ +IMU
   │        │        │         │         │ +Servo
   │        │        │         │         │(Kickstand)
┌──▼────────▼────────▼─────────▼─────────▼────┐
│       MODULE LAYER (Physical Hardware)       │
│  • 2× OLED Eyes (SPI, 30-60 FPS)            │
│  • RGBD Camera (USB to Pi)                   │
│  • 2× Ears (2-DOF, Feetech SCS0009)         │
│  • Neck (3-DOF, Feetech STS3215)            │
│  • Floor Projector (DLP)                     │
│  • Self-Balancing Base:                      │
│    - Hoverboard Motors + ODrive (UART)       │
│    - MPU6050 IMU (200Hz balancing)          │
│    - Kickstand Servo (stationary mode)      │
│  • mmWave Presence Sensor                    │
│  • Microphone Array, Speaker/Beeper          │
└──────────────────────────────────────────────┘

Communication Protocols:
━━━━━━━━ I2C: Pi ↔ All ESP32 modules (commands, sensor data)
- - - - - WiFi: Pi → Cloud AI APIs only (Claude/GPT-4)
━ ━ ━ ━ ━ SPI: ESP32 → OLED displays (high-speed graphics)
━·━·━·━·━ UART: Base ESP32 → ODrive motor controller
```

### Architectural Patterns

- **Layered Architecture (Intelligence → Orchestration → Module)**: Separates high-level reasoning (AI agents) from real-time control (ESP32 firmware), enabling independent development and testing. _Rationale: Matches robotics best practices, clear responsibility boundaries._

- **Smart Peripheral Pattern (I2C Slaves with Embedded Intelligence)**: ESP32 modules act as smart controllers containing full hardware drivers, animation engines, sensor processing, and real-time control loops (e.g., 200Hz self-balancing on Base module), receiving only high-level semantic commands from Pi. _Rationale: Offloads real-time tasks from Linux-based Pi, reduces I2C traffic, enables autonomous execution._

- **Hardware Abstraction via Driver Nodes**: ROS2 driver nodes on Pi translate between ROS2 topics (semantic) and I2C register writes (hardware-specific), isolating hardware details from application logic. _Rationale: Modules replaceable without changing high-level code, standard robotics pattern._

- **Hybrid AI Processing (Local Fast-Path + Cloud Reasoning)**: Hailo accelerator handles time-sensitive inference (Whisper STT) locally, while complex reasoning offloads to cloud LLMs. _Rationale: Balances latency (<3s requirement) with AI capability (GPT-4 quality personality)._

- **Event-Driven Communication (ROS2 Pub/Sub + I2C Interrupts)**: Asynchronous message passing between nodes, interrupt-driven I2C on ESP32s eliminates polling overhead. _Rationale: Responsive (<500ms expression sync), power-efficient, decouples producers/consumers._

- **Repository Pattern (Conversation & Config Storage)**: SQLite abstraction layer for data persistence, encapsulates queries. _Rationale: Testable without database, future migration to cloud storage possible._

- **Modular Firmware Pattern (Per-Module ESP32 Applications)**: Each module self-contained firmware project with OTA partition support. _Rationale: Independent development cycles, gradual rollout of updates, MECE principle enforcement._

- **Autonomous Real-Time Control Pattern (Base Self-Balancing)**: Critical timing loops (200Hz PID balancing) run entirely on ESP32 with hardware timers, Pi sends only abstract navigation commands. _Rationale: Linux cannot guarantee real-time deadlines, ESP32 FreeRTOS provides hard real-time guarantees essential for stability._

---

## ROS2 Node Architecture

All ROS2 nodes run on **Raspberry Pi only**. ESP32 modules are smart I2C slave peripherals.

### Node Structure

```
RASPBERRY PI (ROS2 Humble)
═══════════════════════════════════════════════════════════════

┌─────────────────────────────────────────────────────────────┐
│               HIGH-LEVEL APPLICATION NODES                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  /olaf/personality_coordinator     (Python)                │
│    • Coordinates eyes + ears + neck + beeps                │
│    • Subscribes: /olaf/ai/emotion_command                  │
│    • Publishes: /olaf/head/eye_expression, etc.            │
│                                                             │
│  /olaf/ai_agent                    (Python)                │
│    • Claude/GPT-4 API integration                          │
│    • Whisper STT via Hailo                                 │
│    • Publishes: /olaf/ai/emotion_command                   │
│                                                             │
│  /olaf/navigation                  (Python + Nav2)         │
│    • SLAM (Cartographer/RTAB-Map)                          │
│    • Subscribes: /olaf/base/odometry                       │
│    • Publishes: /olaf/base/velocity_cmd                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                             │
                    ROS2 Topics (internal)
                             │
┌─────────────────────────────────────────────────────────────┐
│          HARDWARE DRIVER NODES (I2C Bridge)                 │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  /olaf/head_driver          (Python + smbus2)              │
│    • I2C Address: 0x08                                     │
│    • Subscribes: /olaf/head/eye_expression, /blink         │
│    • Publishes: /olaf/head/presence, /status               │
│                                                             │
│  /olaf/base_driver          (Python + smbus2)              │
│    • I2C Address: 0x0C                                     │
│    • Subscribes: /olaf/base/velocity_cmd                   │
│    • Publishes: /olaf/base/odometry, /balance_status       │
│    • Sends abstract commands: "move:0.5", "relax"          │
│    • ESP32 handles 200Hz self-balancing autonomously       │
│                                                             │
│  ... (ears, neck, projector drivers)                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
           │          │          │          │          │
           │ I2C      │ I2C      │ I2C      │ I2C      │ I2C
           │ 0x08     │ 0x09     │ 0x0A     │ 0x0B     │ 0x0C
           ▼          ▼          ▼          ▼          ▼
     ┌─────────┐┌─────────┐┌─────────┐┌─────────┐┌─────────┐
     │  ESP32  ││  ESP32  ││  ESP32  ││  ESP32  ││  ESP32  │
     │  HEAD   ││  EARS   ││  NECK   ││PROJECTOR││  BASE   │
     │         ││         ││         ││         ││         │
     │ • I2C   ││ • I2C   ││ • I2C   ││ • I2C   ││ • I2C   │
     │ • SPI   ││ • Servo ││ • Servo ││ • DLP   ││ • UART  │
     │   OLED  ││   I2C   ││   I2C   ││  HDMI   ││  ODrive │
     │ • mmWave││         ││         ││         ││ • IMU   │
     │         ││         ││         ││         ││ • PID   │
     │         ││         ││         ││         ││ • Servo │
     │         ││         ││         ││         ││(Kickstand)│
     └─────────┘└─────────┘└─────────┘└─────────┘└─────────┘
```

### Intelligence Distribution

| Task | **Raspberry Pi** | **ESP32** |
|------|------------------|-----------|
| **AI Reasoning** | ✅ Claude/GPT-4 API calls | ❌ |
| **Personality Logic** | ✅ Choose emotion + intensity | ❌ |
| **SLAM Navigation** | ✅ Map building, path planning | ❌ |
| **Animation Rendering** | ❌ | ✅ Stores frames, renders pixels |
| **Servo Control** | ❌ | ✅ PWM generation, position feedback |
| **Self-Balancing PID** | ❌ | ✅ **200Hz control loop, IMU fusion** |
| **Sensor Polling** | ❌ | ✅ mmWave, IMU, encoders |
| **Display Driver** | ❌ | ✅ SPI protocol, framebuffer |
| **Motor Control** | ❌ | ✅ UART to ODrive, velocity commands |
| **Timing-Critical Loops** | ❌ (Linux not real-time) | ✅ FreeRTOS, microsecond precision |

---

## Tech Stack

### Technology Stack Table

| Category | Technology | Version | Purpose | Rationale |
|----------|-----------|---------|---------|-----------|
| **Orchestrator OS** | Raspberry Pi OS (64-bit) | Debian 12 Bookworm | Operating system for orchestration layer | Official Pi OS, stable, ROS2 Humble compatibility |
| **Orchestrator Language** | Python | 3.11+ | ROS2 nodes, AI integration, orchestration logic | ROS2 rclpy support, rich AI/ML ecosystem |
| **Module Firmware Language** | C/C++ | C++17 | ESP32 embedded firmware | Arduino/ESP-IDF standard, hardware access |
| **Robotics Framework** | ROS2 Humble | LTS (until 2027) | Module coordination, SLAM, navigation | Long-term support, mature ecosystem, Pi OS compatible |
| **Module Communication** | I2C | 400kHz-1MHz | Pi ↔ ESP32 command/sensor data | 5-20ms latency, deterministic, no WiFi overhead |
| **Module MCU** | ESP32-DevKitC | ESP32-WROOM-32 | Smart peripheral controllers per module | 240MHz, 320KB RAM, sufficient for real-time control |
| **Display Interface** | SPI | 10-20 MHz | ESP32 → OLED eye displays | 30-60 FPS animation vs 10-15 FPS (I2C) |
| **OLED Driver** | Adafruit SSD1306 | 2.5.x | SPI mode for eye displays | Proven library, SPI support, animation-ready |
| **AI Accelerator** | Hailo-8L AI Kit | 13 TOPS | Local Whisper STT inference | Eliminates 1-1.5s cloud STT latency, $70 investment |
| **Local AI Model** | Whisper (tiny/base) | OpenAI Whisper | Speech-to-text (Hailo-accelerated) | <200ms latency, offline-capable, accurate |
| **Cloud AI Agent** | Claude API | Claude 3.5 Sonnet | Conversational reasoning, personality generation | Best-in-class personality quality, function calling |
| **Cloud AI Fallback** | OpenAI GPT-4 API | GPT-4 Turbo | Secondary AI provider | Redundancy if Claude unavailable |
| **Motor Controller** | ODrive v3.6 | 3.6 | BLDC motor control (hoverboard wheels) | Closed-loop velocity control, encoder odometry for SLAM |
| **IMU Sensor** | MPU6050 | 6-axis | Self-balancing (gyro + accel) | $2-5, 200Hz update rate, proven for balancing robots |
| **Base Servos** | Standard Hobby Servo | MG90S or SG90 | Kickstand deployment | 2-3 kg·cm torque, PWM control |
| **Ear Servos** | Feetech SCS0009 | Serial bus | 2-DOF ear articulation (4× servos) | Daisy-chainable, position feedback, I2C |
| **Neck Servos** | Feetech STS3215 | Serial bus | 3-DOF neck articulation | Higher torque (15-20 kg·cm), I2C, position feedback |
| **SLAM Library** | Cartographer | ROS2 port | 2D/3D SLAM mapping | Google-maintained, lighter than RTAB-Map, real-time |
| **Navigation Stack** | Nav2 | ROS2 Humble | Path planning, obstacle avoidance | Standard ROS2 navigation, behavior trees |
| **Database** | SQLite | 3.40+ | Conversation history, config, logs | Embedded, zero-config, Python sqlite3 built-in |
| **I2C Library (Pi)** | smbus2 | 0.4.x | Python I2C communication | Pure Python, cross-platform, simple API |
| **I2C Library (ESP32)** | Wire.h | Arduino core | ESP32 I2C slave firmware | Built-in Arduino library, interrupt support |
| **OTA Framework (ESP32)** | ESP32 OTA | Arduino/ESP-IDF | Over-the-air firmware updates | Partition support, rollback, built-in |
| **OTA Server** | Flask | 3.0.x | HTTP server for firmware binaries | Lightweight, Python, easy deployment |
| **Testing Framework (Python)** | pytest | 7.4+ | Unit/integration tests for orchestrator | Standard Python testing, ROS2 compatible |
| **Testing Framework (C++)** | Arduino Unit Test | - | ESP32 firmware unit tests | PlatformIO integration |
| **Build Tool (Firmware)** | PlatformIO | 6.1+ | ESP32 firmware compilation, OTA builds | Superior to Arduino IDE, CLI automation |
| **Build Tool (Orchestrator)** | colcon | ROS2 standard | ROS2 workspace build system | Standard ROS2 build tool |
| **CI/CD** | GitHub Actions | - | Automated testing (future) | Free for public repos, ROS2 actions available |
| **Logging** | Python logging | Built-in | Orchestrator logs | Standard library, ROS2 integration |
| **Monitoring** | ROS2 rqt tools | Built-in | Node monitoring, topic visualization | Built into ROS2, no additional install |
| **Version Control** | Git | 2.40+ | Source code management | Industry standard |
| **3D Modeling** | OnShape | Cloud | 3D CAD for mechanical design | Free for public projects, cloud-based |
| **Wiring Diagrams** | Fritzing | 0.9.x | Visual wiring documentation | Maker-friendly, breadboard view |

---

## Data Models

### I2C Register Map (Standard - All Modules)

```cpp
// shared/i2c_registers.h
// Common registers (0x00-0x0F)

#define REG_MODULE_ID           0x00  // Read-only: Module identification
#define REG_FIRMWARE_VERSION    0x01  // Read-only: Firmware version
#define REG_STATUS              0x02  // Read-only: Module status byte
#define REG_ERROR_CODE          0x03  // Read-only: Last error code
#define REG_COMMAND             0x04  // Write: Command trigger
#define REG_COMMAND_ARG1        0x05  // Write: Command argument 1-4
#define REG_COMMAND_ARG2        0x06
#define REG_COMMAND_ARG3        0x07
#define REG_COMMAND_ARG4        0x08

// Status byte bit flags
#define STATUS_READY            0x01
#define STATUS_BUSY             0x02
#define STATUS_ERROR            0x04
#define STATUS_OTA_MODE         0x08
#define STATUS_CALIBRATING      0x10
```

### Head Module Register Map (I2C 0x08)

```cpp
// Expression Control (0x10-0x1F)
#define REG_EXPRESSION_TYPE     0x10  // Emotion type (0x00-0x06)
#define REG_EXPRESSION_INTENSITY 0x11 // Intensity level (1-5)
#define REG_BLINK_TRIGGER       0x12

// Sensor Data (0x20-0x2F)
#define REG_PRESENCE_DETECTED   0x20
#define REG_PRESENCE_DISTANCE   0x21
#define REG_PRESENCE_ANGLE      0x22

// Expression types
#define EXPR_NEUTRAL  0x00
#define EXPR_HAPPY    0x01
#define EXPR_CURIOUS  0x02
#define EXPR_THINKING 0x03
#define EXPR_CONFUSED 0x04
#define EXPR_SAD      0x05
#define EXPR_EXCITED  0x06
```

### Base Module Register Map (I2C 0x0C)

```cpp
// Movement Control (0x10-0x1F)
#define REG_TARGET_VELOCITY     0x10  // float32 (4 bytes)
#define REG_TARGET_TURN_RATE    0x14  // float32 (4 bytes)
#define REG_MOVEMENT_MODE       0x18

// Balancing Status (0x20-0x2F)
#define REG_BALANCE_STATE       0x20
#define REG_PITCH_ANGLE         0x21  // float32
#define REG_PITCH_RATE          0x25  // float32
#define REG_KICKSTAND_STATE     0x29

// Odometry (0x30-0x4F)
#define REG_ODOMETRY_X          0x30  // float32
#define REG_ODOMETRY_Y          0x34  // float32
#define REG_ODOMETRY_THETA      0x38  // float32

// States
#define STATE_RELAXED         0x00
#define STATE_TRANSITIONING   0x01
#define STATE_BALANCING       0x02
#define STATE_EMERGENCY_STOP  0x03
```

### ROS2 Custom Messages

```python
# olaf_msgs/msg/Expression.msg
string emotion              # 'happy', 'curious', etc.
uint8 intensity             # 1-5
uint8 duration_ms
bool synchronize

# olaf_msgs/msg/BalanceStatus.msg
Header header
string state
float32 pitch_deg
float32 pitch_rate_dps
bool kickstand_deployed
float32 motor_current_left_a
float32 motor_current_right_a

# olaf_msgs/msg/ModuleStatus.msg
Header header
string module_name
uint8 i2c_address
bool online
uint8 firmware_version_major
uint8 firmware_version_minor
string error_message
uint32 uptime_seconds
```

---

## Components

### Head Module

**Responsibility:** Primary sensor hub and emotional expression through animated OLED eyes.

**Hardware:**
- ESP32-WROOM-32
- 2× SSD1306 OLED (128×64, SPI)
- DFRobot SEN0395 mmWave sensor
- INMP441 I2S microphone × 2
- PAM8403 amplifier + speaker

**Firmware Architecture:**
```
head_controller.ino
├── i2c_slave.cpp
├── animation_engine.cpp
├── oled_driver_spi.cpp
├── presence_sensor.cpp
└── beeper.cpp
```

**Key Features:**
- 200Hz animation rendering
- 20Hz presence polling
- I2C interrupt-driven commands

---

### Ears Module

**Responsibility:** 2-DOF articulated ears (Chappie-inspired), 4× Feetech SCS0009 servos.

**Hardware:**
- ESP32-WROOM-32
- 4× Feetech SCS0009 servos

**Firmware:**
```
ears_controller.ino
├── i2c_slave.cpp
├── servo_controller.cpp
├── motion_profiles.cpp
└── position_calibration.cpp
```

---

### Neck Module

**Responsibility:** 3-DOF head articulation (pan ±90°, tilt ±45°, roll ±30°).

**Hardware:**
- ESP32-WROOM-32
- 3× Feetech STS3215 servos

**Firmware:**
```
neck_controller.ino
├── i2c_slave.cpp
├── servo_controller.cpp
├── kinematics.cpp
├── trajectory_planner.cpp
└── safety_limits.cpp
```

---

### Base Module (Self-Balancing)

**Responsibility:** Autonomous two-wheel balancing with 200Hz PID control, ODrive motor coordination, kickstand deployment.

**Hardware:**
- ESP32-WROOM-32 (may upgrade to ESP32-S3)
- MPU6050 IMU (200Hz sampling)
- ODrive v3.6 motor controller
- 2× hoverboard hub motors (350W each)
- MG90S kickstand servo

**Firmware Architecture:**
```
base_controller.ino (most complex)
├── i2c_slave.cpp
├── balancing_controller.cpp    // 200Hz PID loop
├── imu_fusion.cpp
├── odrive_uart.cpp
├── kickstand_control.cpp
├── state_machine.cpp
├── odometry_publisher.cpp
└── safety_monitor.cpp
```

**Key Features:**
- 200Hz real-time PID balancing
- State machine: RELAXED → TRANSITIONING → BALANCING → EMERGENCY_STOP
- Fall detection: |pitch| > 45° triggers kickstand
- 10Hz odometry publication

---

### Projector Module

**Responsibility:** Floor projection for AI-generated content display.

**Hardware:**
- ESP32-WROOM-32 (if DLP controlled)
- TI DLP LightCrafter OR HDMI pico projector
- 12V power supply

**Note:** If HDMI mode, Pi drives projector directly, ESP32 only controls power relay.

---

### Orchestration Layer (Raspberry Pi)

**Hardware:**
- Raspberry Pi 5 8GB + Hailo-8L AI Kit
- 128GB SD card
- Official 5A power supply (via buck converter)

**Software Architecture:**
```
orchestrator/
├── ros2_nodes/
│   ├── hardware_drivers/       # 5× I2C bridge nodes
│   ├── personality/
│   ├── ai_integration/
│   ├── navigation/
│   └── system_monitor/
├── launch/
│   └── olaf_full.launch.py
├── config/
└── ota_server/
```

---

_(Architecture document continues - sections remaining: API Specification, External APIs, Core Workflows, Database Schema, Backend Architecture, Testing Strategy, Coding Standards, Error Handling, Monitoring, Deployment)_

