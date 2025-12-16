# High Level Architecture

## Technical Summary

OLAF is a **distributed embedded robotics system** with hybrid AI intelligence, built on a three-layer architecture. The **Orchestration Layer** (Raspberry Pi 5 16GB + Hailo AI Kit running ROS2 Humble) coordinates four independent **Module Layer** smart peripherals (ESP32-based: Head+Ears, Neck, Torso, Base) via I2C bus communication, plus the Pi Module itself which houses the orchestrator hardware. The **Intelligence Layer** combines local AI acceleration (Hailo-accelerated Whisper STT for <200ms speech recognition) with cloud-based AI agents (Claude/GPT-4 APIs for reasoning and personality generation).

Key architectural decisions include: (1) I2C-only module communication eliminates WiFi complexity and achieves 5-20ms latency vs 80-200ms WiFi/ROS2, (2) ROS2 nodes run exclusively on Pi with ESP32s as smart I2C slaves containing full hardware drivers, animation engines, and real-time control loops, (3) OLED displays (128x64, SPI) for eyes with 30-60 FPS animated expressions, (4) Five independent modules via MECE principle: Head+Ears (eyes + articulated ears at 0x08), Neck (3-DOF pan/tilt/roll + 2 presence sensors at 0x09), Torso (heart display + thermal printer at 0x0A), Base (self-balancing motors at 0x0B), Pi Module (speakerphone + projector + Hailo STT orchestrator), (5) Torso ESP32 controls 2.8" square heart display and thermal printer for physical outputs, (6) Neck ESP32 manages two presence sensors for 360° human detection, (7) Self-balancing two-wheel base with 200Hz PID control and kickstand deployment (both managed by Base ESP32), and (8) OTA firmware updates in V1 enable rapid iteration without robot disassembly. This architecture achieves <3s AI response latency, supports 2-4 hour battery runtime, and enables weekend-sprint modular development per MECE principles.

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

**Package Organization:**
- Documentation: Architecture, brief, guides (documentation/)
- Archive: Previous iterations preserved (archive/) - gitignored
- Project configuration: Makefile, pyproject.toml, LICENSE

```
olaf/
├── documentation/              # Project documentation
│   ├── architecture.md        # Technical architecture (this document)
│   ├── brief.md              # Project brief, MVP scope
│   └── media/                # Images, diagrams
├── archive/                   # Previous iterations (gitignored)
├── Makefile                   # Build convenience wrapper
├── pyproject.toml            # Python dependencies
├── poetry.lock
├── .gitignore
├── README.md                 # Main project README
└── LICENSE
```

**Note:** Firmware, hardware designs, and ROS2 packages will be added as development progresses. Current focus is documentation clarity and foundational architecture.

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
┌──▼──────────┐┌─▼────┐┌─▼────┐┌▼──────────┐
│ HEAD+EARS   ││ NECK ││TORSO ││   BASE    │
│   ESP32     ││ESP32 ││ESP32 ││   ESP32   │
└──┬─────┬────┘└──┬───┘└──┬───┘└┬─────┬────┘
   │SPI  │UART    │UART   │SPI  │UART │UART
   │Eyes │Ears    │Neck   │Heart│1:   │2:
   │OLED │2-DOF   │3-DOF  │2.8" │ODrv │Stand
   │     │(2×)    │+Sens× │Prnt │+IMU │Servo
   │     │        │  2    │     │     │
┌──▼───────────────────────────────────────────────────────────┐
│          MODULE LAYER (Physical Hardware)                    │
│  • HEAD+EARS MODULE (0x08):                                  │
│    - 2× OLED Eyes (128×64, SPI, 30-60 FPS animations)       │
│    - 2× Articulated Ears (2-DOF each, Feetech servos)      │
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
│    - Floor Projector (HDMI, information display)            │
│    - All ROS2 nodes, personality coordination               │
└──────────────────────────────────────────────────────────────┘

Communication Protocols:
━━━━━━━━ I2C: Pi ↔ 4× ESP32 modules (5-20ms latency, commands/sensor data)
- - - - - WiFi: Pi → Cloud AI APIs only (Claude/GPT-4)
━ ━ ━ ━ ━ SPI: ESP32 → OLED displays, heart display (high-speed graphics)
━·━·━·━·━ UART: ESP32 modules → servo controllers, motor controllers
─ ─ ─ ─ ─ USB: RGBD camera, Hailo AI Kit, Speakerphone → Raspberry Pi
━━━━━━━━ HDMI: Raspberry Pi → Floor Projector
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
