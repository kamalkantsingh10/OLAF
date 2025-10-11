# OLAF 🤖

**Open-source AI companion proving embodied AI belongs to builders, not just consumers**

![Project Status](https://img.shields.io/badge/status-in%20development-yellow)
![License](https://img.shields.io/badge/license-MIT-blue)

---

## What is OLAF?

OLAF  is a self-balancing AI companion robot designed to demonstrate that anyone with passion, a 3D printer, and access to modern tools (LLMs, SBCs, affordable electronics) can build their own JARVIS or R2D2.

Unlike commercial AI assistants that position users as passive consumers, OLAF embodies a different philosophy: **when you BUILD your AI partner, it becomes a true collaborator, not a servant.**

OLAF combines:
- **R2D2-style personality** through coordinated expression (OLED eyes, articulated ears, beeps)
- **Autonomous mobility** with self-balancing hoverboard base and SLAM navigation
- **Conversational AI** powered by hybrid local/cloud intelligence (Hailo Whisper + Claude/GPT-4)
- **Floor projection** for visual information display
- **Full documentation** of every design decision, success, and failure

---

## Why OLAF Matters

The technology to build advanced AI companions is now accessible to individual builders:
- **3D printing** has democratized custom hardware
- **LLMs** provide powerful coding assistance and reasoning
- **Modern SBCs** (Raspberry Pi 5 + Hailo AI Kit) bring 13 TOPS of AI acceleration at $150
- **ESP32 modules** enable sophisticated distributed control at $5/unit

Big tech focuses on bringing AI assistants *to* the masses. OLAF proves a different thesis: **the most meaningful AI companions will be the ones people build themselves** - growing alongside their creators through adaptation and co-evolution.

This is the **Linux moment for physical AI** - open, accessible, community-driven embodied intelligence.

---

## Technical Approach

OLAF uses a three-layer modular architecture optimized for power efficiency, low latency, and builder accessibility:

### **Module Layer** (5× ESP32 Smart Peripherals)
Five independent ESP32-WROOM-32 modules handle dedicated functions:
- **Head Module** (0x08): OLED eyes, mmWave presence sensor, microphone array, speaker
- **Ears Module** (0x09): 4× servo-driven articulated ears
- **Neck Module** (0x0A): 3-DOF pan/tilt/roll servos
- **Projector Module** (0x0B): DLP floor projection display
- **Base Module** (0x0C): Self-balancing at 200Hz with MPU6050 IMU, ODrive motor controller, hoverboard motors

**Key Decision**: I2C-only communication (no WiFi on ESP32s) achieves 5-20ms latency vs 80-200ms and saves 1000mA power.

### **Orchestration Layer** (Raspberry Pi 5 + Hailo-8L)
ROS2 Humble framework coordinates all modules through dedicated driver nodes:
- `/olaf/personality_coordinator` - Expression synchronization across channels
- `/olaf/ai_agent` - Hailo Whisper (local STT) + Claude API (reasoning)
- `/olaf/navigation` - Cartographer SLAM + Nav2 path planning
- Hardware driver nodes bridge ROS2 topics to I2C registers

### **Intelligence Layer** (Hybrid AI)
- **Local**: Hailo Whisper (tiny/base) for <200ms speech-to-text
- **Cloud**: Claude 3.5 Sonnet for personality and reasoning (WiFi from Pi only)
- **Target**: <3s end-to-end AI response time (P90)

**Key Decision**: Closed-loop ODrive motor controller enables accurate SLAM odometry (±10cm) vs open-loop alternatives.

---

## Documentation

**Core Documents:**
- [**Product Requirements Document (PRD)**](docs/prd.md) - Complete feature requirements, success metrics, timeline
- [**Technical Architecture**](docs/architecture.md) - Full system design, decisions, tradeoffs, protocols
- [**Architecture Quick Reference**](docs/architecture-quick-ref.md) - 1-page cheat sheet with I2C map, tech stack, troubleshooting
- [**Project Brief**](docs/brief.md) - Original vision and design philosophy

**Development Roadmap:**
- [**Epic 01**: Foundation & Minimal Personality](docs/epics/epic-01-foundation.md) - ROS2 setup, head module, basic expressions
- [**Epic 02**: Mobility & Autonomy](docs/epics/epic-02-mobility.md) - Self-balancing, SLAM, navigation
- [**Epic 03**: AI Integration](docs/epics/epic-03-ai-integration.md) - Whisper STT, Claude integration, conversation
- [**Epic 04**: Conversational AI & Context](docs/epics/epic-04-conversational-ai.md) - Memory, context, function routing

---

## Technical Highlights

| Component | Technology | Decision Rationale |
|-----------|------------|-------------------|
| **Communication** | I2C @ 400kHz-1MHz | 5-20ms latency vs 80-200ms WiFi, saves 1000mA |
| **ROS2 Nodes** | Pi only (no micro-ROS) | Simpler - ESP32s are smart I2C slaves |
| **OLED Display** | SPI (not I2C) | 30-60 FPS vs 10-15 FPS for smooth animation |
| **Motor Control** | ODrive v3.6 closed-loop | Accurate odometry for SLAM (±10cm) |
| **SLAM** | Cartographer | Lighter footprint than RTAB-Map |
| **Self-Balancing** | 200Hz PID on ESP32 | Pi can't guarantee real-time control |
| **Power** | 36V hoverboard battery | 2-4 hour runtime with buck converters |
| **AI Acceleration** | Hailo-8L (13 TOPS) | <200ms local STT vs 1-1.5s cloud |

---

## Repository Structure

```
olaf/
├── docs/                       # PRD, architecture, epics, build guides
├── modules/                    # ESP32 firmware for 5 hardware modules
│   ├── head/                   # OLED eyes, presence sensor, audio
│   ├── ears/                   # Articulated servo ears
│   ├── neck/                   # 3-DOF servo neck
│   ├── projector/              # Floor projection display
│   └── base/                   # Self-balancing + odometry
├── orchestrator/               # Raspberry Pi ROS2 nodes
│   ├── ros2_nodes/
│   │   ├── hardware_drivers/   # I2C bridge to ESP32 modules
│   │   ├── personality/        # Expression coordination
│   │   ├── ai_integration/     # Whisper + Claude
│   │   └── navigation/         # SLAM + Nav2
│   └── ota_server/             # Wireless firmware updates
├── shared/                     # Common headers (I2C registers, protocols)
└── tools/                      # Diagnostics, testing utilities
```

---

## Current Status

**Development Timeline**: 4 months (October 2025 - January 2026)
**Budget**: 1000 CHF (~$1100 USD)
**Build Approach**: Weekly progress updates with full transparency

**Milestones:**
- ✅ PRD and technical architecture complete
- ✅ Epic roadmap defined
- 🟡 Component sourcing (in progress)
- ⏳ Epic 01: Foundation & Minimal Personality (starting)

---

## Build-in-Public

OLAF development follows complete transparency - documenting every decision, success, and failure:

- **Weekly LinkedIn posts** (2-3/week) - Progress updates, technical deep-dives, lessons learned
- **Epic milestone videos** (1 per epic) - Demonstrations and technical explanations
- **Reddit engagement** (1/week) - Community discussions and technical Q&A
- **GitHub activity** - All code, documentation, issues, discussions public from day one

This isn't just about building a robot - it's about proving that **individual builders can create sophisticated AI companions** and sharing that knowledge to enable a community.

---

## Tech Stack Summary

- **Robotics Framework**: ROS2 Humble (LTS until 2027)
- **Orchestrator**: Python 3.11 on Raspberry Pi OS 64-bit (Debian 12)
- **Firmware**: C++17 with Arduino/PlatformIO for ESP32
- **AI**: Hailo Whisper (local) + Claude 3.5 Sonnet (cloud)
- **SLAM**: Google Cartographer + Nav2
- **Database**: SQLite for conversation history
- **OTA**: Flask HTTP server for wireless ESP32 firmware updates

---

## Getting Started

> **Note**: OLAF is in early development (Epic 01). Complete build guides will be published as each epic is completed.

**Prerequisites for builders:**
- Soldering experience
- 3D printer access
- Basic electronics and programming knowledge
- Budget: ~$400-1000 (configurable based on component choices)

**Development workflow:**
1. **Firmware**: PlatformIO → compile → upload via USB (OTA post-V1)
2. **Orchestrator**: `colcon build` → ROS2 workspace
3. **Launch**: `ros2 launch orchestrator olaf_full.launch.py`
4. **Test**: `./tools/diagnostics/olaf-test <module> <command>`

---

## Contributing

OLAF welcomes contributions from the builder community! Once the core framework is proven (post-Epic 01), contributions can include:

- Building your own OLAF variant and sharing improvements
- Creating additional modules following the architecture
- Enhancing documentation and tutorials
- Reporting issues and suggesting features
- Sharing integration guides for new hardware

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines (coming soon).

---

## Inspiration

- **R2D2** (Star Wars) - Expressive non-verbal communication through beeps and movement
- **Chappie** (Film) - Articulated ears and emotional personality development
- **Wall-E** (Film) - Personality conveyed through motion and sound design
- **Linux Movement** - Community-driven open-source alternative to commercial products

---

## License

MIT License - See [LICENSE](LICENSE) for details.

---

## Follow the Journey

- **GitHub Issues**: Questions, discussions, and feature requests welcome
- **LinkedIn**: Weekly progress updates and physical AI insights
- **YouTube**: Epic milestone demonstrations and technical deep-dives
- **Reddit**: r/robotics, r/ROS, r/DIYrobotics community engagement

---

*"The Linux moment for physical AI - proving embodied intelligence belongs to builders, not just consumers."*

**Latest Update**: October 2025 | **Current Epic**: 01 - Foundation & Minimal Personality
