# Olaf 🤖

**An open-source modular personal assistant robot with personality**

Olaf is a personality-first robotics framework that brings physical AI into everyday life. Named after the friendly snowman, Olaf combines R2D2-style charm with modern AI capabilities—communicating through expressive beeps, animated OLED eyes, and Chappie-inspired articulated ears.

![Project Status](https://img.shields.io/badge/status-in%20development-yellow)
![License](https://img.shields.io/badge/license-MIT-blue)

## 🎯 Project Vision

Build a modular robot that:
- **Feels alive** through coordinated personality expression
- **Actually helps** with daily tasks via AI integration
- **Grows with you** through modular expansion
- **Anyone can build** with soldering skills and a 3D printer

## ✨ Key Features

### V1 (4 months)
- 🎭 **Personality Expression**: OLED eyes, 2-DOF articulated ears, 3-DOF neck, R2D2-style beeping
- 🤖 **Conversational AI**: Cloud-powered intelligence (Claude/GPT-4) for natural interaction
- 🚶 **Autonomous Navigation**: SLAM-based movement with RGBD camera and hoverboard base
- 📽️ **Floor Projection**: Information display and visual communication
- 🧩 **Modular Architecture**: Independent ESP32-powered modules with ROS2 communication
- 📚 **Complete Documentation**: 3D files, BOMs, wiring diagrams, build guides

### Post-V1
- Smart home control, diary maintenance, reminders, transit info
- Enhanced personality features (emotional weather, thinking visualization, sleep mode)
- Advanced capabilities (local LLM, object manipulation, extended battery)

## 🏗️ Architecture

**Three-Layer Design:**

1. **Module Layer** - Independent hardware modules (Head, Ears, Neck, Projector, Body, Base)
   - Each powered by ESP32 acting as ROS2 node
   - Embedded intelligence with local firmware
   - MECE principle: Mutually Exclusive, Collectively Exhaustive

2. **Orchestration Layer** - Raspberry Pi 8GB coordinator
   - Python-based orchestration engine
   - Personality coordination and behavior sequencing
   - SLAM navigation and sensor fusion

3. **Intelligence Layer** - Cloud AI
   - Natural language understanding via LLM APIs
   - Decision-making and function routing
   - Context and conversation management

## 📁 Repository Structure

```
olaf/
├── docs/                   # Documentation, build guides, diagrams
├── hardware/              # Physical components
│   ├── 3d-models/        # STL files for 3D printing
│   ├── wiring-diagrams/  # Electrical connection guides
│   └── bom/              # Bills of materials with component links
├── modules/               # Module-specific code
│   ├── head/             # RGBD camera, eyes, presence sensor
│   ├── ears/             # 2-DOF articulated ears
│   ├── neck/             # 3-DOF neck servos
│   ├── projector/        # Floor projection system
│   ├── body/             # LED indicators, status display
│   └── base/             # Hoverboard mobility platform
├── orchestrator/          # Raspberry Pi orchestration
│   ├── personality/      # Expression coordination
│   ├── ai_integration/   # Cloud AI interface
│   └── navigation/       # SLAM, path planning
├── tests/                 # Module and integration tests
└── tools/                 # Setup scripts, utilities
```

## 🛠️ Tech Stack

- **Compute**: Raspberry Pi 8GB + ESP32 modules
- **Communication**: ROS2 Humble, micro-ROS, I2C
- **AI**: Claude/GPT-4 APIs for intelligence
- **Languages**: Python (orchestrator), C/C++ (ESP32 firmware)
- **Navigation**: SLAM with RGBD camera
- **Development**: AI-assisted (100% of code and docs)

## 🎨 Design Principles

- **Personality-First**: Expression coordination as core architecture, not afterthought
- **Weekend Sprints**: Each module buildable in 1-2 weekends
- **Test in Isolation**: Every module works standalone before integration
- **Build in Public**: Complete transparency—successes and failures documented
- **Maker-Accessible**: AliExpress components, 3D printed, configurable cost (~$400-1000)

## 🚀 Getting Started

> **Note**: Project currently in early development. Build guides coming as modules are completed.

### Prerequisites
- Soldering skills
- Python programming knowledge
- Access to 3D printer
- Basic electronics understanding

### Current Status

**Week 1-2**:
- [x] Project brief completed
- [ ] Component ordering in progress
- [ ] Development environment setup
- [ ] First feasibility prototypes

Follow along with weekly updates on [LinkedIn](#) and monthly videos on [YouTube](#).

## 📊 Project Metrics

**Timeline**: 4 months (Oct 2025 - Jan 2026)
**Budget**: 1000 CHF (~$1100 USD)
**Target**: Functional V1 with personality, mobility, AI, and projection

## 🤝 Contributing

Olaf is built in public from day one. Contributions welcome once the core framework is proven!

**Ways to contribute:**
- Build your own Olaf and share improvements
- Create additional modules following the architecture
- Improve documentation and tutorials
- Report issues and suggest features

## 📖 Documentation

- [Project Brief](docs/brief.md) - Complete vision, architecture, and planning
- Build Guides - Coming as modules complete
- Module Specs - Coming soon
- API Documentation - Coming soon

## 🙏 Inspiration

- **R2D2** (Star Wars) - Non-verbal expressive communication
- **Chappie** (Film) - Ear design and emotional development
- **Wall-E** (Film) - Personality through movement and sound

## 📝 License

MIT License - See [LICENSE](LICENSE) for details

## 💬 Connect

Building in public! Follow the journey:
- **LinkedIn**: [Your Profile] - Weekly progress updates + physical AI insights
- **YouTube**: [Your Channel] - Monthly milestone videos
- **GitHub Issues**: Questions, discussions, and feedback welcome

---

*"Named after the friendly snowman, built with the charm of R2D2, designed to be your daily companion."*

**Status**: 🟡 In Development | **Latest Update**: Oct 2025 | **Next Milestone**: First module complete
