# Source Tree

## Overview

This document provides a comprehensive guide to OLAF's repository structure, explaining the purpose of each directory and key files. OLAF uses a **monorepo** structure containing firmware, orchestrator code, hardware designs, documentation, and tooling in a single repository.

**Repository Type:** Monorepo (single repository, multiple components)

**Organization Principle:** MECE (Mutually Exclusive, Collectively Exhaustive) - each module owns its domain exclusively

---

## Repository Structure

### Top-Level Directory Tree

```
olaf/
├── .bmad-core/              # BMAD agent framework (development workflow automation)
├── .claude/                 # Claude Code configuration
├── .content/                # Content planning (LinkedIn posts, YouTube scripts)
├── docs/                    # All project documentation
├── hardware/                # Physical design files (3D models, wiring, BOMs)
├── modules/                 # ESP32 firmware (C/C++) for each hardware module
├── orchestrator/            # Raspberry Pi software (Python/ROS2)
├── tests/                   # Test suites (unit, integration, hardware validation)
├── tools/                   # Utilities (setup, calibration, diagnostics)
├── .gitignore              # Git ignore patterns
├── LICENSE                 # MIT open-source license
└── README.md               # Project overview and getting started guide
```

---

## Directory Details

### `/docs/` - Documentation

**Purpose:** All project documentation including PRD, architecture, build guides, and planning documents.

```
docs/
├── architecture/           # Architecture documentation (sharded)
│   ├── index.md           # Architecture table of contents
│   ├── introduction.md    # Architecture overview and goals
│   ├── high-level-architecture.md
│   ├── ros2-node-architecture.md
│   ├── tech-stack.md      # Complete technology stack table
│   ├── data-models.md     # I2C register maps, ROS2 messages
│   ├── components.md      # Detailed component specifications
│   ├── coding-standards.md # Python/C++ coding standards
│   └── source-tree.md     # This file
├── prd/                   # Product Requirements Document (sharded)
│   ├── index.md           # PRD table of contents
│   ├── goals-and-background-context.md
│   ├── requirements.md    # Functional and non-functional requirements
│   ├── user-interface-design-goals.md
│   ├── technical-assumptions.md
│   ├── epic-list.md       # Epic overview
│   ├── epic-01-foundation.md
│   ├── epic-02-3d-design.md
│   ├── epic-03-personality.md
│   └── epic-04-conversational-ai.md
├── architecture.md        # Consolidated architecture (source for sharding)
├── prd.md                 # Consolidated PRD (source for sharding)
└── brief.md               # Original project brief
```

**Key Files:**
- **`architecture/index.md`** - Start here for architecture navigation
- **`prd/index.md`** - Start here for requirements and epics
- **`architecture/tech-stack.md`** - Technology decisions and rationale
- **`architecture/coding-standards.md`** - Required reading before contributing code
- **`prd/requirements.md`** - Functional (FR1-FR32) and non-functional (NFR1-NFR16) requirements

---

### `/hardware/` - Physical Design Files

**Purpose:** 3D CAD models, wiring diagrams, bills of materials for physical construction.

```
hardware/
├── 3d-models/             # STL files for 3D printing
│   ├── head/              # Head housing, camera mount, OLED bezels
│   ├── ears/              # Ear brackets, servo mounts
│   ├── neck/              # Gimbal structure, servo mounts
│   ├── body/              # Chassis, Pi mounting, power distribution
│   └── base/              # Motor mounts, wheel adapters, kickstand
├── bom/                   # Bills of Materials (CSV/Excel)
│   ├── full-olaf-bom.csv # Complete BOM with supplier links
│   └── module-specific/   # Per-module BOMs (optional builds)
└── wiring-diagrams/       # Fritzing/schematic files
    ├── i2c-bus-topology.png
    ├── power-distribution.png
    └── module-wiring/     # Per-module connection diagrams
```

**Usage:**
- **3D Printing:** All STL files include recommended settings (layer height, infill, supports)
- **BOM:** Includes direct AliExpress/Amazon/Adafruit links, estimated costs, quantity needed
- **Wiring:** Follow diagrams during assembly; validate with continuity tester before powering on

**File Naming Convention:**
- 3D models: `<module>-<part>-v<version>.stl` (e.g., `head-housing-v1.2.stl`)
- Wiring diagrams: `<module>-wiring-diagram.png`

---

### `/modules/` - ESP32 Firmware (C/C++)

**Purpose:** Embedded firmware for each hardware module. Each module is an independent smart peripheral with its own ESP32 controller.

```
modules/
├── README.md              # Module development overview
├── head/                  # Head Module (I2C 0x08)
│   ├── firmware/
│   │   ├── head_controller.ino
│   │   ├── i2c_slave.cpp/h
│   │   ├── oled_driver_spi.cpp/h
│   │   ├── animation_engine.cpp/h
│   │   ├── animations/    # Pre-built emotion animation frames
│   │   └── config.h       # Pin assignments, I2C address
│   ├── platformio.ini     # PlatformIO build configuration
│   ├── test/              # Unit tests (Arduino Unit Test)
│   └── README.md          # Hardware setup, I2C API, troubleshooting
├── ears/                  # Ears Module (I2C 0x09)
│   ├── firmware/
│   │   ├── ears_controller.ino
│   │   ├── servo_control.cpp/h  # Feetech SCS0009 serial bus
│   │   └── config.h
│   ├── platformio.ini
│   └── README.md
├── neck/                  # Neck Module (I2C 0x0A)
│   ├── firmware/
│   │   ├── neck_controller.ino
│   │   ├── servo_control.cpp/h  # Feetech STS3215 serial bus
│   │   └── config.h
│   ├── platformio.ini
│   └── README.md
├── projector/             # Projector Module (I2C 0x0B)
│   ├── firmware/
│   │   ├── projector_controller.ino
│   │   ├── dlp_control.cpp/h
│   │   └── config.h
│   ├── platformio.ini
│   └── README.md
├── base/                  # Base Module (I2C 0x0C) - Self-Balancing
│   ├── firmware/
│   │   ├── base_controller.ino
│   │   ├── balancing_controller.cpp/h  # 200Hz PID loop
│   │   ├── odrive_uart.cpp/h          # ODrive motor control
│   │   ├── kickstand_control.cpp/h
│   │   └── config.h
│   ├── platformio.ini
│   └── README.md
└── body/                  # Body Module (LED indicators, status)
    ├── firmware/
    │   ├── body_controller.ino
    │   └── led_control.cpp/h
    ├── platformio.ini
    └── README.md
```

**Key Concepts:**
- **I2C Slave Architecture:** Each ESP32 acts as intelligent I2C slave (addresses 0x08-0x0C)
- **Embedded Intelligence:** Contains full hardware drivers, animation engines, real-time control loops
- **Semantic Commands:** Receives high-level commands from orchestrator (e.g., "express happy level 3")
- **Standalone Testing:** Each module testable independently without other modules connected

**Development Workflow:**
1. Edit firmware in `firmware/` directory
2. Build with PlatformIO: `pio run -d modules/head`
3. Upload: `pio run -d modules/head -t upload`
4. Test standalone: `olaf-test head --emotion happy --intensity 3`
5. Integration test with orchestrator

**I2C Register Maps:** Documented in `docs/architecture/data-models.md`

---

### `/orchestrator/` - Raspberry Pi Software (Python/ROS2)

**Purpose:** High-level coordination, ROS2 nodes, AI integration, navigation, personality orchestration.

```
orchestrator/
├── ros2_nodes/            # ROS2 node implementations
│   ├── hardware_drivers/  # I2C driver nodes (one per module)
│   │   ├── head_driver_node.py
│   │   ├── ears_driver_node.py
│   │   ├── neck_driver_node.py
│   │   ├── projector_driver_node.py
│   │   └── base_driver_node.py
│   ├── personality/       # Personality coordination
│   │   ├── personality_coordinator_node.py
│   │   └── emotion_mapper.py
│   ├── ai_integration/    # Cloud AI and local Hailo inference
│   │   ├── ai_agent_node.py
│   │   ├── whisper_stt_node.py  # Hailo-accelerated STT
│   │   └── claude_client.py
│   └── navigation/        # SLAM and path planning
│       ├── cartographer_node.py
│       └── nav2_integration.py
├── launch/                # ROS2 launch files
│   ├── olaf_full.launch.py     # Launch all nodes
│   ├── minimal.launch.py       # Personality-only (no navigation)
│   └── test_module.launch.py  # Single module testing
├── config/                # Configuration files
│   ├── ros2_params.yaml   # ROS2 node parameters
│   ├── module_i2c_addresses.yaml
│   └── api-keys.template.env  # Template for .env (actual .env is gitignored)
├── ota_server/            # Over-The-Air firmware update server
│   ├── ota_server.py      # Flask HTTP server
│   └── firmware_binaries/ # Compiled .bin files for ESP32 OTA
├── state_management/      # Conversation context, system state
│   ├── conversation_db.py # SQLite interface
│   └── state_machine.py
├── requirements.txt       # Python dependencies
└── README.md              # Orchestrator setup and usage
```

**Key Concepts:**
- **ROS2 Exclusive:** ALL ROS2 nodes run on Raspberry Pi (ESP32s are NOT ROS2 nodes)
- **Driver Nodes:** Bridge between ROS2 topics and I2C register writes
- **Personality Coordination:** Synchronizes eyes, ears, neck, beeps into unified expressions (<500ms)
- **Hybrid AI:** Local Hailo STT (<200ms) + Cloud LLM reasoning (Claude/GPT-4)
- **SLAM Navigation:** Cartographer for mapping, Nav2 for path planning

**Development Workflow:**
1. Edit Python code in respective directories
2. Source ROS2 workspace: `source /opt/ros/humble/setup.bash`
3. Build: `colcon build --packages-select olaf_orchestrator`
4. Launch: `ros2 launch olaf_orchestrator olaf_full.launch.py`
5. Monitor: `rqt` (ROS2 visualization tools)

**Environment Setup:**
- Copy `config/api-keys.template.env` → `.env`
- Fill in `ANTHROPIC_API_KEY`, `ROS_DOMAIN_ID`
- Never commit `.env` (already in `.gitignore`)

---

### `/tests/` - Test Suites

**Purpose:** Unit tests, integration tests, hardware validation tests.

```
tests/
├── unit/                  # Unit tests (no hardware required)
│   ├── orchestrator/
│   │   ├── personality/
│   │   │   └── test_emotion_mapper.py
│   │   ├── ai_integration/
│   │   │   └── test_claude_client.py
│   │   └── navigation/
│   │       └── test_path_planning.py
│   └── modules/           # ESP32 firmware unit tests
│       ├── head/
│       └── base/
├── integration/           # Cross-component integration tests
│   ├── test_i2c_communication.py
│   ├── test_expression_coordination.py
│   └── test_end_to_end_conversation.py
├── hardware/              # Hardware-in-the-loop tests
│   ├── test_servo_calibration.py
│   ├── test_balancing_stability.py
│   └── test_oled_framerate.py
├── fixtures/              # Test data and mock objects
│   ├── mock_i2c_client.py
│   └── sample_conversations.json
└── README.md              # Testing guide and coverage requirements
```

**Testing Requirements:**
- **Unit Tests:** 70%+ coverage for critical paths (personality, AI, navigation)
- **Integration Tests:** I2C latency (<100ms), expression sync (<500ms)
- **Hardware Tests:** Servo ranges (pan ±90°, tilt ±45°, roll ±30°), PID stability

**Running Tests:**
```bash
# Python unit tests
pytest tests/unit/orchestrator/

# ESP32 firmware tests (PlatformIO)
pio test -d modules/head

# Integration tests (requires hardware)
pytest tests/integration/
```

---

### `/tools/` - Utilities and Scripts

**Purpose:** Setup scripts, calibration tools, diagnostic utilities.

```
tools/
├── setup/                 # First-time environment setup
│   ├── install_ros2.sh
│   ├── install_hailo_drivers.sh
│   └── configure_i2c.sh
├── calibration/           # Hardware calibration
│   ├── calibrate_servos.py    # Servo range calibration
│   ├── calibrate_eyes.py      # OLED display alignment
│   ├── calibrate_imu.py       # MPU6050 for balancing
│   └── calibrate_odrive.py    # Motor controller tuning
├── diagnostics/           # Health checks and troubleshooting
│   ├── test_i2c_bus.py        # Scan for modules, check latency
│   ├── test_battery.py        # Voltage monitoring
│   └── test_module.py         # Standalone module testing (olaf-test CLI)
├── ota/                   # OTA firmware management
│   ├── build_all_firmware.sh  # Compile all ESP32 binaries
│   └── upload_ota.py          # Trigger OTA updates
└── README.md              # Tool usage guide
```

**Common Tools:**
- **`olaf-test`:** Standalone module testing CLI
  - `olaf-test head --emotion happy --intensity 3`
  - `olaf-test neck --pan 45 --tilt 10`
  - `olaf-test base --velocity 0.5`
- **`calibrate_servos.py`:** Interactive servo calibration wizard
- **`test_i2c_bus.py`:** Detect modules, measure latency, validate wiring

---

### `/.bmad-core/` - BMAD Agent Framework

**Purpose:** Development workflow automation using AI agent system (Claude Code integration).

```
.bmad-core/
├── agents/                # AI agent definitions (architect, dev, pm, qa)
├── tasks/                 # Reusable task workflows
├── templates/             # Document templates (PRD, architecture, epics)
├── checklists/            # Quality checklists
├── data/                  # Project-specific data
└── core-config.yaml       # Project configuration
```

**Usage:** Internal development tooling, not required for building/running OLAF.

---

### `/.claude/` - Claude Code Configuration

**Purpose:** Claude Code CLI configuration and custom slash commands.

```
.claude/
└── commands/
    └── BMad/              # Custom BMAD slash commands
```

**Usage:** Development environment configuration, not required for building/running OLAF.

---

### `/.content/` - Content Planning

**Purpose:** LinkedIn posts, YouTube scripts, build-in-public content planning.

```
.content/
├── branding-guide.md
├── linkedin-post-001-launch.md
└── epic-content-plan.md
```

**Usage:** Marketing and community engagement content, not required for building/running OLAF.

---

## Key Configuration Files

### Root Level

| File | Purpose | Notes |
|------|---------|-------|
| `README.md` | Project overview, quick start guide | Start here for newcomers |
| `LICENSE` | MIT open-source license | Permissive open-source |
| `.gitignore` | Git ignore patterns | Excludes `.env`, `*.bin`, build artifacts |

### Orchestrator Configuration

| File | Purpose | Location |
|------|---------|----------|
| `requirements.txt` | Python dependencies | `orchestrator/` |
| `ros2_params.yaml` | ROS2 node parameters | `orchestrator/config/` |
| `api-keys.template.env` | Environment variable template | `orchestrator/config/` |
| `.env` | Actual API keys (gitignored) | `orchestrator/config/` |

### Module Configuration

| File | Purpose | Per Module |
|------|---------|------------|
| `platformio.ini` | PlatformIO build config | Each `modules/<module>/` |
| `config.h` | Pin assignments, I2C address | Each `modules/<module>/firmware/` |

---

## File Naming Conventions

### Documentation
- **Markdown files:** `kebab-case.md` (e.g., `coding-standards.md`, `tech-stack.md`)
- **Epic files:** `epic-<number>-<name>.md` (e.g., `epic-01-foundation.md`)

### Code Files
- **Python:** `snake_case.py` (e.g., `personality_coordinator_node.py`)
- **C++ headers:** `snake_case.h` (e.g., `animation_engine.h`)
- **C++ implementation:** `snake_case.cpp` (e.g., `animation_engine.cpp`)
- **Arduino sketches:** `<module>_controller.ino` (e.g., `head_controller.ino`)

### Configuration Files
- **YAML:** `kebab-case.yaml` (e.g., `ros2-params.yaml`, `module-config.yaml`)
- **Environment:** `.env`, `.env.template`

### Hardware Files
- **3D models:** `<module>-<part>-v<version>.stl` (e.g., `head-housing-v1.2.stl`)
- **BOM:** `<scope>-bom.csv` (e.g., `full-olaf-bom.csv`, `head-module-bom.csv`)
- **Wiring:** `<module>-wiring-diagram.png`

---

## Important Paths Reference

Quick reference for frequently accessed locations:

### Documentation
- Architecture index: `docs/architecture/index.md`
- PRD index: `docs/prd/index.md`
- Tech stack: `docs/architecture/tech-stack.md`
- Coding standards: `docs/architecture/coding-standards.md`
- Requirements: `docs/prd/requirements.md`

### Development
- ROS2 nodes: `orchestrator/ros2_nodes/`
- ESP32 firmware: `modules/<module>/firmware/`
- Launch files: `orchestrator/launch/`
- Tests: `tests/unit/`, `tests/integration/`

### Configuration
- ROS2 parameters: `orchestrator/config/ros2_params.yaml`
- API keys template: `orchestrator/config/api-keys.template.env`
- Module I2C addresses: `orchestrator/config/module_i2c_addresses.yaml`

### Hardware
- 3D models: `hardware/3d-models/<module>/`
- BOM: `hardware/bom/full-olaf-bom.csv`
- Wiring diagrams: `hardware/wiring-diagrams/`

### Tools
- Module testing: `tools/diagnostics/test_module.py`
- Calibration: `tools/calibration/`
- Setup scripts: `tools/setup/`

---

## Navigation Tips

### For New Contributors

1. **Start here:** `README.md` (project overview)
2. **Understand requirements:** `docs/prd/requirements.md`
3. **Learn architecture:** `docs/architecture/index.md`
4. **Review coding standards:** `docs/architecture/coding-standards.md`
5. **Pick a module:** Choose from `modules/` or `orchestrator/ros2_nodes/`
6. **Check the epic:** Review relevant epic in `docs/prd/epic-*.md`

### For Hardware Builders

1. **BOM:** `hardware/bom/full-olaf-bom.csv` (order parts)
2. **3D printing:** `hardware/3d-models/<module>/` (print parts)
3. **Wiring:** `hardware/wiring-diagrams/` (assemble electronics)
4. **Firmware:** `modules/<module>/README.md` (flash ESP32s)
5. **Calibration:** `tools/calibration/` (tune servos, sensors)

### For Software Developers

**Python/ROS2 Development:**
- Node implementations: `orchestrator/ros2_nodes/`
- Tests: `tests/unit/orchestrator/`
- Configuration: `orchestrator/config/`

**ESP32 Firmware Development:**
- Firmware: `modules/<module>/firmware/`
- Tests: `tests/unit/modules/`
- Build config: `modules/<module>/platformio.ini`

### For Documentation Writers

- Architecture docs: `docs/architecture/`
- PRD docs: `docs/prd/`
- Module READMEs: `modules/<module>/README.md`
- Build guides: `hardware/` (future addition)

---

## Development Workflow by Role

### Module Firmware Developer

**Working on Head Module:**
```bash
# Navigate to module
cd modules/head/firmware/

# Edit code
vim head_controller.ino

# Build and upload
pio run -t upload

# Test standalone
olaf-test head --emotion happy --intensity 3

# Run unit tests
pio test

# Update documentation
vim ../README.md
```

### Orchestrator Developer

**Working on Personality Coordination:**
```bash
# Navigate to orchestrator
cd orchestrator/

# Edit node
vim ros2_nodes/personality/personality_coordinator_node.py

# Run tests
pytest ../tests/unit/orchestrator/personality/

# Build ROS2 package
colcon build --packages-select olaf_orchestrator

# Launch for testing
ros2 launch olaf_orchestrator minimal.launch.py
```

### Hardware Designer

**Designing Neck Gimbal:**
```bash
# Navigate to 3D models
cd hardware/3d-models/neck/

# Edit in OnShape (cloud-based)
# Export STL → neck-gimbal-v1.3.stl

# Update BOM
vim ../../bom/full-olaf-bom.csv

# Create wiring diagram
# Save to ../../wiring-diagrams/neck-wiring-diagram.png
```

---

## Gitignore Highlights

**The following are excluded from version control:**

- `.env` files (API keys, secrets)
- `*.bin` (compiled firmware binaries)
- `build/`, `dist/` (build artifacts)
- `__pycache__/`, `*.pyc` (Python cache)
- `.vscode/`, `.idea/` (IDE settings - personal preference)
- `*.log` (log files)

**Template files ARE committed:**
- `api-keys.template.env` (shows required keys without values)

---

## Monorepo Benefits for OLAF

**Why monorepo vs separate repos?**

1. **Single source of truth** - Hardware, firmware, software, docs evolve together
2. **Coordinated versioning** - Tag releases that include all components
3. **Simplified cloning** - `git clone` gives you everything
4. **Cross-component refactoring** - Change I2C protocol affects firmware + orchestrator in one PR
5. **Educational clarity** - Newcomers see complete project structure
6. **Build-in-public goal** - Community can replicate entire system from one repo

---

## Change Log

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2025-10-12 | v1.0 | Initial source tree documentation | Winston (Architect Agent) |

---

## Related Documentation

- **Repository Structure Rationale:** `docs/architecture/high-level-architecture.md#repository-structure`
- **Coding Standards:** `docs/architecture/coding-standards.md`
- **Tech Stack:** `docs/architecture/tech-stack.md`
- **Development Workflow:** `docs/architecture/coding-standards.md#git-workflow-and-code-review`
