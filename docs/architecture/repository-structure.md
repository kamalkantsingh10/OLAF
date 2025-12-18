# Repository Structure

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

**Module Responsibilities:**
- **Head+Ears (0x08):** 2× OLED eyes (GC9A01), 2× ear servos, RGBD camera (USB to Pi), projector control
- **Neck (0x09):** 3× neck servos, kickstand servo, 2× mmWave presence sensors
- **Torso (0x0A):** Heart LCD (ILI9341), thermal printer (EM5820), power LEDs
- **Base (0x0B):** Self-balancing (MPU6050 IMU, ODrive motor control)

**Key Organizational Benefits:**
- **True Modularity:** Each module directory is a complete, deployable unit (firmware + hardware + tests + diagnostics)
- **Co-Located Development:** Everything for a module lives in `modules/{module}/` (code, CAD, docs, tests, tools)
- **Independent Testing:** Module-specific tests in `modules/{module}/tests/`, system tests in top-level `tests/`
- **Clear Ownership:** One person can own an entire module with clear boundaries
- **Simplified Navigation:** Working on head-ears? `cd modules/head-ears` - everything is there
- **Matches Architecture:** Code structure mirrors the physical 4-module smart peripheral architecture
