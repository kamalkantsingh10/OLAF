# Repository Structure

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
| **Base (0x11)** | ESP32 | I2C | ODrive (UART), BNO085 AHRS (I2C), self-balancing PID |
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
