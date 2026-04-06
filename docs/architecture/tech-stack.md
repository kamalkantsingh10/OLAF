# Tech Stack

## Technology Stack Table

| Category | Technology | Version | Purpose | Rationale |
|----------|-----------|---------|---------|-----------|
| **Orchestrator OS** | Ubuntu | 24.04 LTS | Operating system for orchestration layer | Native ROS2 Jazzy support, LTS stability |
| **Orchestrator Language** | Python | 3.11+ | ROS2 nodes, AI integration, orchestration logic | ROS2 rclpy support, rich AI/ML ecosystem |
| **Module Firmware Language** | C/C++ | C++17 | ESP32 embedded firmware (Head, Base only) | Arduino/ESP-IDF standard, hardware access |
| **Robotics Framework** | ROS2 Jazzy | LTS (until 2029) | Module coordination, SLAM, navigation | Long-term support, Ubuntu 24.04 native, latest features |
| **HAT Platform** | Sunfounder Fusion HAT+ | Latest | Pi expansion for PWM, WS2812, I2C, audio | Pre-built interfaces eliminate soldering, 12 PWM channels, WS2812 port, I2S audio |
| **ESP32 Communication** | I2C | 400kHz-1MHz | Pi ↔ ESP32 command/sensor data (Head 0x10, Base 0x11) | 5-20ms latency, deterministic, only 2 modules |
| **Servo Communication** | USB Serial | UART over USB | Pi ↔ Waveshare adapters (neck, ears) | Plug-and-play, no soldering, reliable |
| **Module MCU** | ESP32-S3-DevKitC-1 | ESP32-S3-WROOM-2 (N16R8) | Real-time controllers (Head, Base modules only) | Xtensa LX7 dual-core @ 240MHz, 16MB flash, 8MB PSRAM. Used only where real-time control required. **See:** [esp32-pinout.md](./esp32-pinout.md) |
| **Eye Display Interface** | SPI | 10-20 MHz | Head ESP32 → OLED eye displays | 60 FPS full-color animation |
| **Eye Display Driver** | GC9A01 | 1.28" Round | Color eye displays (2× per Head module) | 240×240 RGB, 65K colors, SPI interface, circular display perfect for eyes |
| **Heart Display** | 4" Raspberry Pi Display | SPI/DSI | Animated heart, status display | Direct Pi control, replaces dedicated Torso ESP32, powered by logic battery |
| **AI Accelerator** | Hailo-8L AI Kit | 26 TOPS | Local Whisper STT inference | Eliminates 1-1.5s cloud STT latency, $70 investment |
| **Local AI Model** | Whisper (tiny/base) | OpenAI Whisper | Speech-to-text (Hailo-accelerated) | <200ms latency, offline-capable, accurate |
| **Cloud AI Agent** | Claude API | Claude 3.5 Sonnet | Conversational reasoning, personality generation | Best-in-class personality quality, function calling |
| **Cloud AI Fallback** | OpenAI GPT-4 API | GPT-4 Turbo | Secondary AI provider | Redundancy if Claude unavailable |
| **Motor Controller** | ODrive v3.6 | 3.6 | BLDC motor control (hoverboard wheels) | Closed-loop velocity control, encoder odometry for SLAM |
| **IMU Sensor** | BNO085 (GY-BNO085) | 9-axis AHRS | Self-balancing on Base ESP32 with on-chip sensor fusion | On-chip AHRS (Hillcrest SH-2) outputs quaternions/euler directly — no complementary filter needed, 200Hz+ update rate, superior accuracy |
| **Servo Controller (Neck)** | Waveshare Bus Servo Adapter (A) | USB mode | Controls 3× neck servos via USB serial | Integrated power + control, USB to Pi, no soldering |
| **Servo Controller (Ears)** | Waveshare Bus Servo Adapter (A) | USB mode | Controls 4× ear servos via USB serial | Integrated power + control, USB to Pi, no soldering |
| **Neck Servos** | Feetech STS3215 | Serial bus | 3-DOF neck articulation (pan/tilt/roll) | 30 kg·cm torque, daisy-chainable, position feedback |
| **Ear Servos** | Feetech SCS0009 | Serial bus | 2-DOF ear articulation (4× servos) | Daisy-chainable, position feedback |
| **Kickstand Servos** | Model Plane Landing Gear | 4.8-7.4V PWM | Retractable kickstand (2× servos) | Standard hobby servos, controlled via Fusion HAT PWM |
| **Indicator LEDs** | WS2812 | 5V addressable | Status indicators (3× 8-LED strips, 24 total) | Daisy-chainable, single data line via Fusion HAT WS2812 port |
| **SLAM Library** | Cartographer | ROS2 port | 2D/3D SLAM mapping | Google-maintained, lighter than RTAB-Map, real-time |
| **Navigation Stack** | Nav2 | ROS2 Humble | Path planning, obstacle avoidance | Standard ROS2 navigation, behavior trees |
| **Database** | SQLite | 3.40+ | Conversation history, config, logs | Embedded, zero-config, Python sqlite3 built-in |
| **I2C Library (Pi)** | smbus2 | 0.4.x | Python I2C communication to ESP32s | Pure Python, cross-platform, simple API |
| **I2C Library (ESP32)** | Wire.h | Arduino core | ESP32 I2C slave firmware | Built-in Arduino library, interrupt support |
| **Serial Library (Pi)** | pyserial | 3.5+ | USB serial to Waveshare adapters | Standard Python serial, works with USB-UART |
| **Servo Library (Pi)** | feetech-servo-sdk | 1.0+ | Feetech STS/SCS servo control | Feetech SDK, position/speed/torque control |
| **WS2812 Library (Pi)** | rpi_ws281x / Fusion HAT SDK | Latest | Control WS2812 LED strips | Native Pi support via Fusion HAT |
| **OTA Framework (ESP32)** | ESP32 OTA | Arduino/ESP-IDF | Over-the-air firmware updates (Head, Base only) | Partition support, rollback, built-in |
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

## Power System

| Component | Voltage | Source | Notes |
|-----------|---------|--------|-------|
| **36V Hoverboard Battery** | 36V | Primary power | Charging port A |
| **36V→12V Buck Converter** | 12V output | From 36V battery | Powers servos (neck, ears) via Waveshare adapters |
| **36V→5V Buck Converter** | 5V output | From 36V battery | Powers Head ESP32, Base ESP32, OLED eyes |
| **ODrive + Motors** | 36V direct | From 36V battery | No conversion needed |
| **2000mAh 7.4V Battery** | 7.4V (2S LiPo) | Logic power | Managed by Fusion HAT, charging port B |
| **Logic System** | 5V regulated | From 2000mAh via HAT | Pi 5, Fusion HAT, AI HAT, 4" display, WS2812, kickstand |

---
