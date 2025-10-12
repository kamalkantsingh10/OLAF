# Tech Stack

## Technology Stack Table

| Category | Technology | Version | Purpose | Rationale |
|----------|-----------|---------|---------|-----------|
| **Orchestrator OS** | Raspberry Pi OS (64-bit) | Debian 12 Bookworm | Operating system for orchestration layer | Official Pi OS, stable, ROS2 Humble compatibility |
| **Orchestrator Language** | Python | 3.11+ | ROS2 nodes, AI integration, orchestration logic | ROS2 rclpy support, rich AI/ML ecosystem |
| **Module Firmware Language** | C/C++ | C++17 | ESP32 embedded firmware | Arduino/ESP-IDF standard, hardware access |
| **Robotics Framework** | ROS2 Humble | LTS (until 2027) | Module coordination, SLAM, navigation | Long-term support, mature ecosystem, Pi OS compatible |
| **Module Communication** | I2C | 400kHz-1MHz | Pi ↔ ESP32 command/sensor data | 5-20ms latency, deterministic, no WiFi overhead |
| **Module MCU** | ESP32-S3-DevKitC-1 | ESP32-S3-WROOM-2 (N16R8) | Smart peripheral controllers per module (all 5 modules) | Xtensa LX7 dual-core @ 240MHz, 512KB SRAM, 8MB PSRAM, 45 GPIO, native USB OTG, vector instructions for edge AI. Standardized across Head/Ears/Neck/Projector/Base for consistency. Octal SPI provides higher memory bandwidth for OLED animation rendering. |
| **Display Interface** | SPI | 10-20 MHz | ESP32 → TFT eye displays | 60 FPS full-color animation (240×240 resolution) |
| **TFT Driver** | GC9A01 | 1.28" Round | Color eye displays (2× per Head module) | 240×240 RGB, 65K colors, SPI interface, circular display perfect for eyes |
| **AI Accelerator** | Hailo-8L AI Kit | 13 TOPS | Local Whisper STT inference | Eliminates 1-1.5s cloud STT latency, $70 investment |
| **Local AI Model** | Whisper (tiny/base) | OpenAI Whisper | Speech-to-text (Hailo-accelerated) | <200ms latency, offline-capable, accurate |
| **Cloud AI Agent** | Claude API | Claude 3.5 Sonnet | Conversational reasoning, personality generation | Best-in-class personality quality, function calling |
| **Cloud AI Fallback** | OpenAI GPT-4 API | GPT-4 Turbo | Secondary AI provider | Redundancy if Claude unavailable |
| **Motor Controller** | ODrive v3.6 | 3.6 | BLDC motor control (hoverboard wheels) | Closed-loop velocity control, encoder odometry for SLAM |
| **IMU Sensor** | MPU6050 | 6-axis | Self-balancing (gyro + accel) | $2-5, 200Hz update rate, proven for balancing robots |
| **Kickstand Servo** | Feetech STS3215 | Serial bus | Kickstand deployment (Base module) | 30 kg·cm torque, controlled via shared servo controller with Neck |
| **Ear Servos** | Feetech SCS0009 | Serial bus | 2-DOF ear articulation (4× servos) | Daisy-chainable, position feedback, dedicated servo controller |
| **Neck Servos** | Feetech STS3215 | Serial bus | 3-DOF neck articulation (3× servos) | 30 kg·cm torque, shared servo controller with kickstand (4 servos total) |
| **Servo Controller (Ears)** | Bus Servo Controller | STSC series | Controls 4× ear servos (SCS0009) | Integrated power supply and control circuit, serial bus interface |
| **Servo Controller (Neck+Kickstand)** | Bus Servo Controller | STSC series | Controls 3× neck + 1× kickstand servos (STS3215) | Shared controller manages 4 servos total, serial bus interface |
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
