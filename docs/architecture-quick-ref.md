# OLAF Architecture Quick Reference

**Version:** 1.0 | **Date:** 2025-10-11 | **1-Page Cheat Sheet**

---

## System Overview

```
INTELLIGENCE (Hybrid AI)
    Whisper STT (Hailo) + Claude/GPT-4 (Cloud)
              ↓ WiFi only
ORCHESTRATION (Raspberry Pi 5 + Hailo)
    ROS2 Humble | Python 3.11 | All nodes here
              ↓ I2C (400kHz-1MHz)
MODULE LAYER (5× ESP32 Smart Peripherals)
    Head | Ears | Neck | Projector | Base
```

---

## Critical Decisions

| Aspect | Decision | Rationale |
|--------|----------|-----------|
| **Module Communication** | I2C only (no WiFi) | 5-20ms latency vs 80-200ms, saves 1000mA power |
| **ROS2 Nodes** | Pi only (no micro-ROS) | Simpler, ESP32s = smart I2C slaves |
| **OLED Interface** | SPI (not I2C) | 30-60 FPS vs 10-15 FPS animation |
| **Motor Controller** | ODrive v3.6 | Closed-loop for SLAM odometry |
| **SLAM Library** | Cartographer | Lighter than RTAB-Map |
| **Self-Balancing** | 200Hz PID on ESP32 | Pi can't guarantee real-time |
| **OTA Updates** | V1 scope | Rapid iteration, no disassembly |

---

## I2C Address Map

| Module | Address | Key Registers |
|--------|---------|---------------|
| **Head** | 0x08 | 0x10: Expression Type, 0x11: Intensity, 0x20: Presence |
| **Ears** | 0x09 | 0x10-0x13: Servo positions |
| **Neck** | 0x0A | 0x10-0x15: Pan/Tilt/Roll (float32) |
| **Projector** | 0x0B | 0x10: Display command |
| **Base** | 0x0C | 0x10: Velocity, 0x20: Balance state, 0x30: Odometry |

**Common Registers (All Modules):**
- 0x00: Module ID
- 0x02: Status byte
- 0x04: Command trigger

---

## Hardware Stack

### Orchestrator
- **Raspberry Pi 5 8GB** + Hailo-8L AI Kit (13 TOPS)
- **OS:** Raspberry Pi OS 64-bit (Debian 12)
- **Power:** 5V @ 5A (via 36V→12V→5V buck converters)

### Modules (5× ESP32-WROOM-32)
| Module | Key Hardware |
|--------|--------------|
| **Head** | 2× OLED (SPI), mmWave (I2C), Mic (I2S), Speaker |
| **Ears** | 4× Feetech SCS0009 servos (I2C) |
| **Neck** | 3× Feetech STS3215 servos (I2C) |
| **Projector** | DLP module OR HDMI pico projector |
| **Base** | MPU6050 IMU, ODrive (UART), Kickstand servo, 2× hoverboard motors |

---

## Software Stack

| Layer | Technology | Purpose |
|-------|-----------|---------|
| **Robotics** | ROS2 Humble (LTS until 2027) | Node coordination, SLAM, navigation |
| **Orchestrator** | Python 3.11 + smbus2 | ROS2 nodes, I2C drivers |
| **Firmware** | C++17 + Arduino/PlatformIO | ESP32 embedded code |
| **Local AI** | Hailo Whisper (tiny/base) | <200ms STT |
| **Cloud AI** | Claude 3.5 Sonnet API | Personality + reasoning |
| **SLAM** | Cartographer + Nav2 | Mapping + path planning |
| **Database** | SQLite | Conversation history |
| **OTA** | Flask HTTP server + ESP32 OTA | Wireless firmware updates |

---

## ROS2 Node Structure

**High-Level Nodes (Pi):**
- `/olaf/personality_coordinator` - Coordinates expressions
- `/olaf/ai_agent` - Claude API + Hailo Whisper
- `/olaf/navigation` - Cartographer SLAM + Nav2

**Driver Nodes (Pi → I2C Bridge):**
- `/olaf/head_driver` → I2C 0x08
- `/olaf/ears_driver` → I2C 0x09
- `/olaf/neck_driver` → I2C 0x0A
- `/olaf/projector_driver` → I2C 0x0B
- `/olaf/base_driver` → I2C 0x0C

---

## Base Module Self-Balancing

**State Machine:**
```
RELAXED (kickstand down, motors idle)
    ↓ CMD_MOVE
TRANSITIONING (retracting kickstand)
    ↓ kickstand up
BALANCING (200Hz PID control)
    ↓ |pitch| > 45° OR CMD_RELAX
EMERGENCY_STOP → RELAXED
```

**Control Loop:** 200Hz (5ms period)
**IMU:** MPU6050 @ 200Hz
**PID Gains (starting):** kp=40.0, ki=0.5, kd=1.2
**Fall Threshold:** |pitch| > 45°

---

## Expression System

**Emotions:** neutral, happy, curious, thinking, confused, sad, excited (0x00-0x06)
**Intensity:** 1-5 (1=subtle, 5=extreme)
**Channels:** Eyes (OLED), Ears (servo), Neck (servo), Beeps (tone)
**Sync Target:** <500ms coordination across all channels

---

## Key File Locations

```
olaf/
├── docs/
│   ├── architecture.md           # Full architecture (this doc's source)
│   ├── architecture-quick-ref.md # This cheat sheet
│   ├── prd.md
│   └── brief.md
├── modules/
│   ├── head/firmware/            # ESP32 head controller
│   ├── base/firmware/            # ESP32 self-balancing code
│   └── ... (ears, neck, projector)
├── orchestrator/
│   ├── ros2_nodes/
│   │   ├── hardware_drivers/     # I2C bridge nodes
│   │   ├── personality/          # Expression coordination
│   │   ├── ai_integration/       # Whisper + Claude
│   │   └── navigation/           # SLAM + Nav2
│   └── ota_server/               # Flask OTA HTTP server
└── shared/
    └── i2c_registers.h           # Shared C++/Python register definitions
```

---

## Communication Protocols

| Protocol | Usage | Speed | Devices |
|----------|-------|-------|---------|
| **I2C** | Pi ↔ ESP32 modules | 400kHz-1MHz | 5 ESP32s |
| **SPI** | ESP32 → OLEDs | 10-20 MHz | 2 displays |
| **UART** | Base ESP32 → ODrive | 115200 baud | Motor controller |
| **I2S** | Mic → Head ESP32 | 16kHz | 2 microphones |
| **WiFi** | Pi → Cloud APIs | 2.4/5GHz | Claude/GPT-4 only |

---

## Power Distribution

```
36V Hoverboard Battery (4-10Ah)
    ├─→ 36V → 12V Buck (10A) → Raspberry Pi (12V→5V adapter)
    ├─→ 36V → 5V Buck (10A) → All ESP32s + Servos + Sensors
    └─→ 36V Direct → ODrive Motor Controller
```

**Runtime Target:** 2-4 hours continuous operation

---

## Development Workflow

1. **Firmware:** PlatformIO → Compile → Upload via USB (or OTA after V1)
2. **Orchestrator:** `colcon build` → ROS2 workspace
3. **Launch:** `ros2 launch orchestrator olaf_full.launch.py`
4. **Test:** `./tools/diagnostics/olaf-test <module> <command>`
5. **Monitor:** `rqt` (ROS2 GUI tools) or `ros2 topic echo`

---

## Performance Targets (from PRD)

| Requirement | Target | Architecture Support |
|-------------|--------|---------------------|
| **NFR1: AI Response** | <3s (P90) | Hailo Whisper saves 1-1.5s |
| **NFR2: Module Latency** | <100ms (P95) | I2C: 5-20ms ✅ |
| **NFR4: Battery Runtime** | 2-4 hours | WiFi disabled on ESP32s |
| **FR11: Expression Sync** | <500ms | I2C fast, coordinated pub |
| **NFR5: SLAM Accuracy** | ±10cm | ODrive closed-loop odometry |

---

## Quick Troubleshooting

| Issue | Check |
|-------|-------|
| Module not responding | `i2cdetect -y 1` (scan I2C bus) |
| Base tipping over | PID tuning, IMU calibration, kickstand timing |
| OLED choppy animation | Verify SPI mode (not I2C) |
| High latency | Confirm WiFi off on ESP32s, check ROS2 QoS |
| OTA fails | Check partition table, verify HTTP server running |

---

## Important URLs

- **Hailo SDK:** https://hailo.ai/developer-zone/
- **ODrive Docs:** https://docs.odriverobotics.com/
- **ROS2 Humble:** https://docs.ros.org/en/humble/
- **Cartographer:** https://google-cartographer-ros.readthedocs.io/

---

**🏗️ Ready to build! Start with Epic 1: Foundation & Minimal Personality**

