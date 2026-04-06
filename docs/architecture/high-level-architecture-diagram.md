# High Level Architecture Diagram

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
│       ORCHESTRATION LAYER (Pi 5 + Fusion HAT+ + AI HAT)         │
│                   Powered by 2000mAh 7.4V Battery               │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ RASPBERRY PI 5 16GB                                        │ │
│  │  • ROS2 Humble (5 driver nodes)                            │ │
│  │  • Personality Coordination  • State Management            │ │
│  │  • SLAM Navigation          • AI Integration               │ │
│  │  • I2C Master (GPIO2/3)     • OTA Server                   │ │
│  └────────────────────────────────────────────────────────────┘ │
│  ┌─────────────────┐ ┌─────────────────┐ ┌──────────────────┐   │
│  │ FUSION HAT+     │ │ AI HAT (Hailo)  │ │ 4" HEART DISPLAY │   │
│  │ • WS2812 port   │ │ • 26 TOPS       │ │ • Animated heart │   │
│  │ • PWM (P0-P3)   │ │ • Whisper STT   │ │ • Status info    │   │
│  │ • I2C bridge    │ │ • Vision models │ │ • SPI from Pi    │   │
│  │ • I2S speaker   │ └─────────────────┘ └──────────────────┘   │
│  └────────┬────────┘                                            │
│           │                                                      │
└───┬───────┼───────┬─────────────────┬───────────────────────────┘
    │       │       │                 │
    │I2C    │PWM    │USB ×2           │WS2812
    │       │       │                 │Data
┌───▼───┐ ┌─▼─────┐ │           ┌─────▼─────────────────────────┐
│ HEAD  │ │KICK-  │ │           │     WS2812 INDICATOR STRIP    │
│ ESP32 │ │STAND  │ │           │     (24 LEDs, 3×8 daisy)      │
│ 0x10  │ │2×Servo│ │           │  ┌────────┬────────┬────────┐ │
└───┬───┘ │4.8-7.4V│ │          │  │Strip 1 │Strip 2 │Strip 3 │ │
    │     │(at base)│ │          │  │Interact│Status  │PID     │ │
┌───▼───┐ └────────┘ │           │  │8 LEDs  │8 LEDs  │8 LEDs  │ │
│2×OLED │             │           └──┴────────┴────────┴────────┘ │
│Eyes   │             │
│SPI    │       ┌─────┴─────────────────────────────┐
└───────┘       │                                   │
                ▼                                   ▼
    ┌───────────────────────┐         ┌───────────────────────┐
    │ WAVESHARE BUS SERVO   │         │ WAVESHARE BUS SERVO   │
    │ ADAPTER (A) - NECK    │         │ ADAPTER (A) - EARS    │
    │ USB Serial            │         │ USB Serial            │
    └───────────┬───────────┘         └───────────┬───────────┘
                │                                 │
    ┌───────────▼───────────┐         ┌───────────▼───────────┐
    │ NECK SERVOS           │         │ EAR SERVOS            │
    │ 3× Feetech STS3215    │         │ 4× Feetech SCS0009    │
    │ Pan / Tilt / Roll     │         │ 2-DOF × 2 ears        │
    │ 30 kg·cm torque       │         │ Daisy-chained         │
    └───────────────────────┘         └───────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    I2C BUS (GPIO2 SDA, GPIO3 SCL)               │
└────────────────────────┬────────────────────────────────────────┘
                         │
         ┌───────────────┴───────────────┐
         │                               │
    ┌────▼────┐                     ┌────▼────┐
    │  HEAD   │                     │  BASE   │
    │  ESP32  │                     │  ESP32  │
    │  0x10   │                     │  0x11   │
    └────┬────┘                     └────┬────┘
         │                               │
    ┌────▼────────────┐         ┌────────▼────────────────────┐
    │ 2× OLED EYES    │         │ MOTOR & BALANCE SYSTEM      │
    │ 128×64, SPI     │         │ ┌─────────┐ ┌─────────────┐ │
    │ 30-60 FPS       │         │ │ BNO085 │ │ ODrive v3.6 │ │
    │ GC9A01 driver   │         │ │ AHRS    │ │ UART        │ │
    │ Eye animations  │         │ │ 200Hz   │ │ 2× BLDC     │ │
    └─────────────────┘         │ └─────────┘ └─────────────┘ │
                                └─────────────────────────────┘
```

## Power Distribution

```
┌─────────────────────────────────────────────────────────────────┐
│              36V HOVERBOARD BATTERY (Charging Port A)           │
└────────┬────────────────────┬────────────────────┬──────────────┘
         │                    │                    │
         │ 36V Direct         │ 36V→12V Buck       │ 36V→5V Buck
         │                    │                    │
    ┌────▼────┐         ┌─────▼─────┐        ┌─────▼─────────────┐
    │ ODrive  │         │ SERVOS    │        │ ESP32s + OLED     │
    │ + BLDC  │         │ Neck ×3   │        │ • Head ESP32      │
    │ Motors  │         │ Ears ×4   │        │ • Base ESP32      │
    │         │         │ (12V in)  │        │ • 2× OLED eyes    │
    └─────────┘         └───────────┘        └───────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│          2000mAh 7.4V BATTERY (Fusion HAT, Charging Port B)     │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                    ┌─────────▼─────────┐
                    │ LOGIC SYSTEM      │
                    │ • Raspberry Pi 5  │
                    │ • Fusion HAT+     │
                    │ • AI HAT (Hailo)  │
                    │ • 4" Heart Display│
                    │ • WS2812 (24 LED) │
                    │ • Kickstand ×2    │
                    └───────────────────┘
```

## Communication Protocols

```
━━━━━━━━ I2C: Pi ↔ 2× ESP32 (Head 0x10, Base 0x11) - 5-20ms latency
─ ─ ─ ─  USB Serial: Pi ↔ 2× Waveshare Adapters (Neck, Ears servos)
- - - -  WiFi: Pi → Cloud AI APIs only (Claude/GPT-4)
━ ━ ━ ━  SPI: Head ESP32 → OLED displays (high-speed graphics)
━·━·━·━  UART: Base ESP32 → ODrive (motor commands, odometry)
────────  WS2812: Pi Fusion HAT → 24 LED strip (single data line)
─·─·─·─  PWM: Pi Fusion HAT → Kickstand servos (long wire to base)
```

## ROS2 Node Topology

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS2 HUMBLE (on Pi 5)                        │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────────┐  │
│  │ olaf_head    │  │ olaf_base    │  │ olaf_indicator        │  │
│  │ I2C → 0x10   │  │ I2C → 0x11   │  │ HAT WS2812            │  │
│  │              │  │              │  │                       │  │
│  │ /head/expr   │  │ /cmd_vel     │  │ /indicator/interact   │  │
│  │ /head/blink  │  │ /odom        │  │ /indicator/status     │  │
│  │ /head/look   │  │ /imu         │  │ /indicator/pid        │  │
│  └──────────────┘  └──────────────┘  └───────────────────────┘  │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────────┐  │
│  │ olaf_neck    │  │ olaf_ears    │  │ olaf_personality      │  │
│  │ USB Serial   │  │ USB Serial   │  │ AI Integration        │  │
│  │              │  │              │  │                       │  │
│  │ /neck/pose   │  │ /ears/emote  │  │ /speech/text          │  │
│  │ /neck/lookat │  │ /ears/perk   │  │ /ai/response          │  │
│  └──────────────┘  └──────────────┘  └───────────────────────┘  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```
