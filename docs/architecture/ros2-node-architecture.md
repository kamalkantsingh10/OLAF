# ROS2 Node Architecture

All ROS2 nodes run on **Raspberry Pi only**. ESP32 modules are smart I2C slave peripherals.

## Node Structure

```
RASPBERRY PI (ROS2 Humble)
═══════════════════════════════════════════════════════════════

┌─────────────────────────────────────────────────────────────┐
│               HIGH-LEVEL APPLICATION NODES                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  /olaf/personality_coordinator     (Python)                │
│    • Coordinates eyes + ears + neck + beeps                │
│    • Subscribes: /olaf/ai/emotion_command                  │
│    • Publishes: /olaf/head/eye_expression, etc.            │
│                                                             │
│  /olaf/ai_agent                    (Python)                │
│    • Claude/GPT-4 API integration                          │
│    • Whisper STT via Hailo                                 │
│    • Publishes: /olaf/ai/emotion_command                   │
│                                                             │
│  /olaf/navigation                  (Python + Nav2)         │
│    • SLAM (Cartographer/RTAB-Map)                          │
│    • Subscribes: /olaf/base/odometry                       │
│    • Publishes: /olaf/base/velocity_cmd                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                             │
                    ROS2 Topics (internal)
                             │
┌─────────────────────────────────────────────────────────────┐
│          HARDWARE DRIVER NODES (I2C Bridge)                 │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  /olaf/head_driver          (Python + smbus2)              │
│    • I2C Address: 0x08                                     │
│    • Subscribes: /olaf/head/eye_expression, /blink         │
│    • Publishes: /olaf/head/presence, /status               │
│                                                             │
│  /olaf/base_driver          (Python + smbus2)              │
│    • I2C Address: 0x0C                                     │
│    • Subscribes: /olaf/base/velocity_cmd                   │
│    • Publishes: /olaf/base/odometry, /balance_status       │
│    • Sends abstract commands: "move:0.5", "relax"          │
│    • ESP32 handles 200Hz self-balancing autonomously       │
│                                                             │
│  ... (ears, neck, projector drivers)                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
           │          │          │          │          │
           │ I2C      │ I2C      │ I2C      │ I2C      │ I2C
           │ 0x08     │ 0x09     │ 0x0A     │ 0x0B     │ 0x0C
           ▼          ▼          ▼          ▼          ▼
     ┌─────────┐┌─────────┐┌─────────┐┌─────────┐┌─────────┐
     │  ESP32  ││  ESP32  ││  ESP32  ││  ESP32  ││  ESP32  │
     │  HEAD   ││  EARS   ││  NECK   ││PROJECTOR││  BASE   │
     │         ││         ││         ││         ││         │
     │ • I2C   ││ • I2C   ││ • I2C   ││ • I2C   ││ • I2C   │
     │ • SPI   ││ • Servo ││ • Servo ││ • DLP   ││ • UART  │
     │   OLED  ││   I2C   ││   I2C   ││  HDMI   ││  ODrive │
     │ • mmWave││         ││         ││         ││ • IMU   │
     │         ││         ││         ││         ││ • PID   │
     │         ││         ││         ││         ││ • Servo │
     │         ││         ││         ││         ││(Kickstand)│
     └─────────┘└─────────┘└─────────┘└─────────┘└─────────┘
```

## Intelligence Distribution

| Task | **Raspberry Pi** | **ESP32** |
|------|------------------|-----------|
| **AI Reasoning** | ✅ Claude/GPT-4 API calls | ❌ |
| **Personality Logic** | ✅ Choose emotion + intensity | ❌ |
| **SLAM Navigation** | ✅ Map building, path planning | ❌ |
| **Animation Rendering** | ❌ | ✅ Stores frames, renders pixels |
| **Servo Control** | ❌ | ✅ PWM generation, position feedback |
| **Self-Balancing PID** | ❌ | ✅ **200Hz control loop, IMU fusion** |
| **Sensor Polling** | ❌ | ✅ mmWave, IMU, encoders |
| **Display Driver** | ❌ | ✅ SPI protocol, framebuffer |
| **Motor Control** | ❌ | ✅ UART to ODrive, velocity commands |
| **Timing-Critical Loops** | ❌ (Linux not real-time) | ✅ FreeRTOS, microsecond precision |

---
