# Components

## Head Module

**Responsibility:** Primary sensor hub and emotional expression through animated OLED eyes.

**Hardware:**
- ESP32-S3-WROOM-2 (N16R8: 16MB Flash, 8MB PSRAM)
- 2× GC9A01 Round TFT Display (1.28", 240×240, SPI)
- DFRobot SEN0395 mmWave sensor
- INMP441 I2S microphone × 2
- PAM8403 amplifier + speaker

**Firmware Architecture:**
```
head_controller.ino
├── i2c_slave.cpp
├── animation_engine.cpp
├── gc9a01_driver_spi.cpp    // Round TFT driver (240×240)
├── presence_sensor.cpp
└── beeper.cpp
```

**Key Features:**
- 60 FPS animation rendering (240×240 round TFT eyes)
- 20Hz presence polling
- I2C interrupt-driven commands
- Full-color expressive eyes (65K colors vs monochrome)

---

## Ears + Neck Module (Shared ESP32)

**Responsibility:** Upper body articulation - 2-DOF ears (4× servos) and 3-DOF neck gimbal (3× servos).

**Hardware:**
- ESP32-S3-WROOM-2 (N16R8: 16MB Flash, 8MB PSRAM)
- 2× Bus Servo Controllers (STSC series compatible), connected via UART:
  - **Controller A (UART1):** 4× Feetech SCS0009 servos (ears only, 2-DOF × 2)
  - **Controller B (UART2):** 3× Feetech STS3215 servos (neck only: pan/tilt/roll, 30 kg·cm torque)

**Firmware Architecture:**
```
ears_neck_controller.ino
├── i2c_slave.cpp
├── ear_servo_controller.cpp       // Controller A via UART1 (SCS0009 × 4)
├── neck_servo_controller.cpp      // Controller B via UART2 (STS3215 × 3)
├── motion_profiles.cpp
├── kinematics.cpp                 // Neck 3-DOF forward/inverse kinematics
└── coordinated_gestures.cpp       // Ears + neck synchronized movements
```

**Key Features:**
- Unified upper-body control (ears + neck coordination)
- Independent ear articulation (2-DOF each: horizontal + vertical)
- 3-DOF neck kinematics (pan ±90°, tilt ±45°, roll ±30°)
- Coordinated gestures (e.g., "look left" = neck pan + ears orient)
- Dual UART interface for simultaneous control of both servo buses

**I2C Address:** 0x09

---

## Base Module (Self-Balancing + Kickstand)

**Responsibility:** Autonomous two-wheel balancing with 200Hz PID control, ODrive motor coordination, kickstand deployment.

**Hardware:**
- ESP32-S3-WROOM-2 (N16R8: 16MB Flash, 8MB PSRAM) - LX7 dual-core for real-time control
- MPU6050 IMU (200Hz sampling, I2C)
- ODrive v3.6 motor controller (UART1)
- 2× Hoverboard hub motors (350W each)
- **1× Feetech STS3215 kickstand servo** (30 kg·cm torque, fall protection, UART2 serial bus)

**Firmware Architecture:**
```
base_controller.ino
├── i2c_slave.cpp
├── balancing_controller.cpp       // 200Hz PID loop (FreeRTOS task)
├── imu_fusion.cpp                 // MPU6050 complementary filter
├── odrive_uart.cpp                // Motor velocity commands via UART1
├── kickstand_control.cpp          // STS3215 servo via UART2 (deploy <200ms)
├── state_machine.cpp              // RELAXED → BALANCING → EMERGENCY
├── odometry_publisher.cpp         // 10Hz wheel encoder data
└── safety_monitor.cpp             // Fall detection: |pitch| > 45°
```

**Key Features:**
- 200Hz real-time PID balancing (ESP32 FreeRTOS guarantees)
- State machine: RELAXED (kickstand down) → TRANSITIONING → BALANCING → EMERGENCY_STOP
- Fall detection triggers kickstand deployment within 200ms
- 10Hz odometry publication to orchestrator (SLAM-grade accuracy)
- Kickstand managed locally for time-critical fall protection

**I2C Address:** 0x0B

---

## Body Module (Heart Display + Projector Control + LEDs)

**Responsibility:** Body-mounted indicators - heart display emotion visualization, projector power management, system status LEDs.

**Hardware:**
- ESP32-S3-WROOM-2 (N16R8: 16MB Flash, 8MB PSRAM)
- 1× Round TFT LCD Display (1.53", 360×360, QSPI, ST77916 driver) - Heart animation
- 1× Relay or MOSFET module (12V projector power switching)
- Nx WS2812B RGB LED strip (addressable, 5-10 LEDs for status indicators)

**Firmware Architecture:**
```
body_controller.ino
├── i2c_slave.cpp
├── heart_animation.cpp
├── st77916_driver_qspi.cpp  // Round TFT driver (360×360, QSPI)
├── projector_power.cpp      // Relay control + status monitoring
└── led_controller.cpp       // WS2812B addressable LED patterns
```

**Key Features:**
- 60 FPS heart animation (emotion-driven BPM 50-120)
- Heart color variations for emotional states (red=default, blue=sad, yellow=happy)
- Projector power switching (ON/OFF via relay)
- RGB LED patterns (status indicators, mood lighting, breathing effects synchronized with heart)
- Coordinated with personality system

**I2C Address:** 0x0A

**Note:** Projector content driven directly by Pi via HDMI; ESP32 only controls power and LEDs.

---


## Orchestration Layer (Raspberry Pi)

**Hardware:**
- Raspberry Pi 5 8GB + Hailo-8L AI Kit
- 128GB SD card
- Official 5A power supply (via buck converter)

**Software Architecture:**
```
ros2/src/orchestrator/
├── ros2_nodes/
│   ├── hardware_drivers/       # 4× I2C bridge nodes (Head, Ears+Neck, Body, Base)
│   ├── personality/
│   ├── ai_integration/
│   ├── navigation/
│   └── system_monitor/
├── launch/
│   └── olaf_full.launch.py
├── config/
├── package.xml
└── setup.py
```

**I2C Module Summary:**
- 0x08: Head (eyes, sensors, audio)
- 0x09: Ears + Neck (7 servos via 2 controllers)
- 0x0A: Body (heart LCD, projector relay, LEDs)
- 0x0B: Base (balancing, odometry, kickstand)

---

_(Architecture document continues - sections remaining: API Specification, External APIs, Core Workflows, Database Schema, Backend Architecture, Testing Strategy, Coding Standards, Error Handling, Monitoring, Deployment)_

