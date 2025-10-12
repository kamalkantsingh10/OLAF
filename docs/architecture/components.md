# Components

## Head Module

**Responsibility:** Primary sensor hub and emotional expression through animated OLED eyes.

**Hardware:**
- ESP32-S3-WROOM-2 (N8R8: 32MB Flash, 8MB PSRAM)
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

## Ears Module

**Responsibility:** 2-DOF articulated ears (Chappie-inspired), 4× Feetech SCS0009 servos.

**Hardware:**
- ESP32-S3-WROOM-2 (N8R8: 32MB Flash, 8MB PSRAM)
- 4× Feetech SCS0009 servos
- Bus Servo Controller (dedicated for ears, STSC series compatible)
  - Integrates servo power supply and control circuit
  - Serial bus interface to ESP32

**Firmware:**
```
ears_controller.ino
├── i2c_slave.cpp
├── servo_bus_controller.cpp    // STSC bus servo driver
├── motion_profiles.cpp
└── position_calibration.cpp
```

---

## Neck Module

**Responsibility:** 3-DOF head articulation (pan ±90°, tilt ±45°, roll ±30°). Shares servo controller with Base module kickstand.

**Hardware:**
- ESP32-S3-WROOM-2 (N8R8: 32MB Flash, 8MB PSRAM)
- 3× Feetech STS3215 servos (30 kg·cm torque)
- Shared Bus Servo Controller (STSC series compatible)
  - Controls 3× neck servos + 1× base kickstand servo
  - Integrates servo power supply and control circuit
  - Serial bus interface to ESP32

**Firmware:**
```
neck_controller.ino
├── i2c_slave.cpp
├── servo_bus_controller.cpp    // STSC bus servo driver (4 servos total)
├── kinematics.cpp
├── trajectory_planner.cpp
└── safety_limits.cpp
```

---

## Base Module (Self-Balancing)

**Responsibility:** Autonomous two-wheel balancing with 200Hz PID control, ODrive motor coordination, kickstand deployment.

**Hardware:**
- ESP32-S3-WROOM-2 (N8R8: 32MB Flash, 8MB PSRAM) - Upgraded for superior LX7 real-time performance (200Hz PID control)
- MPU6050 IMU (200Hz sampling)
- ODrive v3.6 motor controller
- 2× hoverboard hub motors (350W each)
- 1× Feetech STS3215 kickstand servo (30 kg·cm torque)
  - Controlled via shared Bus Servo Controller (managed by Neck module)

**Firmware Architecture:**
```
base_controller.ino (most complex)
├── i2c_slave.cpp
├── balancing_controller.cpp    // 200Hz PID loop
├── imu_fusion.cpp
├── odrive_uart.cpp
├── kickstand_control.cpp       // Commands sent to Neck module's servo controller
├── state_machine.cpp
├── odometry_publisher.cpp
└── safety_monitor.cpp
```

**Key Features:**
- 200Hz real-time PID balancing
- State machine: RELAXED → TRANSITIONING → BALANCING → EMERGENCY_STOP
- Fall detection: |pitch| > 45° triggers kickstand
- 10Hz odometry publication

---

## Projector Module

**Responsibility:** Floor projection for AI-generated content display.

**Hardware:**
- ESP32-S3-WROOM-2 (N8R8: 32MB Flash, 8MB PSRAM)
- TI DLP LightCrafter OR HDMI pico projector
- 12V power supply

**Note:** If HDMI mode, Pi drives projector directly, ESP32 only controls power relay.

---

## Orchestration Layer (Raspberry Pi)

**Hardware:**
- Raspberry Pi 5 8GB + Hailo-8L AI Kit
- 128GB SD card
- Official 5A power supply (via buck converter)

**Software Architecture:**
```
orchestrator/
├── ros2_nodes/
│   ├── hardware_drivers/       # 5× I2C bridge nodes
│   ├── personality/
│   ├── ai_integration/
│   ├── navigation/
│   └── system_monitor/
├── launch/
│   └── olaf_full.launch.py
├── config/
└── ota_server/
```

---

_(Architecture document continues - sections remaining: API Specification, External APIs, Core Workflows, Database Schema, Backend Architecture, Testing Strategy, Coding Standards, Error Handling, Monitoring, Deployment)_

