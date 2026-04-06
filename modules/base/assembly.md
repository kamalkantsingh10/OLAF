# Base Module Assembly

## Components

- ESP32-S3-DevKitC-1 (N16R8)
- BNO085 IMU (GY-BNO085 breakout)
- SSD1306 OLED (0.91" 128x32, I2C)
- ODrive v3.6 motor controller
- 2x Hoverboard BLDC motors (15 pole pairs, hall sensors)
- 36V hoverboard battery
- 36V→5V buck converter (ESP32 power)
- 36V→12V buck converter (servo power)

## Firmware Architecture

```
main.cpp                 — Boot sequence, state machine, serial commands
├── display_driver       — OLED init, messages, 3D wireframe HUD
├── motor_controller     — Enable/disable, balance/linear/turn, safety, auto-recover, calibration
│   └── odrive_uart      — Raw UART ASCII protocol (send/read only)
├── imu                  — BNO085 euler angles at 200Hz (on-chip AHRS)
└── config.h             — All constants (pins, PID gains, safety thresholds, velocity limits)
```

### Source Files

| File | Purpose |
|------|---------|
| `src/main.cpp` | Boot sequence, state machine, serial command handler (CALIBRATE/PIDTEST/STOP) |
| `src/config.h` | Pin assignments, PID constants, safety thresholds, velocity limits |
| `src/motor_controller.h/cpp` | High-level motor interface for PID and navigation |
| `src/odrive_uart.h/cpp` | Low-level ODrive UART ASCII protocol |
| `src/display_driver.h/cpp` | SSD1306 OLED display, messages, real-time HUD |
| `src/imu.h/cpp` | BNO085 IMU driver, euler angles, accuracy reporting |

### MotorController Interface

```cpp
motors.begin();                          // Init ODrive UART
motors.enable();                         // Enter closed-loop control
motors.disable();                        // Idle mode
motors.emergencyStop();                  // Zero velocity + disable

motors.setBalanceVelocity(vel_revs);     // PID pitch correction (both wheels)
motors.setLinearVelocity(mps);           // Forward/backward (navigation/joystick)
motors.setTurnRate(radps);               // Differential steering (navigation/joystick)

motors.update(current_pitch_deg);        // Apply velocities + safety check (call every loop)
motors.calibrate();                      // Full calibration sequence (blocking)

motors.isReady();                        // No errors
motors.isEnabled();                      // Closed-loop active
motors.isSafetyTripped();                // Tilt exceeded threshold
motors.hasError(err0, err1);             // Read ODrive error registers
motors.getAxisVelocity(axis);            // Last commanded velocity
```

### Safety System

- **Emergency stop**: pitch exceeds 45° → motors disabled immediately
- **Auto-recover**: pitch stays below 30° for 1 second → motors re-enable
- **Velocity clamping**: all outputs clamped to ±10 rev/s (ODrive limit)
- **Navigation limits**: linear ±1.0 m/s, turn ±2.0 rad/s

### Serial Commands

| Command | Action |
|---------|--------|
| `CALIBRATE` | Clear errors, run full calibration on both axes, save pre_calibrated |
| `PIDTEST` | P-only balance test with 20Hz CSV telemetry |
| `STOP` | Zero velocity, disable motors |

### ODrive Setup (One-Time)

1. Connect ODrive via USB to PC
2. Run `poetry run python tools/diagnostics/odrive_test_and_calibrate.py`
3. Script configures motor params, calibrates, tests velocity, saves pre_calibrated
4. Future power-ups skip calibration (pre_calibrated flag)

See `modules/base/wiring.md` for full wiring reference and ODrive configuration values.

## Assembly Steps

[To be documented]

## Testing

[To be documented]
