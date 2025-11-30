# ESP32 ODrive Control - Quick Reference

## 🔌 Hardware Wiring

```
ODrive GPIO1 (TX) → ESP32 GPIO16 (RX2)
ODrive GPIO2 (RX) → ESP32 GPIO17 (TX2)
       GND        → ESP32 GND
```

**Already configured:** 115200 baud, ASCII protocol enabled

---

## ⚡ Quick Start Code

```cpp
#include <HardwareSerial.h>

HardwareSerial odriveSerial(2);

void setup() {
  // Initialize ODrive UART
  odriveSerial.begin(115200, SERIAL_8N1, 16, 17);
}

void setVelocity(int axis, float velocity) {
  String cmd = "v " + String(axis) + " " + String(velocity, 2) + "\n";
  odriveSerial.print(cmd);
}

void enableMotors() {
  odriveSerial.print("w axis0.requested_state 8\n");
  odriveSerial.print("w axis1.requested_state 8\n");
}

void loop() {
  enableMotors();
  setVelocity(0, 2.0);  // Left motor 2 rev/s
  setVelocity(1, 2.0);  // Right motor 2 rev/s
  delay(3000);
}
```

---

## 📝 ODrive ASCII Commands

| Command | Description |
|---------|-------------|
| `v 0 2.5\n` | Set axis 0 to 2.5 rev/s |
| `v 1 -1.0\n` | Set axis 1 to -1.0 rev/s (reverse) |
| `w axis0.requested_state 8\n` | Enable motor (closed-loop) |
| `w axis0.requested_state 1\n` | Disable motor (idle) |
| `r axis0.encoder.vel_estimate\n` | Read velocity |

---

## 🤖 Differential Drive (for Balancing)

```cpp
void setDifferentialVelocity(float linear, float angular) {
  const float wheel_base = 0.3;      // 30cm between wheels
  const float wheel_radius = 0.0825; // 6.5" wheels

  float base_vel = linear / (2 * PI * wheel_radius);
  float diff = (angular * wheel_base / 2.0) / (2 * PI * wheel_radius);

  setVelocity(0, base_vel + diff);  // Left
  setVelocity(1, base_vel - diff);  // Right
}

// Usage:
setDifferentialVelocity(0.5, 0);     // 0.5 m/s forward
setDifferentialVelocity(0, 0.5);     // Turn at 0.5 rad/s
setDifferentialVelocity(0.3, 0.2);   // Forward + turn
```

---

## 🎯 Story 6.5 PID Loop Example

```cpp
void loop() {
  static unsigned long last_update = 0;
  unsigned long now = millis();

  if (now - last_update >= 5) {  // 200Hz (5ms)
    last_update = now;

    // 1. Read IMU (MPU6050)
    float pitch = readPitchAngle();

    // 2. PID control
    float motor_vel = pidController.update(0, pitch);

    // 3. Send to ODrive
    setVelocity(0, motor_vel);
    setVelocity(1, motor_vel);
  }
}
```

---

## ✅ What's Already Done

- ✅ ODrive UART enabled
- ✅ Baudrate configured (115200)
- ✅ GPIO pins assigned (GPIO1/GPIO2)
- ✅ Motors calibrated
- ✅ Hall sensors working
- ✅ Velocity control tested

**You just need to:**
1. Wire 3 wires (TX, RX, GND)
2. Upload ESP32 code
3. Done!

---

See `README.md` for full examples and Python testing code.
