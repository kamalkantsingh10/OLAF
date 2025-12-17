# Story 4.9: Develop Base ESP32 Firmware with 200Hz Balancing PID

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 12-16 hours

---

## User Story

**As a** builder,
**I want** ESP32 firmware that implements 200Hz PID control loop for self-balancing using IMU feedback,
**so that** the robot can balance autonomously on two wheels.

---

## Acceptance Criteria

1. ✅ PlatformIO project created with I2C slave (0x0B), IMU, and ODrive UART drivers
2. ✅ 200Hz control loop implemented with precise timing
3. ✅ MPU6050 complementary filter for pitch angle estimation
4. ✅ PID controller for pitch stabilization (tunable gains)
5. ✅ Motor commands sent to ODrive via UART at 200Hz
6. ✅ I2C slave interface for Pi commands (velocity, enable/disable)
7. ✅ Safety features: Tilt angle limits, emergency stop, watchdog
8. ✅ Firmware tested: Robot balances upright for >10 seconds

---

## Implementation Steps

### 1. Create PlatformIO Project

```bash
cd ~/olaf/modules/base/firmware
mkdir -p base-main
cd base-main
pio init --board esp32dev
```

### 2. Configure Dependencies

**Edit `platformio.ini`:**

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200

lib_deps =
    Wire
    adafruit/Adafruit MPU6050 @ ^2.2.4
    ODriveArduino @ ^0.5.2

build_flags =
    -D I2C_SLAVE_ADDRESS=0x0B
    -D CONTROL_FREQ=200
    -D DEBUG_MODE=1
```

### 3. Implement IMU Driver with Complementary Filter

**Create `src/imu_driver.cpp`:**

```cpp
#include "imu_driver.h"
#include <Adafruit_MPU6050.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float pitch = 0.0;  // Pitch angle in degrees
float pitchRate = 0.0;  // Pitch angular velocity in deg/s

const float ALPHA = 0.98;  // Complementary filter coefficient

void IMU_Init() {
    if (!mpu.begin()) {
        Serial.println("MPU6050 init failed!");
        while (1) delay(10);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 initialized");
}

void IMU_Update(float dt) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Calculate pitch from accelerometer (degrees)
    float accelPitch = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;

    // Gyro pitch rate (already in deg/s)
    pitchRate = gyro.gyro.x * 180.0 / PI;

    // Complementary filter
    pitch = ALPHA * (pitch + pitchRate * dt) + (1.0 - ALPHA) * accelPitch;
}

float IMU_GetPitch() { return pitch; }
float IMU_GetPitchRate() { return pitchRate; }
```

### 4. Implement PID Controller

**Create `src/pid_controller.cpp`:**

```cpp
#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd, float dt)
    : kp(kp), ki(ki), kd(kd), dt(dt), integral(0), prevError(0) {}

float PIDController::compute(float setpoint, float measured) {
    float error = setpoint - measured;

    // Proportional term
    float P = kp * error;

    // Integral term with anti-windup
    integral += error * dt;
    integral = constrain(integral, -10.0, 10.0);  // Limit windup
    float I = ki * integral;

    // Derivative term
    float derivative = (error - prevError) / dt;
    float D = kd * derivative;

    prevError = error;

    float output = P + I + D;
    return constrain(output, -20.0, 20.0);  // Motor velocity limit
}

void PIDController::reset() {
    integral = 0;
    prevError = 0;
}

void PIDController::setGains(float kp_, float ki_, float kd_) {
    kp = kp_;
    ki = ki_;
    kd = kd_;
}
```

### 5. Implement ODrive UART Interface

**Create `src/odrive_interface.cpp`:**

```cpp
#include "odrive_interface.h"
#include <HardwareSerial.h>
#include <ODriveArduino.h>

HardwareSerial odriveSerial(2);  // UART2
ODriveArduino odrive(odriveSerial);

void ODrive_Init() {
    odriveSerial.begin(115200, SERIAL_8N1, 16, 17);  // RX=16, TX=17
    delay(500);
    Serial.println("ODrive UART initialized");
}

void ODrive_SetVelocity(float leftVel, float rightVel) {
    odrive.SetVelocity(0, leftVel);   // Axis 0 (left motor)
    odrive.SetVelocity(1, rightVel);  // Axis 1 (right motor)
}

void ODrive_Enable() {
    odriveSerial.println("w axis0.requested_state 8");  // CLOSED_LOOP_CONTROL
    odriveSerial.println("w axis1.requested_state 8");
}

void ODrive_Disable() {
    odriveSerial.println("w axis0.requested_state 1");  // IDLE
    odriveSerial.println("w axis1.requested_state 1");
}
```

### 6. Implement I2C Slave Interface

**Create `src/i2c_slave.cpp`:**

```cpp
#include "i2c_slave.h"
#include <Wire.h>

volatile uint8_t registers[256] = {0};
volatile bool balancingEnabled = false;
volatile float targetVelocity = 0.0;

void onReceive(int numBytes) {
    if (numBytes < 1) return;

    uint8_t regAddr = Wire.read();

    while (Wire.available()) {
        uint8_t value = Wire.read();
        registers[regAddr] = value;

        // Parse commands
        if (regAddr == REG_BALANCE_ENABLE) {
            balancingEnabled = (value == 1);
        }
        else if (regAddr == REG_TARGET_VELOCITY) {
            // Convert uint8 to float (-10 to +10 m/s)
            targetVelocity = (value - 128) / 12.8;
        }

        regAddr++;
    }
}

void onRequest() {
    static uint8_t currentReg = 0;
    Wire.write(registers[currentReg]);
}

void I2C_Slave_Init() {
    registers[REG_DEVICE_ID] = 0x04;  // Base module
    registers[REG_FIRMWARE_VER] = 0x10;  // v1.0

    Wire.begin(I2C_SLAVE_ADDRESS, 21, 22, 400000);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
}

bool I2C_IsBalancingEnabled() { return balancingEnabled; }
float I2C_GetTargetVelocity() { return targetVelocity; }
```

### 7. Implement Main Control Loop

**Create `src/main.cpp`:**

```cpp
#include <Arduino.h>
#include "imu_driver.h"
#include "pid_controller.h"
#include "odrive_interface.h"
#include "i2c_slave.h"

// Control loop timing
const float CONTROL_FREQ = 200.0;  // Hz
const float DT = 1.0 / CONTROL_FREQ;  // 5ms

// PID controller for pitch stabilization
PIDController balancePID(50.0, 5.0, 2.0, DT);

// Safety limits
const float TILT_LIMIT = 30.0;  // degrees
const float VELOCITY_LIMIT = 10.0;  // rev/s

unsigned long lastLoopTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("OLAF Base Module v1.0");

    IMU_Init();
    I2C_Slave_Init();
    ODrive_Init();

    delay(1000);
    Serial.println("System ready - balance mode");
}

void loop() {
    unsigned long currentTime = micros();

    // Maintain 200Hz loop rate (5000 microseconds)
    if (currentTime - lastLoopTime >= 5000) {
        lastLoopTime = currentTime;

        // Update IMU
        IMU_Update(DT);
        float pitch = IMU_GetPitch();

        // Safety check: Disable if tilted too far
        if (abs(pitch) > TILT_LIMIT) {
            ODrive_Disable();
            balancePID.reset();
            Serial.println("TILT LIMIT EXCEEDED - DISABLED");
            return;
        }

        // Balancing control
        if (I2C_IsBalancingEnabled()) {
            // Setpoint: 0 degrees (upright)
            float balanceOutput = balancePID.compute(0.0, pitch);

            // Add forward velocity command from Pi
            float targetVel = I2C_GetTargetVelocity();

            // Motor commands (differential drive)
            float leftVel = balanceOutput + targetVel;
            float rightVel = balanceOutput + targetVel;

            // Clamp to velocity limits
            leftVel = constrain(leftVel, -VELOCITY_LIMIT, VELOCITY_LIMIT);
            rightVel = constrain(rightVel, -VELOCITY_LIMIT, VELOCITY_LIMIT);

            // Send to ODrive
            ODrive_SetVelocity(leftVel, rightVel);

            // Debug output (every 50 loops = 4Hz)
            static int debugCounter = 0;
            if (++debugCounter >= 50) {
                Serial.print("Pitch: "); Serial.print(pitch);
                Serial.print(" | Vel: "); Serial.print(leftVel);
                Serial.print(" / "); Serial.println(rightVel);
                debugCounter = 0;
            }
        }
        else {
            // Balancing disabled - set motors to idle
            ODrive_SetVelocity(0, 0);
        }
    }

    // Small delay to prevent watchdog reset
    delayMicroseconds(100);
}
```

### 8. Tune PID Gains

**Tuning procedure:**

```bash
# 1. Start with low gains: P=10, I=0, D=0
# 2. Increase P until robot oscillates
# 3. Back off P by 20-30%
# 4. Add I to eliminate steady-state error (start with I = P/10)
# 5. Add D to reduce overshoot (start with D = P/5)
# 6. Fine-tune all three gains

# Expected final gains (approximate):
# P = 50-80
# I = 5-15
# D = 2-5

# Adjust via I2C registers (from Pi):
i2cset -y 1 0x0B 0x30 50   # Set P gain
i2cset -y 1 0x0B 0x31 5    # Set I gain
i2cset -y 1 0x0B 0x32 2    # Set D gain
```

### 9. Add Safety Features

**Implement watchdog and limits:**

```cpp
// In main.cpp:

// Watchdog: If no I2C command in 1 second, disable balancing
unsigned long lastI2CCommand = 0;
const unsigned long I2C_TIMEOUT = 1000;  // ms

// In loop():
if (millis() - lastI2CCommand > I2C_TIMEOUT) {
    balancingEnabled = false;
    ODrive_Disable();
}

// Battery voltage monitoring (via ADC)
float batteryVoltage = analogRead(34) * (11.0 / 4095.0);  // Voltage divider
if (batteryVoltage < 32.0) {
    Serial.println("LOW BATTERY - DISABLED");
    ODrive_Disable();
}
```

### 10. Test Balancing

**Testing procedure:**

```bash
# 1. Upload firmware
pio run --target upload

# 2. Monitor serial output
pio device monitor

# 3. Hold robot upright (manually)
# 4. Enable balancing via I2C:
i2cset -y 1 0x0B 0x10 1  # Enable balance mode

# 5. Slowly release robot
# Expected: Robot balances for >10 seconds

# 6. Test forward velocity:
i2cset -y 1 0x0B 0x11 140  # Slight forward (value 140 = +1.25 m/s)

# 7. Emergency stop test:
#    Press E-stop → motors cut power immediately
```

---

## Testing & Validation

**Test 1: Control Loop Timing**
```bash
# Measure loop frequency with oscilloscope on debug pin
# Expected: 200Hz ± 5Hz
```

**Test 2: IMU Data Quality**
```bash
# Pitch angle should track actual tilt
# No excessive noise or drift
```

**Test 3: Balancing Stability**
```bash
# Robot balances upright for >10 seconds
# Responds to small pushes (returns to upright)
```

**Test 4: Safety Features**
```bash
# Disables at 30° tilt
# Disables on low battery
# Disables on I2C timeout
```

---

## Troubleshooting

**Issue 1: Robot Falls Over Immediately**
- **Solution:** Increase P gain, check IMU orientation (pitch axis correct), verify motor directions

**Issue 2: Robot Oscillates**
- **Solution:** Reduce P gain, increase D gain, check control loop timing

**Issue 3: Control Loop Not 200Hz**
- **Solution:** Reduce serial prints, optimize IMU read, check micros() timing

**Issue 4: Motors Don't Respond**
- **Solution:** Check ODrive UART wiring, verify ODrive in closed-loop mode, check velocity limits

---

## Dependencies

**Before this story:**
- Story 4.8: Configure ODrive ✅

**After this story:**
- Story 4.10: Fine-Tune Self-Balancing PID Parameters

---

## References

- [Balancing Robot Theory](https://www.instructables.com/Self-Balancing-Robot/)
- [PID Control Tuning](https://en.wikipedia.org/wiki/PID_controller#Manual_tuning)
- [Complementary Filter](https://www.pieter-jan.com/node/11)

---

## Notes

- **200Hz Critical:** High-frequency control required for fast stabilization response
- **Complementary Filter:** Combines accelerometer (long-term accuracy) with gyro (short-term accuracy)
- **PID Tuning:** Iterative process, expect 1-2 hours of tuning for good performance
- **Safety First:** Always test with E-stop in reach, robot can move suddenly
- **Tilt Limit:** 30° prevents damage if robot falls
- **Future:** Add Kalman filter for better state estimation, cascaded PID for velocity control

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
