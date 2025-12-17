# Story 2.7: Develop Neck ESP32 Firmware

**Epic:** Epic 2 - Neck Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 10-14 hours

---

## User Story

**As a** builder,
**I want** ESP32 firmware that implements I2C slave interface and hardware drivers for all Neck components,
**so that** the Pi can send commands to control neck position, kickstand, and read presence sensors.

---

## Acceptance Criteria

1. ✅ PlatformIO project created in `modules/neck/firmware/`
2. ✅ I2C slave interface implemented at address 0x09 with register map defined
3. ✅ Servo driver implemented: can command 4 servos on daisy-chain UART bus (pan/tilt/roll angles + kickstand deploy/retract)
4. ✅ Presence sensor driver implemented: reads 2 sensors and reports detection state
5. ✅ Basic I2C commands work: `SET_NECK_POSITION`, `DEPLOY_KICKSTAND`, `RETRACT_KICKSTAND`, `GET_PRESENCE_STATE`
6. ✅ Smooth motion curves implemented (optional but recommended for organic movement)
7. ✅ Firmware flashed to ESP32 and responds to I2C commands from Pi

---

## Implementation Steps

### 1. Create PlatformIO Project

```bash
cd ~/olaf/modules/neck/firmware
mkdir -p neck-main
cd neck-main
pio init --board esp32dev
```

### 2. Configure Dependencies

**platformio.ini:**

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
upload_speed = 921600
monitor_speed = 115200

lib_deps =
    Wire
    https://github.com/feetech/SCServo.git

build_flags =
    -D I2C_SLAVE_ADDRESS=0x09
    -D SERVO_PAN_ID=1
    -D SERVO_TILT_ID=2
    -D SERVO_ROLL_ID=3
    -D SERVO_KICKSTAND_ID=4

monitor_filters = esp32_exception_decoder
```

### 3. Define Register Map

**include/register_map.h:**

```cpp
#ifndef REGISTER_MAP_H
#define REGISTER_MAP_H

#define I2C_SLAVE_ADDRESS 0x09

// Device Info (Read-Only)
#define REG_DEVICE_ID       0x00  // RO: 0x02 (Neck Module)
#define REG_FIRMWARE_VER    0x01  // RO: Firmware version
#define REG_STATUS          0x02  // RO: Status flags

// Neck Position Control (Pan/Tilt/Roll)
#define REG_PAN_POSITION    0x10  // RW: Pan angle (0-255, maps to servo range)
#define REG_TILT_POSITION   0x11  // RW: Tilt angle
#define REG_ROLL_POSITION   0x12  // RW: Roll angle
#define REG_MOVEMENT_SPEED  0x13  // RW: Speed (0-255, maps to ms)

// Kickstand Control
#define REG_KICKSTAND_CMD   0x20  // RW: 0=retract, 1=deploy
#define REG_KICKSTAND_STATE 0x21  // RO: Current state (0=retracted, 1=deployed, 2=moving)

// Presence Sensors
#define REG_PRESENCE_FRONT  0x30  // RO: Front sensor (0=clear, 1=detected)
#define REG_PRESENCE_REAR   0x31  // RO: Rear sensor

// Status Flags
#define STATUS_SERVO_READY  (1 << 0)
#define STATUS_SENSOR_READY (1 << 1)
#define STATUS_KICKSTAND_OK (1 << 2)
#define STATUS_ERROR        (1 << 7)

#endif
```

### 4. Implement I2C Slave

**src/i2c_slave.cpp:** (Similar to Story 1.6, adapted for address 0x09)

```cpp
#include <Wire.h>
#include "i2c_slave.h"
#include "register_map.h"

volatile uint8_t registers[256] = {0};

void onReceive(int numBytes) {
    if (numBytes < 1) return;
    uint8_t regAddress = Wire.read();
    while (Wire.available()) {
        registers[regAddress++] = Wire.read();
    }
}

void onRequest() {
    static uint8_t currentReg = 0;
    Wire.write(registers[currentReg]);
}

void I2C_Slave_Init() {
    registers[REG_DEVICE_ID] = 0x02;     // Neck Module
    registers[REG_FIRMWARE_VER] = 0x10;  // v1.0
    Wire.begin(I2C_SLAVE_ADDRESS, 21, 22, 400000);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
}
```

### 5. Implement Servo Driver

**src/servo_driver.cpp:**

```cpp
#include <SCServo.h>
#include "servo_driver.h"
#include "register_map.h"
#include "i2c_slave.h"

SMS_STS servos;
HardwareSerial ServoSerial(1);

#define SERVO_UART_PIN 32

void Servo_Init() {
    ServoSerial.begin(1000000, SERIAL_8N1, SERVO_UART_PIN, SERVO_UART_PIN);
    servos.pSerial = &ServoSerial;

    // Initialize all servos to center position
    servos.WritePos(SERVO_PAN_ID, 2048, 1000);
    servos.WritePos(SERVO_TILT_ID, 2048, 1000);
    servos.WritePos(SERVO_ROLL_ID, 2048, 1000);
    servos.WritePos(SERVO_KICKSTAND_ID, 512, 1000); // Retracted position

    I2C_Write_Register(REG_STATUS, I2C_Read_Register(REG_STATUS) | STATUS_SERVO_READY);
}

void Servo_Update() {
    // Read positions from I2C registers (0-255) and map to servo range
    uint16_t pan = map(I2C_Read_Register(REG_PAN_POSITION), 0, 255, 1024, 3072);
    uint16_t tilt = map(I2C_Read_Register(REG_TILT_POSITION), 0, 255, 1500, 3000);
    uint16_t roll = map(I2C_Read_Register(REG_ROLL_POSITION), 0, 255, 1800, 2300);
    uint16_t speed = map(I2C_Read_Register(REG_MOVEMENT_SPEED), 0, 255, 100, 2000);

    servos.WritePos(SERVO_PAN_ID, pan, speed);
    servos.WritePos(SERVO_TILT_ID, tilt, speed);
    servos.WritePos(SERVO_ROLL_ID, roll, speed);
}

void Kickstand_Update() {
    uint8_t cmd = I2C_Read_Register(REG_KICKSTAND_CMD);
    static uint8_t lastCmd = 0xFF;

    if (cmd != lastCmd) {
        if (cmd == 1) {
            // Deploy kickstand
            servos.WritePos(SERVO_KICKSTAND_ID, 2560, 1500); // 90° deployment
            I2C_Write_Register(REG_KICKSTAND_STATE, 2); // Moving
            delay(1500);
            I2C_Write_Register(REG_KICKSTAND_STATE, 1); // Deployed
        } else {
            // Retract kickstand
            servos.WritePos(SERVO_KICKSTAND_ID, 512, 1500); // 0° retracted
            I2C_Write_Register(REG_KICKSTAND_STATE, 2); // Moving
            delay(1500);
            I2C_Write_Register(REG_KICKSTAND_STATE, 0); // Retracted
        }
        lastCmd = cmd;
    }
}
```

### 6. Implement Presence Sensor Driver

**src/presence_sensors.cpp:**

```cpp
#include <Arduino.h>
#include "presence_sensors.h"
#include "register_map.h"
#include "i2c_slave.h"

#define SENSOR_FRONT_PIN 25
#define SENSOR_REAR_PIN 26

void Sensors_Init() {
    pinMode(SENSOR_FRONT_PIN, INPUT);
    pinMode(SENSOR_REAR_PIN, INPUT);

    I2C_Write_Register(REG_STATUS, I2C_Read_Register(REG_STATUS) | STATUS_SENSOR_READY);
}

void Sensors_Update() {
    bool frontDetected = digitalRead(SENSOR_FRONT_PIN);
    bool rearDetected = digitalRead(SENSOR_REAR_PIN);

    I2C_Write_Register(REG_PRESENCE_FRONT, frontDetected ? 1 : 0);
    I2C_Write_Register(REG_PRESENCE_REAR, rearDetected ? 1 : 0);
}
```

### 7. Implement Main Loop

**src/main.cpp:**

```cpp
#include <Arduino.h>
#include "i2c_slave.h"
#include "servo_driver.h"
#include "presence_sensors.h"

unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 50; // 20 Hz

void setup() {
    Serial.begin(115200);
    Serial.println("Neck Module v1.0");

    I2C_Slave_Init();
    Servo_Init();
    Sensors_Init();

    Serial.println("Initialization complete");
}

void loop() {
    if (millis() - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = millis();

        Servo_Update();
        Kickstand_Update();
        Sensors_Update();
    }
    delay(1);
}
```

### 8. Build and Upload

```bash
cd ~/olaf/modules/neck/firmware/neck-main
pio run --target upload
pio device monitor

# Expected output:
# Neck Module v1.0
# Initialization complete
```

### 9. Test I2C Communication

**On Raspberry Pi:**

```bash
# Detect module
i2cdetect -y 1
# Expected: 0x09 shows up

# Set pan position to center (128)
i2cset -y 1 0x09 0x10 0x80

# Deploy kickstand
i2cset -y 1 0x09 0x20 0x01

# Read front presence sensor
i2cget -y 1 0x09 0x30
```

---

## Testing & Validation

**Test 1: Servo Position Control**
```bash
# Set pan to 0, 128, 255
# Verify servo moves to corresponding positions
```

**Test 2: Kickstand Deployment**
```bash
# Deploy: i2cset -y 1 0x09 0x20 0x01
# Wait 2 seconds
# Read state: i2cget -y 1 0x09 0x21
# Expected: 0x01 (deployed)
```

**Test 3: Presence Detection**
```bash
# Wave hand in front of sensor
# Read: i2cget -y 1 0x09 0x30
# Expected: 0x01 when detected, 0x00 when clear
```

---

## Troubleshooting

**Issue: Servos Jitter**
- **Solution:** Reduce I2C update rate, add smoothing to position commands

**Issue: Kickstand Doesn't Complete Deployment**
- **Solution:** Increase deployment time in firmware, check mechanical binding

---

## Dependencies

**Before this story:**
- Story 2.4: Assemble and Test Neck PCB ✅
- Story 2.6: Kickstand mechanism designed (servo positions known)

**After this story:**
- Story 2.8: Create Neck ROS2 Driver Node

---

## References

- [Feetech Protocol Documentation](http://www.feetechrc.com/)
- [ESP32 I2C Slave](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html)

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
