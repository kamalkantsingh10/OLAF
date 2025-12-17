# Story 1.6: Develop Head+Ears ESP32 Firmware

**Epic:** Epic 1 - Head+Ears Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 12-16 hours

---

## User Story

**As a** builder,
**I want** ESP32 firmware that implements I2C slave interface and hardware drivers for all Head+Ears components,
**so that** the Pi can send commands to control eyes, ears, and projector.

---

## Acceptance Criteria

1. ✅ PlatformIO project created in `modules/head-ears/firmware/`
2. ✅ I2C slave interface implemented at address 0x08 with register map defined
3. ✅ OLED driver implemented: can display images, animations at 30+ FPS
4. ✅ Ear servo driver implemented: can command 4 servos (2 per ear) to positions
5. ✅ Projector control implemented: GPIO commands turn power on/off, PWM/UART controls focus servo
6. ✅ Basic I2C commands work: `SET_EYES`, `SET_EAR_POSITION`, `PROJECTOR_ON/OFF`, `FOCUS_NEAR/FAR`
7. ✅ Firmware flashed to ESP32 and responds to I2C commands from Pi

---

## Implementation Steps

### 1. Create PlatformIO Project Structure

```bash
cd ~/olaf/modules/head-ears/firmware
mkdir -p head-ears-main
cd head-ears-main

# Initialize PlatformIO project
pio init --board esp32dev

# Project structure:
# head-ears-main/
# ├── platformio.ini
# ├── src/
# │   ├── main.cpp
# │   ├── i2c_slave.cpp
# │   ├── oled_driver.cpp
# │   ├── servo_driver.cpp
# │   └── projector_control.cpp
# ├── include/
# │   ├── i2c_slave.h
# │   ├── oled_driver.h
# │   ├── servo_driver.h
# │   ├── projector_control.h
# │   └── register_map.h
# └── lib/ (for custom libraries)
```

### 2. Configure PlatformIO Dependencies

**Edit `platformio.ini`:**

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

; Upload speed
upload_speed = 921600
monitor_speed = 115200

; Libraries
lib_deps =
    adafruit/Adafruit GFX Library @ ^1.11.3
    adafruit/Adafruit SSD1306 @ ^2.5.7
    Wire  ; Built-in I2C library
    SPI   ; Built-in SPI library
    https://github.com/feetech/SCServo.git  ; Feetech servo library
    ESP32Servo @ ^0.13.0  ; For focus servo (PWM)

; Build flags
build_flags =
    -D I2C_SLAVE_ADDRESS=0x08
    -D OLED_WIDTH=128
    -D OLED_HEIGHT=64
    -D DEBUG_MODE=1

; Serial monitor
monitor_filters = esp32_exception_decoder
```

### 3. Define I2C Register Map

**Create `include/register_map.h`:**

```cpp
#ifndef REGISTER_MAP_H
#define REGISTER_MAP_H

// I2C Slave Address
#define I2C_SLAVE_ADDRESS 0x08

// Register Map (8-bit addresses)
// Read/Write registers unless marked RO (read-only)

// Device Info (Read-Only)
#define REG_DEVICE_ID       0x00  // RO: 0x01 (Head+Ears Module)
#define REG_FIRMWARE_VER    0x01  // RO: Firmware version (major.minor)
#define REG_STATUS          0x02  // RO: Status byte (bit flags)

// Eye Control (OLEDs)
#define REG_EYE_MODE        0x10  // RW: Eye mode (0=off, 1=open, 2=blink, 3=animation)
#define REG_EYE_ANIM_ID     0x11  // RW: Animation ID (0-255)
#define REG_EYE_BRIGHTNESS  0x12  // RW: Brightness (0-255)

// Ear Servo Control (Left Ear: Servos 1,2)
#define REG_EAR_L_BASE_POS  0x20  // RW: Left ear base position (0-255, mapped to servo angle)
#define REG_EAR_L_TIP_POS   0x21  // RW: Left ear tip position
#define REG_EAR_L_SPEED     0x22  // RW: Movement speed (0-255)

// Ear Servo Control (Right Ear: Servos 3,4)
#define REG_EAR_R_BASE_POS  0x23  // RW: Right ear base position
#define REG_EAR_R_TIP_POS   0x24  // RW: Right ear tip position
#define REG_EAR_R_SPEED     0x25  // RW: Movement speed

// Projector Control
#define REG_PROJ_POWER      0x30  // RW: Power (0=off, 1=on)
#define REG_PROJ_FOCUS      0x31  // RW: Focus position (0=near, 128=center, 255=far)

// Status Flags (REG_STATUS bit definitions)
#define STATUS_OLED_READY   (1 << 0)
#define STATUS_SERVO_READY  (1 << 1)
#define STATUS_PROJ_READY   (1 << 2)
#define STATUS_ERROR        (1 << 7)

#endif
```

### 4. Implement I2C Slave Interface

**Create `src/i2c_slave.cpp`:**

```cpp
#include <Arduino.h>
#include <Wire.h>
#include "i2c_slave.h"
#include "register_map.h"

// Register storage (volatile for ISR access)
volatile uint8_t registers[256] = {0};

// I2C receive handler
void onReceive(int numBytes) {
    if (numBytes < 1) return;

    uint8_t regAddress = Wire.read();

    // Write operation: register address + data byte(s)
    while (Wire.available()) {
        uint8_t value = Wire.read();
        registers[regAddress] = value;
        regAddress++; // Auto-increment for multi-byte writes
    }
}

// I2C request handler
void onRequest() {
    // Pi is requesting to read from previously set register address
    static uint8_t currentReg = 0;

    // Read last written register address
    Wire.write(registers[currentReg]);
}

void I2C_Slave_Init() {
    // Initialize register values
    registers[REG_DEVICE_ID] = 0x01;     // Head+Ears Module ID
    registers[REG_FIRMWARE_VER] = 0x10;  // v1.0
    registers[REG_STATUS] = 0x00;

    // Initialize I2C slave
    Wire.begin(I2C_SLAVE_ADDRESS, 21, 22, 400000); // SDA=21, SCL=22, 400kHz
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);

    Serial.println("I2C Slave initialized at address 0x08");
}

uint8_t I2C_Read_Register(uint8_t reg) {
    return registers[reg];
}

void I2C_Write_Register(uint8_t reg, uint8_t value) {
    registers[reg] = value;
}
```

**Create `include/i2c_slave.h`:**

```cpp
#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include <Arduino.h>

void I2C_Slave_Init();
uint8_t I2C_Read_Register(uint8_t reg);
void I2C_Write_Register(uint8_t reg, uint8_t value);

#endif
```

### 5. Implement OLED Eye Driver

**Create `src/oled_driver.cpp`:**

```cpp
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "oled_driver.h"
#include "register_map.h"
#include "i2c_slave.h"

// OLED pin definitions (from wiring.md)
#define OLED_DC1   21
#define OLED_CS1   5
#define OLED_RESET 16
#define OLED_DC2   22
#define OLED_CS2   17

Adafruit_SSD1306 leftEye(128, 64, &SPI, OLED_DC1, OLED_RESET, OLED_CS1);
Adafruit_SSD1306 rightEye(128, 64, &SPI, OLED_DC2, OLED_RESET, OLED_CS2);

void OLED_Init() {
    if (!leftEye.begin(SSD1306_SWITCHCAPVCC)) {
        Serial.println("Left OLED init failed");
    }
    if (!rightEye.begin(SSD1306_SWITCHCAPVCC)) {
        Serial.println("Right OLED init failed");
    }

    leftEye.clearDisplay();
    rightEye.clearDisplay();
    leftEye.display();
    rightEye.display();

    I2C_Write_Register(REG_STATUS, I2C_Read_Register(REG_STATUS) | STATUS_OLED_READY);
    Serial.println("OLEDs initialized");
}

void OLED_Update() {
    uint8_t mode = I2C_Read_Register(REG_EYE_MODE);
    uint8_t animID = I2C_Read_Register(REG_EYE_ANIM_ID);

    switch (mode) {
        case 0: // Off
            leftEye.clearDisplay();
            rightEye.clearDisplay();
            break;

        case 1: // Open eyes (static circle)
            leftEye.clearDisplay();
            rightEye.clearDisplay();
            leftEye.fillCircle(64, 32, 20, SSD1306_WHITE);
            rightEye.fillCircle(64, 32, 20, SSD1306_WHITE);
            break;

        case 2: // Blink animation
            static unsigned long lastBlink = 0;
            static bool blinkState = true;

            if (millis() - lastBlink > 2000) {
                blinkState = !blinkState;
                lastBlink = millis();
            }

            if (blinkState) {
                leftEye.fillCircle(64, 32, 20, SSD1306_WHITE);
                rightEye.fillCircle(64, 32, 20, SSD1306_WHITE);
            } else {
                leftEye.clearDisplay();
                rightEye.clearDisplay();
                leftEye.drawLine(44, 32, 84, 32, SSD1306_WHITE); // Blink line
                rightEye.drawLine(44, 32, 84, 32, SSD1306_WHITE);
            }
            break;

        case 3: // Custom animations based on animID
            // Implement various eye expressions
            // TODO: Add more animations (happy, sad, surprised, etc.)
            break;
    }

    leftEye.display();
    rightEye.display();
}
```

### 6. Implement Servo Driver (Ears)

**Create `src/servo_driver.cpp`:**

```cpp
#include <Arduino.h>
#include <SCServo.h>
#include "servo_driver.h"
#include "register_map.h"
#include "i2c_slave.h"

// Servo UART buses
SMS_STS servos;
HardwareSerial SerialLeft(1);  // GPIO26
HardwareSerial SerialRight(2); // GPIO27

// Servo IDs
#define SERVO_LEFT_BASE  1
#define SERVO_LEFT_TIP   2
#define SERVO_RIGHT_BASE 3
#define SERVO_RIGHT_TIP  4

void Servo_Init() {
    // Initialize left ear servo bus
    SerialLeft.begin(1000000, SERIAL_8N1, 26, 26); // Half-duplex UART
    SerialRight.begin(1000000, SERIAL_8N1, 27, 27);

    // Test servos
    servos.pSerial = &SerialLeft;
    servos.WritePos(SERVO_LEFT_BASE, 2048, 500);  // Center position
    servos.WritePos(SERVO_LEFT_TIP, 2048, 500);

    servos.pSerial = &SerialRight;
    servos.WritePos(SERVO_RIGHT_BASE, 2048, 500);
    servos.WritePos(SERVO_RIGHT_TIP, 2048, 500);

    I2C_Write_Register(REG_STATUS, I2C_Read_Register(REG_STATUS) | STATUS_SERVO_READY);
    Serial.println("Servos initialized");
}

void Servo_Update() {
    // Read positions from I2C registers (0-255) and map to servo positions (0-4096)
    uint16_t leftBase = map(I2C_Read_Register(REG_EAR_L_BASE_POS), 0, 255, 0, 4095);
    uint16_t leftTip = map(I2C_Read_Register(REG_EAR_L_TIP_POS), 0, 255, 0, 4095);
    uint16_t rightBase = map(I2C_Read_Register(REG_EAR_R_BASE_POS), 0, 255, 0, 4095);
    uint16_t rightTip = map(I2C_Read_Register(REG_EAR_R_TIP_POS), 0, 255, 0, 4095);
    uint16_t speed = map(I2C_Read_Register(REG_EAR_L_SPEED), 0, 255, 100, 2000); // ms

    // Update left ear servos
    servos.pSerial = &SerialLeft;
    servos.WritePos(SERVO_LEFT_BASE, leftBase, speed);
    servos.WritePos(SERVO_LEFT_TIP, leftTip, speed);

    // Update right ear servos
    servos.pSerial = &SerialRight;
    servos.WritePos(SERVO_RIGHT_BASE, rightBase, speed);
    servos.WritePos(SERVO_RIGHT_TIP, rightTip, speed);
}
```

### 7. Implement Projector Control

**Create `src/projector_control.cpp`:**

```cpp
#include <Arduino.h>
#include <ESP32Servo.h>
#include "projector_control.h"
#include "register_map.h"
#include "i2c_slave.h"

#define PROJ_POWER_PIN 25
#define FOCUS_SERVO_PIN 32

Servo focusServo;

void Projector_Init() {
    pinMode(PROJ_POWER_PIN, OUTPUT);
    digitalWrite(PROJ_POWER_PIN, LOW); // Projector off

    focusServo.attach(FOCUS_SERVO_PIN);
    focusServo.write(90); // Center position

    I2C_Write_Register(REG_STATUS, I2C_Read_Register(REG_STATUS) | STATUS_PROJ_READY);
    Serial.println("Projector control initialized");
}

void Projector_Update() {
    // Power control
    uint8_t power = I2C_Read_Register(REG_PROJ_POWER);
    digitalWrite(PROJ_POWER_PIN, power ? HIGH : LOW);

    // Focus control (0-255 maps to 0-180 degrees)
    uint8_t focusPos = I2C_Read_Register(REG_PROJ_FOCUS);
    int servoAngle = map(focusPos, 0, 255, 0, 180);
    focusServo.write(servoAngle);
}
```

### 8. Implement Main Loop

**Create `src/main.cpp`:**

```cpp
#include <Arduino.h>
#include "i2c_slave.h"
#include "oled_driver.h"
#include "servo_driver.h"
#include "projector_control.h"
#include "register_map.h"

unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 33; // 30 Hz update rate

void setup() {
    Serial.begin(115200);
    Serial.println("Head+Ears Module v1.0");

    // Initialize subsystems
    I2C_Slave_Init();
    OLED_Init();
    Servo_Init();
    Projector_Init();

    Serial.println("Initialization complete");
}

void loop() {
    // Update all subsystems at fixed rate (30 Hz)
    if (millis() - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = millis();

        OLED_Update();
        Servo_Update();
        Projector_Update();
    }

    // Small delay to prevent WDT resets
    delay(1);
}
```

### 9. Build and Upload Firmware

```bash
cd ~/olaf/modules/head-ears/firmware/head-ears-main

# Build firmware
pio run

# Upload to ESP32 (ensure USB cable connected)
pio run --target upload

# Monitor serial output
pio device monitor

# Expected output:
# Head+Ears Module v1.0
# I2C Slave initialized at address 0x08
# OLEDs initialized
# Servos initialized
# Projector control initialized
# Initialization complete
```

### 10. Test I2C Communication from Raspberry Pi

**On Raspberry Pi:**

```bash
# Scan I2C bus
i2cdetect -y 1
# Expected: 0x08 shows up

# Test write: Set eye mode to "open" (mode=1)
i2cset -y 1 0x08 0x10 0x01

# Expected: OLEDs display open eyes (circles)

# Test ear movement: Set left ear base position to 128 (center)
i2cset -y 1 0x08 0x20 0x80

# Expected: Left ear base servo moves to center position

# Test projector power on
i2cset -y 1 0x08 0x30 0x01

# Expected: Projector powers on (or LED indicator lights up)
```

---

## Testing & Validation

**Test 1: I2C Slave Response**
```bash
i2cget -y 1 0x08 0x00  # Read device ID
# Expected: 0x01

i2cget -y 1 0x08 0x01  # Read firmware version
# Expected: 0x10 (v1.0)
```

**Test 2: OLED Animation Performance**
```bash
# Measure frame rate with oscilloscope or serial logging
# Target: 30 FPS (33ms per frame)
```

**Test 3: Servo Position Accuracy**
```bash
# Command servo to multiple positions (0, 64, 128, 192, 255)
# Verify with protractor or visual inspection
# Positions should be proportional
```

**Test 4: Concurrent Operation**
```bash
# Test all subsystems simultaneously:
# - Eyes blinking
# - Ears moving
# - Projector on with focus adjustments
# Verify no lag or crashes
```

---

## Troubleshooting

**Issue 1: I2C Address Not Detected by Pi**
- **Solution:** Check I2C_Slave_Init() called, verify SDA/SCL pins, check pull-up resistors

**Issue 2: OLEDs Not Displaying**
- **Solution:** Verify SPI wiring, check OLED power, try different library (SH1106 vs SSD1306)

**Issue 3: Servos Not Responding**
- **Solution:** Check UART baud rate, verify servo IDs, test with Feetech debug tool

**Issue 4: ESP32 Crashes or Reboots**
- **Solution:** Increase stack size, check for memory leaks, disable WDT temporarily

**Issue 5: I2C Communication Unreliable**
- **Solution:** Reduce I2C speed to 100kHz, add delays in onReceive handler, check for bus conflicts

---

## Dependencies

**Before this story:**
- Story 1.4: Assemble and Test Head+Ears PCB ✅
- PlatformIO installed (Story 0.4)

**After this story:**
- Story 1.7: Create Head+Ears ROS2 Driver Node

---

## References

- [ESP32 I2C Slave Mode](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html)
- [Adafruit GFX Graphics Library](https://learn.adafruit.com/adafruit-gfx-graphics-library)
- [Feetech SCS Library](https://github.com/ftservo/SCServo)
- [PlatformIO Documentation](https://docs.platformio.org/)

---

## Notes

- **Firmware Size:** Expect ~500KB compiled binary (fits easily in ESP32 4MB flash)
- **I2C Speed:** 400kHz recommended, can go to 1MHz if needed
- **Register Map:** Design allows future expansion (256 registers available)
- **Thread Safety:** I2C handlers use ISR, minimize operations in onReceive/onRequest
- **Animation Library:** Consider creating eye expression library (happy, sad, surprised) in Phase 2

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
