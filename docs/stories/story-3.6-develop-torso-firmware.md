# Story 3.6: Develop Torso ESP32 Firmware

**Epic:** Epic 3 - Torso Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 8-10 hours

---

## User Story

**As a** builder,
**I want** ESP32 firmware that implements I2C slave interface and hardware drivers for the heart display and thermal printer,
**so that** the Pi can send commands to display animations and print messages.

---

## Acceptance Criteria

1. ✅ PlatformIO project created in `modules/torso/firmware/`
2. ✅ I2C slave interface implemented at address 0x09 with register map defined
3. ✅ Heart display driver implemented: can display animations at 30+ FPS
4. ✅ Thermal printer driver implemented: can receive text and print
5. ✅ Basic I2C commands work: `SET_HEART_ANIMATION`, `PRINT_TEXT`, `PRINT_STATUS`
6. ✅ Firmware flashed to ESP32 and responds to I2C commands from Pi
7. ✅ Test printing a message and displaying heart animation

---

## Implementation Steps

### 1. Create PlatformIO Project Structure

```bash
cd ~/olaf/modules/torso/firmware
mkdir -p torso-main
cd torso-main

# Initialize PlatformIO project
pio init --board esp32dev

# Project structure:
# torso-main/
# ├── platformio.ini
# ├── src/
# │   ├── main.cpp
# │   ├── i2c_slave.cpp
# │   ├── heart_display.cpp
# │   └── thermal_printer.cpp
# ├── include/
# │   ├── i2c_slave.h
# │   ├── heart_display.h
# │   ├── thermal_printer.h
# │   └── register_map.h
# └── lib/
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
    adafruit/Adafruit ST7735 and ST7789 Library @ ^1.10.0
    adafruit/Adafruit Thermal Printer Library @ ^1.4.0
    Wire  ; Built-in I2C library

; Build flags
build_flags =
    -D I2C_SLAVE_ADDRESS=0x09
    -D HEART_DISPLAY_WIDTH=240
    -D HEART_DISPLAY_HEIGHT=240
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
#define I2C_SLAVE_ADDRESS 0x09

// Register Map (8-bit addresses)
// Read/Write registers unless marked RO (read-only)

// Device Info (Read-Only)
#define REG_DEVICE_ID       0x00  // RO: 0x03 (Torso Module)
#define REG_FIRMWARE_VER    0x01  // RO: Firmware version (major.minor)
#define REG_STATUS          0x02  // RO: Status byte (bit flags)

// Heart Display Control
#define REG_HEART_MODE      0x10  // RW: Display mode (0=off, 1=static, 2=beat, 3=animation)
#define REG_HEART_ANIM_ID   0x11  // RW: Animation ID (0-255)
#define REG_HEART_COLOR_R   0x12  // RW: Red component (0-255)
#define REG_HEART_COLOR_G   0x13  // RW: Green component (0-255)
#define REG_HEART_COLOR_B   0x14  // RW: Blue component (0-255)
#define REG_HEART_SPEED     0x15  // RW: Animation speed (0-255)

// Thermal Printer Control
#define REG_PRINT_CMD       0x20  // RW: Print command (0=idle, 1=print_buffer, 2=feed, 3=reset)
#define REG_PRINT_STATUS    0x21  // RO: Printer status (0=ready, 1=busy, 2=error)
#define REG_PRINT_BUFFER    0x22  // RW: Start of print buffer (multi-byte write)

// Status Flags (REG_STATUS bit definitions)
#define STATUS_HEART_READY    (1 << 0)
#define STATUS_PRINTER_READY  (1 << 1)
#define STATUS_ERROR          (1 << 7)

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

// Print buffer for text
char printBuffer[256] = {0};
volatile uint8_t printBufferIndex = 0;

// I2C receive handler
void onReceive(int numBytes) {
    if (numBytes < 1) return;

    uint8_t regAddress = Wire.read();

    // Write operation: register address + data byte(s)
    while (Wire.available()) {
        uint8_t value = Wire.read();

        // Special handling for print buffer
        if (regAddress >= REG_PRINT_BUFFER) {
            if (printBufferIndex < 255) {
                printBuffer[printBufferIndex++] = (char)value;
                printBuffer[printBufferIndex] = '\0';
            }
        } else {
            registers[regAddress] = value;
        }

        regAddress++; // Auto-increment for multi-byte writes
    }
}

// I2C request handler
void onRequest() {
    static uint8_t currentReg = 0;
    Wire.write(registers[currentReg]);
}

void I2C_Slave_Init() {
    // Initialize register values
    registers[REG_DEVICE_ID] = 0x03;     // Torso Module ID
    registers[REG_FIRMWARE_VER] = 0x10;  // v1.0
    registers[REG_STATUS] = 0x00;

    // Initialize I2C slave
    Wire.begin(I2C_SLAVE_ADDRESS, 21, 22, 400000); // SDA=21, SCL=22, 400kHz
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);

    Serial.println("I2C Slave initialized at address 0x09");
}

uint8_t I2C_Read_Register(uint8_t reg) {
    return registers[reg];
}

void I2C_Write_Register(uint8_t reg, uint8_t value) {
    registers[reg] = value;
}

char* I2C_Get_Print_Buffer() {
    return printBuffer;
}

void I2C_Clear_Print_Buffer() {
    printBufferIndex = 0;
    printBuffer[0] = '\0';
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
char* I2C_Get_Print_Buffer();
void I2C_Clear_Print_Buffer();

#endif
```

### 5. Implement Heart Display Driver

**Create `src/heart_display.cpp`:**

```cpp
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "heart_display.h"
#include "register_map.h"
#include "i2c_slave.h"

// Display pin definitions (2.8" round display)
#define TFT_CS    5
#define TFT_DC    16
#define TFT_RST   17
#define TFT_MOSI  23
#define TFT_SCLK  18

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Animation variables
unsigned long lastAnimFrame = 0;
uint8_t animFrame = 0;

void Heart_Init() {
    tft.init(240, 240, SPI_MODE2);
    tft.setRotation(0);
    tft.fillScreen(ST77XX_BLACK);

    I2C_Write_Register(REG_STATUS, I2C_Read_Register(REG_STATUS) | STATUS_HEART_READY);
    Serial.println("Heart display initialized");
}

void drawHeart(int16_t x, int16_t y, uint16_t size, uint16_t color) {
    // Draw a simple heart shape using circles and triangle
    int16_t r = size / 4;

    // Two circles at top
    tft.fillCircle(x - r, y, r, color);
    tft.fillCircle(x + r, y, r, color);

    // Triangle at bottom
    tft.fillTriangle(
        x - size/2, y,
        x + size/2, y,
        x, y + size,
        color
    );
}

void Heart_Update() {
    uint8_t mode = I2C_Read_Register(REG_HEART_MODE);
    uint8_t animID = I2C_Read_Register(REG_HEART_ANIM_ID);
    uint8_t r = I2C_Read_Register(REG_HEART_COLOR_R);
    uint8_t g = I2C_Read_Register(REG_HEART_COLOR_G);
    uint8_t b = I2C_Read_Register(REG_HEART_COLOR_B);
    uint8_t speed = I2C_Read_Register(REG_HEART_SPEED);

    uint16_t color = tft.color565(r, g, b);
    uint16_t animDelay = map(speed, 0, 255, 500, 50); // Slower = higher delay

    switch (mode) {
        case 0: // Off
            tft.fillScreen(ST77XX_BLACK);
            break;

        case 1: // Static heart
            tft.fillScreen(ST77XX_BLACK);
            drawHeart(120, 80, 100, color);
            break;

        case 2: // Beating heart animation
            if (millis() - lastAnimFrame > animDelay) {
                lastAnimFrame = millis();

                tft.fillScreen(ST77XX_BLACK);

                // Pulse effect (size oscillates)
                uint8_t size = 80 + 20 * sin(animFrame * 0.1);
                drawHeart(120, 80, size, color);

                animFrame++;
            }
            break;

        case 3: // Custom animations based on animID
            if (millis() - lastAnimFrame > animDelay) {
                lastAnimFrame = millis();

                switch (animID) {
                    case 0: // Rainbow heart
                        {
                            uint16_t rainbowColor = tft.color565(
                                (sin(animFrame * 0.1) + 1) * 127,
                                (sin(animFrame * 0.1 + 2) + 1) * 127,
                                (sin(animFrame * 0.1 + 4) + 1) * 127
                            );
                            tft.fillScreen(ST77XX_BLACK);
                            drawHeart(120, 80, 100, rainbowColor);
                        }
                        break;

                    case 1: // Multiple hearts
                        tft.fillScreen(ST77XX_BLACK);
                        for (int i = 0; i < 3; i++) {
                            drawHeart(60 + i * 60, 80, 50, color);
                        }
                        break;

                    default:
                        // Default to static heart
                        tft.fillScreen(ST77XX_BLACK);
                        drawHeart(120, 80, 100, color);
                        break;
                }

                animFrame++;
            }
            break;
    }
}
```

**Create `include/heart_display.h`:**

```cpp
#ifndef HEART_DISPLAY_H
#define HEART_DISPLAY_H

void Heart_Init();
void Heart_Update();

#endif
```

### 6. Implement Thermal Printer Driver

**Create `src/thermal_printer.cpp`:**

```cpp
#include <Arduino.h>
#include "Adafruit_Thermal.h"
#include "thermal_printer.h"
#include "register_map.h"
#include "i2c_slave.h"

// Printer UART (typically Serial2 on ESP32)
#define PRINTER_RX 16
#define PRINTER_TX 17

Adafruit_Thermal printer(&Serial2);

void Printer_Init() {
    Serial2.begin(19200, SERIAL_8N1, PRINTER_RX, PRINTER_TX);

    delay(500); // Wait for printer to initialize
    printer.begin();
    printer.setDefault();

    I2C_Write_Register(REG_STATUS, I2C_Read_Register(REG_STATUS) | STATUS_PRINTER_READY);
    I2C_Write_Register(REG_PRINT_STATUS, 0); // Ready

    Serial.println("Thermal printer initialized");
}

void Printer_Update() {
    uint8_t cmd = I2C_Read_Register(REG_PRINT_CMD);

    if (cmd == 0) {
        // Idle - nothing to do
        return;
    }

    // Set status to busy
    I2C_Write_Register(REG_PRINT_STATUS, 1);

    switch (cmd) {
        case 1: // Print buffer contents
            {
                char* buffer = I2C_Get_Print_Buffer();
                if (strlen(buffer) > 0) {
                    printer.println(buffer);
                    printer.feed(2);
                    I2C_Clear_Print_Buffer();
                    Serial.print("Printed: ");
                    Serial.println(buffer);
                }
            }
            break;

        case 2: // Feed paper
            printer.feed(3);
            Serial.println("Fed paper");
            break;

        case 3: // Reset printer
            printer.begin();
            printer.setDefault();
            Serial.println("Printer reset");
            break;

        default:
            Serial.println("Unknown print command");
            break;
    }

    // Clear command and set status to ready
    I2C_Write_Register(REG_PRINT_CMD, 0);
    I2C_Write_Register(REG_PRINT_STATUS, 0);
}
```

**Create `include/thermal_printer.h`:**

```cpp
#ifndef THERMAL_PRINTER_H
#define THERMAL_PRINTER_H

void Printer_Init();
void Printer_Update();

#endif
```

### 7. Implement Main Loop

**Create `src/main.cpp`:**

```cpp
#include <Arduino.h>
#include "i2c_slave.h"
#include "heart_display.h"
#include "thermal_printer.h"
#include "register_map.h"

unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 33; // 30 Hz update rate

void setup() {
    Serial.begin(115200);
    Serial.println("Torso Module v1.0");

    // Initialize subsystems
    I2C_Slave_Init();
    Heart_Init();
    Printer_Init();

    Serial.println("Initialization complete");
}

void loop() {
    // Update all subsystems at fixed rate (30 Hz)
    if (millis() - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = millis();

        Heart_Update();
        Printer_Update();
    }

    // Small delay to prevent WDT resets
    delay(1);
}
```

### 8. Build and Upload Firmware

```bash
cd ~/olaf/modules/torso/firmware/torso-main

# Build firmware
pio run

# Upload to ESP32 (ensure USB cable connected)
pio run --target upload

# Monitor serial output
pio device monitor

# Expected output:
# Torso Module v1.0
# I2C Slave initialized at address 0x09
# Heart display initialized
# Thermal printer initialized
# Initialization complete
```

### 9. Test I2C Communication from Raspberry Pi

**On Raspberry Pi:**

```bash
# Scan I2C bus
i2cdetect -y 1
# Expected: 0x09 shows up

# Test write: Set heart mode to "beat" (mode=2)
i2cset -y 1 0x09 0x10 0x02

# Expected: Heart display shows beating animation

# Set heart color to red
i2cset -y 1 0x09 0x12 255  # Red
i2cset -y 1 0x09 0x13 0    # Green
i2cset -y 1 0x09 0x14 0    # Blue

# Test printer: Write "Hello OLAF!" to buffer
# (This is complex via i2cset - better to use Python script)
```

### 10. Create Python Test Script

**Create `test_torso.py`:**

```python
#!/usr/bin/env python3
import smbus2
import time

bus = smbus2.SMBus(1)
TORSO_ADDR = 0x09

# Test heart display
print("Setting heart to beat mode (red)...")
bus.write_byte_data(TORSO_ADDR, 0x10, 2)  # Beat mode
bus.write_byte_data(TORSO_ADDR, 0x12, 255)  # Red
bus.write_byte_data(TORSO_ADDR, 0x13, 0)    # Green
bus.write_byte_data(TORSO_ADDR, 0x14, 0)    # Blue
bus.write_byte_data(TORSO_ADDR, 0x15, 128)  # Medium speed

time.sleep(3)

# Test printer
print("Sending text to printer...")
message = "Hello OLAF!\n"
for char in message:
    bus.write_byte_data(TORSO_ADDR, 0x22, ord(char))

# Trigger print
bus.write_byte_data(TORSO_ADDR, 0x20, 1)  # Print command

print("Test complete!")
bus.close()
```

```bash
python3 test_torso.py
```

---

## Testing & Validation

**Test 1: I2C Slave Response**
```bash
i2cget -y 1 0x09 0x00  # Read device ID
# Expected: 0x03

i2cget -y 1 0x09 0x01  # Read firmware version
# Expected: 0x10 (v1.0)
```

**Test 2: Heart Display Animation**
```bash
# Test all modes: off (0), static (1), beat (2), animation (3)
for mode in 0 1 2 3; do
    echo "Testing mode $mode"
    i2cset -y 1 0x09 0x10 $mode
    sleep 3
done
```

**Test 3: Printer Text Output**
```python
# Use Python script to send longer messages
# Verify text prints clearly and paper feeds correctly
```

**Test 4: Concurrent Operation**
```bash
# Heart animating while printer prints
# Verify no lag or crashes
```

---

## Troubleshooting

**Issue 1: Display Shows Nothing**
- **Solution:** Check SPI wiring, verify display power (3.3V or 5V depending on model), try different ST7789 library settings

**Issue 2: Printer Not Printing**
- **Solution:** Check UART baud rate (19200), verify printer power (5-9V, 2A+), check paper loaded, test with Adafruit example

**Issue 3: I2C Address Not Detected**
- **Solution:** Verify I2C_Slave_Init() called, check SDA/SCL pins (GPIO 21, 22), check pull-up resistors

**Issue 4: Characters Garbled on Print**
- **Solution:** Check baud rate, verify UART TX/RX not swapped, check printer DIP switch settings

**Issue 5: ESP32 Crashes with Printer Commands**
- **Solution:** Increase stack size, check for buffer overflow in print buffer, add delay between printer commands

---

## Dependencies

**Before this story:**
- Story 3.4: Assemble and Test Torso PCB ✅
- PlatformIO installed (Story 0.4)
- Heart display and thermal printer hardware tested

**After this story:**
- Story 3.7: Create Torso ROS2 Driver Node

---

## References

- [ESP32 I2C Slave Mode](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html)
- [Adafruit ST7789 Library](https://github.com/adafruit/Adafruit-ST7735-Library)
- [Adafruit Thermal Printer Library](https://github.com/adafruit/Adafruit-Thermal-Printer-Library)
- [PlatformIO Documentation](https://docs.platformio.org/)

---

## Notes

- **Firmware Size:** Expect ~600KB compiled binary (fits in ESP32 4MB flash)
- **I2C Speed:** 400kHz recommended for responsiveness
- **Print Buffer:** Limited to 255 characters per message (can be extended if needed)
- **Display Animations:** Keep animations simple for 30 FPS performance
- **Power Consumption:** Printer draws significant current (1-2A) during printing - ensure adequate power supply
- **Future Enhancements:** Add image printing support, more heart animations, status LEDs

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
