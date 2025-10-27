# Head Module - Dual Eye Displays

## Overview
The Head Module provides OLAF's expressive eyes using two GC9A01 round TFT displays (240×240 pixels each). It communicates with the Raspberry Pi orchestrator via I2C and renders eye animations via SPI.

**I2C Address:** `0x08`

## Hardware

### Components
- **MCU:** ESP32-S3-DevKitC-1 (ESP32-S3-WROOM-2, N16R8)
  - 16MB Flash, 8MB PSRAM
  - Dual-core Xtensa LX7 @ 240MHz
- **Displays:** 2× GC9A01 Round TFT (1.28", 240×240, SPI)
- **Communication:**
  - I2C slave to Raspberry Pi (400kHz)
  - SPI to displays (20MHz)

### Pin Mapping

#### SPI Display Connections

**Shared Signals (Both Displays):**
| GC9A01 Pin | Function       | ESP32-S3 GPIO | Notes                    |
|------------|----------------|---------------|--------------------------|
| VCC        | Power          | 3.3V          | Shared power rail        |
| GND        | Ground         | GND           | Common ground            |
| SCL (CLK)  | SPI Clock      | GPIO12        | 20MHz SPI clock          |
| SDA (MOSI) | SPI Data Out   | GPIO11        | Master out, slave in     |
| RES (RST)  | Reset          | GPIO4         | Shared reset             |
| DC (A0)    | Data/Command   | GPIO2         | Shared D/C select        |
| LED        | Backlight PWM  | GPIO10        | Optional brightness ctrl |

**Individual Chip Select:**
| Display   | CS Pin | ESP32-S3 GPIO | Notes                     |
|-----------|--------|---------------|---------------------------|
| Left Eye  | CS     | GPIO5         | Manual CS control in code |
| Right Eye | CS     | GPIO15        | Manual CS control in code |

#### I2C Connection to Raspberry Pi

| Pi Pin   | Function  | ESP32-S3 GPIO | Notes                        |
|----------|-----------|---------------|------------------------------|
| GPIO2    | I2C SDA   | GPIO8         | Data line with pull-ups      |
| GPIO3    | I2C SCL   | GPIO9         | Clock line with pull-ups     |
| GND      | Ground    | GND           | **CRITICAL: Common ground!** |
| 5V (opt) | Power     | VIN           | Or use separate USB power    |

## Software Configuration

### PlatformIO Configuration

**File:** `platformio.ini`

The TFT_eSPI library configuration is defined via build flags (survives `pio clean`):

```ini
build_flags =
    -DUSER_SETUP_LOADED=1
    -DGC9A01_DRIVER=1
    -DUSE_HSPI_PORT=1        # Critical for ESP32-S3
    -DTFT_WIDTH=240
    -DTFT_HEIGHT=240
    -DTFT_MOSI=11
    -DTFT_SCLK=12
    -DTFT_DC=2
    -DTFT_RST=4
    -DTFT_BL=10
    -DSPI_FREQUENCY=20000000
```

**Note:** `TFT_CS` is **not defined** to allow manual chip select control for dual displays.

### Firmware Configuration

**File:** `firmware/config.h`

Key constants:
```cpp
#define I2C_SLAVE_ADDRESS 0x08
constexpr uint8_t kI2cSdaPin = 8;
constexpr uint8_t kI2cSclPin = 9;
constexpr uint32_t kI2cFrequency = 400000;  // 400kHz

constexpr uint8_t CS_LEFT_EYE = 5;
constexpr uint8_t CS_RIGHT_EYE = 15;
```

## Building and Uploading

### Prerequisites
- PlatformIO Core or PlatformIO IDE
- USB connection to ESP32-S3

### Build
```bash
cd modules/head
pio run
```

### Upload
```bash
pio run -t upload
```

### Monitor Serial Output
```bash
pio device monitor
```

Expected output:
```
========================================
  OLAF Head Module
  Dual Eyes + I2C Slave
========================================

Initializing displays...
✓ Displays initialized

Initializing I2C slave...
  Address: 0x08
  SDA Pin: GPIO8
  SCL Pin: GPIO9
✓ I2C slave initialized

Setup complete!
Waiting for I2C commands from Pi...
```

## Testing

### 1. Verify I2C Connection

On Raspberry Pi:
```bash
sudo i2cdetect -y 1
```

Expected output (device at address 0x08):
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         08 -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
```

### 2. Send Text to Displays

```bash
# Send "HELLO" as hex bytes
sudo i2ctransfer -y 1 w5@0x08 0x48 0x45 0x4c 0x4c 0x4f

# Send "OLAF"
sudo i2ctransfer -y 1 w4@0x08 0x4f 0x4c 0x41 0x46
```

Text should appear on both eye displays.

### 3. Check Serial Monitor

ESP32 should show:
```
[I2C RX] Received 5 bytes: HELLO
[Display] Showing text: HELLO
```

## Troubleshooting

### Displays Not Working

**Symptom:** Blank screens or garbage pixels

**Solutions:**
- Verify 3.3V power connection
- Check all SPI wiring (especially GPIO11, GPIO12)
- Ensure `USE_HSPI_PORT` is defined in platformio.ini
- Reduce SPI frequency if using long jumper wires

### I2C Not Detected

**Symptom:** `i2cdetect` shows no device at 0x08

**Solutions:**
- **CRITICAL:** Verify common ground between Pi and ESP32
- Check SDA/SCL not swapped (Pi GPIO2→ESP32 GPIO8, Pi GPIO3→ESP32 GPIO9)
- Verify ESP32 firmware is running (check serial monitor)
- Check I2C pull-up resistors (Pi has internal 1.8kΩ pull-ups)

### ESP32-S3 Crashes on Boot

**Symptom:** Guru Meditation Error (StoreProhibited)

**Solutions:**
- Ensure `USE_HSPI_PORT=1` is in platformio.ini build flags
- This is a critical fix for ESP32-S3 with espressif32 platform >= 6.7.0
- Verify TFT_eSPI configuration matches pin assignments

### Only One Display Works

**Symptom:** Left or right eye blank

**Solutions:**
- Check individual CS wiring (GPIO5 for left, GPIO15 for right)
- Verify CS pins are set as OUTPUT in code
- Check display power connections (each needs VCC and GND)

## Technical Notes

### Dual Display Rendering Strategy

Both displays share SPI bus signals (SCK, MOSI, DC, RST), but have independent CS pins:

```cpp
// Select LEFT eye
digitalWrite(CS_LEFT_EYE, LOW);
digitalWrite(CS_RIGHT_EYE, HIGH);
tft.fillScreen(TFT_BLACK);
tft.drawString("LEFT", 120, 120);
digitalWrite(CS_LEFT_EYE, HIGH);

// Select RIGHT eye
digitalWrite(CS_RIGHT_EYE, LOW);
tft.fillScreen(TFT_BLACK);
tft.drawString("RIGHT", 120, 120);
digitalWrite(CS_RIGHT_EYE, HIGH);
```

### I2C Communication Protocol

**Current Implementation (Story 1.3):**
- Simple text display via I2C
- Receives ASCII bytes and displays on both eyes

**Future (Story 1.4):**
- Register-based command protocol
- Expression commands (emotion type + intensity)
- Animation control
- Status reporting

### Performance

**SPI Bandwidth:**
- 20MHz SPI clock
- 240×240 pixels × 2 displays = 115,200 pixels/frame
- RGB565 format = 2 bytes/pixel = 230,400 bytes/frame
- Theoretical max: ~87 FPS
- Target: 60 FPS (achievable with current config)

**I2C Latency:**
- 400kHz I2C bus
- Typical command: <10ms round-trip
- Meets <100ms requirement for expression updates

## Hardware Photos

Assembly photos available in: `hardware/media/head-module/`

## Bill of Materials

See: `hardware/bom/epic1_bom.csv`

**Head Module Components:**
- ESP32-S3-DevKitC-1 (N16R8): 4.04 CHF
- GC9A01 Display (×2): 5.98 CHF
- Breadboard Kit: 3.19 CHF
- USB Cables (×2): 5.00 CHF

## References

- [Story 1.3: Head Module Hardware Assembly](../../docs/stories/1.3.head-module-hardware-assembly.md)
- [Architecture: Tech Stack](../../docs/architecture/tech-stack.md)
- [Architecture: Components](../../docs/architecture/components.md)
- [TFT_eSPI Library](https://github.com/Bodmer/TFT_eSPI)

## Change Log

| Date       | Version | Changes                                      | Author |
|------------|---------|----------------------------------------------|--------|
| 2025-10-27 | v1.0    | Initial README with pin mapping and testing  | James  |
