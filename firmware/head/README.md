# Head Module - Eye Expression System

## Overview
The Head Module provides OLAF's expressive eyes using two GC9A01 round TFT displays (240×240 pixels each). It renders emotion-driven eye expressions with 60 FPS animation, controlled via I2C commands from the Raspberry Pi orchestrator.

**I2C Address:** `0x08`

**Features (Story 1.4):**
- 7 expression types (neutral, happy, curious, thinking, confused, sad, excited)
- 5 intensity levels (1=subtle to 5=extreme)
- Synchronized blink animations
- 60 FPS smooth animation on both eyes
- Register-based I2C command protocol

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

### 2. Test Expression Commands (Story 1.4)

```bash
# Read module ID (should return 0x08)
sudo i2cget -y 1 0x08 0x00

# Read status (should return 0x01 = READY)
sudo i2cget -y 1 0x08 0x02

# Set HAPPY expression with medium intensity
sudo i2cset -y 1 0x08 0x10 0x01    # Expression type = 1 (happy)
sudo i2cset -y 1 0x08 0x11 0x03    # Intensity = 3 (moderate)

# Set EXCITED expression with maximum intensity
sudo i2cset -y 1 0x08 0x10 0x06    # Expression type = 6 (excited)
sudo i2cset -y 1 0x08 0x11 0x05    # Intensity = 5 (extreme)

# Trigger blink
sudo i2cset -y 1 0x08 0x12 0x01    # Any value triggers blink

# Test all expressions
for expr in {0..6}; do
  echo "Testing expression $expr"
  sudo i2cset -y 1 0x08 0x10 $expr
  sleep 2
done
```

### 3. Check Serial Monitor

ESP32 should show:
```
╔════════════════════════════════════════════╗
║      OLAF Head Module - Eye Expressions    ║
║      Story 1.4: ESP32 Firmware             ║
╚════════════════════════════════════════════╝

[I2C] Slave initialized at 0x08 - Head Module
[Display] ✓ Dual displays initialized
[Expression] Engine initialized - NEUTRAL expression

========== Performance Report ==========
Frames rendered: 300
Avg frame time: 14.2 ms (70.4 FPS)
Min frame time: 12 ms
Max frame time: 18 ms
✓ Frame rate target achieved
========================================
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

**Story 1.4 Implementation - Register Map:**

| Register | Address | Type | Description |
|----------|---------|------|-------------|
| Module ID | 0x00 | Read | Returns 0x08 (head module identifier) |
| Firmware Version | 0x01 | Read | Firmware version (0x01 = v0.1) |
| Status | 0x02 | Read | Status flags (READY=0x01, BUSY=0x02, ERROR=0x04) |
| Error Code | 0x03 | Read | Last error code (0x00 = no error) |
| Command | 0x04 | Write | Generic command register |
| Expression Type | 0x10 | Write | Emotion type (0-6) |
| Expression Intensity | 0x11 | Write | Intensity level (1-5) |
| Blink Trigger | 0x12 | Write | Trigger blink (write any value) |

**Expression Types:**
- 0 = NEUTRAL: Calm, centered pupils
- 1 = HAPPY: Wide pupils, upward gaze
- 2 = CURIOUS: Large pupils, looking at something
- 3 = THINKING: Pupils up-right (memory access)
- 4 = CONFUSED: Wandering, asymmetric pupils
- 5 = SAD: Small pupils, downward gaze
- 6 = EXCITED: Very wide pupils, rapid movements

**Intensity Levels:**
- 1 = Subtle (minimal changes)
- 2 = Light (noticeable but restrained)
- 3 = Moderate (clear expression, balanced)
- 4 = Strong (exaggerated, theatrical)
- 5 = Extreme (maximum effect, cartoon-like)

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

## Firmware Architecture (Story 1.4)

**File Structure:**
```
modules/head/
├── firmware/
│   ├── config.h                 # Hardware pin configuration
│   ├── i2c_slave.h/.cpp        # I2C slave with register map
│   ├── gc9a01_driver_spi.h/.cpp # Dual display driver
│   └── eye_expression.h/.cpp    # Expression animation engine
├── src/
│   └── main.cpp                 # Main controller loop
├── platformio.ini               # Build configuration
└── README.md                    # This file
```

**Component Responsibilities:**
- **i2c_slave**: Handles I2C communication, register map, command parsing
- **gc9a01_driver_spi**: Manages dual GC9A01 displays with CS switching
- **eye_expression**: Renders expressions with smooth animation and blinking
- **main.cpp**: Integrates all components, main control loop

## References

- [Story 1.3: Head Module Hardware Assembly](../../docs/stories/1.3.head-module-hardware-assembly.md)
- [Story 1.4: Head Module ESP32 Firmware](../../docs/stories/1.4.head-module-esp32-firmware.md)
- [Architecture: Tech Stack](../../docs/architecture/tech-stack.md)
- [Architecture: Components](../../docs/architecture/components.md)
- [Architecture: Data Models (Register Maps)](../../docs/architecture/data-models.md)
- [TFT_eSPI Library](https://github.com/Bodmer/TFT_eSPI)

## Change Log

| Date       | Version | Changes                                      | Author |
|------------|---------|----------------------------------------------|--------|
| 2025-10-27 | v1.0    | Initial README with pin mapping and testing  | James  |
| 2025-10-28 | v2.0    | Updated for Story 1.4 expression engine      | James  |
