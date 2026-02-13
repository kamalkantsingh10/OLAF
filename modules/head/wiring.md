# Head Module Wiring

## Overview

ESP32-S3-DevKitC-1 (N16R8) controlling:
- 2x GC9A01 round OLED eye displays (SPI)
- 1x WS2812 8-LED strip (face status indicator)
- I2C slave connection to Raspberry Pi

## Wiring Diagram

```
                                    ESP32-S3-DevKitC-1 (N16R8)
                              ┌─────────────────────────────────────┐
                              │              USB-C                  │
                              │               ┌─┐                   │
                              │               └─┘                   │
        ┌─────────────────────┤ LEFT (J1)              RIGHT (J3)  ├─────────────────────┐
        │                     │                                     │                     │
        │    3V3 ─── [1]  [2] ─── 3V3                    GND ─── [1]  [2] ─── TX         │
        │    RST ─── [3]  [4] ─── GPIO4 ◄── WS2812      GPIO43     [3]  [4] ─── GPIO1 ──►│── Right Eye CS
        │         ─── [5]  [6] ─── GPIO5                GPIO44 RX  [5]  [6] ─── GPIO2 ──►│── Left Eye CS
        │         ─── [7]  [8] ─── GPIO6                GPIO42 ─── [7]  [8] ─── GPIO41 ─►│── RST (shared)
        │         ─── [9] [10] ─── GPIO7                GPIO41 ─── [9] [10] ─── GPIO40 ─►│── DC (shared)
        │        ─── [11] [12] ─── GPIO8 ◄── I2C SDA    GPIO40 ── [11] [12] ─── GPIO39 ─►│── MOSI (shared)
        │        ─── [13] [14] ─── GPIO3                GPIO39 ── [13] [14] ─── GPIO38 ─►│── SCLK (shared)
        │        ─── [15] [16] ─── GPIO9 ◄── I2C SCL    GPIO38 ── [15] [16] ─── GPIO37   │
        │        ─── [17] [18] ─── GPIO10               GPIO37 ── [17] [18] ─── GPIO36   │
        │        ─── [19] [20] ─── GPIO11               GPIO36 ── [19] [20] ─── GPIO0    │
        │        ─── [21] [22] ─── GPIO12               GPIO35 ── [21] [22] ─── GPIO45   │
        │         5V ─── [23] [24] ─── GND              GPIO48 ── [23] [24] ─── GPIO47   │
        │                     │                         GPIO21 ── [25] [26] ─── GND      │
        └─────────────────────┤                                     ├─────────────────────┘
                              │               ┌─┐                   │
                              │              BOOT                   │
                              └─────────────────────────────────────┘
```

## Pin Assignments

### I2C (to Raspberry Pi) - LEFT SIDE

| Function | GPIO | ESP32 Pin | Pi GPIO | Notes |
|----------|------|-----------|---------|-------|
| I2C SDA  | 8    | J1-12     | GPIO 2  | 4.7kΩ pull-up to 3.3V |
| I2C SCL  | 9    | J1-15     | GPIO 3  | 4.7kΩ pull-up to 3.3V |

**I2C Address:** `0x08`

### WS2812 LED Strip - LEFT SIDE

| Function | GPIO | ESP32 Pin | Notes |
|----------|------|-----------|-------|
| DATA     | 4    | J1-4      | Single wire, 3.3V logic OK for WS2812 |

### SPI Displays (Both Eyes) - RIGHT SIDE

| Function | GPIO | ESP32 Pin | Notes |
|----------|------|-----------|-------|
| SCLK     | 38   | J3-10     | Shared clock |
| MOSI     | 39   | J3-9      | Shared data |
| DC       | 40   | J3-8      | Shared data/command |
| RST      | 41   | J3-7      | Shared reset |
| CS Left  | 2    | J3-5      | Left eye chip select |
| CS Right | 1    | J3-4      | Right eye chip select |

## Component Wiring Details

### WS2812 8-LED Strip

```
    WS2812 Strip (8 LEDs)
    ┌─────────────────────────────────────────────────────────┐
    │  [0] [1] [2] [3] │ [4] [5] [6] [7]                      │
    │   ◄── LEFT ────    ──── RIGHT ──►                       │
    │       edges → center ← edges                            │
    ├─────────────────────────────────────────────────────────┤
    │  VCC ─────────────────────────► 5V (from buck converter)│
    │  GND ─────────────────────────► GND (common ground)     │
    │  DIN ─────────────────────────► GPIO4 (ESP32 J1-4)      │
    └─────────────────────────────────────────────────────────┘

    Note: WS2812 runs on 5V but accepts 3.3V logic from ESP32.
    For long runs (>30cm), add 300-500Ω resistor on data line.
```

### GC9A01 OLED Displays (x2)

```
    Left Eye (GC9A01)              Right Eye (GC9A01)
    ┌───────────────┐              ┌───────────────┐
    │ VCC ──► 3.3V  │              │ VCC ──► 3.3V  │
    │ GND ──► GND   │              │ GND ──► GND   │
    │ SCL ──► GPIO38│ (shared)     │ SCL ──► GPIO38│ (shared)
    │ SDA ──► GPIO39│ (shared)     │ SDA ──► GPIO39│ (shared)
    │ DC  ──► GPIO40│ (shared)     │ DC  ──► GPIO40│ (shared)
    │ RST ──► GPIO41│ (shared)     │ RST ──► GPIO41│ (shared)
    │ CS  ──► GPIO2 │ (unique)     │ CS  ──► GPIO1 │ (unique)
    │ BL  ──► 3.3V  │ (always on)  │ BL  ──► 3.3V  │ (always on)
    └───────────────┘              └───────────────┘
```

### I2C Connection to Raspberry Pi

```
    Raspberry Pi 5                    ESP32-S3
    ┌────────────────┐                ┌────────────────┐
    │                │                │                │
    │ GPIO 2 (SDA) ──┼───────────────►│ GPIO 8 (SDA)   │
    │                │                │                │
    │ GPIO 3 (SCL) ──┼───────────────►│ GPIO 9 (SCL)   │
    │                │                │                │
    │ GND ───────────┼───────────────►│ GND            │
    │                │                │                │
    └────────────────┘                └────────────────┘

    Note: Pi 5 has built-in 1.8kΩ pull-ups on SDA/SCL.
    No external pull-up resistors needed for runs <30cm.
```

## Power Distribution

```
    36V Battery
        │
        ▼
    ┌───────────────┐
    │ 36V→5V Buck   │
    └───────┬───────┘
            │ 5V
            ├──────────────────► ESP32 5V pin (J1-21)
            │
            └──────────────────► WS2812 VCC

    ESP32 Internal 3.3V Regulator
            │
            ├──────────────────► GC9A01 Left Eye VCC
            │
            └──────────────────► GC9A01 Right Eye VCC
```

## I2C Register Map

**Slave Address:** `0x08`

### Status Registers (Read-only)

| Register | Address | Description |
|----------|---------|-------------|
| STATUS | 0x00 | Module status (0x00=OK, 0x01=READY, 0x02=BUSY, 0x04=ERROR) |
| COMMAND | 0x01 | Reserved for future commands |
| MODULE_ID | 0x02 | Returns slave address (0x08) |
| FIRMWARE_VER | 0x03 | Firmware version byte |
| ERROR_CODE | 0x04 | Last error code |

### Expression Registers (Write)

| Register | Address | Values | Description |
|----------|---------|--------|-------------|
| EXPRESSION_TYPE | 0x10 | 0-6 | 0=neutral, 1=happy, 2=sad, 3=surprised, 4=angry, 5=sleepy, 6=wink |
| EXPRESSION_INTENSITY | 0x11 | 1-5 | 1=subtle, 5=extreme |
| BLINK_TRIGGER | 0x12 | any | Write any value to trigger a blink |

### Look Direction Registers (Write)

| Register | Address | Values | Description |
|----------|---------|--------|-------------|
| LOOK_X | 0x20 | -100 to +100 | Signed byte: negative=left, positive=right |
| LOOK_Y | 0x21 | -100 to +100 | Signed byte: negative=down, positive=up |

### System Status Register (Write)

| Register | Address | Values | Description |
|----------|---------|--------|-------------|
| SYSTEM_STATUS | 0x30 | 0-5 | Controls eye wake state and LED strip animation |

### Quick Test Commands (from Pi)

```bash
# Detect device
i2cdetect -y 1          # Should show 0x08

# Wake eyes
i2cset -y 1 0x08 0x30 0x01   # WOKE_UP

# Set to speaking mode (required before expressions show)
i2cset -y 1 0x08 0x30 0x04   # SPEAKING

# Set expression
i2cset -y 1 0x08 0x10 0x01   # HAPPY

# Trigger blink
i2cset -y 1 0x08 0x12 0x01

# Look right
i2cset -y 1 0x08 0x20 0x64   # X = +100

# Look left (signed: -100 = 0x9C)
i2cset -y 1 0x08 0x20 0x9c

# Put to sleep
i2cset -y 1 0x08 0x30 0x05   # GOING_IDLE
```

## LED Strip States

| State | I2C Value | Animation |
|-------|-----------|-----------|
| IDLE | 0x00 | Off |
| WOKE_UP | 0x01 | Edges sweep inward → flash |
| LISTENING | 0x02 | Blue pulse from center |
| PROCESSING | 0x03 | Chase toward center |
| SPEAKING | 0x04 | Rainbow colors popping |
| GOING_IDLE | 0x05 | Fade edges inward → off |

## Notes

1. **I2C Pull-ups:** Pi 5 has built-in 1.8kΩ pull-ups on GPIO 2/3 — sufficient for short runs (<30cm). External 4.7kΩ not required.
2. **WS2812 Data:** GPIO4 chosen for clean signal, no conflicts
3. **Display Backlight:** Connected directly to 3.3V (always on, no PWM control)
4. **Ground:** All components share common ground with ESP32
5. **Reserved GPIOs:** Do not use GPIO 26-32 (flash) or GPIO 33-37 (PSRAM)
