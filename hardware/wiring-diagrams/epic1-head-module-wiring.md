# Epic 1 - Head Module Wiring Diagram

## Overview

This document describes the wiring connections for the Epic 1 Head Module, which consists of:
- 1× ESP32-S3-DevKitC-1 (ESP32-S3-WROOM-2, N16R8)
- 2× GC9A01 Round TFT Displays (1.28", 240×240 pixels)
- 1× Raspberry Pi 5 (8GB RAM)

## Connection Summary

```
Raspberry Pi 5
     |
     | I2C Bus (GPIO2/GPIO3)
     |
     v
ESP32-S3 (Head Module)
     |
     | SPI Bus (Shared)
     |
     +------+-------+
     |              |
     v              v
Left Eye         Right Eye
(GC9A01)         (GC9A01)
```

## Detailed Wiring

### 1. I2C Connection: Raspberry Pi → ESP32-S3

**Purpose:** Command communication from orchestrator to head module

| Raspberry Pi 5 Pin | Signal | ESP32-S3 GPIO | Notes |
|-------------------|--------|---------------|-------|
| GPIO 2 (Pin 3)    | SDA    | GPIO 8        | I2C Data line |
| GPIO 3 (Pin 5)    | SCL    | GPIO 9        | I2C Clock line (400kHz) |
| GND (Pin 6/9/14)  | GND    | GND           | **CRITICAL: Common ground** |
| 5V (Pin 2/4) *optional* | 5V | VIN         | Power ESP32 from Pi (or use separate USB) |

**I2C Address:** ESP32 Head Module = `0x08`

**Pull-up Resistors:** Raspberry Pi has internal 1.8kΩ pull-ups on GPIO2/GPIO3 (usually sufficient)

**Wiring Notes:**
- Keep I2C wires short (<30cm) for reliable 400kHz operation
- Use twisted pair or keep SDA/SCL close together to reduce interference
- **Common ground is mandatory** - I2C will not work without it

---

### 2. SPI Connection: ESP32-S3 → GC9A01 Displays (Shared Signals)

**Purpose:** High-speed data transfer for 60 FPS eye animations

#### Shared SPI Bus (Both Displays)

| GC9A01 Pin | Signal | ESP32-S3 GPIO | Notes |
|-----------|--------|---------------|-------|
| VCC       | Power  | 3.3V          | Shared 3.3V power rail |
| GND       | Ground | GND           | Common ground |
| SCL (CLK) | SPI Clock | GPIO 12    | 20MHz SPI clock (shared) |
| SDA (MOSI)| SPI Data  | GPIO 11    | Master Out Slave In (shared) |
| RES (RST) | Reset     | GPIO 4     | Shared reset line |
| DC (A0)   | Data/Cmd  | GPIO 2     | Shared Data/Command select |
| LED       | Backlight | GPIO 10    | Optional PWM brightness control (shared) |

**SPI Configuration:** HSPI port, 20MHz clock frequency

#### Individual Chip Select (CS) Pins

| Display   | CS Signal | ESP32-S3 GPIO | Notes |
|-----------|-----------|---------------|-------|
| Left Eye  | CS        | GPIO 5        | Manual CS control in firmware |
| Right Eye | CS        | GPIO 15       | Manual CS control in firmware |

**CS Control Logic:**
- Active LOW: Pull CS low to select display
- Inactive HIGH: Pull CS high to deselect display
- Only one display should be selected at a time for rendering

---

## Physical Layout

### Breadboard Layout (Epic 1 Test Setup)

```
+----------------------------------+
|         Raspberry Pi 5           |
|  [GPIO 2] --SDA--> [I2C Bus]     |
|  [GPIO 3] --SCL--> [I2C Bus]     |
|  [GND]    ---------> [Common]    |
+----------------------------------+
         |
         | (I2C: SDA, SCL, GND)
         |
         v
+----------------------------------+
|      ESP32-S3-DevKitC-1          |
|  [GPIO 8]  <--I2C SDA            |
|  [GPIO 9]  <--I2C SCL            |
|                                   |
|  [GPIO 11] --SPI MOSI-->         |
|  [GPIO 12] --SPI CLK-->          |
|  [GPIO 2]  --DC-->               |
|  [GPIO 4]  --RST-->              |
|  [GPIO 10] --LED (Backlight)     |
|                                   |
|  [GPIO 5]  --CS--> Left Eye      |
|  [GPIO 15] --CS--> Right Eye     |
+----------------------------------+
         |              |
         | (SPI Bus)    | (SPI Bus)
         v              v
   +----------+    +----------+
   | GC9A01   |    | GC9A01   |
   | Left Eye |    | Right Eye|
   +----------+    +----------+
```

---

## Power Supply

### Option 1: USB Power (Development)

- **ESP32-S3:** USB-C cable to PC/power bank (5V, 500mA)
- **Raspberry Pi 5:** USB-C power adapter (5V, 5A recommended)
- **Displays:** Powered from ESP32 3.3V rail (each draws ~100mA)

**Total Current Draw:**
- ESP32-S3: ~200mA (idle), ~400mA (active SPI)
- 2× GC9A01: ~200mA combined (backlight on)
- **Total:** ~600mA (within ESP32 3.3V regulator capacity)

### Option 2: Pi-Powered ESP32 (Production)

- Connect Pi 5V pin → ESP32 VIN (5V input)
- ESP32 on-board regulator provides 3.3V for displays
- **Ensure Pi power supply can handle extra 600mA**

---

## Wire Management

### Cable Lengths (Epic 1 Breadboard Setup)

| Connection Type | Recommended Length | Max Length | Notes |
|----------------|-------------------|------------|-------|
| I2C (Pi → ESP32) | 10-15cm | 30cm | Short wires reduce noise |
| SPI (ESP32 → Display) | 10cm | 20cm | Critical: Long SPI wires cause glitches |
| Power (GND/VCC) | As needed | - | Use thicker gauge for power |

### Wire Gauge Recommendations

- **Signal wires (I2C, SPI):** 24-26 AWG (standard jumper wires)
- **Power/Ground:** 22-24 AWG (thicker for lower resistance)

### Color Coding (Recommended)

- **Red:** VCC (3.3V or 5V)
- **Black:** GND (ground)
- **Yellow:** I2C SDA
- **Orange:** I2C SCL
- **Blue:** SPI MOSI
- **Green:** SPI CLK
- **White:** CS pins
- **Purple:** DC, RST, LED

---

## Testing Checklist

### Pre-Power Checks

- [ ] **Common ground connected** between Pi and ESP32
- [ ] **No shorts** between VCC and GND (use multimeter continuity test)
- [ ] **I2C pull-ups present** (Pi internal pull-ups sufficient)
- [ ] **SPI wires correct** (MOSI, CLK, DC, RST, CS)
- [ ] **CS pins independent** (left eye GPIO 5, right eye GPIO 15)

### Post-Power Checks

- [ ] **ESP32 boots** (LED on, serial output visible)
- [ ] **Displays powered** (backlights on, even if blank)
- [ ] **I2C detected:** Run `sudo i2cdetect -y 1` on Pi (should show `08`)
- [ ] **Both eyes render:** Test firmware shows content on both displays

### Common Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| ESP32 not detected on I2C | Missing common ground | Verify GND connection |
| Displays show garbage | SPI wiring incorrect or too long | Check wiring, shorten cables |
| Only one eye works | CS pin not connected | Verify CS wiring (GPIO 5, GPIO 15) |
| Eyes flicker | Power supply insufficient | Use shorter SPI cables or better PSU |

---

## Firmware Configuration Reference

The pin assignments are defined in the firmware and must match the physical wiring:

**File:** `firmware/head/src/config.h`

```cpp
// I2C Configuration
#define I2C_SLAVE_ADDRESS 0x08
constexpr uint8_t kI2cSdaPin = 8;
constexpr uint8_t kI2cSclPin = 9;

// SPI Display Pins
constexpr uint8_t CS_LEFT_EYE = 5;
constexpr uint8_t CS_RIGHT_EYE = 15;
// Other SPI pins defined in platformio.ini build flags
```

**File:** `firmware/head/platformio.ini`

```ini
build_flags =
    -DTFT_MOSI=11
    -DTFT_SCLK=12
    -DTFT_DC=2
    -DTFT_RST=4
    -DTFT_BL=10
```

**⚠️ IMPORTANT:** If you change physical wiring, update these configuration values and reflash firmware.

---

## References

- [Story 1.3: Head Module Hardware Assembly](../../docs/stories/1.3.head-module-hardware-assembly.md)
- [Story 1.4: Head Module ESP32 Firmware](../../docs/stories/1.4.head-module-esp32-firmware.md)
- [Firmware: Head Module README](../../firmware/head/README.md)
- [ESP32-S3 Pinout](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html)
- [GC9A01 Datasheet](https://www.buydisplay.com/download/ic/GC9A01A.pdf)
- [Raspberry Pi GPIO Pinout](https://pinout.xyz/)

---

## Change Log

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2025-11-02 | v1.0 | Initial wiring documentation for Epic 1 | Gilfoyle (Dev Agent) |
