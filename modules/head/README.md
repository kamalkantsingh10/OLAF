# Head Module

**Controller:** ESP32-S3 (I2C address 0x10)

## Overview

The Head module controls OLAF's expressive OLED eyes. It runs on a dedicated ESP32 to achieve 30-60 FPS eye animations with real-time responsiveness.

## Hardware

- 2x GC9A01 round OLED displays (240x240, SPI)
- ESP32-S3-DevKitC-1 (N16R8)

## Interfaces

| Interface | Purpose |
|-----------|---------|
| I2C (slave) | Commands from Pi (expressions, blink, look direction) |
| SPI x2 | OLED eye displays |

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/head/expression` | String | Set eye expression (happy, sad, curious, etc.) |
| `/head/blink` | Bool | Trigger blink animation |
| `/head/look` | Point | Look direction (x, y coordinates) |

## Power

- 5V from 36V→5V buck converter
- ESP32 provides 3.3V to OLEDs

## Directory Structure

```
head/
├── firmware/           # ESP32 firmware (PlatformIO)
│   ├── src/
│   ├── include/
│   ├── test/
│   └── platformio.ini
├── hardware/
│   ├── mechanical/     # 3D models
│   └── bom.csv
├── tests/              # Integration tests
├── diagnostics/        # Eye test patterns
├── scripts/            # Build, flash, calibrate
├── wiring.md           # Pin assignments
├── assembly.md         # Assembly guide
└── README.md
```
