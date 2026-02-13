# Head Module

**Controller:** ESP32-S3 (I2C address 0x08)

## Overview

The Head module controls OLAF's expressive OLED eyes. It runs on a dedicated ESP32 to achieve 30-60 FPS eye animations with real-time responsiveness.

## Hardware

- ESP32-S3-DevKitC-1 (N16R8)
- 2x GC9A01 round OLED displays (240x240, SPI)
- 1x WS2812 8-LED strip (face status indicator)

## Interfaces

| Interface | Purpose |
|-----------|---------|
| I2C (slave) | Commands from Pi (expressions, blink, look direction, LED state) |
| SPI x2 | OLED eye displays |
| GPIO4 | WS2812 LED strip data |

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/head/expression` | String | Set eye expression (happy, sad, curious, etc.) |
| `/head/blink` | Bool | Trigger blink animation |
| `/head/look` | Point | Look direction (x, y coordinates) |

## OTA Firmware Updates

The head module supports over-the-air firmware updates via ArduinoOTA (WiFi).

**Prerequisites:**
- ESP32 connected to same WiFi network as development machine
- mDNS hostname: `olaf-head.local`
- OTA password: configured in `firmware/src/main.cpp`

**Flash via PlatformIO:**
```bash
cd modules/head/firmware
pio run -t upload
```

PlatformIO is configured with `upload_protocol = espota` and `upload_port = olaf-head.local`, so `pio run -t upload` automatically uses OTA.

**Partition scheme:** `default_16MB.csv` — dual 6.5MB app partitions with automatic rollback support. If a new firmware fails to boot, the ESP32 reverts to the previous working partition.

**First-time flash:** OTA requires working firmware already on the ESP32. For initial flash or recovery, use USB:
```bash
pio run -t upload --upload-port /dev/ttyUSB0
```

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
