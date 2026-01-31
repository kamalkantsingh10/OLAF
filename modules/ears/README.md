# Ears Module

**Controller:** Raspberry Pi (via USB serial to Waveshare Bus Servo Adapter)

## Overview

The Ears module controls OLAF's articulated ears for emotional expression. Two 2-DOF ears (4 servos total) are controlled directly by the Pi via a Waveshare Bus Servo Adapter connected over USB.

## Hardware

- 4x Feetech SCS0009 serial bus servos
- 1x Waveshare Bus Servo Adapter (A) in USB mode
- 2x ear mechanisms (2-DOF each: rotate + tilt)

## Interfaces

| Interface | Purpose |
|-----------|---------|
| USB Serial | Pi to Waveshare adapter |
| Serial bus | Adapter to SCS0009 servos (daisy-chained) |

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ears/emotion` | String | Set ear pose for emotion (alert, relaxed, sad, etc.) |
| `/ears/perk` | Bool | Quick perk-up animation |
| `/ears/left_pose` | Float32[2] | Left ear angles [rotate, tilt] |
| `/ears/right_pose` | Float32[2] | Right ear angles [rotate, tilt] |

## Power

- 12V from 36V→12V buck converter (to Waveshare adapter)
- Waveshare adapter provides servo power

## Directory Structure

```
ears/
├── hardware/
│   ├── mechanical/     # 3D models for ear mechanisms
│   └── bom.csv
├── tests/              # Integration tests
├── diagnostics/        # Servo test scripts
├── scripts/            # Calibration scripts
├── assembly.md         # Assembly guide
└── README.md
```

## Notes

- No firmware required - controlled by ROS2 node `olaf_ears`
- Servos are daisy-chained on serial bus
- USB connection provides reliable communication without custom wiring
