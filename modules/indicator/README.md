# Indicator Module

**Controller:** Raspberry Pi (via Fusion HAT+ WS2812 port)

## Overview

The Indicator module provides visual feedback through 24 addressable RGB LEDs arranged in 3 strips of 8 LEDs each. Controlled directly by the Pi through the Fusion HAT's WS2812 port.

## Hardware

- 3x WS2812 8-LED strips (24 LEDs total, daisy-chained)
- Connected to Fusion HAT+ WS2812 port (ZH1.5 connector)

## LED Strip Assignments

| Strip | LEDs | Purpose |
|-------|------|---------|
| Strip 1 | 0-7 | **Interaction** — listening (blue pulse), thinking (yellow spin), speaking (green wave) |
| Strip 2 | 8-15 | **Module Status** — health indicators for Head, Base, Neck, Ears, AI |
| Strip 3 | 16-23 | **PID Visualization** — self-balancing state (tilt direction, correction intensity) |

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/indicator/interaction` | String | Set interaction state (listening, thinking, speaking, idle) |
| `/indicator/status` | UInt8[5] | Module health status array [head, base, neck, ears, ai] |
| `/indicator/pid` | Float32[3] | PID visualization [pitch, correction, intensity] |
| `/indicator/raw` | UInt8[72] | Direct LED control (24 LEDs × RGB) |

## Power

- 5V from 2000mAh battery via Fusion HAT

## Directory Structure

```
indicator/
├── hardware/
│   ├── mechanical/     # 3D models for LED strip mounts
│   └── bom.csv
├── tests/              # Integration tests
├── diagnostics/        # LED test patterns
└── README.md
```

## Notes

- No firmware required - controlled by ROS2 node `olaf_indicator`
- Single data line connection via Fusion HAT
- All 3 strips daisy-chained (data out → data in)
