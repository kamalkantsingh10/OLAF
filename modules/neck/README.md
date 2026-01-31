# Neck Module

I2C Address: **0x09**

## Overview

The Neck module serves as OLAF's voice interaction status indicator and houses the neck articulation servos. Key features:

- **Voice Indicator**: 8× WS2812B LED strip displays real-time voice interaction states
- **Neck Servos**: 3× Feetech STS3215 servos (pan, tilt, roll)
- **Kickstand**: 1× Feetech STS3215 servo for stability deployment
- **mmWave Sensors**: 2× HLK-LD2461 24GHz radar for 360° presence detection
- **ESP32-S3**: 240MHz dual-core MCU with 16MB flash, 8MB PSRAM
- **OTA Updates**: Over-the-air firmware updates via WiFi

## Hardware

See `hardware/` directory for PCB designs and 3D models.

## Firmware

See `firmware/` directory for ESP32 code.

### Voice Indicator Animations

The Neck module features a `VoiceIndicator` class that controls the 8× WS2812B LED strip with 7 distinct animation states for voice interaction feedback. All animations are **perfectly symmetrical** across the center for visual balance.

#### Animation States

| State | Trigger | Visual Effect | Technical Details |
|-------|---------|---------------|-------------------|
| **WAKE_UP** | Wake word detected | Fast power-up burst from center outward with color shift (Blue → Cyan → White), ending in bright flash | 70ms steps, expands symmetrically from center |
| **LISTENING** | Actively listening | Fast blue pulse waves radiating from center with subtle sparkles for organic feel | 30ms updates, 1s wave cycle, dual-wave overlaps |
| **RESPONDING** | Speaking answer | Dynamic symmetrical VU meter with fast-rotating rainbow gradient and frequent brightness peaks | 40ms updates, rainbow rotation, 2-3 simultaneous peaks |
| **WORKING** | Processing task | Rapid bouncing chase from edges to center and back with cyan→purple gradient and comet tail | 35ms updates, smooth bounce motion, bright leading edge |
| **RECEIVING_TRANSMISSION** | Receiving data | Fast symmetrical data stream with mixed colors (white/cyan/blue) and quick fade trails | 50ms updates, 2-3 packet pairs, 60% fade rate |
| **GOING_TO_SLEEP** | Preparing to sleep | Sunset fade transitioning through color temperatures (warm amber → deep red → off) | 1.8s total duration, smooth color/brightness fade |
| **SLEEPING** | Idle state | Gentle breathing pulse in dark blue with subtle center-outward brightness variation | 3.5s breath cycle, very dim (5-15 brightness) |

#### Usage Example

```cpp
#include "voice_indicator.h"

CRGB leds[8];
VoiceIndicator voiceIndicator(leds, 8);

void setup() {
  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, 8);
  FastLED.setBrightness(5);
  voiceIndicator.begin();
}

void loop() {
  voiceIndicator.update();  // Non-blocking, call every loop

  // Change state based on voice interaction
  // voiceIndicator.setState(VoiceIndicator::LISTENING);
}
```

#### Animation Design Philosophy

- **Symmetry**: All patterns mirror perfectly across the center LED axis
- **Dynamic Motion**: Fast update rates (30-70ms) create fluid, organic movement
- **Energy Variation**: Brightness peaks, color shifts, and random elements prevent static feel
- **State Clarity**: Each animation has distinct visual signature for instant recognition
- **Performance**: Non-blocking updates, optimized for ESP32-S3 at 240MHz

#### Files

- `firmware/src/voice_indicator.h` - Class interface and state definitions
- `firmware/src/voice_indicator.cpp` - Animation implementations
- `firmware/src/main.cpp` - Integration example with test cycle

## Wiring

See `wiring.md` for pin assignments and connections.

## Assembly

See `assembly.md` for build instructions.

## Testing

See `tests/` directory for module-specific tests.

## Diagnostics

See `diagnostics/` directory for diagnostic tools.
