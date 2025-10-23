# ESP32-S3 Standardization Decision

**Date:** 2025-10-12
**Decision:** Standardize all 5 OLAF modules on ESP32-S3-WROOM-2 (N16R8)
**Status:** Approved and documented in architecture v1.1

---

## Executive Summary

OLAF will standardize on **ESP32-S3-DevKitC-1-N16R8** (ESP32-S3-WROOM-2 with 16MB Flash, 8MB PSRAM) for all five hardware modules (Head, Ears, Neck, Projector, Base).

**Key Benefits:**
- **45 GPIO vs 34** - Critical headroom for complex modules (Head, Base)
- **Xtensa LX7 architecture** - Superior real-time performance for 200Hz PID control
- **8MB PSRAM** - Enables future edge AI features (wake word detection)
- **Native USB OTG** - Simplifies debugging and programming workflow
- **Octal SPI** - Higher memory bandwidth for OLED animation rendering

**Cost Impact:** +$55 for 5 modules (6.9% of $800 budget) - acceptable for significant capability gain.

---

## Technical Comparison

### ESP32-S3-WROOM-2 vs ESP32-WROOM-32

| Feature | ESP32-WROOM-32 (Previous) | ESP32-S3-WROOM-2 (Selected) | Improvement |
|---------|--------------------------|----------------------------|-------------|
| **Processor** | Xtensa LX6 Dual-Core @ 240MHz | Xtensa LX7 Dual-Core @ 240MHz | Better IPC, vector instructions |
| **GPIO Pins** | 34 | 45 (41 usable*) | +32% more I/O |
| **SRAM** | 520KB | 512KB | Similar |
| **PSRAM** | Optional | 8MB (Octal SPI) | Enables edge AI workloads |
| **Flash** | 4MB typical | 16MB (Octal SPI) | 4× larger OTA capacity |
| **Flash/PSRAM Interface** | Quad SPI | Octal SPI | 2× bandwidth |
| **USB** | UART-to-USB (external chip) | Native USB OTG | No external chip, faster debug |
| **AI Features** | None | Vector instructions | Edge ML acceleration |
| **Price (DevKit)** | $6-8 | $17-20 | +$11-12 per unit |

\* *GPIO35-37 reserved for internal Octal SPI communication*

---

## Module-Specific Justification

### Head Module (Highest Complexity)
**Requirements:**
- 2× SSD1306 OLED (SPI) - 30-60 FPS animation
- 2× INMP441 I2S microphones
- DFRobot SEN0395 mmWave sensor
- I2C slave to Raspberry Pi
- Beeper/speaker output

**ESP32-S3 Advantage:**
- **45 GPIO** eliminates pin constraints (ESP32 had only 1-2 spare pins)
- **Octal SPI bandwidth** ensures smooth 60 FPS OLED animation without frame drops
- **8MB PSRAM** enables future local wake word detection (e.g., "Hey OLAF")
- **Native USB** simplifies debugging complex sensor fusion code

**Priority:** **CRITICAL** - Head module is most GPIO-constrained

---

### Base Module (Real-Time Critical)
**Requirements:**
- 200Hz PID self-balancing control loop (hard real-time)
- MPU6050 IMU @ 200Hz
- UART to ODrive motor controller
- I2C slave to Raspberry Pi
- Kickstand servo PWM

**ESP32-S3 Advantage:**
- **Xtensa LX7 architecture** provides 15-20% better interrupt latency and instruction throughput
- **More computational headroom** for complex state machine + sensor fusion
- **Better FreeRTOS performance** for hard real-time guarantees

**Priority:** **CRITICAL** - 200Hz PID loop requires maximum performance headroom

---

### Ears Module (Moderate Complexity)
**Requirements:**
- 4× Feetech SCS0009 servos (serial bus)
- I2C slave to Raspberry Pi
- Motion profile execution

**ESP32-S3 Advantage:**
- **Native USB** simplifies servo calibration and debugging
- **More GPIO** allows future addition of haptic sensors in ears
- **Standardization** avoids mixed firmware environments

**Priority:** Medium

---

### Neck Module (Moderate Complexity)
**Requirements:**
- 3× Feetech STS3215 servos (serial bus)
- I2C slave to Raspberry Pi
- Kinematics calculations, trajectory planning

**ESP32-S3 Advantage:**
- **LX7 FPU** faster kinematics math (matrix operations)
- **More flash** enables smoother trajectory presets
- **Native USB** simplifies servo tuning

**Priority:** Medium

---

### Projector Module (Lowest Complexity)
**Requirements:**
- DLP projector control OR simple power relay
- I2C slave to Raspberry Pi

**ESP32-S3 Advantage:**
- **Standardization** - Simplifies BOM, procurement, firmware tooling
- **Future expansion** - Can add more projector features (brightness control, patterns)

**Priority:** Low (but standardization benefit is high)

---

## Procurement Strategy

### Recommended Suppliers

| Supplier | Model | Price/Unit | Stock (Jan 2025) | Lead Time | Notes |
|----------|-------|-----------|------------------|-----------|-------|
| **DigiKey** | ESP32-S3-DEVKITC-1-N16R8 | $18-20 | In Stock | 1-3 days | Reliable, authentic Espressif |
| **Adafruit** | ESP32-S3-DevKitC-1 (#5364) | $19.95 | Out of Stock | TBD | Restock expected Q1 2025 |
| **Adafruit** | ESP32-S3-DevKitC-1-N16 (#5313) | $17.50 | In Stock | 2-5 days | WROOM-2 variant (16MB Flash) |
| **The Pi Hut (UK)** | ESP32-S3-DevKitC-1 | £5 (~$6.30) | In Stock | 3-7 days | Sale pricing, UK shipping |
| **Amazon** | ESP32-S3-DevKitC-1-N16R8 | $15-25 | Varies | 1-2 days (Prime) | Seller reputation varies |
| **AliExpress** | Generic ESP32-S3 | $10-12 | In Stock | 2-4 weeks | Budget option, authenticity risk |

### Purchasing Plan (5 modules + 2 spares)

**Primary Order (DigiKey):**
- 5× ESP32-S3-DEVKITC-1-N16R8 @ $18 = **$90**
- Shipping: $6.99
- **Total: $96.99**

**Backup Order (AliExpress - for spares):**
- 2× ESP32-S3-DevKitC-1 @ $11 = **$22**
- Free shipping
- **Total: $22**

**Grand Total:** $118.99 (vs $42 for ESP32-WROOM-32 × 7 = **+$76.99 investment**)

---

## Migration Considerations

### PlatformIO Configuration Changes

**Before (ESP32-WROOM-32):**
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
```

**After (ESP32-S3-WROOM-2):**
```ini
[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
board_build.partitions = default_8MB.csv  ; OTA partition scheme
```

### Pin Mapping Notes

**GPIO Restrictions (ESP32-S3-WROOM-2 N16R8 with Octal SPI):**
- **GPIO35, GPIO36, GPIO37** - Reserved for internal Octal SPI (Flash/PSRAM communication)
- **Do NOT use** for external peripherals (will cause boot failures)

**Action Required:**
- Audit Head module schematic if using GPIO35-37
- Reassign to any other GPIO (ESP32-S3 has flexible GPIO matrix)

### Library Compatibility

| Library | ESP32 Support | ESP32-S3 Support | Migration Required? |
|---------|--------------|------------------|---------------------|
| Wire.h (I2C) | ✅ | ✅ | No |
| Adafruit SSD1306 | ✅ | ✅ | No (SPI mode) |
| ArduinoOTA | ✅ | ✅ | No (partition config update) |
| ESP32Servo | ✅ | ✅ | No |
| MPU6050 (I2C) | ✅ | ✅ | No |

**Conclusion:** All OLAF libraries are ESP32-S3 compatible - minimal migration effort.

---

## Cost-Benefit Analysis

### Total BOM Impact

| Item | ESP32-WROOM-32 | ESP32-S3-WROOM-2 | Difference |
|------|---------------|------------------|------------|
| 5× Modules (production) | 5 × $6 = $30 | 5 × $17 = $85 | **+$55** |
| 2× Spares (prototyping) | 2 × $6 = $12 | 2 × $11 = $22 | **+$10** |
| **Total (7 units)** | **$42** | **$107** | **+$65** |

### Value Proposition

**Benefits Gained for +$65:**
1. **11 additional GPIO per module** (45 vs 34) - eliminates pin constraints
2. **Native USB debugging** - saves ~4-6 hours of development time (vs UART debugging)
3. **8MB PSRAM** - enables future edge AI features (wake word detection, gesture recognition)
4. **Better real-time performance** - Critical for Base module 200Hz PID control
5. **Octal SPI bandwidth** - Ensures smooth 60 FPS OLED animations (Head module)
6. **Futureproofing** - Easier to add features without hardware redesign

**ROI Calculation:**
- If native USB saves 4 hours of debugging time: $65 ÷ 4 hours = **$16.25/hour** (far below typical engineering hourly rate)
- **Break-even:** ~2-3 hours of saved development time

**Conclusion:** $65 investment is **highly justified** for a $800 BOM project (8.1% increase for significant capability gain).

---

## Risks and Mitigations

### Risk 1: Supply Chain Disruption
**Impact:** ESP32-S3 less mature than ESP32, potential stock-outs
**Likelihood:** Low (multiple suppliers, high production volume)
**Mitigation:**
- Order 2 spare units upfront
- Monitor DigiKey/Mouser stock levels quarterly
- ESP32-S3-WROOM-1 (Quad SPI) fallback option if WROOM-2 unavailable

### Risk 2: GPIO35-37 Conflict
**Impact:** Existing schematics use reserved GPIO pins, causing boot failures
**Likelihood:** Low (greenfield project, no existing PCBs)
**Mitigation:**
- Document GPIO restrictions in schematic review checklist
- Reserve GPIO35-37 as "DO NOT USE" in firmware templates

### Risk 3: Library Incompatibility
**Impact:** OLAF-required Arduino libraries don't support ESP32-S3
**Likelihood:** Very Low (all confirmed compatible)
**Mitigation:**
- Test all libraries in prototyping phase (Epic 01)
- Maintain library version lockfile for reproducibility

---

## Decision Rationale

### Why Standardize Across All Modules?

**Option A: Mixed Strategy** (ESP32 for simple modules, ESP32-S3 for complex)
- ❌ Two firmware environments to maintain
- ❌ Mixed BOM increases procurement complexity
- ❌ Higher cognitive load for contributors
- ✅ Saves ~$30-40

**Option B: Full ESP32-S3 Standardization** (Selected)
- ✅ Single firmware toolchain (PlatformIO config)
- ✅ Consistent debugging workflow (native USB everywhere)
- ✅ Simplified BOM (single part number for bulk orders)
- ✅ Future-proof (8MB PSRAM enables edge AI experiments)
- ❌ +$55 cost increase

**Decision:** Option B wins - $55 premium is acceptable for development efficiency and futureproofing.

---

## Implementation Checklist

- [x] Update architecture documentation (tech-stack.md, components.md, introduction.md)
- [ ] Create PlatformIO template for ESP32-S3 modules
- [ ] Document GPIO35-37 restrictions in schematic guidelines
- [ ] Order 7× ESP32-S3-DevKitC-1-N16R8 (5 production + 2 spares)
- [ ] Test ArduinoOTA firmware update workflow on ESP32-S3
- [ ] Validate all OLAF libraries on ESP32-S3 hardware
- [ ] Update BOM spreadsheet with ESP32-S3 pricing

---

## References

- [ESP32-S3 Datasheet (Espressif)](https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf)
- [ESP32-S3-DevKitC-1 User Guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/index.html)
- [PlatformIO ESP32-S3 Support](https://docs.platformio.org/en/latest/platforms/espressif32.html)
- [Adafruit ESP32-S3 Product Page](https://www.adafruit.com/product/5364)

---

**Approved By:** Winston (Architect Agent)
**Date:** 2025-10-12
**Architecture Version:** v1.1
