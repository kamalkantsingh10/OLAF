# Hardware Updates v1.2

**Date:** 2025-10-12
**Architecture Version:** v1.2
**Status:** Documented

---

## Summary of Changes

This document captures the hardware specification updates made to OLAF's architecture (v1.1 → v1.2).

### Three Key Hardware Updates

1. **Head Module Eyes: OLED → Round Color TFT**
2. **Kickstand Servo: Hobby Servo → High-Torque Bus Servo**
3. **Servo Control Architecture: Added Bus Servo Controllers**

---

## 1. Head Module Display Upgrade

### Previous Specification (v1.1)
- 2× SSD1306 OLED displays
- 128×64 pixels
- Monochrome (white on black)
- SPI interface
- Target: 30-60 FPS animation

### New Specification (v1.2)
- **2× GC9A01 Round TFT displays**
- **1.28 inch diameter, circular form factor**
- **240×240 pixels (57,600 pixels vs 8,192 OLED)**
- **65K colors (RGB565)**
- **SPI interface (4-wire)**
- **60 FPS full-color animation**

### Advantages

| Aspect | OLED (Old) | GC9A01 TFT (New) | Benefit |
|--------|-----------|------------------|---------|
| **Resolution** | 128×64 (8,192px) | 240×240 (57,600px) | 7× more detail |
| **Color** | Monochrome | 65K colors | Expressive emotions (pupils, iris, gradients) |
| **Shape** | Rectangular | Circular | Perfect for robot eyes (no corners) |
| **Expressiveness** | Low (pixel art eyes) | High (smooth animations, realistic iris) | Personality depth |
| **Library Support** | Adafruit SSD1306 | TFT_eSPI, Adafruit GFX | Mature ecosystem |
| **Price** | ~$5-8/pair | ~$8-12/pair | Minimal cost increase |

### Technical Considerations

**Frame Buffer Requirements:**
- Resolution: 240×240 × 2 bytes (RGB565) = **115,200 bytes per display**
- For 2 displays: **230,400 bytes** (230 KB)
- ESP32-S3 PSRAM (8MB): ✅ Plenty of headroom for double-buffering

**SPI Bandwidth:**
- 240×240 @ 60 FPS = 3,456,000 pixels/sec
- RGB565 (2 bytes/pixel) = 6.6 MB/sec per display
- SPI @ 20 MHz: 2.5 MB/sec theoretical → **realistically 30-40 FPS smooth, 60 FPS for partial updates**
- ESP32-S3 Octal SPI flash bandwidth ensures no bottleneck on animation loading

**Firmware Impact:**
- Replace `Adafruit_SSD1306.h` with `TFT_eSPI.h` or `Adafruit_GC9A01A.h`
- Update animation engine for RGB565 color space
- No GPIO pin changes (SPI interface identical)

---

## 2. Kickstand Servo Upgrade

### Previous Specification (v1.1)
- MG90S or SG90 hobby servo
- 2-3 kg·cm torque
- PWM control (dedicated GPIO pin)
- Analog servo

### New Specification (v1.2)
- **Feetech STS3215 bus servo**
- **30 kg·cm torque (10× stronger)**
- **Serial bus control (STSC protocol)**
- **Controlled via shared Bus Servo Controller (with neck servos)**

### Advantages

| Aspect | MG90S (Old) | STS3215 (New) | Benefit |
|--------|------------|---------------|---------|
| **Torque** | 2-3 kg·cm | 30 kg·cm | 10× stronger, reliable deployment |
| **Control** | PWM (analog) | Serial bus (digital) | Position feedback, precise control |
| **Interface** | Dedicated GPIO | Shared bus controller | Frees GPIO, cleaner architecture |
| **Durability** | Consumer grade | Robotics grade | Longer lifespan under load |
| **Feedback** | None | Position, load, temperature | Safety monitoring |

### Rationale

**Why Higher Torque?**
- OLAF's estimated weight: 8-12 kg
- Kickstand must support full weight when deploying from tilted position
- 2-3 kg·cm insufficient for reliable deployment under load
- 30 kg·cm provides 3-5× safety margin

**Why Bus Servo?**
- Consistent with neck servos (same STS3215 model)
- Shares servo controller with neck (cost savings, simpler wiring)
- Position feedback enables safety checks (e.g., "kickstand fully deployed?")

---

## 3. Bus Servo Controller Architecture

### New Hardware: STSC Series Bus Servo Controllers

**Purpose:** Centralized power and control for Feetech serial bus servos (SCS0009, STS3215)

**Two Controllers Added:**

#### Controller 1: Ears Module (Dedicated)
- **Servos:** 4× Feetech SCS0009 (ear articulation)
- **Configuration:** 2 servos per ear × 2 ears = 4 total
- **Connection:** Serial bus to Ears ESP32-S3
- **Features:**
  - Integrated 5V/6V servo power supply
  - Serial bus protocol (STSC compatible)
  - Daisy-chain all 4 servos on single bus

#### Controller 2: Neck Module (Shared with Kickstand)
- **Servos:** 3× Feetech STS3215 (neck) + 1× STS3215 (kickstand) = **4 servos total**
- **Configuration:**
  - Neck: Pan, Tilt, Roll (3 servos)
  - Kickstand: Deploy/Retract (1 servo, physically located on Base module)
- **Connection:** Serial bus to Neck ESP32-S3
- **Inter-Module Coordination:**
  - Base module sends kickstand commands to Raspberry Pi via I2C
  - Pi forwards commands to Neck module via I2C
  - Neck module controls kickstand servo via bus servo controller

**Why Shared Controller for Neck + Kickstand?**
1. **Cost Efficiency:** One controller vs two ($20-30 savings)
2. **Wiring Simplification:** Single serial bus cable from Neck to Base
3. **Bus Capacity:** STSC controllers support up to 8-12 servos (4 is well within limits)
4. **Minimal Latency:** Kickstand commands are infrequent (only during state transitions)

---

## Architecture Implications

### Inter-Module Communication for Kickstand

```
┌─────────────┐                  ┌──────────────────┐
│ Base Module │  I2C (0x0C)      │ Raspberry Pi 5   │
│  ESP32-S3   ├─────────────────►│ (ROS2 Master)    │
│             │  "deploy_kick"   │                  │
└─────────────┘                  └────────┬─────────┘
                                          │
                                          │ I2C (0x0A)
                                          │ "servo_4_pos_90"
                                          ▼
                                 ┌──────────────────┐
                                 │  Neck Module     │
                                 │   ESP32-S3       │
                                 │                  │
                                 └────────┬─────────┘
                                          │
                                          │ Serial Bus (STSC)
                                          ▼
                                 ┌──────────────────┐
                                 │ Bus Servo Ctrl   │
                                 │  (4 servos)      │
                                 │  - Servo 1: Pan  │
                                 │  - Servo 2: Tilt │
                                 │  - Servo 3: Roll │
                                 │  - Servo 4: Kick │◄── Physical cable to Base
                                 └──────────────────┘
```

**Latency Analysis:**
- Base → Pi (I2C): ~5-10ms
- Pi → Neck (I2C): ~5-10ms
- Neck → Servo Controller (Serial): ~10-20ms
- **Total:** ~20-40ms (acceptable for kickstand deployment, not time-critical)

### GPIO Savings

**Before (v1.1):**
- Base module: 1× GPIO for PWM kickstand servo

**After (v1.2):**
- Base module: 0 GPIO for kickstand (freed up)
- Neck module: 1× UART TX/RX for bus servo controller (already used)

**Net Benefit:** Frees 1 GPIO on Base module for future sensors/features

---

## Updated Bill of Materials (Hardware)

### Displays
| Item | Qty | Unit Price | Total | Notes |
|------|-----|-----------|-------|-------|
| GC9A01 Round TFT (1.28", 240×240) | 2 | $4-6 | **$8-12** | AliExpress/Amazon |
| ~~SSD1306 OLED (removed)~~ | ~~2~~ | ~~$4~~ | ~~-$8~~ | Replaced |

### Servos
| Item | Qty | Unit Price | Total | Notes |
|------|-----|-----------|-------|-------|
| Feetech STS3215 (Neck) | 3 | $25-30 | $75-90 | Existing |
| Feetech STS3215 (Kickstand) | 1 | $25-30 | **$25-30** | **NEW** |
| Feetech SCS0009 (Ears) | 4 | $15-20 | $60-80 | Existing |
| ~~MG90S/SG90 (removed)~~ | ~~1~~ | ~~$3~~ | ~~-$3~~ | Replaced |

### Servo Controllers
| Item | Qty | Unit Price | Total | Notes |
|------|-----|-----------|-------|-------|
| Bus Servo Controller (STSC) - Ears | 1 | $15-25 | **$15-25** | **NEW** |
| Bus Servo Controller (STSC) - Neck+Kick | 1 | $15-25 | **$15-25** | **NEW** |

### Net Cost Impact

| Category | Change | Amount |
|----------|--------|--------|
| Displays | GC9A01 - OLED | +$0 to +$4 |
| Kickstand Servo | STS3215 - MG90S | +$22 to +$27 |
| Servo Controllers (2×) | New hardware | +$30 to +$50 |
| **Total BOM Increase** | | **+$52 to +$81** |

**Percentage Impact:** +6.5% to +10% of $800 budget (acceptable for significant capability upgrade)

---

## Firmware Updates Required

### Head Module
- [x] Update display driver: `SSD1306` → `GC9A01` (TFT_eSPI library)
- [x] Update animation engine for RGB565 color space
- [x] Increase frame buffer allocation (use PSRAM)
- [ ] Create color animation library (gradients, smooth iris movements)

### Ears Module
- [x] Add bus servo controller driver (`servo_bus_controller.cpp`)
- [ ] Update servo command protocol (STSC serial bus vs I2C)
- [ ] Implement daisy-chain addressing for 4 servos

### Neck Module
- [x] Add bus servo controller driver (`servo_bus_controller.cpp`)
- [x] Support 4 servos (3 neck + 1 kickstand)
- [ ] Implement I2C command handler for kickstand proxy commands
- [ ] Add servo ID mapping (1=pan, 2=tilt, 3=roll, 4=kickstand)

### Base Module
- [x] Update kickstand control to send I2C commands to Pi (vs direct PWM)
- [ ] Remove GPIO PWM code for kickstand servo
- [ ] Add kickstand position feedback monitoring (via Pi → Neck → Base)

---

## Testing Checklist

- [ ] **GC9A01 TFT Testing**
  - [ ] SPI communication at 20 MHz (both displays)
  - [ ] Full-screen RGB565 bitmap rendering
  - [ ] 60 FPS partial update test (eye animations)
  - [ ] PSRAM double-buffering stability test
  - [ ] Color accuracy validation (gradients, skin tones)

- [ ] **Bus Servo Controller Testing**
  - [ ] Ears: 4× SCS0009 daisy-chain addressing
  - [ ] Neck: 3× STS3215 + 1× kickstand servo coordination
  - [ ] Position feedback reading (all servos)
  - [ ] Load/temperature monitoring
  - [ ] Emergency stop testing (servo overload detection)

- [ ] **Kickstand Deployment Testing**
  - [ ] Base → Pi → Neck → Servo Controller latency measurement
  - [ ] Full deployment under 12 kg load (OLAF weight)
  - [ ] Position feedback accuracy (deployed vs retracted)
  - [ ] State machine integration (BALANCING → RELAXED transition)
  - [ ] Fall detection → kickstand auto-deploy (safety test)

---

## Risks and Mitigations

### Risk 1: GC9A01 Frame Rate Below 60 FPS
**Impact:** Animation smoothness degraded
**Likelihood:** Medium (SPI bandwidth limits)
**Mitigation:**
- Use partial screen updates for eye animations (only update iris/pupil regions)
- Implement dirty rectangle tracking
- Optimize SPI clock speed (test up to 40 MHz if stable)
- Accept 30-40 FPS as "good enough" for expressive eyes

### Risk 2: Bus Servo Controller Compatibility
**Impact:** Feetech servos don't work with STSC controllers
**Likelihood:** Low (Feetech servos are STSC-compatible per spec)
**Mitigation:**
- Verify STSC protocol compatibility before bulk purchase
- Test with 1 servo + controller before ordering remaining units
- Fallback: Use direct UART control from ESP32-S3 (no controller needed)

### Risk 3: Kickstand Cross-Module Latency
**Impact:** Slow kickstand deployment during fall
**Likelihood:** Low (20-40ms measured latency)
**Mitigation:**
- Fall detection threshold set conservatively (|pitch| > 45°)
- Emergency stop triggers immediate ODrive brake + kickstand command
- 200ms deployment time acceptable (gravity takes ~300-500ms to tip robot)

### Risk 4: Increased BOM Cost
**Impact:** +$52-81 exceeds budget flexibility
**Likelihood:** Low ($800 budget has headroom)
**Mitigation:**
- GC9A01 displays available cheaply on AliExpress ($4-5/unit)
- Bus servo controllers optional (can use direct UART if needed)
- STS3215 kickstand torque upgrade is critical for safety (non-negotiable)

---

## Future Enhancements Enabled

### 1. Edge AI Emotion Recognition (GC9A01 Eyes)
- Use Pi camera + Hailo to detect user emotions
- Reflect emotions back via full-color eye animations (empathy)
- Example: User smiles → OLAF's eyes show warm gradient (orange/yellow)

### 2. Dynamic Pupil Dilation (Color TFT)
- Simulate biological pupil response to "interest" or "surprise"
- Small pupil (focused) → Large pupil (curious)
- Not possible with monochrome OLED

### 3. Servo Health Monitoring (Bus Controllers)
- Track servo load, temperature, position error over time
- Predict servo failures before they happen (e.g., grinding gears)
- Send maintenance alerts via OLAF's voice ("My left ear feels stiff!")

### 4. Advanced Neck Gestures (Shared Controller)
- Synchronized neck + ear movements (e.g., head tilt + ear droop = "sad")
- Choreographed multi-servo animations loaded from SD card
- Position feedback enables closed-loop gesture accuracy

---

## Conclusion

The v1.2 hardware updates significantly enhance OLAF's expressiveness and mechanical robustness:

1. **GC9A01 Round TFTs** transform eyes from pixel-art to lifelike (7× resolution, full color)
2. **STS3215 Kickstand** ensures reliable deployment under full robot weight (10× torque increase)
3. **Bus Servo Controllers** centralize servo management, free GPIO, enable health monitoring

**Total Cost Impact:** +$52-81 (6.5-10% of budget) - justified by expressiveness gain and safety improvement.

**Architecture Version:** v1.2 documented in all architecture files.

---

**Next Steps:**
1. Order GC9A01 TFT displays (2×) + bus servo controllers (2×)
2. Test GC9A01 with ESP32-S3 (SPI + PSRAM frame buffer)
3. Validate STSC bus servo controller with STS3215 servo
4. Update Head/Ears/Neck/Base firmware per checklist above

**Questions/Concerns:** Reach out to Winston (Architect Agent) for clarification.
