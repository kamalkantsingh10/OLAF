# OLAF Phase 1 - Build Tracking Sheet

**Project:** OLAF Phase 1: Hardware Build & ROS2 Foundation
**Architecture:** Fusion HAT (2 ESP32 + Pi-controlled peripherals)
**Started:** [Date TBD]
**Target Completion:** [Date TBD]
**Status:** Not Started

---

## Progress Overview

| Epic | Stories Complete | Total Stories | Progress |
|------|------------------|---------------|----------|
| Epic 0: ROS2 Foundation Setup | 0 | 5 | 0% |
| Epic 1: Head Module Build | 0 | 5 | 0% |
| Epic 2: Base Module Build | 0 | 8 | 0% |
| Epic 3: Neck Module Build | 0 | 4 | 0% |
| Epic 4: Ears Module Build | 0 | 4 | 0% |
| Epic 5: Indicators, Heart & Kickstand | 0 | 5 | 0% |
| Epic 6: End-to-End Demo Script | 0 | 4 | 0% |
| **TOTAL** | **0** | **35** | **0%** |

---

## Epic 0: ROS2 Foundation Setup

**Goal:** Establish ROS2 Jazzy on Ubuntu 24.04 with Fusion HAT, I2C, and USB serial configured

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 0.1 | Install Ubuntu 24.04 and ROS2 Jazzy on Raspberry Pi | ⬜ Not Started | | | |
| 0.2 | Create ROS2 Workspace with Module-First Structure | ✅ Done | | | |
| 0.3 | Configure Fusion HAT and I2C Communication | ⬜ Not Started | | | |
| 0.4 | Configure USB Serial for Waveshare Adapters | ⬜ Not Started | | | |
| 0.5 | Set Up Development Tools and Dependencies | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

---

## Epic 1: Head Module Build

**Goal:** Complete Head module (ESP32 I2C 0x10) with 2× GC9A01 round OLED eyes

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 1.1 | Breadboard Head Components and Test Connectivity | ⬜ Not Started | | | |
| 1.2 | Design Head Module Wiring and Assembly | ⬜ Not Started | | | |
| 1.3 | Develop Head ESP32 Firmware | ⬜ Not Started | | | |
| 1.4 | Create Head ROS2 Driver Node | ⬜ Not Started | | | |
| 1.5 | Design and 3D Print Head Enclosure | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

**Key Components:**
- [ ] ESP32-S3-DevKitC-1 (N16R8)
- [ ] 2× GC9A01 round displays (240×240, 1.28", SPI)
- [ ] I2C cable to Pi
- [ ] 5V power from 36V→5V buck

---

## Epic 2: Base Module Build

**Goal:** Complete Base module (ESP32 I2C 0x11) with hoverboard platform, ODrive, 200Hz PID balancing

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 2.1 | Source and Disassemble Hoverboard for Parts | ⬜ Not Started | | | Hoverboard source: |
| 2.2 | Breadboard Base Components and Test Connectivity | ⬜ Not Started | | | |
| 2.3 | Design Power Distribution System | ⬜ Not Started | | | Critical: power budget |
| 2.4 | Assemble Base Platform | ⬜ Not Started | | | |
| 2.5 | Configure ODrive for Hoverboard Motors | ⬜ Not Started | | | ODrive model: |
| 2.6 | Develop Base ESP32 Firmware with 200Hz Balancing PID | ⬜ Not Started | | | |
| 2.7 | Fine-Tune Self-Balancing PID Parameters | ⬜ Not Started | | | PID tuning log |
| 2.8 | Create Base ROS2 Driver Node | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

**Key Components:**
- [ ] ESP32-S3-DevKitC-1 (N16R8)
- [ ] MPU6050 IMU
- [ ] ODrive v3.6 motor controller
- [ ] 36V hoverboard battery (10S Li-ion)
- [ ] 2× hoverboard BLDC motors (350W each)
- [ ] Buck converters: 36V→12V, 36V→5V
- [ ] Skateboard suspension (iron trucks)

**Power Budget:**
- Pi 5 + Hailo: ~25W (from 7.4V logic battery)
- 2× ESP32s: ~4W
- Servos: ~20W peak
- Displays/Sensors: ~5W
- ODrive + Motors: ~100W+ peak
- **Total from 36V:** ~130W peak

---

## Epic 3: Neck Module Build

**Goal:** Complete Neck module with 3× STS3215 servos via USB Waveshare adapter

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 3.1 | Test Waveshare Adapter with STS3215 Servos | ⬜ Not Started | | | |
| 3.2 | Design Neck Mechanical Assembly | ⬜ Not Started | | | |
| 3.3 | Assemble and Test Neck Module | ⬜ Not Started | | | |
| 3.4 | Create Neck ROS2 Driver Node | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

**Key Components:**
- [ ] Waveshare Bus Servo Adapter (A) - USB mode
- [ ] 3× Feetech STS3215 servos (30 kg·cm)
- [ ] 12V power from 36V→12V buck
- [ ] 3D printed mounts and brackets

---

## Epic 4: Ears Module Build

**Goal:** Complete Ears module with 4× SCS0009 servos via USB Waveshare adapter

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 4.1 | Test Waveshare Adapter with SCS0009 Servos | ⬜ Not Started | | | |
| 4.2 | Design Ear Mechanical Assembly | ⬜ Not Started | | | |
| 4.3 | Assemble and Test Ears Module | ⬜ Not Started | | | |
| 4.4 | Create Ears ROS2 Driver Node | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

**Key Components:**
- [ ] Waveshare Bus Servo Adapter (A) - USB mode
- [ ] 4× Feetech SCS0009 servos
- [ ] 12V power from 36V→12V buck
- [ ] 3D printed ear shapes and mounts

---

## Epic 5: Indicators, Heart & Kickstand Build

**Goal:** Complete WS2812 indicators, 4" heart display, and kickstand mechanism

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 5.1 | Configure and Test WS2812 LED Strips | ⬜ Not Started | | | |
| 5.2 | Create Indicator ROS2 Driver Node | ⬜ Not Started | | | |
| 5.3 | Configure Heart Display | ⬜ Not Started | | | |
| 5.4 | Configure and Test Kickstand Servos | ⬜ Not Started | | | |
| 5.5 | Create Kickstand Control Integration | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

**Key Components:**
- [ ] 24× WS2812 LEDs (3×8 strips, daisy-chained)
- [ ] 4" Raspberry Pi Display (SPI/DSI)
- [ ] 2× Model plane landing gear servos (4.8-7.4V)
- [ ] Fusion HAT+ (WS2812 port + PWM channels)

---

## Epic 6: End-to-End Demo Script

**Goal:** Create and execute demo script validating all modules via ROS2

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 6.1 | Create End-to-End Demo Script | ⬜ Not Started | | | |
| 6.2 | Create Demo Launch File | ⬜ Not Started | | | |
| 6.3 | Test and Record End-to-End Demo | ⬜ Not Started | | | Video link: |
| 6.4 | Document Phase 1 Completion | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

**Demo Sequence:**
1. Deploy kickstand → 2. Blink eyes (expressions) → 3. Move ears (emotions) → 4. Tilt head (pan/tilt/roll) → 5. Animate heart → 6. Flash indicators → 7. Retract kickstand → 8. Balance → 9. Move forward/backward

---

## Build Milestones

| Milestone | Target Date | Actual Date | Status |
|-----------|-------------|-------------|--------|
| ROS2 Foundation Setup Complete | | | ⬜ |
| Head Module Working (eyes animate) | | | ⬜ |
| Base Module Balancing Achieved | | | ⬜ |
| Neck Module Working (3-DOF) | | | ⬜ |
| Ears Module Working (expressions) | | | ⬜ |
| Indicators/Heart/Kickstand Working | | | ⬜ |
| End-to-End Demo Success | | | ⬜ |
| Phase 1 Complete (v1.0-phase1 tagged) | | | ⬜ |

---

## Architecture Summary

### Module Communication

| Module | Interface | Address/Port | Controller |
|--------|-----------|--------------|------------|
| Head (eyes) | I2C | 0x10 | ESP32-S3 |
| Base (balance) | I2C | 0x11 | ESP32-S3 |
| Neck (servos) | USB Serial | /dev/waveshare_neck | Pi via Waveshare |
| Ears (servos) | USB Serial | /dev/waveshare_ears | Pi via Waveshare |
| Indicators (LEDs) | WS2812 | Fusion HAT | Pi direct |
| Heart (display) | SPI/DSI | Framebuffer | Pi direct |
| Kickstand (servos) | PWM | Fusion HAT P0/P1 | Pi direct |

### Power Distribution

| Rail | Source | Loads |
|------|--------|-------|
| 36V | Hoverboard battery | ODrive, motors |
| 12V | 36V→12V buck | Neck servos, Ear servos (via Waveshare) |
| 5V | 36V→5V buck | Head ESP32, Base ESP32, OLEDs |
| 5V | 7.4V→5V (Fusion HAT) | Pi 5, AI HAT, Heart display, WS2812, Kickstand |

---

## Cost Tracking

| Category | Budgeted | Actual | Variance | Notes |
|----------|----------|--------|----------|-------|
| Hoverboard (motors, battery) | | | | |
| ESP32-S3 Modules (2×) | | | | |
| Raspberry Pi 5 16GB | | | | |
| Hailo AI Kit | | | | |
| Fusion HAT+ | | | | |
| GC9A01 Displays (2×) | | | | |
| 4" Pi Display | | | | |
| Waveshare Adapters (2×) | | | | |
| STS3215 Servos (3×) | | | | |
| SCS0009 Servos (4×) | | | | |
| Landing Gear Servos (2×) | | | | |
| WS2812 LED Strips | | | | |
| ODrive v3.6 | | | | |
| MPU6050 IMU | | | | |
| Buck Converters | | | | |
| Skateboard Suspension | | | | |
| 3D Printing Filament | | | | |
| Misc Components | | | | |
| **TOTAL** | | | | |

---

## Known Issues & Workarounds

| Issue # | Module | Description | Workaround | Resolution | Status |
|---------|--------|-------------|------------|------------|--------|
| | | | | | |

---
