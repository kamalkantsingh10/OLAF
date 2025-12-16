# OLAF Phase 1 - Build Tracking Sheet

**Project:** OLAF Phase 1: Hardware Build & ROS2 Foundation
**Started:** [Date TBD]
**Target Completion:** [Date TBD]
**Status:** Not Started

---

## Progress Overview

| Epic | Stories Complete | Total Stories | Progress |
|------|------------------|---------------|----------|
| Epic 0: ROS2 Foundation Setup | 0 | 4 | 0% |
| Epic 1: Head+Ears Module Build | 0 | 8 | 0% |
| Epic 2: Neck Module Build | 0 | 9 | 0% |
| Epic 3: Torso Module Build | 0 | 9 | 0% |
| Epic 4: Base Module Build | 0 | 12 | 0% |
| Epic 5: End-to-End Demo Script | 0 | 4 | 0% |
| **TOTAL** | **0** | **46** | **0%** |

---

## Epic 0: ROS2 Foundation Setup

**Goal:** Establish ROS2 Humble workspace on Raspberry Pi with I2C tools configured

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 0.1 | Install ROS2 Humble on Raspberry Pi | ⬜ Not Started | | | |
| 0.2 | Create ROS2 Workspace with Module-First Structure | ⬜ Not Started | | | |
| 0.3 | Configure I2C Communication Tools | ⬜ Not Started | | | |
| 0.4 | Set Up Development Tools and Dependencies | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

---

## Epic 1: Head+Ears Module Build

**Goal:** Complete Head+Ears module (I2C 0x08) with OLED eyes, articulated ears, projector control, OAK-D-Pro camera

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 1.1 | Breadboard Head+Ears Components and Test Connectivity | ⬜ Not Started | | | |
| 1.2 | Design Head+Ears Custom PCB in Fritzing | ⬜ Not Started | | | |
| 1.3 | Order and Receive Head+Ears PCB from Elecrow | ⬜ Not Started | | | Order #: |
| 1.4 | Assemble and Test Head+Ears PCB | ⬜ Not Started | | | |
| 1.5 | Design and 3D Print Head+Ears Enclosure | ⬜ Not Started | | | |
| 1.6 | Develop Head+Ears ESP32 Firmware | ⬜ Not Started | | | |
| 1.7 | Create Head+Ears ROS2 Driver Node | ⬜ Not Started | | | |
| 1.8 | Mount Head+Ears Module to Robot Frame | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

**Key Components:**
- [ ] 2× OLED displays (128×64, SPI)
- [ ] 4× Feetech servos (2-DOF per ear)
- [ ] Floor projector + optocoupler + focus servo
- [ ] OAK-D-Pro RGBD camera
- [ ] ESP32 module

---

## Epic 2: Neck Module Build

**Goal:** Complete Neck module (I2C 0x09) with 3-DOF pan/tilt/roll, kickstand servo, presence sensors

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 2.1 | Breadboard Neck Components and Test Connectivity | ⬜ Not Started | | | |
| 2.2 | Design Neck Custom PCB in Fritzing | ⬜ Not Started | | | |
| 2.3 | Order and Receive Neck PCB from Elecrow | ⬜ Not Started | | | Order #: |
| 2.4 | Assemble and Test Neck PCB | ⬜ Not Started | | | |
| 2.5 | Design and 3D Print Neck Enclosure and Servo Mounts | ⬜ Not Started | | | |
| 2.6 | Design Kickstand Mechanism with Single Servo | ⬜ Not Started | | | Metal parts needed |
| 2.7 | Develop Neck ESP32 Firmware | ⬜ Not Started | | | |
| 2.8 | Create Neck ROS2 Driver Node | ⬜ Not Started | | | |
| 2.9 | Mount Neck Module to Robot Frame | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

**Key Components:**
- [ ] 4× Feetech STS3215 servos (3-DOF + kickstand, daisy-chained)
- [ ] 2× Presence sensors (mmWave or PIR)
- [ ] Kickstand metal components (aluminum rod, steel pad)
- [ ] ESP32 module

---

## Epic 3: Torso Module Build

**Goal:** Complete Torso module (I2C 0x0A) with Raspberry Pi 5 housing, heart display, thermal printer

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 3.1 | Breadboard Torso Components and Test Connectivity | ⬜ Not Started | | | |
| 3.2 | Design Torso Custom PCB in Fritzing | ⬜ Not Started | | | |
| 3.3 | Order and Receive Torso PCB from Elecrow | ⬜ Not Started | | | Order #: |
| 3.4 | Assemble and Test Torso PCB | ⬜ Not Started | | | |
| 3.5 | Assemble Torso Enclosure with Kitchen Bin and Metal Reinforcement | ⬜ Not Started | | | Kitchen bin + metal bars |
| 3.6 | Develop Torso ESP32 Firmware | ⬜ Not Started | | | |
| 3.7 | Create Torso ROS2 Driver Node | ⬜ Not Started | | | |
| 3.8 | Install and Configure Raspberry Pi 5 in Torso Module | ⬜ Not Started | | | |
| 3.9 | Mount Torso Module to Robot Frame | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

**Key Components:**
- [ ] Raspberry Pi 5 16GB
- [ ] Hailo AI Kit (26 TOPS, PCIe)
- [ ] 2.8" square display (heart animation, SPI)
- [ ] Thermal printer
- [ ] Speakerphone (USB)
- [ ] ESP32 module

---

## Epic 4: Base Module Build

**Goal:** Complete Base module (I2C 0x0B) with hoverboard motors, 36V battery, power distribution, self-balancing

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 4.1 | Source and Disassemble Hoverboard for Parts | ⬜ Not Started | | | Hoverboard source: |
| 4.2 | Breadboard Base Components and Test Connectivity | ⬜ Not Started | | | |
| 4.3 | Design Power Distribution System for 36V Battery | ⬜ Not Started | | | Critical: power budget! |
| 4.4 | Design Base Custom PCB in Fritzing | ⬜ Not Started | | | 2oz copper! |
| 4.5 | Order and Receive Base PCB from Elecrow | ⬜ Not Started | | | Order #: |
| 4.6 | Assemble and Test Base PCB | ⬜ Not Started | | | SAFETY FIRST! |
| 4.7 | Assemble Base Platform with Skateboard Suspension | ⬜ Not Started | | | Iron trucks for wheels |
| 4.8 | Configure ODrive for Hoverboard Motors | ⬜ Not Started | | | ODrive model: |
| 4.9 | Develop Base ESP32 Firmware with 200Hz Balancing PID | ⬜ Not Started | | | |
| 4.10 | Fine-Tune Self-Balancing PID Parameters | ⬜ Not Started | | | PID tuning log |
| 4.11 | Create Base ROS2 Driver Node | ⬜ Not Started | | | |
| 4.12 | Mount Base Module and Connect All Modules | ⬜ Not Started | | | Final integration! |

**Epic Status:** ⬜ Not Started

**Key Components:**
- [ ] Hoverboard: 2× BLDC motors (350W), 36V battery (10S Li-ion, 4-5Ah)
- [ ] ODrive S1 motor controller
- [ ] MPU6050 IMU
- [ ] Buck converters: 36V→12V, 36V→5V (6A+), 36V→3.3V (2A+)
- [ ] Fuse, emergency stop switch, charge port
- [ ] ESP32 module

**Power Budget:**
- Pi 5 + Hailo: ~25W
- 4× ESP32s: ~8W
- Servos: [Calculate]
- Displays/Sensors: ~5W
- ODrive + Motors: ~100W+ peak
- **Total: [TBD]**

---

## Epic 5: End-to-End Demo Script

**Goal:** Create and execute demo script validating all modules via ROS2

| ID | Story | Status | Started | Completed | Notes/Blockers |
|----|-------|--------|---------|-----------|----------------|
| 5.1 | Create End-to-End Demo Script | ⬜ Not Started | | | |
| 5.2 | Create Demo Launch File | ⬜ Not Started | | | |
| 5.3 | Test and Record End-to-End Demo | ⬜ Not Started | | | Video link: |
| 5.4 | Document Phase 1 Completion and Lessons Learned | ⬜ Not Started | | | |

**Epic Status:** ⬜ Not Started

**Demo Sequence:**
1. Deploy kickstand → 2. Blink eyes → 3. Move ears → 4. Tilt head → 5. Animate heart → 6. Print "Phase 1 Complete!" → 7. Retract kickstand → 8. Balance → 9. Move forward/backward

---

## Build Milestones

| Milestone | Target Date | Actual Date | Status |
|-----------|-------------|-------------|--------|
| ROS2 Foundation Setup Complete | | | ⬜ |
| First Module Working (Head+Ears or choice) | | | ⬜ |
| Two Modules Working | | | ⬜ |
| Three Modules Working | | | ⬜ |
| All Four Modules Working | | | ⬜ |
| Base Module Balancing Achieved | | | ⬜ |
| End-to-End Demo Success | | | ⬜ |
| Phase 1 Complete (v1.0-phase1 tagged) | | | ⬜ |

---


## Cost Tracking

| Category | Budgeted | Actual | Variance | Notes |
|----------|----------|--------|----------|-------|
| Hoverboard Parts | | | | |
| PCBs (Elecrow) | | | | 4 boards + shipping |
| ESP32 Modules (4×) | | | | |
| Raspberry Pi 5 16GB | | | | |
| Hailo AI Kit | | | | |
| OAK-D-Pro Camera | | | | |
| Servos (Feetech) | | | | 12 total |
| OLED Displays | | | | |
| Thermal Printer | | | | |
| ODrive Controller | | | | |
| Buck Converters | | | | |
| 3D Printing Filament | | | | Head+Ears, Neck only |
| Kitchen Bin (Torso) | | | | Durable plastic |
| Skateboard Suspension (Base) | | | | Iron trucks |
| Metal Flat Bars | | | | Steel/aluminum |
| Electronic Components | | | | Resistors, caps, etc. |
| Mechanical Parts | | | | Fasteners, hardware |
| Misc/Contingency | | | | |
| **TOTAL** | | | | |

---


## Known Issues & Workarounds

| Issue # | Module | Description | Workaround | Resolution | Status |
|---------|--------|-------------|------------|------------|--------|
| | | | | | |

---

