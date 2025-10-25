# Epic List

Based on the requirements, technical architecture, and V1 scope from the brief, here's the proposed epic breakdown for Olaf V1:

## Epic Structure Overview (13 Epics - Optimized Build Flow)

**Epic 1: Foundation & I2C Communication (Eye Displays)**
**Goal:** Establish ROS2 + I2C infrastructure with minimal working personality—dual eye LCDs showing emotion-driven expressions. Prove I2C + SPI architecture end-to-end. Validates Head Module (I2C 0x08) before physical integration in Epic 3. Use temporary bench power supply (full power system deferred to Epic 5).

**Duration:** 1.5 weeks | **Value:** Quick win proving I2C + SPI architecture, expressive eye displays, Head Module validation, initial design concept validation

---

**Epic 2: Complete OnShape Design & Community Feedback**
**Goal:** Design ALL mechanical components in OnShape (head housing, ears, neck gimbal, core torso, power enclosure, projector bay, base platform). Export STLs. Post complete design to Reddit/maker communities for feedback (3-7 day cycle). Incorporate feedback and mark design as "V1.0 Draft - Ready to Print."

**Duration:** 2-3 weeks | **Value:** Complete validated design before printing, community feedback incorporated early, STLs ready for all subsequent epics

---

**Epic 3: Head Module - Complete Build**
**Goal:** 3D print head housing (with Epic 2 feedback incorporated). Assemble dual GC9A01 round TFT eyes (240×240), OAK-D Pro camera, mmWave presence sensor, microphone array, speaker/beeper. ESP32 firmware for expressive eye animations and sensor integration.

**Duration:** 2-3 weeks | **Value:** Expressive eyes with presence detection, complete sensory input, validated head assembly

---

**Epic 4: Ears + Neck Module - Complete Build**
**Goal:** 3D print ear mounts and neck gimbal (with Epic 2 feedback incorporated). Assemble 2-DOF articulated ears (4× Feetech SCS0009 servos) and 3-DOF neck gimbal (pan ±90°, tilt ±45°, roll ±30° with 3× Feetech STS3215 servos). Shared ESP32 bus servo controller. Coordinated kinematics engine for ear+neck gestures.

**Duration:** 2-3 weeks | **Value:** Full head articulation, independent ear movement, coordinated emotional expression

---

**Epic 5: Core Torso & Power System - Complete Build**
**Goal:** 3D print core torso panels and power enclosure (with Epic 2 feedback incorporated). Assemble complete torso: Raspberry Pi mounting, **NEW heart LCD display integration** (1.53" Round TFT 360×360 ST77916 QSPI, Body Module I2C 0x0A), status indicators (RGB LEDs for power/battery/system status), component bays. Build power system: 36V hoverboard battery + BMS, buck converters (36V→5V, 36V→12V), charging circuit, power distribution harness, safety (fuses, emergency cutoff, voltage monitoring). Thermal testing and cable management.

**Duration:** 2-3 weeks | **Value:** Complete power infrastructure, integrated heart display in body, professional component housing, safe charging system

---

**Epic 6: Self-Balancing Base - Complete Build**
**Goal:** 3D print base platform (with Epic 2 feedback incorporated). Assemble two-wheel balancing system: hoverboard motors + wheels, ODrive motor controller, MPU6050 IMU. Implement 200Hz PID control, kickstand servo deployment for fall protection, odometry integration for SLAM readiness.

**Duration:** 3-4 weeks | **Value:** Active mobility foundation, autonomous balancing, navigation-ready odometry

---

**Epic 7: Full Robot Integration & Cable Management**
**Goal:** Integrate all physical modules into unified structure: Head + Ears/Neck → Torso → Base. Complete cable routing (power distribution, I2C buses, data lines). Install all I2C connections between Raspberry Pi and ESP32 modules. Structural assembly validation, stand test, balance check, full system power-on test.

**Duration:** 2-3 weeks | **Value:** Complete robot assembly, professional cable routing, accessible maintenance, validated mechanical integration

---

**Epic 8: DLP Floor Projector Integration**
**Goal:** 3D print projector mounting bracket (angled bay from Epic 2 design). Install DLP projector with 30-45° downward angle for floor projection. Implement projector power control via optocoupler circuit (ESP32 GPIO → optocoupler → projector power button). HDMI routing from Raspberry Pi. ESP32 firmware for I2C-based projector control. ROS2 driver node for projector power management and display output (floor graphics, gestures, information display).

**Duration:** 2-3 weeks | **Value:** Information display capability, coordinated projection with personality gestures, visual communication channel

---

**Epic 9: Basic Personality System**
**Goal:** Implement 2D emotion model (7 emotion types × 5 intensity levels) with coordinated expression across all channels: eyes (animation patterns), ears (position/movement), neck (gestures), heart LCD (BPM/color), status LEDs (color/breathing), beeper (tones), and floor projection (visual cues). Personality orchestrator node synchronizes all expression modules.

**Duration:** 2-3 weeks | **Value:** Full non-verbal personality expression, emotion synchronization across 7 channels, cohesive character

---

**Epic 10: SLAM & Spatial Awareness**
**Goal:** Enable OAK-D Pro stereo depth sensing, ROS2 Nav2 SLAM integration (Google Cartographer or RTAB-Map), apartment map building, localization. Integration with base odometry for accurate positioning.

**Duration:** 3-4 weeks | **Value:** Autonomous mapping, spatial understanding, navigation foundation

---

**Epic 11: Autonomous Navigation & Obstacle Avoidance**
**Goal:** Implement Nav2 path planning, dynamic obstacle avoidance (real-time depth sensing), balancing coordination during movement. Safe room-to-room navigation on SLAM maps.

**Duration:** 2-3 weeks | **Value:** Full autonomous mobility, collision-free navigation, coordinated balancing

---

**Epic 12: Computer Vision & Image Processing**
**Goal:** Deploy Hailo-8L accelerated models: object detection, person tracking, face recognition, gesture recognition. Visual understanding for contextual interactions and personality responses.

**Duration:** 2-3 weeks | **Value:** Visual intelligence, person identification, gesture-based commands

---

**Epic 13: AI Agent Framework, Voice & Polish**
**Goal:** Integrate Hailo-accelerated Whisper STT (<200ms latency), cloud agent reasoning (Claude/GPT-4), tool use framework, function calling, multi-step planning. Implement Quiet Mode, Power Saving Mode. System integration testing, build documentation finalization. Prepare V1 MVP for community release.

**Duration:** 3-4 weeks | **Value:** Voice-controlled AI companion, embodied agent reasoning, polished system, community-ready release

---

## Total Timeline

**Estimated Duration:** 28-39 weeks (7-10 months of weekend development)
**Milestone Cadence:** Every 1.5-4 weeks (13 milestones)
**Build-in-Public Content:** 13 major posts + monthly YouTube videos

## Key Design Philosophy Changes

✅ **Epic 2: All design upfront** - Get community feedback before printing anything expensive
✅ **Epic 5: Power before base** - Core torso + power system established before mobility
✅ **Epic 5: Heart LCD integration** - NEW 1.53" heart display (ST77916) integrated into torso (separate from Epic 1 eyes)
✅ **Epic 7: Dedicated integration epic** - Ensures proper assembly and cable management
✅ **Epic 8: Projector gets dedicated epic** - Optocoupler control, mounting, display coordination
✅ **Build flow: Design → Print → Assemble** - Each component epic includes 3D printing + assembly with feedback changes applied

