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

**Epic 3: Complete Head Assembly (Head + Ears + Neck)**
**Goal:** 3D print head housing, ear mounts, and neck gimbal (with Epic 2 feedback incorporated). Assemble complete integrated head system: dual GC9A01 round TFT eyes (240×240), mmWave presence sensor, 2-DOF articulated ears (4× Feetech SCS0009 servos), 3-DOF neck gimbal (pan ±90°, tilt ±45°, roll ±30° with 3× Feetech STS3215 servos). Shared ESP32 bus servo controller. Coordinated kinematics engine for ear+neck gestures. ESP32 firmware for expressive eye animations, sensor integration, and coordinated ear+neck movement.

**Duration:** 3-4 weeks | **Value:** Complete expressive head assembly with articulation, coordinated emotional expression across eyes+ears+neck, full upper-body mobility validated

**Note:** Audio handled by USB conference mic+speaker connected to Raspberry Pi (not embedded in head). OAK-D Pro camera deferred to Epic 10.

---

**Epic 4: ~~Ears + Neck Module - Complete Build~~ [MERGED INTO EPIC 3]**
**Status:** This epic has been merged with Epic 3 for more logical build flow. Head, ears, and neck are mechanically integrated components and are assembled together in Epic 3.

**Original Goal:** 3D print ear mounts and neck gimbal. Assemble 2-DOF articulated ears (4× Feetech SCS0009 servos) and 3-DOF neck gimbal (3× Feetech STS3215 servos).

**New Location:** See Epic 3: Complete Head Assembly

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

**Epic 9: Super Expressive Personality System**
**Goal:** Transform OLAF into a super expressive companion with research-backed multi-modal emotion system featuring 21 emotional states, organic randomization (±10-15% variation prevents repetition), emotional decay (emotions fade realistically over time), and micro-movements (continuous subtle adjustments create "living" feel) across all 7 modalities: eyes (manga-style with shape morphing, pupil variations), ears (Doberman-inspired 2-DOF positioning), neck (WALL-E head tilts), sound (R2-D2 pitch/patterns), torso screen (contextual symbols), LEDs (emotion-coded colors), and body (hoverboard lean/rock). Built on 2024 HRI research + Disney animation principles. Achieves Claptrap over-enthusiastic personality with Vector-style organic movement.

**Duration:** 5-6 weeks | **Value:** Complete transformation to engaging companion with human-like expressiveness, user-validated (>80% emotion recognition), research-backed design, foundation for AI integration (Epic 13)

---

**Epic 10: SLAM & Spatial Awareness**
**Goal:** Install and integrate OAK-D Pro camera hardware (mounting, wiring, ROS2 driver setup). Enable stereo depth sensing, ROS2 Nav2 SLAM integration (Google Cartographer or RTAB-Map), apartment map building, localization. Integration with base odometry for accurate positioning.

**Duration:** 3-4 weeks | **Value:** Visual SLAM capability, autonomous mapping, spatial understanding, navigation foundation

**Note:** OAK-D Pro camera hardware integration happens in this epic (moved from Epic 3) since this is where the camera is functionally used.

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
**Goal:** Integrate USB conference microphone + speaker hardware. Implement Hailo-accelerated Whisper STT (<200ms latency), cloud agent reasoning (Claude/GPT-4), tool use framework, function calling, multi-step planning. Implement Quiet Mode, Power Saving Mode. System integration testing, build documentation finalization. Prepare V1 MVP for community release.

**Duration:** 3-4 weeks | **Value:** Voice-controlled AI companion, embodied agent reasoning, polished system, community-ready release

---

## Total Timeline

**Estimated Duration:** 31-43 weeks (8-11 months of weekend development)
**Total Epics:** 12 epics (numbered 1-3, 5-13; Epic 4 merged into Epic 3)
**Milestone Cadence:** Every 1.5-6 weeks (12 milestones)
**Build-in-Public Content:** 12 major posts + monthly YouTube videos
**Note:** Epic 9 duration increased from 2-3 weeks to 5-6 weeks for comprehensive Super Expressive System

## Key Design Philosophy Changes

✅ **Epic 2: All design upfront** - Get community feedback before printing anything expensive
✅ **Epic 5: Power before base** - Core torso + power system established before mobility
✅ **Epic 5: Heart LCD integration** - NEW 1.53" heart display (ST77916) integrated into torso (separate from Epic 1 eyes)
✅ **Epic 7: Dedicated integration epic** - Ensures proper assembly and cable management
✅ **Epic 8: Projector gets dedicated epic** - Optocoupler control, mounting, display coordination
✅ **Epic 9: Research-backed Super Expressive System** - 21 emotional states with organic randomization, micro-movements, and multi-modal coordination based on 2024 HRI research + Disney animation principles (replaces basic 7-emotion system)
✅ **Build flow: Design → Print → Assemble** - Each component epic includes 3D printing + assembly with feedback changes applied

