# Epic List

Based on the requirements, technical architecture, and V1 scope from the brief, here's the proposed epic breakdown for Olaf V1:

## Epic Structure Overview (12 Epics for Faster Milestones)

**Epic 1: Foundation & Core I2C Communication**
**Goal:** Establish ROS2 + I2C infrastructure, power system, and prove architecture with minimal working personality (eye blink + heart beat animation). Includes body module with heart LCD, projector control, and status LEDs.

**Duration:** 2-3 weeks | **Value:** Quick win proving I2C + SPI architecture, heart display emotion feedback

---

**Epic 2: Head Module - Eyes & Sensors**
**Goal:** Complete head assembly with dual round TFT eyes (GC9A01 240×240), mmWave presence sensor, microphone array, speaker/beeper. Design and 3D print head housing with integrated sensor mounts.

**Duration:** 2-3 weeks | **Value:** Expressive eyes with presence detection, complete sensory input

---

**Epic 3: Ears - 2-DOF Articulation**
**Goal:** Implement articulated ears with 4× Feetech SCS0009 servos (2-DOF per ear), bus servo controller, and 3D printed ear mounts. Shares ESP32 with neck module.

**Duration:** 1-2 weeks | **Value:** Independent ear movement for emotional expression

---

**Epic 4: Neck - 3-DOF Gimbal**
**Goal:** Build 3-DOF neck gimbal (pan ±90°, tilt ±45°, roll ±30°) with 3× Feetech STS3215 servos, kinematics engine, and 3D printed gimbal structure. Shares ESP32 with ears module.

**Duration:** 2-3 weeks | **Value:** Full head articulation, coordinated ear+neck gestures

---

**Epic 5: Self-Balancing Base**
**Goal:** Achieve autonomous two-wheel balancing with 200Hz PID control, ODrive motor coordination, MPU6050 IMU, and kickstand servo deployment. Design and 3D print base platform.

**Duration:** 3-4 weeks | **Value:** Active mobility foundation, fall protection, SLAM-ready odometry

---

**Epic 6: Body Chassis & Integration**
**Goal:** Design and assemble body chassis housing Raspberry Pi, battery, buck converters, with complete cable management. 3D print modular chassis panels. Integrate all modules into unified physical structure.

**Duration:** 2-3 weeks | **Value:** Complete robot assembly, professional cable routing, accessible maintenance

---

**Epic 7: Basic Personality System**
**Goal:** Implement 2D emotion model (7 types × 5 intensities) with coordinated expression across eyes, ears, neck, heart, and beeps. Personality orchestrator coordinates all expression channels.

**Duration:** 2-3 weeks | **Value:** Full non-verbal personality expression, emotion synchronization

---

**Epic 8: SLAM & Spatial Awareness**
**Goal:** Enable OAK-D Pro depth sensing, ROS2 Nav2 SLAM (Google Cartographer), apartment map building, and localization. Integration with base odometry.

**Duration:** 3-4 weeks | **Value:** Autonomous mapping, spatial understanding, navigation foundation

---

**Epic 9: Autonomous Navigation & Obstacle Avoidance**
**Goal:** Implement Nav2 path planning, dynamic obstacle avoidance, and balancing coordination for safe room-to-room movement. Integration with SLAM maps.

**Duration:** 2-3 weeks | **Value:** Full autonomous mobility, collision-free navigation

---

**Epic 10: Computer Vision & Image Processing**
**Goal:** Deploy Hailo-8L accelerated object detection, person tracking, face recognition, and gesture recognition. Visual understanding for contextual interactions.

**Duration:** 2-3 weeks | **Value:** Visual intelligence, person identification, gesture-based commands

---

**Epic 11: AI Agent Framework & Voice**
**Goal:** Integrate Hailo-accelerated Whisper STT (<200ms latency), cloud agent reasoning (Claude/GPT-4), tool use framework, function calling, and multi-step planning. Conversational AI control.

**Duration:** 3-4 weeks | **Value:** Voice-controlled AI companion, embodied agent reasoning

---

**Epic 12: Floor Projection & Polish**
**Goal:** Add DLP projector with coordinated gestures, implement Quiet Mode, Power Saving Mode, system integration testing, finalize build documentation, and prepare V1 MVP for community release.

**Duration:** 2-3 weeks | **Value:** Information display, system polish, community-ready release

---

## Total Timeline

**Estimated Duration:** 26-36 weeks (6-9 months of weekend development)
**Milestone Cadence:** Every 1-3 weeks (12 milestones vs original 7)
**Build-in-Public Content:** 12 major posts + monthly YouTube videos

