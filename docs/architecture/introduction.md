# Introduction

This document outlines the complete fullstack architecture for **OLAF (Open Loveable AI Friend)**, an open-source modular AI companion robot. Unlike traditional software applications, OLAF is a **physical embodied AI system** that integrates embedded hardware (ESP32 modules), real-time robotic control (ROS2), local AI acceleration (Hailo AI Kit), and cloud AI reasoning (Claude/GPT-4 agents).

This architecture document serves as the single source of truth for implementing OLAF's three-layer system:
1. **Module Layer**: Independent hardware modules (Head, Ears, Neck, Projector, Base) each powered by ESP32-S3-WROOM-2 (standardized across all modules)
2. **Orchestration Layer**: Raspberry Pi 5 + Hailo AI Kit coordinating behaviors, AI, and navigation
3. **Intelligence Layer**: Hybrid local AI (Whisper STT) + cloud AI agents for reasoning and personality

The unified approach treats the embedded firmware, orchestration software, and AI integration as a cohesive fullstack system designed for AI-assisted development and maker replicability.

## Starter Template or Existing Project

**Status**: N/A - Greenfield project with pre-defined architectural constraints from PRD.

**Existing Decisions:**
- **Repository structure**: Monorepo confirmed in Technical Assumptions (already documented with clear folder structure)
- **Core technology commitments**:
  - ROS2 Humble for orchestration layer communication
  - Raspberry Pi 5 8GB + Hailo AI Kit for orchestration
  - ESP32-S3-WROOM-2 (N8R8) microcontrollers for module layer (all 5 modules standardized)
  - Python 3.10+ for orchestrator
  - C/C++ (Arduino/ESP-IDF) for module firmware
  - I2C communication between Pi and ESP32-S3 modules (WiFi for cloud API only)
  - SPI for OLED displays (high-speed graphics, leverages Octal SPI bandwidth)
  - ODrive motor controller for base mobility

**Architecture Type**: This is a **hybrid physical robotics + AI orchestration system**:
- **Physical layer**: Embedded systems (ESP32-S3 smart peripherals with edge AI capability)
- **Orchestration layer**: Raspberry Pi running ROS2 + Python
- **Intelligence layer**: Hybrid local AI (Hailo) + cloud AI agents (Claude/GPT-4 API)

## Change Log

| Date | Version | Description | Author |
|------|---------|-------------|--------|
| 2025-10-12 | v1.2 | Updated displays to GC9A01 round TFT (240×240 color), kickstand to STS3215 (30 kg·cm), added bus servo controllers (dedicated for ears, shared for neck+kickstand) | Winston (Architect Agent) |
| 2025-10-12 | v1.1 | Standardized all modules on ESP32-S3-WROOM-2 (N8R8) for enhanced performance (45 GPIO, LX7 architecture, 8MB PSRAM for edge AI) | Winston (Architect Agent) |
| 2025-10-11 | v1.0 | Initial architecture document | Winston (Architect Agent) |

---
