# High Level Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│              INTELLIGENCE LAYER (Hybrid AI)                     │
│  Local: Whisper STT (Hailo) | Cloud: Agent (Claude/GPT-4)      │
│  • Speech Recognition (Hailo-accelerated, local <200ms)         │
│  • Agent Reasoning & Tool Use (cloud, WiFi)                     │
│  • Multi-step Planning & Context Management                     │
└──────────────────────┬──────────────────────────────────────────┘
                       │
                       │ HTTPS/REST (WiFi - Cloud Only)
                       │
┌──────────────────────▼──────────────────────────────────────────┐
│          ORCHESTRATION LAYER (Raspberry Pi 5 16GB + Hailo)      │
│  • Personality Coordination  • SLAM Navigation                  │
│  • Local AI (Whisper, Vision) • Sensor Fusion                  │
│  • State Management         • Tool Execution                    │
│  • I2C Master Controller    • OTA Server (HTTP)                 │
└──┬──────┬──────┬──────┬────────────────────────────────────────┘
   │      │      │      │
   │ 0x08 │ 0x09 │ 0x0A │ 0x0B  I2C (Wired Only)
   │      │      │      │
┌──▼────────────────┐┌─▼────┐┌─▼────┐┌▼──────────┐
│ HEAD+EARS         ││ NECK ││TORSO ││   BASE    │
│   ESP32           ││ESP32 ││ESP32 ││   ESP32   │
└┬──┬───┬────┬─────┘└──┬───┘└──┬───┘└┬─────┬────┘
 │  │   │    │HDMI     │UART   │SPI  │UART │UART
 │  │   │    │(Pi)     │Neck   │Heart│1:   │2:
 │  │   │    │Proj     │3-DOF  │2.8" │ODrv │Stand
 │  │   │    │+Focus   │+Sens× │Prnt │+IMU │Servo
 │  │   │    │         │  2    │     │     │
 │  │   │UART
 │  │   │Ears
 │  │   │2-DOF
 │  │   │(2×)
 │  │SPI
 │  │Eyes
 │  │OLED
 │GPIO
 │Proj Pwr
┌──▼───────────────────────────────────────────────────────────┐
│          MODULE LAYER (Physical Hardware)                    │
│  • HEAD+EARS MODULE (0x08):                                  │
│    - 2× OLED Eyes (128×64, SPI, 30-60 FPS animations)       │
│    - 2× Articulated Ears (2-DOF each, Feetech servos)      │
│    - Floor Projector (HDMI from Pi, controlled by ESP32):   │
│      • Power: Optocoupler switching (GPIO-controlled)       │
│      • Focus: Linear servo (PWM/UART, auto-focus)           │
│    - RGBD Camera + IMU (USB to Pi)                          │
│  • NECK MODULE (0x09):                                       │
│    - 3-DOF Servo Array (pan/tilt/roll, Feetech STS3215)    │
│    - 2× Presence Sensors (360° human detection)             │
│    - Smooth organic motion curves                           │
│  • TORSO MODULE (0x0A):                                      │
│    - 2.8" Square Display (animated beating heart, SPI)      │
│    - Thermal Printer (lists, reminders, recipes)            │
│    - Raspberry Pi Housing, Battery Pack, LED indicators     │
│  • BASE MODULE (0x0B):                                       │
│    - 2× Hoverboard BLDC Motors + ODrive (UART1)             │
│    - MPU6050 IMU (200Hz self-balancing PID)                 │
│    - Servo Kickstand (Feetech STS3215, UART2)              │
│  • PI MODULE (Orchestrator, no I2C address):                │
│    - Raspberry Pi 5 16GB + Hailo AI Kit (26 TOPS, PCIe)    │
│    - Speakerphone (USB, for voice I/O)                      │
│    - HDMI Output (video to Head+Ears projector)             │
│    - All ROS2 nodes, personality coordination               │
└──────────────────────────────────────────────────────────────┘

Communication Protocols:
━━━━━━━━ I2C: Pi ↔ 4× ESP32 modules (5-20ms latency, commands/sensor data)
- - - - - WiFi: Pi → Cloud AI APIs only (Claude/GPT-4)
━ ━ ━ ━ ━ SPI: ESP32 → OLED displays, heart display (high-speed graphics)
━·━·━·━·━ UART: ESP32 modules → servo controllers, motor controllers
─ ─ ─ ─ ─ USB: RGBD camera, Hailo AI Kit, Speakerphone → Raspberry Pi
━━━━━━━━ HDMI: Raspberry Pi → Head+Ears Module (Floor Projector video)
─·─·─·─·─ GPIO: Head+Ears ESP32 → Optocoupler → Projector Power
━·━·━·━·━ PWM/UART: Head+Ears ESP32 → Linear Servo (Projector Focus)
```
