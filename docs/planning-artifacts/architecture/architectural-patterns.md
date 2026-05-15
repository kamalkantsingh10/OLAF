# Architectural Patterns

- **Layered Architecture (Intelligence → Orchestration → Module)**: Separates high-level reasoning (AI agents) from real-time control (ESP32 firmware) and direct hardware control (Pi via Fusion HAT), enabling independent development and testing. _Rationale: Matches robotics best practices, clear responsibility boundaries._

- **Hybrid Control Topology (ESP32 + Pi Direct)**: Real-time critical tasks (eye animations at 60 FPS, self-balancing at 200Hz) run on dedicated ESP32 controllers, while latency-tolerant hardware (servos, LEDs, kickstand) is controlled directly by Pi via Fusion HAT and USB adapters. _Rationale: Minimizes ESP32 count (4→2), eliminates soldering, leverages pre-built interfaces while preserving real-time guarantees where needed._

- **Smart Peripheral Pattern (I2C Slaves with Embedded Intelligence)**: Head and Base ESP32 modules act as smart controllers containing full hardware drivers, animation engines, and real-time control loops (200Hz self-balancing on Base, 60 FPS eye rendering on Head), receiving only high-level semantic commands from Pi via I2C. _Rationale: Offloads real-time tasks from Linux-based Pi, reduces I2C traffic, enables autonomous execution._

- **Direct HAT Control Pattern (Pi-Controlled Peripherals)**: Neck servos, ear servos, indicator LEDs, and kickstand are controlled directly by Pi through Fusion HAT interfaces (USB serial for Waveshare adapters, WS2812 data line for LEDs, PWM for kickstand servos). ROS2 nodes manage these peripherals without intermediate microcontrollers. _Rationale: Eliminates unnecessary ESP32s, simplifies wiring (USB cables vs. custom PCBs), reduces firmware maintenance burden._

- **Hardware Abstraction via Driver Nodes**: Driver Python modules on Pi isolate hardware details (I2C registers for ESP32s, USB serial for Waveshare adapters, HAT APIs for LEDs/PWM) from application logic. _Rationale: Modules replaceable without changing high-level code, standard robotics pattern._ **Phase 2 caveat (2026-05-15):** the expression engine consumes these drivers **in-process** via Protocol-shaped adapters, not over intermediate ROS 2 topics. The per-module semantic topics (`Expression`/`EarsPose`/`Gesture`/`HeartRate`) were retired; any low-level ROS topic surface that remains is for inspection/debugging only.

- **Expression Engine Pattern (Subscribe-only renderer of canonical events)** *(Phase 2, replaces "Hybrid AI Processing")*: AI/STT/reasoning/personality run in the sibling `olaf_companion` pipeline, which publishes 4 canonical ROS 2 topics. This repo runs a single long-lived **expression engine** that subscribes only, validates the schema-3 envelope fail-fast, and renders each canonical name to pose/LED/eye/heart via a body-side `expression_map.yaml`. It never publishes back. _Rationale: single-writer-per-topic prevents drift; rendering decisions are data, owned consumer-side; the wire is the contract. See `docs/sprint-change-proposal-2026-05-15.md`._

- **Event-Driven Communication (ROS2 Pub/Sub)**: Asynchronous message passing between ROS2 nodes enables loose coupling. ESP32 modules use interrupt-driven I2C to minimize latency. _Rationale: Responsive (<500ms expression sync), power-efficient, decouples producers/consumers._

- **Repository Pattern (Conversation & Config Storage)**: SQLite abstraction layer for data persistence, encapsulates queries. _Rationale: Testable without database, future migration to cloud storage possible._

- **Reduced Firmware Footprint (2 ESP32 Modules Only)**: Only Head (eyes) and Base (balancing) require dedicated ESP32 firmware. All other hardware managed by Pi ROS2 nodes. _Rationale: Fewer firmware projects to maintain, faster iteration, reduced OTA complexity._

- **Autonomous Real-Time Control Pattern (Base Self-Balancing)**: Critical timing loops (200Hz PID balancing) run entirely on Base ESP32 with hardware timers, Pi sends only abstract navigation commands. _Rationale: Linux cannot guarantee real-time deadlines, ESP32 FreeRTOS provides hard real-time guarantees essential for stability._

- **Dual Battery Isolation (Power + Logic)**: 36V hoverboard battery powers high-current systems (motors, servos, ESP32s) via buck converters, while dedicated 2000mAh 7.4V battery powers logic systems (Pi, HATs, display, LEDs) via Fusion HAT. _Rationale: Isolates sensitive electronics from motor noise, enables independent charging, prevents brownouts during motor startup._

---
