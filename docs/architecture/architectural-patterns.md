# Architectural Patterns

- **Layered Architecture (Intelligence → Orchestration → Module)**: Separates high-level reasoning (AI agents) from real-time control (ESP32 firmware), enabling independent development and testing. _Rationale: Matches robotics best practices, clear responsibility boundaries._

- **Smart Peripheral Pattern (I2C Slaves with Embedded Intelligence)**: ESP32 modules act as smart controllers containing full hardware drivers, animation engines, sensor processing, and real-time control loops (e.g., 200Hz self-balancing on Base module, projector power/focus control on Head+Ears module), receiving only high-level semantic commands from Pi. Pi sends HDMI video to projector but Head+Ears ESP32 controls power (GPIO → optocoupler) and focus (linear servo via PWM/UART), responding to I2C commands like `PROJECTOR_ON/OFF` and `FOCUS_NEAR/FAR`. _Rationale: Offloads real-time tasks from Linux-based Pi, reduces I2C traffic, enables autonomous execution, and follows separation of concerns (Pi=content, ESP32=hardware control)._

- **Hardware Abstraction via Driver Nodes**: ROS2 driver nodes on Pi translate between ROS2 topics (semantic) and I2C register writes (hardware-specific), isolating hardware details from application logic. _Rationale: Modules replaceable without changing high-level code, standard robotics pattern._

- **Hybrid AI Processing (Local Fast-Path + Cloud Reasoning)**: Hailo accelerator handles time-sensitive inference (Whisper STT) locally, while complex reasoning offloads to cloud LLMs. _Rationale: Balances latency (<3s requirement) with AI capability (GPT-4 quality personality)._

- **Event-Driven Communication (ROS2 Pub/Sub + I2C Interrupts)**: Asynchronous message passing between nodes, interrupt-driven I2C on ESP32s eliminates polling overhead. _Rationale: Responsive (<500ms expression sync), power-efficient, decouples producers/consumers._

- **Repository Pattern (Conversation & Config Storage)**: SQLite abstraction layer for data persistence, encapsulates queries. _Rationale: Testable without database, future migration to cloud storage possible._

- **Modular Firmware Pattern (Per-Module ESP32 Applications)**: Each module self-contained firmware project with OTA partition support. _Rationale: Independent development cycles, gradual rollout of updates, MECE principle enforcement._

- **Autonomous Real-Time Control Pattern (Base Self-Balancing)**: Critical timing loops (200Hz PID balancing) run entirely on ESP32 with hardware timers, Pi sends only abstract navigation commands. _Rationale: Linux cannot guarantee real-time deadlines, ESP32 FreeRTOS provides hard real-time guarantees essential for stability._

---
