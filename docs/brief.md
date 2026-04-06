c# Project Brief: Olaf

## OLAF- What is it?

Olaf is a **personality-first, modular robotics framework** that **brings AI agents to physical life**. Unlike existing platforms that bolt personality onto functional robots, Olaf is designed from the ground up to embody AI agents with **emotional engagement**, local AI processing (Hailo AI Kit + Whisper), and practical assistance capabilities through modular expansion. Think of it as giving Claude, GPT-4, or any AI agent a physical body with personality.


## Problem Statement

AI assistants (Alexa, Siri, ChatGPT) remain fundamentally disembodied—voices in speakers or text in apps. Meanwhile, physical robotics projects force impossible choices:

- **Commercial robots** (Boston Dynamics, Vector) - Expensive ($1000s+), closed-source, no customization
- **Educational platforms** (Poppy, Niryo) - Great for mechanics, lacking AI integration and personality
- **ROS2 robots** (Linorobot2, Andino) - Excellent navigation, zero personality or assistant features
- **Hobbyist kits** (Otto DIY) - Affordable and charming but limited capabilities, no AI
- **Closed AI robots** (HIWONDER) - Bridge AI and robotics but generic design, closed ecosystem

**The Gap:** No open-source framework integrates modular architecture + AI conversation + personality expression + practical assistance + maker-accessible approach + community ecosystem.

**The Impact:** Makers must choose personality OR functionality OR affordability. No shared framework exists for personality orchestration or emotional expression coordination. Physical AI remains fragmented across disciplines (roboticists don't integrate AI; AI practitioners don't build robots).



## Proposed Solution

A robot that **feels alive** through coordinated personality expression, **helps with daily tasks** through AI integration, and **grows with you** through modular expansion—all built transparently so anyone with soldering skills and a 3D printer can create their own companion and contribute modules back to the community.


**Three-Layer Design:**

1. **Module Layer (Physical + Embedded Intelligence)**
   - Independent hardware modules: Head (RGBD camera + OLED eyes + floor projector), Ears (2-DOF Chappie-inspired), Neck (3-DOF servo), Torso (2.8" square display showing beating heart + thermal printer), Base (self-balancing two-wheel platform)
   - Each module powered by ESP32 microcontroller acting as smart I2C slave (addresses 0x08-0x0C)
   - ESP32s contain full hardware drivers, animation engines, sensor processing, real-time control loops (Base: 200Hz self-balancing PID)
   - Modules communicate with Pi via I2C bus (5-20ms latency); NO WiFi on ESP32s for power efficiency
   - MECE Principle: Each module owns its domain exclusively, no cross-dependencies

2. **Orchestration Layer (Raspberry Pi 5 8GB + Hailo AI Kit)**
   - Central coordinator running Python-based ROS2 nodes (ROS2 Humble, LTS until 2027)
   - **All ROS2 nodes run on Pi**: personality_coordinator, ai_agent, navigation, 5× hardware driver nodes
   - **Driver nodes**: Act as I2C bridges translating ROS2 topics into I2C register writes to ESP32 modules
   - **Hailo-8L AI accelerator:** 13 TOPS for local AI inference (Whisper STT <200ms, eliminating 1-1.5s cloud STT delay)
   - Receives high-level commands from AI agent layer
   - Translates abstract agent decisions into high-level I2C commands sent to modules
   - Example: "Express excitement" → I2C writes to Head (eyes), Ears, Neck, Torso (heart) modules → ESP32s execute locally cached animations
   - Handles Google Cartographer SLAM, Nav2 path planning, SQLite conversation storage

3. **Intelligence Layer (Hybrid: Local Hailo + Cloud AI Agents)**
   - **Local AI (Hailo-accelerated):** Whisper speech-to-text (<200ms latency), real-time vision processing, fast response loops
   - **Cloud AI Agents (Claude 3.5 Sonnet for V1):** Natural language understanding, reasoning, complex decision-making
   - **Agent Framework:** Tool use, function calling, multi-step task planning (framework TBD during implementation)
   - Context maintenance: SQLite conversation history, user preferences across power cycles
   - Personality engine: Generate R2D2-style emotional responses matched to context (emotion type + intensity 1-5)
   - **Embodied intelligence:** AI agents see (RGBD camera), hear (Hailo Whisper), move (self-balancing base + SLAM), and express (coordinated eyes/ears/neck/beeps)


**Why This Will Be Different than Others?**

- **Linorobot2** proved ROS2 + ESP32 works for makers → Olaf adds AI + personality layers
- **Poppy** proved modular robotics builds community → Olaf adds practical AI assistance
- **Otto DIY** proved personality sells → Olaf adds serious capabilities
- **R2D2** proved beeps > words for emotional connection → Olaf embraces non-verbal communication
- **Your public build** provides authentic documentation → Learners follow real development, not sanitized tutorials



## MVP Scope

### Core Features (Must Have for V1)

**1. Personality Expression System**
- **OLED Eyes (2x)**: Animated expressions showing emotions (happy, curious, thinking, confused, sad)
- **Articulated Ears (2x, 2-DOF each)**: Chappie-inspired ears with independent movement for directional attention and emotion
- **Articulated Neck (3-DOF)**: Pan, tilt, and roll movements for head orientation and expressive gestures
- **Torso Heart Display (2.8" square)**: Animated beating heart that changes rhythm and style with emotional state
- **R2D2-Style Beeping**: Tone-based communication system with emotional inflection matching context
- **Coordinated Expression**: Orchestrator synchronizes eyes + ears + neck + heart + beeps for unified emotional states
- **Rationale**: Personality is the core differentiator; without engaging expression, Olaf is just another functional robot

**2. Conversational AI Integration**
- **Voice Input**: Microphone array for voice command capture
- **Cloud AI Processing**: Integration with Claude/GPT-4 API for natural language understanding
- **Context Maintenance**: Conversation history and state management
- **Function Routing**: AI decides which modules to activate based on user intent
- **Response Generation**: AI generates appropriate R2D2-style beep patterns and physical responses
- **Rationale**: Intelligence layer bridges human intent to robot actions; core to "assistant" value proposition

**3. Mobility & Navigation**
- **Self-Balancing Base**: Two-wheel inverted pendulum with 200Hz PID control on ESP32 Base module (Linux can't guarantee real-time)
- **BNO085 IMU (GY-BNO085)**: 9-axis AHRS with on-chip sensor fusion — outputs euler angles/quaternions directly at 200Hz, no complementary filter needed
- **Kickstand System**: Servo-deployed kickstand for stationary mode (RELAXED state) vs. BALANCING state
- **ODrive Motor Controller**: Closed-loop velocity control of hoverboard BLDC motors via UART from ESP32
- **RGBD Camera**: Depth sensing for obstacle detection and mapping
- **Cartographer SLAM**: Google-maintained library for autonomous apartment navigation (lighter than RTAB-Map)
- **Follow-Me Mode**: Basic person-following capability using vision + Nav2 path planning
- **Rationale**: Physical presence in the space; mobility enables "companion" vs. "desk toy" positioning; self-balancing saves floor space

**4. Information Display**
- **Floor Projector (head-mounted)**: Projects information, visualizations, and text onto floor
- **Thermal Printer (torso)**: Prints lists, reminders, recipes, and text for physical takeaways
- **Dynamic Content**: Projector displays charts, visual aids; printer outputs portable information
- **Expression Enhancement**: Projects visual elements to complement beeps (emojis, thinking dots, etc.)
- **Rationale**: Information delivery without screen-staring; unique interaction modality; physical printouts for reference

**5. Modular Architecture (Proven)**
- **Independent Modules**: Head, Ears, Neck, Torso, Base each operate as smart I2C slave peripherals
- **ESP32 per Module**: Embedded intelligence with local firmware, hardware drivers, real-time control loops
- **I2C Communication**: 5-20ms latency, deterministic timing, 400kHz-1MHz bus speed (I2C addresses 0x08-0x0C)
- **ROS2 on Pi Only**: All ROS2 nodes run on orchestrator; driver nodes translate topics to I2C register writes
- **Power Efficiency**: No WiFi on ESP32s saves ~1000mA, extends battery runtime to 2-4 hours
- **Module Testing Framework**: Each module testable in isolation via CLI tool (`olaf-test <module> <command>`)
- **OTA Updates**: V1 includes Over-The-Air firmware updates via HTTP Flask server on Pi
- **Rationale**: Validates core architectural thesis; enables weekend development; critical for future extensibility; real-time guarantee for balancing

**6. Build Documentation**
- **3D Print Files**: All STL files for physical components with print settings
- **BOM with Links**: Complete parts list with AliExpress/supplier links and pricing
- **Wiring Diagrams**: Visual guides for all electrical connections
- **Setup Guides**: Step-by-step instructions for module assembly and software installation (AI-generated)
- **Troubleshooting**: Common issues and solutions documented as encountered
- **Rationale**: Replicability is core value proposition; documentation validates build-in-public commitment


## Technical Considerations

### Platform Requirements

**Target Platform:**
- Mobile robot in indoor apartment (2-3 feet height)
- Flat surfaces: hardwood, tile, carpet
- Typical home lighting

**Hardware:**
- **Compute:** Raspberry Pi 5 16GB + Hailo AI Kit (26 TOPS), Multiple ESP32s per module
- **Sensors:** RGBD camera + IMU, mmWave presence sensor, microphone array
- **Display:** 2x OLED eyes (128x64), floor projector (head-mounted), 2.8" square display (torso heart), thermal printer (torso), status LEDs
- **Connectivity:** WiFi (cloud AI), I2C (inter-module)

**Software Stack:**
- **Orchestration:** Python 3.x, ROS2 Humble, Hailo Whisper STT, custom personality engine, cloud AI (Claude/GPT-4 API)
- **Module Firmware:** C/C++ (Arduino/ESP-IDF), micro-ROS, I2C drivers
- **Data:** SQLite (conversation history), JSON (config)
- **Infrastructure:** Self-hosted on Pi, GitHub (code/docs), YouTube/LinkedIn (content)

**Performance Targets:**
- AI response: <3s (90% of interactions)
- Navigation: 10Hz obstacle detection
- Module communication: <100ms latency
- Battery: 2-4 hours runtime

### Architecture Considerations

**System Design:**
- **Modular:** Each hardware module = independent ESP32-powered service
- **ROS2 Communication:** ESP32s act as ROS2 nodes (pub/sub topics), I2C for physical backbone
- **Orchestration:** Raspberry Pi 5 + Hailo AI Kit coordinates modules via ROS2 topics
- **Hybrid AI:** Local (Hailo Whisper STT, vision) + Cloud (Claude/GPT-4 reasoning)

**Key Integrations:**
- ROS2 topics (commands, sensor data, expressions), SLAM (RTAB-Map/Cartographer), Agent framework (tool use, function calling - TBD), Cloud AI APIs (HTTP REST), Audio I/O, GPIO/I2C drivers

**Security:**
- API keys in environment variables (not committed)
- Conversation data stays local on Pi
- Cloud calls over HTTPS
- Open-source license (MIT or Apache 2.0)

---

