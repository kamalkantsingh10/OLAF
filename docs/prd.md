# Olaf Product Requirements Document (PRD)

## Goals and Background Context

### Goals

- Deliver a functional V1 modular personal assistant robot with engaging personality expression (OLED eyes, articulated ears/neck, R2D2-style beeping)
- Enable conversational AI integration for natural language interaction and intelligent task routing
- Implement autonomous SLAM navigation for mobility around indoor apartment environments
- Provide practical information display via floor projection system
- Prove modular MECE architecture enabling independent weekend-sprint development of each module
- Create complete build-in-public documentation (3D files, BOM, wiring diagrams, setup guides) for community replication
- Establish credibility in physical AI space through consistent weekly content (LinkedIn posts, monthly YouTube videos)
- Enable daily meaningful interactions (2-3+ per day) making Olaf a genuine companion, not a desk toy

### Background Context

AI assistants remain trapped in screens and speakers—Alexa, Siri, ChatGPT exist only as disembodied voices. Meanwhile, existing robotics projects force impossible choices: commercial robots are expensive closed-source black boxes, educational platforms lack AI integration and personality, functional ROS2 robots are purely utilitarian, and simple hobbyist kits sacrifice all advanced capabilities. No open-source framework combines modular architecture, AI conversation, physical personality expression, practical assistance, maker-accessible budget (~$400-$1000), and community contribution ecosystem.

Olaf bridges this gap as a personality-first robotics framework. Built around a three-layer architecture (Module Layer with independent ESP32-powered hardware, Orchestration Layer on Raspberry Pi 8GB coordinating behaviors, Intelligence Layer via cloud AI), Olaf orchestrates multiple expression channels (eyes, ears, neck, beeps) to create emergent personality while proving practical value through navigation and information display. The MECE principle ensures each module owns its domain exclusively, enabling weekend development sprints, independent testing, and progressive capability enhancement.

```
┌─────────────────────────────────────────────────────────────┐
│                   INTELLIGENCE LAYER                        │
│            (Cloud AI - Claude/GPT-4 API)                    │
│  • Natural Language Understanding                           │
│  • Decision Making & Function Routing                       │
│  • Context & Conversation History                           │
└──────────────────────┬──────────────────────────────────────┘
                       │ HTTPS/REST
┌──────────────────────▼──────────────────────────────────────┐
│             ORCHESTRATION LAYER                             │
│           (Raspberry Pi 8GB - Python/ROS2)                  │
│  • Personality Coordination  • SLAM Navigation              │
│  • Module Discovery         • Sensor Fusion                 │
│  • Behavior Translation     • State Management              │
└──┬────────┬────────┬────────┬────────┬──────────────────────┘
   │        │        │        │        │
   │ ROS2   │ ROS2   │ ROS2   │ ROS2   │ ROS2 Topics
   │ Topic  │ Topic  │ Topic  │ Topic  │ Topic
   │        │        │        │        │
┌──▼───┐ ┌─▼────┐ ┌─▼────┐ ┌─▼──────┐ ┌▼─────┐
│ HEAD │ │ EARS │ │ NECK │ │PROJECT-│ │ BASE │
│      │ │      │ │      │ │  OR    │ │      │
│ESP32 │ │ESP32 │ │ESP32 │ │ ESP32  │ │ESP32 │
└──┬───┘ └──┬───┘ └──┬───┘ └───┬────┘ └──┬───┘
   │        │        │         │         │
┌──▼────────▼────────▼─────────▼─────────▼────┐
│          MODULE LAYER (Hardware)             │
│  • OLED Eyes      • 2-DOF Ears (2x)          │
│  • RGBD Camera    • 3-DOF Neck               │
│  • Microphone     • Floor Projector          │
│  • PIR Sensor     • Hoverboard Motors        │
│  • Speaker/Beeps  • LED Indicators           │
└──────────────────────────────────────────────┘
```

This project validates that physical AI companions can be both emotionally engaging and genuinely useful—built transparently so the maker community can replicate, extend, and contribute.

### Change Log

| Date | Version | Description | Author |
|------|---------|-------------|--------|
| 2025-10-10 | v1.0 | Initial PRD draft | John (PM Agent) |

---

## Requirements

### Functional

**Architecture Foundation**
1. **FR1:** Each hardware module (Head, Ears, Neck, Projector, Base) shall operate as an independent ROS2 node with its own ESP32 controller
2. **FR2:** Modules shall communicate via standardized ROS2 topics for commands, sensor data, and status updates
3. **FR3:** Each module shall be testable in standalone mode without requiring other modules to be connected
4. **FR4:** The system shall provide a test harness framework for standalone module validation supporting automated and manual testing
5. **FR5:** The system shall gracefully degrade functionality when optional modules are disconnected according to the following matrix:

| Missing Module | System Behavior |
|---------------|-----------------|
| Projector | Conversation continues; visual info delivered via beeps/speech only |
| Ears | Expression uses eyes + neck + beeps; reduced emotional range |
| Neck | Expression uses eyes + ears + beeps; stationary head orientation |
| RGBD Camera | Navigation disabled; stationary mode only; conversation functional |
| Base | Stationary companion mode; all other functions operational |

**Personality Expression System**
6. **FR6:** The OLED eye displays shall support a two-dimensional expression system with emotion types (happy, curious, thinking, confused, sad, excited, neutral) and intensity levels (1-5 scale where 1=subtle, 5=extreme)
7. **FR7:** Eye expressions shall be validated through user testing showing >80% correct emotion type identification and >70% correct intensity assessment (±1 level tolerance)
8. **FR8:** The articulated ears shall support 2 degrees of freedom each (vertical tilt, horizontal rotation), enabling independent directional movement and emotional gestures with intensity mapping (1=minimal movement, 5=exaggerated movement)
9. **FR9:** The articulated neck shall support 3 degrees of freedom (pan ±90°, tilt ±45°, roll ±30°) for head orientation and expressive movements with intensity-scaled animation speed (see FAQ for pan/tilt/roll definitions)
10. **FR10:** The system shall generate R2D2-style beeping sounds with tonal variations (pitch, duration, rhythm) mapped to emotion type and intensity level
11. **FR11:** The orchestrator shall coordinate eyes, ears, neck, and beeps into unified emotional states (e.g., "express excitement level 4" triggers synchronized high-energy ear wiggle + wide-eye animation + rapid happy beep sequence) with <500ms synchronization tolerance between expression channels
12. **FR12:** The system shall support dynamic emotion intensity escalation during conversations (e.g., enthusiasm builds from level 1→5 across multi-turn interaction)

**Conversational AI Integration**
13. **FR13:** The system shall capture voice input via microphone array for voice command detection
14. **FR14:** The system shall integrate with cloud AI API (Claude API for V1) to process natural language with conversational understanding
15. **FR15:** The system shall maintain conversation history and context across multiple interactions with persistence across power cycles
16. **FR16:** The AI layer shall route user requests to appropriate module functions based on intent analysis (e.g., "show me a recipe" → activate projector + fetch content)
17. **FR17:** The system shall implement retry logic with exponential backoff (initial 1s, max 10s, 3 attempts) for cloud AI API failures before gracefully falling back to "thinking" expression and error acknowledgment
18. **FR18:** The system shall log interaction events (timestamp, type, duration, user satisfaction signal if available) to enable usage analysis and daily interaction tracking

**Mobility & Navigation**
19. **FR19:** The mobility base shall support differential drive control using repurposed hoverboard motors
20. **FR20:** The system shall implement SLAM (Simultaneous Localization and Mapping) for autonomous navigation within mapped indoor environments using RGBD camera depth data
21. **FR21:** The system shall detect obstacles using RGBD camera depth sensing during navigation with collision avoidance behaviors

**Information Display**
22. **FR22:** The floor projector shall display text, images, charts, and dynamic content on floor surfaces
23. **FR23:** The projection system shall support AI-generated visual elements (emojis, thinking indicators, status information) to complement personality expression
24. **FR24:** The system shall detect human presence using mmWave sensor (V1 implementation) to enable context-aware behaviors and proactive interaction initiation

### Non-Functional

**Performance**
1. **NFR1:** AI response latency shall be under 3 seconds (P90) for voice interactions from command capture to action initiation
2. **NFR2:** Module communication latency between orchestrator and any module shall be under 100ms (P95)
3. **NFR3:** Navigation obstacle detection processing shall operate at minimum 10Hz for real-time responsiveness
4. **NFR4:** Battery runtime shall support 2-4 hours of continuous operation (conversation + moderate navigation)
5. **NFR5:** SLAM navigation accuracy shall achieve ±10cm positioning accuracy in mapped indoor environments
6. **NFR6:** Projection system shall display readable text (minimum 18pt equivalent) at 2 meters distance in typical indoor lighting (200-500 lux)

**Cost & Accessibility**
7. **NFR7:** Component costs shall align with configuration tiers:
   - Minimal Olaf (~$400): Head (eyes, mic, speaker) + Ears + Neck + Base (stationary/simple wheels) - Personality focus
   - Standard Olaf (~$700): Minimal + Hoverboard base + Basic RGBD camera - Adds mobility
   - Full Olaf (~$1000): Standard + Floor projector + High-quality RGBD camera - Complete feature set
8. **NFR8:** All components shall be sourceable from maker-accessible vendors (AliExpress, Amazon, Adafruit, standard electronics distributors) with BOM including direct supplier links

**Infrastructure & Security**
9. **NFR9:** The system shall operate on standard home WiFi networks (802.11n minimum, 2.4GHz or 5GHz) for cloud AI connectivity
10. **NFR10:** API keys and credentials shall be stored in environment variables and excluded from version control with template configuration files provided
11. **NFR11:** Conversation data shall be stored locally on Raspberry Pi by default; cloud transmission limited to API inference calls only
12. **NFR12:** All cloud AI communication shall use HTTPS encryption with certificate validation

**Documentation & Development**
13. **NFR13:** Complete documentation (3D print files with settings, BOM with costs, wiring diagrams, setup guides) shall be provided for each module to enable independent replication
14. **NFR14:** The codebase shall be released under MIT open-source license
15. **NFR15:** Module development shall be scoped for weekend sprint completion (1-2 weekends per module) with the following exceptions requiring multi-weekend effort: SLAM integration (estimated 3-4 weekends), Personality orchestration engine (estimated 2-3 weekends)
16. **NFR16:** The system shall support modular configuration allowing builders to omit expensive modules (projector, RGBD camera, hoverboard base) without breaking core personality and conversation functionality

---

### FAQ: Technical Terminology

**Q: What do Pan, Tilt, and Roll mean for the neck movement (FR9)?**

**A:** These are the three rotational axes that give Olaf's head natural, expressive movement:

```
                    TILT (±45°)
                   Look up/down
                        ↑
                        |
         _______________●_______________
        /               |               \
       /          [  OLAF'S  ]           \
      /           [  HEAD    ]            \
     /                 |                   \
    ←──────────────────●──────────────────→  PAN (±90°)
                       |                     Look left/right
                       |
                  ROLL (±30°)
              Tilt head sideways
            (like "aww, cute" gesture)
```

- **PAN (Yaw) ±90°**: Horizontal rotation left/right (like shaking head "no")
  - Use: Track moving person, look toward sound, scanning behavior

- **TILT (Pitch) ±45°**: Vertical rotation up/down (like nodding "yes")
  - Use: Look at floor projection, look up when thinking, excited head bobs

- **ROLL (Bank) ±30°**: Sideways tilt (like leaning head on shoulder)
  - Use: "Confused" head tilt, curious gesture, playful personality

**Example - "Express Curiosity Level 3":**
- Pan: +15° (slight turn toward sound)
- Tilt: +10° (slight upward gaze)
- Roll: +20° (head tilt "huh?")
- Eyes: Curious animation
- Ears: Both forward (alert)
- Beeps: Ascending questioning tone
- All synchronized within 500ms

**Why these ranges?**
- Pan ±90°: Nearly 180° view without full rotation (avoids wire tangling)
- Tilt ±45°: Covers floor projection and tall people without extreme angles
- Roll ±30°: Expressive without looking unstable (>45° seems unnatural)

---

## User Interface Design Goals

Based on the project context, Olaf's "UI" is fundamentally different from traditional screen-based interfaces—it's a **multi-modal physical interaction system** combining R2D2-style non-verbal communication with floor projection for information display.

### Olaf Physical Design

```
                     ╔═══════════════════╗
                   ╔═╝    EAR MODULE    ╚═╗
                  ║   (2-DOF Articulated)  ║
                  ║    [Servo + ESP32]     ║
                  ╚═╗                   ╔═╝
                    ║                   ║
        ┌───────────╨═══════════════════╨───────────┐
        │         HEAD MODULE (ESP32)               │
        │  ┌─────────────────────────────────┐      │
        │  │  [OAK-D Pro RGBD Camera]        │      │  ← Vision + Depth
        │  │   ● ▓▓▓▓▓▓▓ ●                   │      │
        │  └─────────────────────────────────┘      │
        │                                            │
        │   ╔═══════╗         ╔═══════╗             │
        │   ║ ◉   ◉ ║         ║ ◉   ◉ ║  ← OLED     │
        │   ║  \_/  ║  EYES   ║  ^_^  ║     Eyes    │
        │   ╚═══════╝         ╚═══════╝             │
        │                                            │
        │   [mmWave Sensor] [Microphone Array]      │  ← Presence + Voice
        │           [Speaker/Beeper]                 │  ← Audio Output
        └────────────────┬───────────────────────────┘
                         │
                    ╔════╧════╗
                    ║  NECK   ║  ← 3-DOF Articulation
                    ║ MODULE  ║     (Pan/Tilt/Roll)
                    ║ (ESP32) ║     Servo Array
                    ╚════╤════╝
         ┌───────────────┴───────────────┐
         │       BODY MODULE              │
         │                                │
         │   [Raspberry Pi 8GB]           │  ← Orchestrator
         │   [Battery Pack]               │
         │   [LED Status Indicators]      │
         │   [I2C Hub + Power Dist]       │
         │                                │
         │  ┌──────────────────────────┐  │
         │  │   DLP PROJECTOR          │  │  ← Floor Projection
         │  │    (ESP32 Control)       │  │     (angled downward)
         │  └──────────────────────────┘  │
         └────────────┬───────────────────┘
                      │
         ╔════════════╧════════════╗
         ║   BASE MODULE (ESP32)   ║
         ║                         ║
         ║  [Hoverboard Motors]    ║  ← Differential Drive
         ║   ●─────────────────●   ║     Mobility Platform
         ║   Left        Right     ║
         ║                         ║
         ╚═════════════════════════╝

    Height: ~50-60cm  |  Width: ~35-40cm
```

### Overall UX Vision

Olaf's user experience prioritizes **emotional connection over efficiency**. Interactions should feel like communicating with a companion rather than operating a device. The interface design emphasizes:

- **Non-verbal communication first**: R2D2-style beeps, expressive movements, and animated eyes create personality without relying on human speech
- **Physical presence**: Olaf occupies space, moves, and orients toward users—leveraging embodiment for natural interaction
- **Ambient awareness**: Context-sensitive behaviors (mmWave presence detection, conversation history) make interactions feel thoughtful rather than transactional
- **Progressive disclosure**: Information starts with simple beeps/gestures, escalates to floor projection only when detailed content is needed
- **Playful engagement**: Personality intensity escalation, dynamic expressions, and unpredictable micro-behaviors keep interactions fresh

**Design Philosophy**: "Feel alive first, be useful second"—users should emotionally connect with Olaf before evaluating utility.

### Key Interaction Paradigms

**Voice-First Input**
- Primary interaction mode: Natural language voice commands
- No wake word required when human presence detected (always listening in presence mode)
- Conversational follow-ups supported via context maintenance (FR15)
- **Quiet Mode**: Activated via voice command or time-based (10 PM - 7 AM default) - reduces beep volume by 70%, replaces audible acknowledgments with visual-only feedback (eye blinks, LED pulses), navigation disabled to prevent motor noise
- Fallback: Physical touch sensors (future enhancement) for voice-unavailable scenarios

**Multi-Channel Feedback**
- **Immediate acknowledgment** (<300ms): Eyes blink, ears perk up, brief beep (or LED pulse in Quiet Mode) to confirm heard
- **Processing indication** (while AI thinks): Animated "thinking" eyes, ears twitch, contemplative beeps (muted in Quiet Mode), optional projection of animated dots
- **Response delivery**: Coordinated expression (eyes + ears + neck + beeps) matched to content emotion and intensity
- **Information display**: DLP floor projection for text/images, accompanied by directional gestures (neck tilt toward projection, pointing ear movement)

**Spatial Interaction**
- **Approach behavior**: Olaf detects person entering room (mmWave sensor), orients head (pan) toward them, escalates greeting based on context (time since last interaction)
- **Follow-mode**: Maintains conversational distance (1.5-2m) when following, adjusts based on user movement speed
- **Stationary positioning**: When not mobile, Olaf uses head orientation to maintain "eye contact" and show attention

**Emotional Mirroring**
- AI sentiment analysis drives emotion type selection (happy user query → happy response expression)
- Conversation intensity escalation (longer conversation → higher intensity expressions up to level 5)
- Contextual appropriateness (weather report on rainy day → sad beeps level 2, recipe success → excited beeps level 4)

### Core Screens and Views

**Conceptual Interaction States** (Olaf has states, not "screens"):

1. **Idle/Ambient State**
   - Slow breathing-like eye animation (neutral emotion, intensity 1)
   - Occasional random ear twitches and neck micro-adjustments to appear "alive"
   - Soft ambient beeps every 30-60 seconds (disabled in Quiet Mode)
   - Transitions to Alert State on presence detection

2. **Alert/Listening State**
   - Eyes widen (curious emotion, intensity 2-3)
   - Ears orient forward (alert position)
   - Neck pans toward sound source
   - Ready-to-listen beep tone (or amber LED pulse in Quiet Mode)
   - Active voice capture engaged

3. **Processing/Thinking State**
   - Animated "thinking" eye pattern (thinking emotion, intensity 2-4 based on query complexity)
   - Ears twitch occasionally
   - Neck may tilt slightly (roll ±10°)
   - Contemplative beep rhythm (muted in Quiet Mode)
   - Optional floor projection: animated "..." dots

4. **Response/Expression State**
   - Full coordinated expression (emotion + intensity from AI)
   - Eyes, ears, neck, beeps synchronized <500ms (FR11)
   - Duration: 2-5 seconds for emotional expressions, sustained during speech/projection
   - Intensity level maps to animation exaggeration

5. **Information Display State**
   - Neck tilts down toward floor projection (-30° to -45°)
   - One ear may "point" toward projection area
   - Eyes show "presenting" animation (neutral emotion, intensity 2)
   - Informational beeps (tone patterns indicate content type: recipe vs weather vs reminder) - muted in Quiet Mode
   - DLP projection active with readable text/images (NFR6: 2m readable distance)

6. **Navigation State**
   - Eyes forward, scanning animation (neutral/curious emotion, intensity 1-2)
   - Ears in neutral position or perked if following person
   - Neck oriented in travel direction with micro-adjustments for obstacles
   - Movement beeps (rhythmic tones matching movement speed) - navigation disabled in Quiet Mode to prevent motor noise
   - Only active when OAK-D Pro camera present (FR5 degradation matrix)

### Accessibility

**Level: Basic Accessibility (V1), WCAG AA Target (V2+)**

**V1 Considerations:**
- **Visual**: DLP floor projection text meets minimum size/contrast (NFR6: 18pt equivalent at 2m)
- **Auditory**: All critical information available through beeps AND projection (multi-modal redundancy); Quiet Mode supports hearing-sensitive users
- **Motor**: Voice-first interaction requires no physical manipulation
- **Cognitive**: Simple, consistent interaction model (speak → Olaf responds)

**Known V1 Limitations** (deferred to V2):
- No screen reader support (physical robot, non-traditional UI)
- Beep-only mode challenging for hearing-impaired users (V2: add visual-only mode with enhanced projection/LED indicators)
- Voice-only input excludes speech-impaired users (V2: add mobile app or physical button interface)
- Floor projection requires ability to look down (V2: explore alternative display positions)

**Inclusive Design Commitment**: V2 will prioritize multi-modal alternatives ensuring core functionality accessible without relying solely on vision, hearing, or speech.

### Branding

**Aesthetic: "Retro-Futurism Meets Friendly Companion"**

- **Visual Language**: Inspired by 1970s-80s sci-fi (R2D2, Wall-E aesthetic) combined with modern minimalism
- **Color Palette**:
  - Primary: White/light gray chassis (3D printed, clean look)
  - Accent: Cyan/blue for OLED eyes (high contrast, approachable)
  - Status LEDs: Amber (processing), green (ready), red (error), blue (Quiet Mode active)
  - Projection: White light DLP (adaptable to surface colors)
- **Form Factor**: Rounded edges, no sharp corners—approachable, non-threatening silhouette (50-60cm height)
- **Sound Design**: Melodic beeps (not harsh/robotic), musical intervals for pleasantness, R2D2 as primary reference
  - Quiet Mode sound profile: Sub-bass rumbles (<100Hz) for tactile feedback without disturbing sleep
- **Movement Style**: Smooth, organic motion curves (not mechanical jerks)—easing functions for servo movements
- **Personality Archetype**: "Curious helpful friend" not "efficient servant"—okay to be playful/imperfect

**Anti-Patterns to Avoid**:
- ❌ Sleek black monolith (intimidating, corporate)
- ❌ Humanoid facial features (uncanny valley risk)
- ❌ Synthetic speech (prefer beeps for character, reserve speech for critical info only)
- ❌ Overly polished/perfect (users should feel comfortable with DIY aesthetic)

### Target Device and Platforms

**Platform: Physical Robot (Custom Hardware)**

**Primary Components**:
- **Compute**: Raspberry Pi 8GB (Raspberry Pi OS)
- **Displays**:
  - 2x OLED displays (128x64 or similar) for eyes
  - DLP projector (floor projection, 480p minimum resolution, angled downward 30-45°)
  - Status LEDs (body indicators, RGB capable for mode signaling)
- **Input**:
  - Microphone array (USB or I2S)
  - mmWave sensor (24GHz, human presence detection with direction/distance)
- **Output**: Speaker/beeper module, servo motors (ears, neck), DC motors (base)
- **Sensors**: OAK-D Pro RGBD camera (4K RGB + stereo depth, onboard AI acceleration)

**Environmental Constraints**:
- **Indoor only**: Flat surfaces (hardwood, tile, low-pile carpet)
- **Lighting**: Typical home lighting (200-500 lux)—DLP projection requires dimmer areas for best visibility
- **WiFi required**: Cloud AI dependency (NFR9)
- **Operating space**: Minimum 2m × 2m clear area for navigation testing

**Not Supported in V1**:
- Outdoor operation (weather resistance, bright sunlight)
- Stairs/multi-level navigation
- Rough terrain (gravel, thick carpet >1 inch pile)
- Offline mode (cloud AI required)

---

## Technical Assumptions

### Repository Structure

**Type: Monorepo**

The project will use a single repository containing all modules, orchestration code, documentation, and hardware designs. This simplifies dependency management and ensures all components version together.

```
olaf/
├── docs/                    # Documentation, PRD, architecture, build guides
│   ├── prd.md
│   ├── branding-guide.md
│   ├── architecture.md
│   └── build-guides/
├── hardware/               # 3D models, wiring diagrams, BOMs
│   ├── 3d-models/         # STL files organized by module
│   ├── wiring/            # Fritzing diagrams, connection tables
│   └── bom/               # Bills of materials with supplier links
├── modules/                # Module-specific firmware and configs
│   ├── head/              # ESP32 firmware: OAK-D Pro, eyes, mic, mmWave
│   ├── ears/              # ESP32 firmware: 2-DOF servo control (Feetech SCS0009)
│   ├── neck/              # ESP32 firmware: 3-DOF servo control (STS3215)
│   ├── projector/         # ESP32 firmware: DLP control
│   ├── body/              # ESP32 firmware: LED indicators, status
│   └── base/              # ESP32 firmware: ODrive motor control interface
├── orchestrator/           # Raspberry Pi Python application
│   ├── personality/       # Expression coordination engine
│   ├── ai_integration/    # Cloud AI (Claude API) interface
│   ├── navigation/        # SLAM, path planning (ROS2)
│   ├── state_management/  # Conversation context, system state
│   └── ros2_nodes/        # ROS2 node implementations
├── tests/                  # Module and integration tests
│   ├── unit/              # Per-module unit tests
│   ├── integration/       # Cross-module integration tests
│   └── fixtures/          # Test harnesses, mock data
├── tools/                  # Setup scripts, utilities
│   ├── setup/             # First-time environment setup
│   ├── calibration/       # Servo calibration, sensor tuning, ODrive config
│   └── diagnostics/       # Module health checks
├── config/                 # Configuration templates
│   ├── api-keys.template.env
│   ├── module-config.yaml
│   └── ros2-params/
└── README.md
```

**Rationale:**
- Single source of truth for all project components
- Simplifies cloning and getting started (one `git clone`)
- Coordinated versioning (hardware + software + docs evolve together)
- Easier dependency management (Python packages, ROS2 workspaces in one place)
- Aligns with educational goal (newcomers see complete project structure)

### Service Architecture

**Type: Modular Microservices within Monorepo**

Each hardware module functions as an independent microservice (ESP32 + firmware) communicating via ROS2 topics. The orchestration layer (Raspberry Pi) acts as coordinator without tight coupling to individual modules.

**Key Architectural Decisions:**

**1. Module Layer (ESP32 Microcontrollers):**
- Each module runs standalone firmware (C/C++ using Arduino framework or ESP-IDF)
- **micro-ROS integration:** Each ESP32 hosts a logical ROS2 node
  - Publishes sensor data to topics (e.g., `/olaf/head/presence`, `/olaf/base/odometry`)
  - Subscribes to command topics (e.g., `/olaf/neck/move`, `/olaf/eyes/expression`)
- **Local behavior execution:** Pre-programmed movement patterns run on ESP32 (reduces latency)
  - Example: "Express happy level 3" → ESP32 executes cached servo sequence
  - Orchestrator sends high-level commands, module handles low-level timing
- **I2C for hardware communication:** Power distribution and module discovery via I2C bus
  - Each module has unique I2C address
  - Orchestrator polls for module presence at startup

**2. Orchestration Layer (Raspberry Pi 8GB):**
- Python 3.10+ application running on Raspberry Pi OS (Debian-based)
- **ROS2 Humble or Iron:** Communication backbone for all modules
  - Orchestrator runs master ROS2 nodes for personality, navigation, AI integration
  - Bridges cloud AI decisions to module-level commands
- **Personality Coordination Engine:** 
  - Receives emotion type + intensity from AI layer
  - Translates to synchronized module commands (eyes, ears, neck, beeps)
  - Ensures <500ms coordination (FR11)
- **SLAM Navigation:** Likely using **RTAB-Map** or **Cartographer** with OAK-D Pro depth data
  - Real-time mapping and localization
  - Path planning for autonomous movement
  - Obstacle avoidance using depth sensing
- **SQLite for local storage:**
  - Conversation history and context
  - Interaction logs (FR18)
  - Module configuration and calibration data

**3. Intelligence Layer (Cloud AI - Claude API):**
- RESTful API integration over HTTPS
- **Anthropic Claude API** for V1 (standardize on single provider)
  - Natural language understanding
  - Intent classification and function routing
  - Sentiment analysis → emotion type + intensity mapping
- **Conversation context management:**
  - Orchestrator maintains message history locally
  - Sends rolling context window to API (last N messages + system prompt)
  - Persists to SQLite across power cycles (FR15)
- **Retry logic:** Exponential backoff (1s, 2s, 4s, max 10s, 3 attempts) per FR17
- **Graceful degradation:** On API failure, fall back to "thinking" expression + cached responses

**Communication Flow Example:**

```
User speaks: "Hey Olaf, what's the weather?"
    ↓
1. HEAD module (mmWave + Mic) detects presence, captures audio
    ↓ ROS2 topic: /olaf/head/audio_data
2. Orchestrator receives audio, sends to Cloud AI (Claude API)
    ↓ HTTPS POST with audio transcription
3. Claude API responds: Intent=weather_query, Emotion=curious, Intensity=2
    ↓
4. Orchestrator:
   - Queries weather API (local fetch)
   - Generates coordinated expression (curious level 2)
   - Sends commands:
       /olaf/eyes/expression → "curious:2"
       /olaf/ears/position → "forward_alert"
       /olaf/neck/move → "slight_tilt"
       /olaf/projector/display → [weather data formatted]
    ↓
5. Modules execute simultaneously (<500ms sync)
    ↓
6. User sees: Olaf tilts head curiously, ears forward, eyes animate,
              projects weather on floor, beeps in curious pattern
```

**Rationale:**
- **Modularity:** Individual modules testable/replaceable without affecting others (FR3, FR4)
- **Responsiveness:** Local firmware execution reduces latency (NFR2: <100ms)
- **Scalability:** Easy to add new modules (just publish/subscribe to topics)
- **Fault tolerance:** Module failures don't crash orchestrator (graceful degradation per FR5)
- **Cloud AI:** Leverages state-of-the-art models without local compute constraints (NFR1: <3s response)

### Testing Requirements

**Type: Hybrid Testing Strategy (Unit + Integration + Manual)**

**Unit Testing:**
- **Module firmware:** Each ESP32 module has standalone test suite
  - Servo range validation (Feetech SCS0009 ears: 2-DOF, STS3215 neck: 3-DOF with pan ±90°, tilt ±45°, roll ±30°)
  - Sensor data integrity (OAK-D Pro depth accuracy, mmWave presence detection)
  - Expression rendering (OLED eyes display correct animations)
- **Orchestrator components:** Python unit tests using `pytest`
  - Personality engine logic (emotion + intensity → module commands)
  - AI integration (mocked API responses, intent parsing)
  - Navigation algorithms (path planning on simulated maps)
- **Coverage target:** 70%+ for critical paths (personality, navigation, AI)

**Integration Testing:**
- **Module-to-orchestrator:** ROS2 topic communication validation
  - Latency tests (NFR2: <100ms module communication)
  - Command-response verification (send `/olaf/neck/move`, confirm STS3215 servo movement)
  - Graceful degradation tests (disconnect module mid-operation, verify FR5 behavior)
- **End-to-end workflows:** Simulated user interactions
  - Voice command → AI processing → expression coordination → projection display
  - Navigation scenarios (move to point, avoid obstacle, follow person) with ODrive motor control
  - Multi-turn conversations with context persistence

**Manual/Physical Testing:**
- **Personality validation:** User testing per FR7 (>80% emotion ID, >70% intensity assessment)
  - Show 20 random expressions to 5+ testers
  - Record identification accuracy, adjust animations based on feedback
- **Navigation accuracy:** SLAM precision testing per NFR5 (±10cm)
  - Mark waypoints in apartment, measure Olaf's arrival accuracy
  - Test on different floor surfaces (hardwood, tile, carpet)
- **Battery runtime:** Continuous operation test per NFR4 (2-4 hours)
  - Full charge → idle/conversation/moderate navigation → measure time to shutdown
  - Optimize power consumption based on results

**Test Harness (FR4):**
- **Module standalone test tool:** CLI utility for manual testing
  - `olaf-test neck --pan 45` → Commands STS3215 neck servo to pan 45° for visual verification
  - `olaf-test ears --tilt 30` → Commands SCS0009 ear servos to tilt 30°
  - `olaf-test eyes --emotion happy --intensity 3` → Renders expression
  - `olaf-test base --odrive-velocity 0.5` → Commands ODrive motors at 0.5 m/s
  - Logs module responses, validates ROS2 topic publishing
- **CI/CD Integration (V2):** Automated testing on GitHub Actions
  - V1: Manual test execution before major commits
  - V2: Full CI pipeline with hardware-in-the-loop testing (if feasible)

**Rationale:**
- **Unit tests:** Catch regressions early in development (especially personality engine complexity)
- **Integration tests:** Validate ROS2 communication and latency requirements
- **Manual tests:** Physical robot behavior (expression recognition, navigation) requires human assessment
- **Hybrid approach:** Balances automated validation with real-world performance verification

### Additional Technical Assumptions and Requests

**Power Management:**
- **Battery:** Hoverboard battery pack (36V, typically 4-10Ah depending on hoverboard model)
  - **Voltage:** 36V nominal (10S Li-ion, 42V fully charged, 30V cutoff)
  - **Capacity:** Estimated 4000-8000mAh (depends on salvaged hoverboard model)
  - **Expected runtime:** 2-4 hours continuous operation (NFR4) depending on capacity and usage
- **Power Distribution:**
  - **36V → 12V Buck Converter (10A):** Powers Raspberry Pi 8GB via secondary 12V→5V converter, ODrive motor controller, DLP projector
  - **36V → 5V Buck Converter (10A):** Powers ESP32 modules, servos (Feetech SCS0009, STS3215), OAK-D Pro camera, sensors
  - Both converters rated for 10A continuous to handle peak loads
- **Charging:** Hoverboard's original charging circuit board (BMS - Battery Management System)
  - Retains hoverboard main circuit board for integrated charging functionality
  - Standard hoverboard charger (42V, 2A typical) used for recharging
  - BMS handles cell balancing, over-charge/discharge protection
- **Battery Monitoring:**
  - Voltage monitoring via ADC (ESP32 or Raspberry Pi GPIO)
  - Low-battery warning at 32V (≈20% capacity): LED indicator + beep alert
  - Auto-shutdown at 30V (battery cutoff) to prevent over-discharge damage
  - Display remaining % via calculation: (V_current - 30) / (42 - 30) × 100

**Servo Specifications:**
- **Ears (2x modules, 2-DOF each):** Feetech SCS0009 serial bus servos
  - **Protocol:** Serial communication (likely half-duplex TTL UART)
  - **DOF per ear:** Vertical tilt + horizontal rotation = 2 servos per ear module
  - **Total servos:** 4x SCS0009 (2 ears × 2 DOF)
  - **Torque:** Check spec sheet (typically 8-12 kg·cm for this series)
  - **Speed:** Adjustable via protocol (intensity mapping: level 1=slow, level 5=fast)
  - **Voltage:** 5V (powered by 36V→5V buck converter)
- **Neck (1x module, 3-DOF):** Feetech STS3215 serial bus servos
  - **Protocol:** Serial communication (ST series, likely compatible with SCS protocol)
  - **DOF:** Pan, tilt, roll = 3 servos total
  - **Torque:** Higher than SCS0009 (typically 15-20 kg·cm for neck support)
  - **Range:** Pan ±90°, tilt ±45°, roll ±30° per FR9
  - **Voltage:** 5V or 6V (check spec, use 5V from buck converter)
  - **Position feedback:** Built-in position sensing for closed-loop control

**Motor Control (Base Module):**
- **Motors:** Hoverboard brushless DC (BLDC) hub motors (2x, differential drive)
  - **Type:** Typically 350W per wheel (700W total peak power)
  - **Voltage:** 36V nominal
  - **Encoder:** Hall sensors built into motors (for position/velocity feedback)
- **Controller:** ODrive motor controller
  - **Model:** ODrive v3.6 or ODrive S1 (V1: likely v3.6 for cost, V2: S1 for compactness)
  - **Channels:** Dual-channel (controls both left and right motors)
  - **Interface:** UART, USB, or CAN bus communication with Raspberry Pi
  - **Power:** Powered directly from 36V hoverboard battery
  - **Functionality:**
    - Closed-loop velocity control (SLAM navigation requires precise speed control)
    - Encoder feedback for odometry (wheel rotation → distance traveled)
    - Current limiting to prevent battery over-draw
  - **ROS2 Integration:** ESP32 base module or Raspberry Pi directly communicates with ODrive via serial
    - Publishes `/olaf/base/odometry` (wheel encoder data for SLAM)
    - Subscribes to `/olaf/base/velocity_cmd` (target linear/angular velocity from navigation)

**Development Environment:**
- **Primary OS:** Linux (Ubuntu 22.04 LTS or Raspberry Pi OS for orchestrator dev)
  - ROS2 Humble natively supported on Ubuntu 22.04
  - Cross-compilation for ESP32 via PlatformIO or Arduino CLI
- **IDE:** VS Code with extensions:
  - PlatformIO (ESP32 firmware development)
  - Python (orchestrator development)
  - ROS extension (topic monitoring, launch files)
- **Version Control:** Git with GitHub for hosting
  - Branch strategy: `main` (stable), `develop` (active work), feature branches
  - Semantic versioning for releases (v1.0.0, v1.1.0, etc.)

**Networking:**
- **WiFi:** 2.4GHz preferred (longer range, better obstacle penetration than 5GHz)
  - Raspberry Pi 8GB has dual-band WiFi (802.11ac)
  - ESP32 modules: 2.4GHz only (sufficient for local ROS2 communication)
- **Local network:** Orchestrator and modules communicate over local WiFi (no internet required for ROS2)
- **Cloud connectivity:** Orchestrator requires internet for Claude API calls (NFR9)
- **Fallback:** If WiFi drops, Olaf enters "offline mode" (stationary, no AI, pre-programmed expressions only)

**Security Considerations:**
- **API Key Management:**
  - Store in `.env` file (excluded via `.gitignore`)
  - Template provided: `config/api-keys.template.env`
  - Load via Python `dotenv` library at runtime
  - **Never commit keys to repository** (use pre-commit hooks to prevent)
- **Conversation Privacy:**
  - All conversation data stored locally on Raspberry Pi (SQLite database)
  - Cloud AI API calls: Send only necessary context (last 5-10 messages, no full history)
  - Option to disable logging (privacy mode for sensitive conversations)
  - HTTPS encryption for all API calls (NFR12)
- **Network Security:**
  - Raspberry Pi firewall: Allow only outbound HTTPS, ROS2 ports on local network
  - No remote access by default (SSH disabled or key-only auth)
  - Optional: VPN for remote debugging (not required for V1)

**Third-Party Libraries & APIs:**
- **ROS2 Packages:**
  - `micro_ros_agent` (ESP32 to ROS2 bridge)
  - `rtabmap_ros` or `cartographer_ros` (SLAM)
  - `navigation2` (path planning, obstacle avoidance)
  - `cv_bridge` (OAK-D Pro image processing)
  - `odrive_ros2` (ODrive motor controller integration, community package or custom node)
- **Python Libraries:**
  - `anthropic` (Claude API client)
  - `rclpy` (ROS2 Python bindings)
  - `opencv-python` (computer vision, if needed beyond OAK-D Pro SDK)
  - `pygame` or `pyaudio` (sound generation for beeps)
  - `python-dotenv` (environment variable management)
  - `pytest` (unit testing)
- **ESP32 Libraries:**
  - `micro_ros_arduino` (ROS2 node on ESP32)
  - `Adafruit_SSD1306` (OLED eye display driver)
  - `FeetechServo` or custom library (SCS0009, STS3215 serial servo control)
  - Custom libraries for DLP projector control (vendor-specific)

**Hardware Sourcing:**
- **Primary Vendors:**
  - AliExpress (servos, sensors, ESP32 boards) - budget-friendly
  - Adafruit (quality sensors, well-documented modules) - reliability
  - Amazon (Raspberry Pi, buck converters, quick shipping) - convenience
- **Specific Components:**
  - **OAK-D Pro:** Luxonis official site or authorized distributors (~$300)
  - **DLP Projector:** TI DLP LightCrafter or compatible mini projector module (~$150-250)
  - **mmWave Sensor:** 24GHz radar module (e.g., DFRobot SEN0395 or similar, ~$20-40)
  - **Hoverboard (salvage):** Used hoverboards (eBay, local classifieds, ~$50-100 for broken units)
    - Extract: Battery pack, motors with wheels, BMS/charging circuit, motor controller (if not using ODrive)
  - **ODrive:** ODrive Robotics official site (~$120-200 depending on version)
  - **Feetech SCS0009:** AliExpress, RobotShop (~$15-25 each, need 4x)
  - **Feetech STS3215:** AliExpress, RobotShop (~$25-40 each, need 3x)
  - **Buck Converters (36V→12V, 36V→5V, 10A):** AliExpress, Amazon (~$10-20 each)
- **BOM Maintenance:** Keep supplier links updated in `hardware/bom/` (NFR8)

**AI-Assisted Development:**
- **Expectation:** 100% of code and documentation leverages AI assistance (Claude, GPT-4, GitHub Copilot)
  - Code generation: Boilerplate, module templates, ROS2 nodes, ODrive integration
  - Documentation: Build guides, API docs, servo calibration procedures, troubleshooting (auto-generated with human review)
  - Debugging: AI-assisted error analysis, solution suggestions
  - Optimization: Performance tuning, code refactoring recommendations
- **Workflow:**
  - Describe desired functionality to AI
  - Review generated code/docs for correctness
  - Iterate with AI on refinements
  - Commit final human-reviewed version
- **Rationale:** Aligns with brief's emphasis on AI-assisted workflow, accelerates development, educational for learners

**Deployment Strategy:**
- **V1 Target:** Self-hosted on Raspberry Pi (no external servers, no containers)
  - Direct Python execution via systemd service (auto-start on boot)
  - ROS2 launch files for module orchestration
  - Manual updates: SSH into Pi, `git pull`, restart services
- **V2+ Considerations:**
  - Docker containers for orchestrator (easier deployment, dependency isolation)
  - Over-the-air (OTA) updates for ESP32 firmware
  - Web dashboard for monitoring/configuration (Flask or FastAPI backend)

---

**Rationale & Assumptions:**

**Key Technical Decisions:**

1. **Monorepo:** Simplifies onboarding (one clone), ensures versioning consistency, educational clarity
2. **ROS2 as communication backbone:** Industry-standard robotics framework, rich ecosystem, supports modularity
3. **micro-ROS for ESP32:** Enables ESP32 modules to act as first-class ROS2 nodes without Raspberry Pi bottleneck
4. **Claude API (single provider for V1):** Reduces complexity vs. multi-provider support, can add GPT-4 in V2
5. **Hoverboard salvage:** Reuses battery + motors + BMS = cost-effective, eco-friendly, proven components
6. **ODrive motor controller:** Professional-grade BLDC control, ROS2 integration available, supports SLAM odometry requirements
7. **Feetech serial servos:** Daisy-chainable (reduces wiring), position feedback, programmable speed (intensity mapping)
8. **Dual buck converters (36V→12V, 36V→5V):** Clean power distribution, isolates high-power (motors) from low-power (logic) circuits
9. **Hybrid testing:** Automated tests catch regressions, manual tests validate physical robot behavior
10. **Local-first data:** Privacy-conscious, reduces cloud dependency, faster response for non-AI tasks
11. **AI-assisted development:** Dogfooding modern AI tools, accelerates development, teaches learners AI workflows

**Assumptions Made:**
- Hoverboard battery (36V, 4-8Ah) provides 2-4 hours runtime with efficient power management
- ODrive can interface with Raspberry Pi or ESP32 via UART/USB for motor control
- Feetech servos (SCS0009, STS3215) provide sufficient torque for ear/neck articulation under head weight
- 10A buck converters handle peak current draw (Pi: ~3A, servos: ~2A each × 7 servos peak = ~14A total, staggered usage keeps average <10A per rail)
- Raspberry Pi 8GB sufficient for orchestrator workload (ROS2 + Python + SLAM + SQLite)
- ESP32 boards handle micro-ROS + local firmware execution without performance issues
- Home WiFi networks have stable <50ms latency for local ROS2 communication
- Claude API latency + network round-trip achieves <3s total (P90) per NFR1
- OAK-D Pro depth accuracy meets ±10cm SLAM requirement (NFR5)
- Hoverboard BMS safely handles charging without modifications (standard 42V charger compatible)

**Open Questions for Architect:**
- Should we use ROS2 Humble (LTS until 2027) or Iron (newer, but shorter support)?
- SLAM library choice: RTAB-Map (feature-rich) vs. Cartographer (Google-maintained, simpler)?
- ODrive communication: Direct from Raspberry Pi (simpler) or via ESP32 base module (more modular)?
- Servo daisy-chaining: Single serial bus for all 7 servos or separate buses per module (ears, neck)?
- Power sequencing: Should Raspberry Pi boot before or after motor controller initialization?
- mmWave sensor interface: I2C, UART, or GPIO communication with head ESP32?
- DLP projector control: HDMI from Raspberry Pi (simple) or ESP32-controlled (modular)?
- Buck converter placement: Centralized in body or distributed per module (trade-off: wiring complexity vs. fault isolation)?

---

## Epic List

Based on the requirements, technical architecture, and V1 scope from the brief, here's the proposed epic breakdown for Olaf V1:

### Epic Structure Overview

**Epic 1: Foundation, Infrastructure & Minimal Personality**
**Goal:** Establish project infrastructure, power system, ROS2 communication, and deliver a minimal working personality expression (single eye blink + beep) proving the three-layer architecture works end-to-end.

**Epic 2: 3D Design & Physical Structure**
**Goal:** Design all mechanical components in OnShape (head housing, ear mounts, neck gimbal, body chassis, base platform), produce printable STL files, print and assemble physical structure, validate mechanical ranges and fitment.

**Epic 3: Complete Personality Expression System**
**Goal:** Expand minimal personality to full two-dimensional emotion system (7 types × 5 intensities) with coordinated expression across eyes, ears, neck, and beeps.

**Epic 4: Conversational AI Integration**
**Goal:** Enable voice-based interaction with cloud AI (Claude API) for natural language understanding, intent routing, and emotion-driven responses with context persistence.

**Epic 5: Autonomous Navigation & Mobility**
**Goal:** Implement SLAM-based navigation using OAK-D Pro depth sensing and ODrive motor control for autonomous apartment movement with obstacle avoidance.

**Epic 6: Information Display & Projection**
**Goal:** Integrate DLP floor projection system for displaying AI-generated content with coordinated physical gestures.

**Epic 7: System Integration, Polish & Cross-Cutting Features**
**Goal:** Complete end-to-end integration, implement Quiet Mode, mmWave presence detection, Power Saving Mode, finalize build documentation, conduct user testing, and prepare for community release.

---

_See `.content/epic-content-plan.md` for detailed content strategy aligned with each epic (LinkedIn posts, YouTube videos, GitHub activity)_

---

## Epic 1: Foundation, Infrastructure & Minimal Personality

**Epic Goal:** Establish the complete development foundation including repository structure, ROS2 environment, power distribution system (hoverboard battery + buck converters), and basic ESP32-to-Raspberry Pi communication. Deliver a minimal working personality—a single OLED eye that can blink and display one simple emotion (neutral), plus a test beep—proving the three-layer architecture (Intelligence → Orchestration → Module) works before building complex features or mechanical structures.

**Duration:** 2-3 weeks (Weeks 1-2)

**Prerequisites:** None (first epic)

**Value Delivered:** Quick win proving architecture viability, enables all future development, provides "Olaf blinks at you!" moment within first 2-3 weekends.

---

### Story 1.1: Repository Setup & Project Structure

**As a** developer building Olaf,  
**I want** a well-organized monorepo with clear structure and documentation,  
**so that** I can clone the project, understand the layout, and start contributing immediately.

#### Acceptance Criteria:

1. Repository created on GitHub with name `olaf` under appropriate account
2. Directory structure matches Technical Assumptions specification:
   ```
   olaf/
   ├── docs/
   ├── hardware/
   ├── modules/
   ├── orchestrator/
   ├── tests/
   ├── tools/
   ├── config/
   └── README.md
   ```
3. README.md contains:
   - Project overview (personality-first AI companion)
   - Architecture diagram (Intelligence → Orchestration → Module layers)
   - Quick start instructions (clone, dependencies, setup)
   - Link to PRD (`docs/prd.md`)
   - Link to branding guide (`docs/branding-guide.md`)
   - MIT License badge
4. `.gitignore` configured to exclude:
   - Python `__pycache__/`, `*.pyc`
   - ROS2 build artifacts (`build/`, `install/`, `log/`)
   - Environment files (`.env`)
   - IDE configs (`.vscode/`, `.idea/`)
5. `config/api-keys.template.env` created with placeholder variables:
   ```
   ANTHROPIC_API_KEY=your_claude_api_key_here
   WEATHER_API_KEY=your_weather_api_key_here
   ```
6. Initial commit pushed with message: "Initial repository structure and project documentation"
7. GitHub repository description set: "Open-source modular AI companion robot with personality-first design"
8. Topics tagged: `robotics`, `ros2`, `ai`, `open-source`, `esp32`, `physical-ai`

**Dependencies:** None

**Estimated Effort:** 2-4 hours

---

### Story 1.2: Power System Assembly & Validation

**As a** robot builder,  
**I want** a reliable power distribution system using salvaged hoverboard components,  
**so that** all modules receive stable voltage and I can safely charge the battery.

#### Acceptance Criteria:

1. **Battery Selection:**
   - Hoverboard battery salvaged (36V nominal, 4-10Ah capacity documented)
   - Battery voltage measured: 36-42V (confirms healthy cells)
   - BMS (Battery Management System) circuit board identified and retained

2. **Buck Converter Installation:**
   - 36V → 12V buck converter (10A) installed and secured
   - 36V → 5V buck converter (10A) installed and secured
   - Converter output voltages measured under no-load: 12V ±0.2V, 5V ±0.1V

3. **Load Testing:**
   - 12V rail tested with 3A load (Raspberry Pi equivalent): voltage stable 12V ±0.3V
   - 5V rail tested with 2A load (servos + ESP32s): voltage stable 5V ±0.2V
   - Both converters loaded simultaneously: no voltage sag >5%

4. **Battery Monitoring:**
   - Voltage divider circuit connected to ADC input (Raspberry Pi GPIO or ESP32)
   - ADC reads battery voltage accurately (±0.5V of multimeter measurement)
   - Low-battery threshold set at 32V (≈20% capacity)

5. **Charging Circuit:**
   - Hoverboard charging port retained and accessible
   - Standard 42V hoverboard charger connects successfully
   - Charging tested: Battery voltage rises from 36V → 42V over 2-4 hours
   - BMS balances cells (verified by no error LEDs on BMS board)

6. **Safety:**
   - Fuse or circuit breaker installed on battery main output (30-40A rating)
   - Power switch (physical toggle) installed for emergency cutoff
   - Wiring uses appropriate gauge: 18-20 AWG for power rails, 22-24 AWG for signals
   - Connectors secure (JST-XH or screw terminals, no loose Dupont on power)

7. **Documentation:**
   - Wiring diagram created (Fritzing or hand-drawn, photographed)
   - BOM updated with battery source, buck converters, connectors
   - Power system photo uploaded to `hardware/wiring/power-system.jpg`

**Dependencies:** Story 1.1 (repo to commit documentation)

**Estimated Effort:** 6-10 hours (includes salvaging hoverboard, wiring, testing)

---

### Story 1.3: Raspberry Pi ROS2 Setup

**As a** robotics developer,  
**I want** ROS2 Humble installed and configured on Raspberry Pi 8GB,  
**so that** I can run the orchestration layer and communicate with ESP32 modules via micro-ROS.

#### Acceptance Criteria:

1. **OS Installation:**
   - Raspberry Pi OS (64-bit, Debian Bookworm-based) flashed to SD card (minimum 32GB)
   - Pi boots successfully, SSH enabled (or direct monitor/keyboard access)
   - WiFi configured (2.4GHz network for ESP32 compatibility)
   - Static IP assigned (optional but recommended for development)

2. **ROS2 Installation:**
   - ROS2 Humble installed following official docs (apt packages)
   - Environment sourced: `source /opt/ros/humble/setup.bash` in `.bashrc`
   - Verification: `ros2 --version` returns "ros2 doctor: humble"

3. **micro-ROS Agent:**
   - `micro_ros_agent` package installed via apt or built from source
   - Agent launches successfully: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
   - No errors in agent terminal output

4. **Development Tools:**
   - Python 3.10+ installed (comes with Raspberry Pi OS)
   - `pip` installed, virtual environment created: `python3 -m venv ~/olaf_env`
   - ROS2 Python bindings available: `import rclpy` works in Python

5. **Network Configuration:**
   - Raspberry Pi and development laptop on same network
   - Ping successful between Pi and laptop
   - ROS2 discovery working: `ros2 topic list` shows topics from laptop

6. **Repository Clone:**
   - Repository cloned to Raspberry Pi: `git clone <repo_url> ~/olaf`
   - Directory structure visible: `ls ~/olaf` shows all folders

7. **Documentation:**
   - Setup script created: `tools/setup/install_ros2_humble.sh`
   - README updated with ROS2 setup instructions
   - Troubleshooting notes added for common issues (WiFi config, permissions)

**Dependencies:** Story 1.1 (repo), Story 1.2 (power to boot Pi)

**Estimated Effort:** 4-6 hours (includes learning ROS2 if new)

---

### Story 1.4: Minimal Head Module - Hardware Assembly

**As a** hardware builder,  
**I want** a minimal head module with one OLED eye and ESP32,  
**so that** I can test visual feedback before building the complete dual-eye system.

#### Acceptance Criteria:

1. **Components Acquired:**
   - 1x ESP32 development board (DevKit v1 or similar)
   - 1x OLED display (128x64 pixels, I2C interface, SSD1306 driver)
   - Jumper wires (I2C: SDA, SCL, VCC, GND)
   - Breadboard or prototype PCB for temporary assembly

2. **Wiring:**
   - OLED connected to ESP32 via I2C:
     - OLED VCC → ESP32 3.3V (or 5V rail from buck converter if OLED supports it)
     - OLED GND → ESP32 GND
     - OLED SDA → ESP32 GPIO21 (default I2C SDA)
     - OLED SCL → ESP32 GPIO22 (default I2C SCL)
   - Power connection: ESP32 VIN ← 5V from buck converter (or USB for initial testing)

3. **I2C Communication Test:**
   - Upload I2C scanner sketch to ESP32
   - OLED detected at address 0x3C or 0x3D (confirms wiring)
   - Serial monitor shows "I2C device found at address 0x3C"

4. **Basic Display Test:**
   - Upload Adafruit SSD1306 example sketch (Hello World)
   - OLED displays text "Hello Olaf!"
   - Screen updates without flickering

5. **Physical Mounting (Temporary):**
   - Components secured to breadboard or cardboard base (prevents shorts)
   - Cables strain-relieved (won't pull out during testing)
   - Module can sit on desk without tipping over

6. **Documentation:**
   - Wiring photo uploaded: `hardware/wiring/head-module-minimal.jpg`
   - Component list added to BOM: ESP32, OLED, wires
   - Wiring notes documented in `modules/head/README.md`

**Dependencies:** Story 1.2 (5V power available)

**Estimated Effort:** 2-4 hours

---

### Story 1.5: Minimal Head Module - Firmware (Eye Blink)

**As a** firmware developer,  
**I want** ESP32 firmware that displays a neutral expression and blinks on command,  
**so that** I can validate the orchestrator → module communication pipeline.

#### Acceptance Criteria:

1. **Development Environment:**
   - PlatformIO or Arduino IDE configured for ESP32
   - Libraries installed:
     - `Adafruit_SSD1306` (OLED driver)
     - `Adafruit_GFX` (graphics primitives)
     - `micro_ros_arduino` (ROS2 node on ESP32)

2. **Neutral Expression Graphics:**
   - Neutral eye sprite created (simple circle with dot pupil, or peaceful closed curve)
   - Sprite stored as bitmap array in firmware: `const uint8_t neutral_eye[]`
   - Sprite renders correctly on OLED (centered, appropriate size)

3. **Blink Animation:**
   - Blink sequence defined:
     - Frame 1: Full open (neutral eye)
     - Frame 2: Half closed (top eyelid lowered)
     - Frame 3: Fully closed (horizontal line)
     - Frame 4: Half closed (opening)
     - Frame 5: Full open
   - Blink duration: 300-500ms total (fast enough to feel natural)
   - Animation plays smoothly (no visible tearing)

4. **micro-ROS Integration:**
   - ESP32 connects to micro-ROS agent running on Raspberry Pi (UDP port 8888)
   - ROS2 node created: `/olaf/head` with namespace
   - Subscribes to topic: `/olaf/head/eye_cmd` (message type: `std_msgs/String` or custom)
   - On message "blink" received → triggers blink animation

5. **Standalone Testing:**
   - Firmware uploaded to ESP32
   - ESP32 connects to WiFi (hardcoded SSID/password for now)
   - micro-ROS agent running on Pi shows: "Node '/olaf/head' connected"
   - Command line test: `ros2 topic pub /olaf/head/eye_cmd std_msgs/String "data: 'blink'"` → eye blinks

6. **Continuous Operation:**
   - Module runs continuously for 30 minutes without crashing
   - Can blink on command multiple times (tested 10x)
   - Memory leak check: ESP32 free heap stable over time

7. **Code Quality:**
   - Code committed to `modules/head/firmware/`
   - README documents build/upload process
   - Pin definitions clearly commented (I2C pins, power)

**Dependencies:** Story 1.3 (ROS2 + micro-ROS agent), Story 1.4 (hardware assembled)

**Estimated Effort:** 6-8 hours (includes micro-ROS learning curve)

---

### Story 1.6: Minimal Audio - Beeper Hardware & Test Tone

**As a** hardware builder,  
**I want** a simple beeper/speaker that can play test tones,  
**so that** I can validate audio output before implementing the full sound library.

#### Acceptance Criteria:

1. **Component Selection:**
   - Passive piezo buzzer (simplest, works with PWM) OR
   - Small speaker (3W, 8Ω) + audio amplifier module (PAM8403 or similar)
   - Choice documented with rationale (piezo = simpler, speaker = better quality)

2. **Wiring:**
   - If piezo: Connect to ESP32 GPIO pin (e.g., GPIO25) + GND
   - If speaker: Amplifier powered from 5V rail, audio input from ESP32 DAC or PWM pin
   - Wiring diagram updated in `hardware/wiring/audio-system.jpg`

3. **Test Tone Firmware:**
   - Simple Arduino sketch uploaded to ESP32 (can reuse head module ESP32 or separate one)
   - Generates 440Hz tone (A4 note) via PWM or DAC
   - Plays tone for 500ms when triggered

4. **ROS2 Integration:**
   - ESP32 subscribes to `/olaf/audio/beep_cmd` topic
   - Message format: `std_msgs/Int16` (frequency in Hz) or `std_msgs/String` ("beep")
   - On message received → plays tone at specified frequency (or default 440Hz)

5. **Testing:**
   - Command: `ros2 topic pub /olaf/audio/beep_cmd std_msgs/String "data: 'beep'"` → beep plays
   - Adjustable volume (if speaker): potentiometer on amplifier or PWM duty cycle
   - No distortion or crackling at moderate volume

6. **Integration with Head Module (Optional for Story, Required Later):**
   - If same ESP32 as head: Both eye and beep commands work independently
   - If separate ESP32: Both nodes visible in `ros2 node list`

7. **Documentation:**
   - Component added to BOM
   - Audio wiring documented
   - Firmware committed to `modules/body/firmware/` or `modules/head/firmware/` (depending on location)

**Dependencies:** Story 1.2 (power), Story 1.3 (ROS2)

**Estimated Effort:** 3-5 hours

---

### Story 1.7: Basic Orchestrator - Blink & Beep Coordinator

**As a** software developer,  
**I want** a minimal orchestrator node that sends coordinated blink + beep commands,  
**so that** I can prove the three-layer architecture (orchestrator → modules) works.

#### Acceptance Criteria:

1. **Orchestrator Setup:**
   - Python ROS2 node created: `orchestrator/minimal_demo.py`
   - Node name: `/olaf/orchestrator`
   - Imports: `rclpy`, `std_msgs.msg`

2. **Publishers Created:**
   - Publisher to `/olaf/head/eye_cmd` (for eye blink)
   - Publisher to `/olaf/audio/beep_cmd` (for beep)

3. **Coordinated Action Function:**
   - Function `blink_and_beep()` implemented:
     ```python
     def blink_and_beep(self):
         # Publish eye blink command
         self.eye_pub.publish(String(data='blink'))
         # Publish beep command (simultaneously or with small delay)
         self.beep_pub.publish(String(data='beep'))
     ```

4. **Test Interface:**
   - Timer-based trigger: Every 5 seconds, call `blink_and_beep()` (for continuous demo)
   - OR service call: Create ROS2 service `/olaf/test_blink_beep` that triggers action on request

5. **Execution:**
   - Launch orchestrator: `ros2 run olaf_orchestrator minimal_demo`
   - Verify in `ros2 node list`: `/olaf/orchestrator` appears
   - Verify publishers: `ros2 topic info /olaf/head/eye_cmd` shows orchestrator as publisher

6. **End-to-End Test:**
   - All three nodes running: micro-ROS agent + head module + audio module + orchestrator
   - Orchestrator triggers blink_and_beep
   - **Expected behavior:** Eye blinks AND beep plays within <500ms of each other
   - Latency measured (rough estimate via stopwatch or logs): <500ms between commands

7. **Logging:**
   - Orchestrator logs actions: `self.get_logger().info("Triggered blink and beep")`
   - Logs visible in terminal during execution

8. **Code Quality:**
   - Code committed to `orchestrator/minimal_demo.py`
   - Launch file created: `orchestrator/launch/minimal_demo.launch.py` (optional but nice)
   - README updated with orchestrator run instructions

**Dependencies:** Story 1.5 (eye blink firmware), Story 1.6 (beep firmware)

**Estimated Effort:** 4-6 hours

---

### Story 1.8: Test Harness CLI - Manual Module Testing

**As a** developer or tester,  
**I want** a command-line tool to manually test individual modules,  
**so that** I can debug issues without running the full orchestrator.

#### Acceptance Criteria:

1. **CLI Tool Created:**
   - Python script: `tools/diagnostics/olaf-test`
   - Shebang line: `#!/usr/bin/env python3`
   - Executable permission: `chmod +x tools/diagnostics/olaf-test`

2. **Commands Implemented:**
   - `olaf-test blink` → Publishes blink command to `/olaf/head/eye_cmd`
   - `olaf-test beep` → Publishes beep command to `/olaf/audio/beep_cmd`
   - `olaf-test list` → Lists all active ROS2 nodes (shows module connectivity)

3. **ROS2 Integration:**
   - Script uses `rclpy` to publish messages
   - Creates temporary node: `/olaf/test_harness`
   - Publishes message, waits 1 second, shuts down cleanly

4. **Usage:**
   - From terminal: `./tools/diagnostics/olaf-test blink` → eye blinks
   - From terminal: `./tools/diagnostics/olaf-test beep` → beep plays
   - Help message: `./tools/diagnostics/olaf-test --help` shows available commands

5. **Error Handling:**
   - If ROS2 not running: Clear error message "ROS2 not initialized. Is ros2 daemon running?"
   - If invalid command: "Unknown command. Use --help for usage."

6. **Documentation:**
   - Tool usage documented in `tools/diagnostics/README.md`
   - Main README links to test harness documentation

**Dependencies:** Story 1.5 (modules to test), Story 1.7 (orchestrator as reference)

**Estimated Effort:** 3-4 hours

---

### Story 1.9: Battery Monitoring & Low-Battery Warning

**As a** robot operator,  
**I want** visual and audio alerts when battery is low,  
**so that** I can recharge before unexpected shutdown damages the battery or loses work.

#### Acceptance Criteria:

1. **Voltage Sensing:**
   - ADC input reads battery voltage via voltage divider (36V → 3.3V range)
   - Voltage reading published to ROS2 topic: `/olaf/battery/voltage` (Float32, in volts)
   - Reading accuracy: ±0.5V compared to multimeter

2. **Battery Percentage Calculation:**
   - Formula: `percentage = (V_current - 30) / (42 - 30) * 100`
   - 42V = 100%, 36V ≈ 50%, 30V = 0%
   - Percentage published to `/olaf/battery/percentage` (Int16, 0-100)

3. **Low-Battery Threshold:**
   - Threshold: 32V (≈20% capacity)
   - When voltage drops below 32V: Low-battery warning triggered

4. **Warning Indicators:**
   - **Visual:** LED on body (if available) blinks red, OR eye displays warning symbol
   - **Audio:** Three beeps (440Hz, 200ms each, 200ms gaps) played immediately
   - **Log:** ROS2 logger warning: "Low battery! 20% remaining. Please recharge."

5. **Persistent Warning:**
   - Warning repeats every 60 seconds until battery recharged or system shutdown
   - Prevents battery over-discharge (30V cutoff protection via BMS)

6. **Testing:**
   - Simulate low battery: Manually lower voltage (adjust voltage divider temporarily) or drain battery to 32V
   - Verify warning triggers at correct voltage
   - Verify warning repeats until voltage rises above threshold

7. **Integration:**
   - Battery monitoring node runs automatically on orchestrator (or dedicated ESP32)
   - Added to system startup (launch file or systemd service for future)

8. **Documentation:**
   - Battery monitoring code committed to `orchestrator/battery_monitor.py` or `modules/body/firmware/`
   - Voltage divider calculation documented with schematic

**Dependencies:** Story 1.2 (battery + monitoring circuit), Story 1.5 or 1.6 (LED or beep for warning)

**Estimated Effort:** 4-5 hours

---

### Story 1.10: 30-Minute Continuous Operation Test

**As a** quality assurance tester,  
**I want** to run Olaf continuously for 30 minutes without failures,  
**so that** I can validate system stability before moving to Epic 2.

#### Acceptance Criteria:

1. **Test Setup:**
   - All modules running: Raspberry Pi orchestrator + head module (eye) + audio module (beep)
   - Battery fully charged (42V, 100%)
   - Test environment: Room temperature, stable WiFi

2. **Test Procedure:**
   - Start orchestrator with continuous blink + beep (every 5 seconds)
   - Monitor for 30 minutes (set timer)
   - Observe: terminal logs, eye behavior, beep sounds, battery voltage

3. **Success Criteria:**
   - **No crashes:** All ROS2 nodes remain active for full 30 minutes
   - **No missed commands:** Eye blinks every 5 seconds (360 blinks total), beeps every 5 seconds (360 beeps)
   - **Stable latency:** Blink + beep coordination remains <500ms throughout test
   - **Memory stable:** ESP32 free heap doesn't decrease (check at start, 15min, 30min)
   - **Battery drain:** Voltage drops from 42V → 40-41V (≈2-5% drain expected for minimal load)

4. **Failure Handling:**
   - If crash occurs: Log error, investigate cause, fix bug, restart test
   - If commands missed: Check ROS2 topic echo, verify network stability
   - If latency degrades: Profile code, optimize bottlenecks

5. **Metrics Collected:**
   - Total runtime: 30:00 minutes
   - Total blinks: ~360 (verify via log count)
   - Total beeps: ~360 (verify via log count)
   - Battery voltage at start: X.XX V
   - Battery voltage at end: X.XX V
   - Average latency: <500ms (sampled 10 times during test)

6. **Pass/Fail:**
   - **PASS:** All success criteria met, no crashes
   - **FAIL:** Any crash, >5% missed commands, latency >500ms for >10% of samples

7. **Documentation:**
   - Test results logged in `tests/integration/epic1_30min_test_results.md`
   - Pass/fail status, metrics, any issues encountered
   - Screenshot or video of final terminal output (proof of 30min runtime)

**Dependencies:** All Epic 1 stories (1.1-1.9)

**Estimated Effort:** 1 hour (test execution) + 2-4 hours (bug fixes if needed)

---

### Story 1.11: Epic 1 Documentation & Handoff

**As a** future contributor or builder,  
**I want** comprehensive documentation of Epic 1 work,  
**so that** I can understand the foundation and replicate the setup.

#### Acceptance Criteria:

1. **README Updates:**
   - Main `README.md` updated with:
     - Quick start guide (clone → setup → run minimal demo)
     - Link to Epic 1 completion status
     - Photos of working system (eye blinking, breadboard setup)

2. **Hardware Documentation:**
   - `hardware/bom/epic1_bom.csv` created with all components:
     - Hoverboard battery, buck converters, ESP32, OLED, wires, connectors
     - Supplier links (AliExpress, Amazon)
     - Costs (actual prices paid)
   - `hardware/wiring/` contains:
     - Power system diagram
     - Head module wiring photo
     - Audio wiring photo

3. **Software Documentation:**
   - `modules/head/README.md` explains:
     - Hardware connections
     - Firmware build/upload process
     - How to test standalone
   - `orchestrator/README.md` explains:
     - How to run orchestrator
     - ROS2 topics used
     - Coordination logic

4. **Setup Scripts:**
   - `tools/setup/install_ros2_humble.sh` tested on fresh Raspberry Pi
   - `tools/setup/configure_wifi.sh` (optional) for easy network setup

5. **Troubleshooting Guide:**
   - `docs/troubleshooting.md` created with common issues:
     - "ESP32 won't connect to WiFi" → solution
     - "OLED shows garbage" → I2C address check
     - "Beep doesn't play" → wiring/pin check
     - "ROS2 topics not visible" → network discovery fix

6. **Photos/Videos:**
   - At least 3 photos:
     - Power system assembled
     - Head module (ESP32 + OLED) working
     - Full Epic 1 setup on desk
   - Optional: 30-second video of blink + beep demo

7. **GitHub Milestone:**
   - GitHub milestone "Epic 1: Foundation" created
   - All Epic 1 stories closed, linked to milestone
   - Milestone marked complete

8. **Content Creation:**
   - LinkedIn post drafted (see `.content/epic-content-plan.md` Week 2 Post #3)
   - "Olaf's First Blink!" with photo/video, explanation of power system
   - Post published or scheduled

**Dependencies:** All Epic 1 stories complete (1.1-1.10)

**Estimated Effort:** 4-6 hours (documentation writing, photo editing, content creation)

---

## Epic 1 Summary

**Total Stories:** 11  
**Estimated Total Effort:** 45-60 hours (2-3 weeks for solo builder working weekends)

**Key Deliverables:**
- ✅ Repository structure established
- ✅ Power system operational (hoverboard battery + buck converters)
- ✅ ROS2 Humble running on Raspberry Pi
- ✅ Minimal head module: 1 eye blinks on command
- ✅ Audio module: Beeper plays test tone
- ✅ Orchestrator coordinates eye + beep
- ✅ Test harness for manual module testing
- ✅ Battery monitoring with low-battery warnings
- ✅ 30-minute continuous operation validated
- ✅ Complete documentation for replication

**Success Criteria Met:**
- Three-layer architecture proven (Intelligence → Orchestration → Module)
- ROS2 + micro-ROS communication working
- Power system stable and safe
- "Quick win" achieved: Olaf blinks at you!
- Foundation ready for Epic 2 (3D design) and Epic 3 (full personality)

**Next Epic:** Epic 2 - 3D Design & Physical Structure

---
