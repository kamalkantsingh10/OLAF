# Project Brief: Olaf

## Executive Summary

**Olaf** is an open-source modular personal assistant robot that brings physical AI into everyday life through expressive personality and practical assistance. Named after the friendly snowman, Olaf combines R2D2-style charm with modern AI capabilities—communicating through expressive beeps, animated OLED eyes, and Chappie-inspired articulated ears, while using floor projection to display information when needed.

**Primary Problem:** AI assistants are trapped in screens and speakers. There's no accessible, open-source path for makers to build a physical AI companion that's both genuinely useful and has engaging personality. Existing robots are either expensive black boxes or purely hobbyist projects lacking real AI integration.

**Target Audience:** DIY robotics builders who can solder, code Python, and access 3D printers—people who want to build physical AI in public and contribute to an evolving open-source ecosystem.

**The Solution:** A modular robot architecture where each component (head with vision/OLED eyes, articulated ears/neck, projection system, hoverboard-based mobility) operates independently via ESP32 modules, coordinated by a Raspberry Pi 8GB running local orchestration while leveraging cloud AI for intelligence. Built around the MECE principle (Mutually Exclusive, Collectively Exhaustive), each module can be developed in weekend sprints, tested in isolation, and shared with the community.

**Key Value Proposition:**
- **Physical AI personality** - Expressive movements (3-DOF neck, 2-DOF ears), emotional OLED eyes, R2D2-style communication that feels alive
- **Practical assistance** - SLAM navigation, floor projection for visual information, conversational AI for questions and companionship
- **Complete build transparency** - Weekly LinkedIn progress updates, monthly YouTube milestone videos, full documentation and 3D files on GitHub
- **Modular & replicable** - AliExpress-friendly components, configurable capabilities (remove projector/RGBD to reduce cost), weekend-sized development chunks
- **Community-driven evolution** - Others can build, enhance, and contribute modules back to the ecosystem

**V1 Scope (4 months):** Mobile robot with personality expression (beeps, eyes, ear/neck movement), conversational AI, SLAM navigation, floor projection, and modular architecture proven through independent module development. Post-V1 expansions include diary maintenance, smart home control, reminders, and community-contributed capabilities.

---

## Problem Statement

**Current State & Pain Points:**

The world of AI assistants has evolved dramatically, but they remain fundamentally disembodied—Alexa, Siri, ChatGPT, and Claude exist only as voices in speakers or text in apps. Meanwhile, physical robotics projects exist in distinct silos:

1. **Commercial robots** (Boston Dynamics, Anki Vector) - Expensive ($1000s-$10,000s), closed-source, limited customization, no community contribution
2. **Educational platforms** (Poppy Humanoid ~$8K, Niryo One ~$2K) - Open-source and modular but focused on mechanical learning, lacking AI conversation and personality
3. **Functional robotics** (Linorobot2, Andino) - Excellent ROS2 navigation platforms but purely utilitarian, no personality or assistant features
4. **Simple hobbyist kits** (Otto DIY ~$100) - Affordable and charming but extremely limited, Arduino-based with no AI integration
5. **Closed AI robots** (HIWONDER TurboPi) - Bridge AI and robotics but generic design, closed ecosystem, limited modularity

**For makers who want to build physical AI with personality**, no existing framework combines all the necessary elements. Current options force impossible choices:

- **Choose mechanical education** (Poppy) → Get modularity but no AI or personality
- **Choose navigation capability** (Linorobot2) → Get ROS2 and SLAM but no emotional expression or assistant features
- **Choose AI integration** (HIWONDER) → Get ChatGPT but lose open-source modularity and personality-first design
- **Choose affordability** (Otto DIY) → Get accessible kit but sacrifice all advanced capabilities

**The Core Gap:**

No open-source project integrates:
- ✅ Modular architecture (weekend-sprint development)
- ✅ AI conversation capabilities (cloud LLM integration)
- ✅ Physical personality expression (R2D2-style character)
- ✅ Practical assistance features (navigation, information display)
- ✅ Maker-accessible budget (~$1000)
- ✅ Community contribution ecosystem

**Why Existing Solutions Fall Short:**

- **ROS2 platforms** - Provide excellent robotics infrastructure but no framework for **personality orchestration** or **emotional expression coordination**
- **Educational robots** - Teach mechanical/electrical skills but don't bridge to modern AI capabilities (LLMs, vision models)
- **Commercial AI assistants** - Smart speakers remain disembodied; physical robots are closed and expensive
- **Hobbyist projects** - Scattered tutorials for individual components (ESP32, servos, displays) but no holistic architecture for **emergent personality** from coordinated modules

**Impact of the Problem:**

- **Wasted potential** - Makers must choose between personality OR functionality OR affordability
- **Reinventing wheels** - No shared framework for AI orchestration, emotional expression, or module coordination
- **Fragmented community** - Navigation experts, AI enthusiasts, and character designers work in separate silos
- **Physical AI remains abstract** - Despite LLM advances, embodied AI projects lack accessible on-ramps
- **No personality-first framework** - Existing robotics platforms treat expression as afterthought, not core design principle

**Urgency & Importance:**

Physical AI is the next frontier—AI needs embodiment to be truly helpful in daily life. The technical pieces exist (ROS2, ESP32, affordable sensors, cloud AI APIs), but no framework **brings them together with personality as the foundation**. Without this integration, physical AI development remains:
- Locked behind corporate/academic walls (high budgets, closed-source)
- Fragmented across disciplines (roboticists don't integrate AI; AI practitioners don't build robots)
- Missing the emotional engagement that makes companions compelling (function without character)

The maker community is ready—proven by successful ROS2 adoption (Linorobot2) and educational robot communities (Poppy, Otto). What's missing is a **personality-first, AI-integrated, modular framework** that makers can build, extend, and share.

---

## Proposed Solution

**Core Concept:**

Olaf is a **personality-first, modular robotics framework** that bridges AI intelligence and physical expression through a distributed architecture. Unlike existing platforms that bolt personality onto functional robots, Olaf is designed from the ground up for **emotional engagement** while proving practical assistance capabilities through modular expansion.

**The Architecture:**

**Three-Layer Design:**

1. **Module Layer (Physical + Embedded Intelligence)**
   - Independent hardware modules: Head (RGBD camera + OLED eyes), Ears (2-DOF Chappie-inspired), Neck (3-DOF servo), Body (projection system), Base (hoverboard wheel platform)
   - Each module powered by ESP32 microcontroller running embedded firmware
   - Modules communicate via I2C hub + ROS2 topics
   - MECE Principle: Each module owns its domain exclusively, no cross-dependencies

2. **Orchestration Layer (Raspberry Pi 8GB)**
   - Central coordinator running Python-based orchestration engine
   - Receives high-level commands from AI layer
   - Translates abstract intentions into coordinated module behaviors
   - Example: "Express excitement" → coordinate ear wiggle + eye animation + happy beep sequence
   - Handles SLAM navigation, sensor fusion, module discovery

3. **Intelligence Layer (Cloud AI)**
   - Conversational AI (Claude/GPT-4 API) for natural language understanding
   - Decision-making: Route user requests to appropriate module functions
   - Context maintenance: Remember conversation history, user preferences
   - Personality engine: Generate R2D2-style emotional responses matched to context

**Key Differentiators:**

**1. Personality Orchestration Framework**
- Not just "add some LEDs"—systematic coordination of multiple expression channels
- Inspired by human body muscle memory: orchestrator sends high-level commands ("greet enthusiastically"), modules execute learned behavior sequences
- Emotional state machine: Curious, excited, thinking, confused, content states drive expression

**2. Modular Weekend Development**
- Each module completable in 1-2 weekend sprints
- Independent testing: Modules work standalone before integration
- Progressive capability: Robot functional with minimal modules, enhanced as you add more
- Community contribution: Others develop modules without touching core framework

**3. Build-in-Public Documentation**
- Complete 3D print files, BOM with AliExpress links, wiring diagrams
- Weekly progress journals on LinkedIn (successes + failures)
- Monthly YouTube milestone videos ("Part 1: Building the Head")
- GitHub repository with module templates, setup scripts, testing frameworks

**4. Configurable Capability Tiers**
- **Minimal Olaf** (~$400): Basic personality (eyes, ears, neck, voice), stationary
- **Standard Olaf** (~$700): + Mobility (hoverboard base, SLAM navigation)
- **Full Olaf** (~$1000): + Projection system, RGBD camera, full assistant features
- Builders choose features based on budget/needs

**5. Physical AI Integration**
- Cloud AI for intelligence, local execution for responsiveness
- Sensor data → Raspberry Pi → Cloud AI decision → Local orchestration → Module execution
- Example flow: "Show me a good pasta recipe" → AI decides to project → Orchestrator activates projector + fetches content → Displays on floor while beeping encouragingly

**How This Solves the Problem:**

✅ **Bridges AI and Robotics** - ROS2 foundation + cloud LLM integration + personality orchestration
✅ **Modular Development** - Weekend sprints, independent testing, progressive enhancement
✅ **Personality-First** - Expression coordination as core architecture, not afterthought
✅ **Maker-Accessible** - $400-$1000 configurable tiers, AliExpress components, 3D printed
✅ **Community Ecosystem** - Module templates, complete docs, public build process
✅ **Actually Useful** - Conversation, navigation, information display, smart home (post-V1)

**Why This Will Succeed Where Others Haven't:**

- **Linorobot2** proved ROS2 + ESP32 works for makers → Olaf adds AI + personality layers
- **Poppy** proved modular robotics builds community → Olaf adds practical AI assistance
- **Otto DIY** proved personality sells → Olaf adds serious capabilities
- **R2D2** proved beeps > words for emotional connection → Olaf embraces non-verbal communication
- **Your public build** provides authentic documentation → Learners follow real development, not sanitized tutorials

**High-Level Vision:**

A robot that **feels alive** through coordinated personality expression, **helps with daily tasks** through AI integration, and **grows with you** through modular expansion—all built transparently so anyone with soldering skills and a 3D printer can create their own companion and contribute modules back to the community.

---

## Target Users

### Primary User Segment: "Physical AI Enthusiasts"

**Demographic/Profile:**
- **Age:** 25-45 years old
- **Technical background:** Software developers, embedded systems engineers, hobbyist makers, robotics students
- **Skills:** Can solder, comfortable with Python, familiar with 3D printing, basic electronics knowledge
- **Resources:** Access to 3D printer (personal or makerspace), budget of $400-$1000, dedicated workspace
- **Geography:** Global, with strong representation in maker-friendly communities (US, Europe, Asia)

**Current Behaviors & Workflows:**
- Spends weekends on personal projects (Arduino/Raspberry Pi builds, home automation, IoT)
- Follows AI developments closely (uses ChatGPT/Claude, experiments with APIs)
- Active in online communities (Reddit r/robotics, Hackaday, YouTube maker channels)
- Frustrated by lack of integration between AI software skills and physical hardware interests
- Orders components from AliExpress, waits weeks for delivery, plans projects around shipping times

**Specific Needs & Pain Points:**
- Wants to build something **meaningful**, not just another blinking LED project
- Craves projects that combine multiple skill domains (software + hardware + AI)
- Needs **incremental buildability**—can't commit 6 months to all-or-nothing project
- Desires social proof and content creation opportunities (LinkedIn posts, YouTube builds)
- Frustrated by robotics projects that feel "mechanical" without personality
- Wants to learn modern AI integration in practical, hands-on way

**Goals They're Trying to Achieve:**
- Build a physically present AI companion that demonstrates real capabilities
- Develop expertise at intersection of AI and robotics for career/portfolio
- Create shareable content that establishes technical authority
- Have a daily-use project that's both functional and engaging
- Learn ROS2, SLAM, computer vision through concrete application

### Secondary User Segment: "Build-Along Learners"

**Demographic/Profile:**
- **Age:** 18-35 years old
- **Technical background:** CS students, junior developers, career switchers into tech, self-taught programmers
- **Skills:** Strong software skills (Python), limited hardware experience, curious about robotics
- **Resources:** Budget-conscious ($400-700), may use school/library 3D printers, smaller workspace
- **Motivation:** Learn by replicating rather than designing from scratch

**Current Behaviors & Workflows:**
- Follows tutorial-based learning (YouTube builds, online courses, documentation)
- Builds portfolio projects for resume/interviews
- Active on platforms like LinkedIn, Twitter/X for learning in public
- Interested in AI but hasn't found hands-on robotics entry point
- Prefers step-by-step guides with clear milestones

**Specific Needs & Pain Points:**
- Intimidated by "design your own robot" - needs concrete reference implementation
- Wants **proven design** with clear instructions and troubleshooting
- Needs to understand "why" behind technical decisions (educational focus)
- Budget-sensitive—appreciates configurable cost tiers
- Fears getting stuck mid-project without community support

**Goals They're Trying to Achieve:**
- Complete a impressive portfolio project that combines AI + hardware
- Learn robotics fundamentals through practical experience
- Build confidence in hardware/electronics through successful project
- Create documentation of their build journey for personal brand
- Potentially contribute small improvements once they understand the system

---

## Goals & Success Metrics

### Business Objectives

- **Complete functional V1 robot within 4 months** - Mobile robot with personality expression, conversational AI, and SLAM navigation operational by end of Month 4
- **Build authority in physical AI space** - Establish credibility through consistent public documentation, resulting in measurable audience growth and engagement
- **Create replicable framework** - Produce complete documentation (3D files, BOM, code, setup guides) enabling others to replicate Olaf independently
- **Prove modular architecture viability** - Demonstrate that MECE-based module design enables independent development and graceful fault tolerance

### User Success Metrics

- **First module operational within 2 weeks** - Head module (eyes, basic expression) working standalone to prove quick wins
- **Weekly visible progress** - Each week produces demonstrable milestone (new module, integration success, behavior implementation)
- **Daily interaction achieved by Month 4** - Olaf becomes part of daily routine through conversation and presence
- **Content creation milestones met** - 2-3 LinkedIn posts per week (1 progress update + 1-2 physical AI insights), 1 monthly YouTube video (4 videos), continuous GitHub commits

### Key Performance Indicators (KPIs)

- **Technical KPIs:**
  - **Module independence**: Each module testable standalone without other modules present - Target: 100% of modules
  - **AI response latency**: Time from voice command to action initiation - Target: <3 seconds for 90% of interactions
  - **Navigation accuracy**: SLAM-based movement to target locations - Target: ±10cm accuracy in mapped environments
  - **AI-assisted development**: Coding and documentation produced with AI assistance (Claude/GPT-4) - Target: 100% AI-assisted workflow

- **Build-in-Public KPIs:**
  - **Documentation completeness**: Every completed module has BOM, wiring diagram, 3D files, setup guide (AI-generated) - Target: 100%
  - **Content consistency**: LinkedIn posts - Target: 32-48 posts over 4 months (2-3/week: 1 progress update + 1-2 physical AI insights)
  - **Video milestones**: Major builds documented - Target: 4 YouTube videos (1/month)
  - **GitHub activity**: Regular commits showing continuous progress - Target: 3-5 commits/week average

- **Engagement KPIs (Optional/Aspirational):**
  - **Audience growth**: LinkedIn followers/engagement increase - Baseline tracking, no specific target
  - **Repository stars**: GitHub interest indicator - Baseline tracking, no specific target
  - **Replication attempts**: Others attempting to build Olaf - Target: At least 1 external builder by Month 6 post-launch

- **Personal Success KPIs:**
  - **Learning objectives met**: Gain proficiency in ROS2, SLAM, ESP32 firmware, AI orchestration - Self-assessed monthly
  - **"Cool factor" validated**: Robot generates positive reactions when demonstrated - Qualitative feedback from demos
  - **Daily utility achieved**: At least 2-3 meaningful interactions with Olaf per day by Month 4

---

## MVP Scope

### Core Features (Must Have for V1)

**1. Personality Expression System**
- **OLED Eyes (2x)**: Animated expressions showing emotions (happy, curious, thinking, confused, sad)
- **Articulated Ears (2x, 2-DOF each)**: Chappie-inspired ears with independent movement for directional attention and emotion
- **Articulated Neck (3-DOF)**: Pan, tilt, and roll movements for head orientation and expressive gestures
- **R2D2-Style Beeping**: Tone-based communication system with emotional inflection matching context
- **Coordinated Expression**: Orchestrator synchronizes eyes + ears + neck + beeps for unified emotional states
- **Rationale**: Personality is the core differentiator; without engaging expression, Olaf is just another functional robot

**2. Conversational AI Integration**
- **Voice Input**: Microphone array for voice command capture
- **Cloud AI Processing**: Integration with Claude/GPT-4 API for natural language understanding
- **Context Maintenance**: Conversation history and state management
- **Function Routing**: AI decides which modules to activate based on user intent
- **Response Generation**: AI generates appropriate R2D2-style beep patterns and physical responses
- **Rationale**: Intelligence layer bridges human intent to robot actions; core to "assistant" value proposition

**3. Mobility & Navigation**
- **Hoverboard Base**: Repurposed hoverboard wheels and motors for differential drive
- **RGBD Camera**: Depth sensing for obstacle detection and mapping
- **SLAM Navigation**: Simultaneous localization and mapping for autonomous apartment navigation
- **Follow-Me Mode**: Basic person-following capability using vision
- **Rationale**: Physical presence in the space; mobility enables "companion" vs. "desk toy" positioning

**4. Information Display**
- **Floor Projector**: Projects information, visualizations, and text onto floor
- **Dynamic Content**: Can display charts, recipes, reminders, status information
- **Expression Enhancement**: Projects visual elements to complement beeps (emojis, thinking dots, etc.)
- **Rationale**: Information delivery without screen-staring; unique interaction modality

**5. Modular Architecture (Proven)**
- **Independent Modules**: Head, Ears, Neck, Projection, Base each operate standalone
- **ESP32 per Module**: Embedded intelligence with local firmware
- **ROS2 Communication**: Standardized topics for inter-module messaging
- **I2C + Power Hub**: Physical connectivity architecture
- **Module Testing Framework**: Each module testable in isolation before integration
- **Rationale**: Validates core architectural thesis; enables weekend development; critical for future extensibility

**6. Build Documentation**
- **3D Print Files**: All STL files for physical components with print settings
- **BOM with Links**: Complete parts list with AliExpress/supplier links and pricing
- **Wiring Diagrams**: Visual guides for all electrical connections
- **Setup Guides**: Step-by-step instructions for module assembly and software installation (AI-generated)
- **Troubleshooting**: Common issues and solutions documented as encountered
- **Rationale**: Replicability is core value proposition; documentation validates build-in-public commitment

### Out of Scope for MVP

- **Diary Maintenance**: Conversation logging and daily summary generation → V2
- **Smart Home Control**: Light control, device integration → V2
- **Bill Reminders**: Calendar integration and proactive notifications → V2
- **Transit Information**: Real-time bus/train schedules → V2
- **Object Manipulation**: Arms, grippers, physical item interaction → V2+
- **Multi-Robot Coordination**: Olaf-to-Olaf communication → V2+
- **Local LLM**: On-device AI models instead of cloud → V2+ (performance/cost trade-offs)
- **Custom PCB**: Using development boards for V1, custom integration later → V2+
- **Hot-Plugging**: Module auto-discovery at runtime → V2 (V1 uses fixed configuration)

### MVP Success Criteria

**V1 is successful when:**

1. ✅ **Personality Validation**: Olaf elicits emotional responses (smiles, "aww" reactions) from people who interact with it
2. ✅ **Conversational Capability**: Can answer questions, tell jokes, engage in basic multi-turn conversations
3. ✅ **Autonomous Navigation**: Moves around apartment reliably without human intervention, avoiding obstacles
4. ✅ **Information Utility**: Successfully displays requested information via projection (recipes, weather, etc.)
5. ✅ **Module Independence**: Each module works standalone; removing one module doesn't break others
6. ✅ **Daily Interaction**: You naturally interact with Olaf 2-3 times per day without forcing it
7. ✅ **Build Replicability**: Complete documentation exists for someone else to build their own Olaf
8. ✅ **Content Output**: 4 YouTube milestone videos published, 32-48 LinkedIn posts completed, active GitHub repository

---

## Post-MVP Vision

### Phase 2 Features (Months 5-8)

**Assistant Utility Features:**
- **Diary Maintenance** - Daily conversation summarization, automatic journal generation
- **Smart Home Integration** - Control lights, thermostats, and other IoT devices via voice
- **Calendar & Reminders** - Proactive notifications for bills, appointments, tasks
- **Transit Information** - Real-time bus/train schedules, commute planning
- **Context-Aware Interruptions** - Learns when you're busy vs. available, interrupts appropriately

**Enhanced Personality Features:**
- **Emotional Weather Reports** - Weather delivery with emotional context (sad beeps for rain, excited for sunshine)
- **Thinking Visualization** - Visible "thinking" states with animated eyes, ear twitches, projected dots while AI processes
- **Sleep Mode Personality** - Nightlight eyes, breathing movements, constellation projections, responds to sounds without full wake
- **Excitement Escalation** - Enthusiasm builds through conversation (level 1-5) with coordinated expression intensity
- **Module Status Dashboard** - "Check yourself" command projects module health diagram with proud beeps

### Long-Term Vision (1-2 Years)

**Advanced AI Capabilities:**
- **Local LLM Option** - On-device AI models for privacy-conscious users or offline operation
- **Visual Memory** - RGBD camera captures daily activities, "What did I work on yesterday?" queries
- **Learning Behaviors** - Olaf learns your habits and preferences over time
- **Collaborative Problem Solving** - Projects mind maps during conversations, helps think through problems

**Physical Enhancements:**
- **Object Manipulation** - Simple gripper arm for fetching small items
- **Improved Mobility** - Faster motors, climbing small obstacles, outdoor capability
- **Enhanced Expression** - Additional DOF, RGB lighting, sound effects library expansion
- **Battery Upgrade** - Extended runtime, wireless charging station

**Technical Enhancements:**
- **Hot-Plugging** - Dynamic module discovery at runtime
- **Custom PCB** - Integrated circuit board replacing development boards for cleaner build
- **Improved SLAM** - Better mapping accuracy, multi-floor support
- **Voice Enhancement** - Better noise cancellation, far-field microphones

**Ecosystem Development:**
- **Module Marketplace** - Platform for discovering and sharing custom modules (if community emerges)
- **Behavior Library** - Shareable orchestration patterns and personality profiles
- **Multi-Olaf Coordination** - Multiple robots communicate and coordinate (if multiple units built)
- **Mobile App** - Companion app for remote control, status monitoring, configuration

**Content & Education:**
- **Tutorial Series** - Comprehensive video course on building Olaf from scratch
- **Workshop Format** - In-person or virtual workshops for group builds
- **Academic Partnerships** - University course integration for robotics/AI education
- **Conference Talks** - Present Olaf at maker faires, robotics conferences, AI meetups

### Expansion Opportunities

**Alternative Form Factors:**
- **Mini-Olaf** - Desk-sized version (~20cm) for smaller spaces/budgets
- **Olaf Pro** - Industrial-grade components for research/education institutions
- **Specialized Variants** - Security patrol version, elder care companion, educational assistant

**Commercial Possibilities (If Interest Emerges):**
- **Kit Sales** - Pre-sourced component bundles for easier building
- **Consulting Services** - Help others build custom robotic platforms
- **Licensing Opportunities** - Framework adoption by educational institutions or companies

**Research Applications:**
- **Embodied AI Research** - Platform for testing AI-robotics integration theories
- **HCI Studies** - Personality expression impact on human-robot interaction
- **Multi-Agent Systems** - Coordination algorithms between multiple Olafs
- **Edge AI Deployment** - Testing local AI models in physical contexts

---

## Technical Considerations

### Platform Requirements

**Target Platforms:**
- Mobile robot operating in indoor apartment environment (50-60cm height)
- Supports navigation on flat surfaces (hardwood, tile, carpet)
- Operates in typical home lighting conditions

**Compute Requirements:**
- Raspberry Pi 8GB RAM for orchestration layer
- Multiple ESP32 microcontrollers for module-level processing (each ESP32 acts as a ROS2 node)
- Cloud API access for AI inference (Claude/GPT-4)
- Internet connectivity (WiFi) required for cloud AI

**Sensors:**
- RGBD camera (head module) with built-in IMU for depth sensing, vision, and orientation
- Human presence sensor (PIR/mmWave in head module) for detecting people nearby
- Microphone array (head module) for voice input

**Indicators:**
- LED indicators on body for system status (power, WiFi, processing, error states)
- OLED eyes for primary expression
- Optional RGB LEDs for ambient lighting/mood indication

**Performance Requirements:**
- AI response latency: <3 seconds for 90% of interactions
- Navigation obstacle detection: Real-time processing at 10Hz minimum
- Module communication: <100ms latency between orchestrator and modules
- Battery runtime: 2-4 hours continuous operation (initial target)

### Technology Preferences

**Frontend (Projection/Display):**
- Floor projection system (pico projector or repurposed component)
- OLED displays (128x64 or similar) for eyes
- LED indicators for system status
- Simple graphics rendering, no complex UI framework needed

**Backend (Orchestration Layer):**
- Python 3.x on Raspberry Pi OS
- ROS2 Humble or newer for module communication
- Custom orchestration engine for personality coordination
- RESTful API integration for cloud AI services

**Module Firmware (ESP32):**
- C/C++ using Arduino framework or ESP-IDF
- micro-ROS for ROS2 integration (each ESP32 hosts a ROS2 node)
- I2C for inter-module hardware communication
- Local behavior execution (pre-programmed movement patterns)

**Database:**
- SQLite for local conversation history and state management
- JSON files for configuration and module definitions
- No complex database infrastructure needed for V1

**Hosting/Infrastructure:**
- Self-hosted on Raspberry Pi (no external servers)
- Cloud AI via API calls (Anthropic Claude API, OpenAI GPT-4 API)
- GitHub for code repository and documentation hosting
- YouTube for video content, LinkedIn for progress updates

### Architecture Considerations

**Repository Structure:**
```
olaf/
├── docs/                    # Documentation, build guides, diagrams
├── hardware/               # 3D models (STL), wiring diagrams, BOMs
├── modules/                # Module-specific code
│   ├── head/              # RGBD camera, eyes, presence sensor
│   ├── ears/              # 2-DOF articulated ears
│   ├── neck/              # 3-DOF neck servos
│   ├── projector/         # Floor projection system
│   ├── body/              # LED indicators, status display
│   └── base/              # Hoverboard mobility platform, IMU
├── orchestrator/           # Raspberry Pi orchestration engine
│   ├── personality/       # Expression coordination
│   ├── ai_integration/    # Cloud AI interface
│   └── navigation/        # SLAM, path planning
├── tests/                  # Module and integration tests
└── tools/                  # Setup scripts, utilities
```

**Service Architecture:**
- **Modular Design**: Each hardware module is an independent service (ESP32 + firmware)
- **ROS2 Node Architecture**: Each ESP32 acts as a logical ROS2 node, hosting node services and publishing/subscribing to topics
- **Central Orchestrator**: Raspberry Pi runs orchestration engine coordinating modules
- **Cloud AI Layer**: External API calls for intelligence (stateless from robot perspective)
- **ROS2 Topics**: Standardized pub/sub for module communication
- **I2C Hub**: Physical communication backbone for power and data

**Integration Requirements:**
- ROS2 topic definitions for standard messages (movement commands, sensor data, expressions)
- Cloud AI API integration (HTTP REST calls with retry logic)
- SLAM library integration (likely RTAB-Map or similar for RGBD)
- Audio input/output libraries for voice and beeping
- GPIO/I2C libraries for hardware control
- Presence detection logic for human-aware behaviors

**Security/Compliance:**
- API keys stored in environment variables, not committed to repository
- Conversation data stays local on Raspberry Pi by default
- Cloud AI calls over HTTPS
- No PII collection beyond what user explicitly shares in conversation
- Open-source license (likely MIT or Apache 2.0) for framework code

---

