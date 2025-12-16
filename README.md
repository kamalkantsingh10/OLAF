# 🤖 OLAF

**An open-source AI companion robot. Built in public.**




## The Journey: 20 Weeks and Counting

<table>
  <tr>
    <td width="33%" align="center">
      <img src="docs/media/sketch.jpeg" alt="Week 0: Initial Sketch" width="100%"><br>
      <b>Week 0</b><br>Initial concept sketch
    </td>
    <td width="33%" align="center">
      <img src="docs/media/cad-design.webp" alt="Week 8: CAD Design" width="100%"><br>
      <b>Week 8</b><br>3D CAD design complete
    </td>
    <td width="33%" align="center">
      <img src="docs/media/current.jpeg" alt="Week 20: First Module Working" width="100%"><br>
      <b>Week 20</b><br>First light! Eyes blinking (build in progress)
    </td>
  </tr>
</table>

**An active build-in-public project.** Built while relocating cities and starting a new job. Learning CAD, ROS2, embedded systems, and AI integration as I go—all with AI coding assistants accelerating development. The journey continues!

---

## What is OLAF?

OLAF (Open Lovable AI Friend) is a **personality-first robotics framework** that brings AI agents into physical form. Think R2D2's charm meeting modern AI capabilities—a 2-3 foot tall companion that communicates through expressive OLED eyes, articulated ears (Chappie-inspired), animated heart display, R2D2-style beeps, and floor projection for information display.

**This is not another voice assistant in a box.**

When you BUILD your AI companion, something fundamental changes. It becomes a partner, not a servant. JARVIS challenges Tony Stark's ideas. R2D2 takes initiative. The difference? Tony BUILT JARVIS. Luke CHOSE R2D2 and they grew together. **Building creates relationship, and relationship enables true collaboration.**

---

## The Problem We're Solving

AI assistants (Alexa, Siri, ChatGPT, Claude) have impressive reasoning but remain fundamentally **disembodied**—trapped in screens and speakers. Meanwhile, building physical AI with personality forces impossible choices:

| Option | What You Get | What You Lose |
|--------|-------------|---------------|
| **Commercial robots** (Boston Dynamics, Vector) | Polish, capability | $1000s+, closed-source, no customization |
| **Educational platforms** (Poppy, Niryo) | Modularity, learning | AI integration, personality, high cost |
| **ROS2 robots** (Linorobot2, Andino) | Excellent navigation | Zero personality, assistant features |
| **Hobbyist kits** (Otto DIY) | Affordable, charming | Limited capabilities, no AI |
| **Closed AI robots** (HIWONDER) | AI + robotics bridge | Generic design, closed ecosystem |

**The Core Gap:** No open-source framework integrates modular architecture + AI conversation + personality expression + practical assistance + maker-accessible approach + community ecosystem.

**The Opportunity:** All the pieces are finally here—3D printers, powerful LLMs, modern SBCs (Raspberry Pi 5), AI coding assistants, local AI accelerators (Hailo). **This is the democratization moment.** The future of embodied AI belongs to builders, not just consumers.

---

## Architecture: Three Layers, Four Modules

### The Three-Layer Design

```
┌─────────────────────────────────────────────────────────────────┐
│              INTELLIGENCE LAYER (Hybrid AI)                     │
│                                                                 │
│  Local AI (Hailo-accelerated):                                 │
│    • Whisper STT (<200ms latency)                              │
│    • Real-time vision processing                               │
│                                                                 │
│  Cloud AI Agents (Claude/GPT-4):                               │
│    • Natural language understanding                            │
│    • Complex reasoning & decision-making                       │
│    • Tool use, function calling, multi-step planning           │
│                                                                 │
└──────────────────────┬──────────────────────────────────────────┘
                       │ HTTPS/REST + Local Inference
                       │
┌──────────────────────▼──────────────────────────────────────────┐
│              ORCHESTRATION LAYER                                │
│         Raspberry Pi 5 16GB + Hailo AI Kit (26 TOPS)            │
│                     Python • ROS2 Humble                        │
│                                                                 │
│  • Personality Coordinator (sync eyes/ears/neck/heart/beeps)   │
│  • AI Agent Orchestration (tool routing, context management)   │
│  • SLAM Navigation (Cartographer)                              │
│  • Sensor Fusion & State Management                            │
│  • Module Discovery (I2C communication)                        │
│                                                                 │
└──┬──────┬──────┬──────┬────────────────────────────────────┘
   │      │      │      │ ROS2 Topics (pub/sub)
   │      │      │      │ I2C Physical Backbone
   │      │      │      │
┌──▼──────────┐┌─▼────┐┌─▼────┐┌▼──────────┐
│ HEAD+EARS   ││ NECK ││TORSO ││   BASE    │
│   ESP32     ││ESP32 ││ESP32 ││   ESP32   │
│   (0x08)    ││(0x09)││(0x0A)││   (0x0B)  │
└─────────────┘└──────┘└──────┘└───────────┘
```

### The Four Modules (MECE Principle)

Each module owns its domain exclusively—no cross-dependencies. All powered by ESP32 microcontrollers acting as smart I2C peripherals.

**1. HEAD+EARS Module (I2C 0x08)**
- **Hardware:**
  - 2× OLED eyes (128×64, SPI-driven 30-60 FPS)
  - 2× Chappie-inspired articulated ears (2-DOF each, Feetech serial servos)
  - Floor projector with ESP32-controlled power (optocoupler) and focus (linear servo)
  - RGBD camera + IMU (USB to Pi)
- **Personality:**
  - Animated eye expressions (happy, curious, thinking, confused, sad)
  - Ear movements for directional attention and emotional expression (perked up = alert, drooping = sad)
  - Smooth emotional transitions
- **Intelligence:** Vision input for SLAM/navigation, presence detection for context-aware behaviors
- **Smart Control:** Pi sends HDMI video to projector; ESP32 controls power and auto-focus

**2. NECK Module (I2C 0x09)**
- **Hardware:** 3-DOF servo array (pan/tilt/roll, Feetech STS3215), 2× presence sensors
- **Personality:** Head orientation, expressive gestures (head tilts, nods, shakes)
- **Movement:** Smooth organic motion curves (easing functions, not mechanical jerks)
- **Intelligence:** 360° human detection via dual presence sensors

**3. TORSO Module (I2C 0x0A)**
- **Hardware:** 2.8" square display (animated beating heart), thermal printer, Raspberry Pi housing, battery pack, LED status indicators
- **Personality:** Heart rhythm changes with emotional state (fast = excited, slow = calm, irregular = confused)
- **Output:** Thermal printer outputs lists, reminders, recipes for physical takeaways
- **Power:** Houses main battery and power distribution to all modules

**4. BASE Module (I2C 0x0B) - Self-Balancing**
- **Hardware:** Two-wheel inverted pendulum, hoverboard BLDC motors, ODrive controller (UART), MPU6050 IMU (200Hz), servo kickstand
- **Control:** 200Hz PID balancing loop (real-time guarantee on ESP32, Linux can't do this)
- **Mobility:** SLAM navigation, follow-me mode, obstacle avoidance
- **Safety:** Autonomous kickstand deployment when stopping

### Why This Architecture?

**Modular MECE (Mutually Exclusive, Collectively Exhaustive):**
- Each module completable in 1-2 weekend sprints
- Independent testing: modules work standalone before integration
- Progressive capability: robot functional with minimal modules, enhanced as you add more
- Community contribution: others can develop modules without touching core framework

**Hybrid Intelligence:**
- **Local AI (Hailo):** Fast sensing, <200ms STT eliminates cloud delay, privacy-friendly
- **Cloud Agents:** Sophisticated reasoning, tool use, multi-step planning
- Best of both worlds: speed + sophistication

**ROS2 Foundation:**
- Industry-standard robotics middleware
- Proven pub/sub architecture for module communication
- Extensive ecosystem (SLAM, navigation, sensor fusion)
- All ROS2 nodes run on Pi; ESP32s act as I2C bridges

---

## Key Features

### Personality Expression (The Core Differentiator)

**Multi-channel coordinated expression:**
- 🎭 **OLED Eyes:** 7 emotion types × 5 intensity levels (35 unique expressions)
- 👂 **Articulated Ears:** Directional attention + emotional positioning
- 🎯 **Neck Movement:** 3-DOF gestures (pan/tilt/roll)
- ❤️ **Heart Display:** Beating rhythm synchronized with emotional state
- 🎵 **R2D2 Beeps:** Musical intervals (not harsh tones), emotion-matched inflection
- 🎬 **Orchestrated Sync:** All channels coordinated <500ms for unified emotional states

**Example:** "Express excitement level 4"
→ Orchestrator sends I2C commands to Head (eyes wide, pupils dilated), Ears (perked forward), Neck (small bouncing motion), Torso (heart racing), Speaker (rapid high-pitched beeps)
→ ESP32s execute locally cached animations simultaneously
→ Result: Coherent excited expression across all channels

### Intelligence & Interaction

- 🗣️ **Voice-first input:** Microphone array + Hailo Whisper STT (<200ms local processing)
- 🧠 **Cloud AI agents:** Claude/GPT-4 for natural language, reasoning, tool use
- 🔊 **Context maintenance:** SQLite conversation history, user preferences across power cycles
- 🎯 **Function routing:** AI decides which modules to activate (projection, printer, movement)
- 👁️ **Vision:** RGBD camera for SLAM, obstacle detection, person following

### Physical Outputs

- 📽️ **Floor projector:** Information display without screen-staring (charts, recipes, reminders)
- 🖨️ **Thermal printer:** Physical printouts for lists, notes (tangible takeaways)
- 🎵 **Beeps + Gestures:** Non-verbal communication (avoids uncanny valley)

### Mobility & Navigation

- 🚶 **Self-balancing base:** Two-wheel inverted pendulum (saves floor space vs. 4-wheel)
- 🗺️ **SLAM navigation:** Google Cartographer for autonomous apartment navigation
- 👤 **Follow-me mode:** Vision-based person tracking + Nav2 path planning
- 🛑 **Obstacle avoidance:** Real-time depth sensing (RGBD camera)

---


## Current Status

**🔄 Active Development (Week 20):**

This is a work-in-progress build. The journey from sketch to functioning robot continues!

**What's Working:**
- ✅ Head module: OLED eyes blinking with cyan animations
- ✅ 3D CAD design complete for all modules
- ✅ Initial firmware for head module operational
- ✅ Modular architecture validated conceptually

**Currently Building:**
- 🔨 Physical assembly of remaining modules (ears, neck, torso, base)
- 🔨 3D printing and fitting parts
- 🔨 ESP32 firmware development for each module
- 🔨 I2C communication between modules
- 🔨 ROS2 integration layer

**Not Yet Started:**
- ⏳ AI intelligence layer integration (Hailo Whisper STT)
- ⏳ Personality coordinator (coordinated expression across modules)
- ⏳ SLAM navigation
- ⏳ Self-balancing base with PID tuning
- ⏳ Thermal printer integration

**Realistic Timeline:**
This is a complex build being done in spare time while working full-time. Progress is incremental, iterative, and fully documented as it happens.


---

## License & Credits

**License:** Open Source (TBD: MIT or Apache 2.0)

**Builder:** [Kamal Singh](https://www.linkedin.com/in/kamal-singh)
- 15 years in tech (5 at Amazon)
- 1 year intentional AI deep-dive
- MBA + Full-stack capability (hardware + software + AI)

**Inspiration:**
- R2D2 (Star Wars) - Non-verbal personality, beeps > words
- JARVIS (Iron Man) - AI partner, not servant
- Chappie - Expressive ears, emotional engagement
- Wall-E - Retro-futurism meets friendly companion

**Built with:**
- AI Coding Assistants (Claude, GPT-4) - 100% AI-assisted development
- Raspberry Pi 5 + Hailo AI Kit - Local AI acceleration
- ROS2 Humble - Robotics middleware
- ESP32 - Module intelligence
- OnShape - 3D CAD design
- PlatformIO - Firmware development

---

## Tags

`#BuildInPublic` `#Robotics` `#ROS2` `#PhysicalAI` `#OpenSource` `#MakerMovement` `#AI` `#LearnInPublic` `#EmbodiedAI` `#ModularRobotics`

---

<p align="center">
  <i>"Feel alive first, be useful second."</i>
</p>

<p align="center">
  <b>Star ⭐ this repo if you believe the future of AI should be open and collaborative!</b>
</p>
