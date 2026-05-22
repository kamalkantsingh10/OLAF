# 🤖 OLAF

**An open-source AI companion robot. Built in public.**

<p align="center">
  <a href="https://youtube.com/shorts/9D4Txb85EnM">
    <img src="https://img.youtube.com/vi/9D4Txb85EnM/hqdefault.jpg" alt="OLAF cycling through its expressions — click to watch" width="360"><br>
    <b>▶ Watch OLAF cycle through its expressions</b>
  </a>
</p>

---

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
      <b>Week 20</b><br>Head alive — eyes, ears & neck expressing
    </td>
  </tr>
</table>

**An active build-in-public project.** Built while relocating cities and starting a new job — learning CAD, ROS2, embedded systems, and AI integration as I go, with AI coding assistants accelerating development. The journey continues!

---

## What is OLAF?

OLAF is a **personality-first robotics framework** that brings AI agents into physical form. Think R2D2's charm meeting modern AI capabilities — a 2-3 foot tall companion that communicates through expressive OLED eyes, articulated ears (Chappie-inspired), an animated heart display, R2D2-style beeps, and LED status indicators.

**This is not another voice assistant in a box.**

When you BUILD your AI companion, something fundamental changes. It becomes a partner, not a servant. JARVIS challenges Tony Stark's ideas. R2D2 takes initiative. The difference? Tony BUILT JARVIS. Luke CHOSE R2D2 and they grew together. **Building creates relationship, and relationship enables true collaboration.**

---

## The Problem We're Solving

AI assistants (Alexa, Siri, ChatGPT, Claude) have impressive reasoning but remain fundamentally **disembodied** — trapped in screens and speakers. Meanwhile, building physical AI with personality forces impossible choices:

| Option | What You Get | What You Lose |
|--------|-------------|---------------|
| **Commercial robots** (Boston Dynamics, Vector) | Polish, capability | $1000s+, closed-source, no customization |
| **Educational platforms** (Poppy, Niryo) | Modularity, learning | AI integration, personality, high cost |
| **ROS2 robots** (Linorobot2, Andino) | Excellent navigation | Zero personality, assistant features |
| **Hobbyist kits** (Otto DIY) | Affordable, charming | Limited capabilities, no AI |
| **Closed AI robots** (HIWONDER) | AI + robotics bridge | Generic design, closed ecosystem |

**The Core Gap:** No open-source framework integrates modular architecture + AI conversation + personality expression + practical assistance + maker-accessible approach + community ecosystem.

**The Opportunity:** All the pieces are finally here — 3D printers, powerful LLMs, modern SBCs (Raspberry Pi 5), AI coding assistants, local AI accelerators (Hailo). **This is the democratization moment.** The future of embodied AI belongs to builders, not just consumers.

---

## Architecture: Three Layers, Five Modules

> **Repo scope (Phase 2, 2026-05-15).** This repository is the **OLAF body / expression engine** — the Orchestration + Module layers below. The Intelligence Layer (STT, reasoning, personality, mood/emotion decisions) is a **separate sibling project, `olaf_companion`**, which publishes 4 canonical ROS 2 topics (`mood`, `activity`, `speech_emotion`, `vocalization`). This repo subscribes to those and renders them on the body. The "personality-first" vision is unchanged at the *system* level; it's just split across two repos. See `docs/sprint-change-proposal-2026-05-15.md`.

```
┌─────────────────────────────────────────────────────────────────┐
│   INTELLIGENCE LAYER (Hybrid AI) — sibling repo olaf_companion  │
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
                       │ 4 canonical ROS 2 topics
                       │ (mood · activity · speech_emotion · vocalization)
┌──────────────────────▼──────────────────────────────────────────┐
│              ORCHESTRATION LAYER  ◀── THIS REPO                 │
│      Raspberry Pi 5 16GB + Fusion HAT+ + Hailo AI Kit          │
│               Ubuntu 24.04 • ROS2 Jazzy                         │
│                                                                 │
│  • Expression Engine (subscribes to olaf_companion's 4 topics) │
│  • Composes activity → mood → speech_emotion → vocalization    │
│  • Renders pose/LED/eye/heart from expression_map.yaml         │
│  • Direct Hardware Control (servos, LEDs, kickstand via HAT)   │
│  • I2C Master for ESP32 modules                                │
│                                                                 │
└──┬───────┬───────┬─────────────────┬───────────────────────────┘
   │       │       │                 │
   │I2C    │USB×2  │HAT WS2812       │HAT PWM
   │       │       │                 │
┌──▼───┐ ┌─▼─────────────┐ ┌────────▼───┐ ┌──▼──────┐
│ HEAD │ │ WAVESHARE USB │ │ INDICATOR  │ │KICKSTAND│
│ESP32 │ │ Neck + Ears   │ │ 24 LEDs    │ │ Servos  │
│(0x10)│ │ Servo Adapters│ │ (3 strips) │ │(at base)│
└──────┘ └───────────────┘ └────────────┘ └──▲──────┘
┌──▼───┐                                      │
│ BASE │                                      │
│ESP32 │                                      │
│(0x11)│                                      │
└──────┘
```

### The Five Modules (MECE Principle)

Each module owns its domain exclusively — no cross-dependencies. **Hybrid control:** ESP32s for real-time tasks, Pi direct control for latency-tolerant hardware.

**1. HEAD Module (ESP32, I2C 0x10)**
- **Hardware:** 2× GC9A01 round OLED eyes (240×240, SPI 30-60 FPS), ESP32-S3 for real-time eye animation
- **Personality:** Animated eye expressions, smooth emotional transitions, blink patterns
- **Why ESP32:** 60 FPS animation requires dedicated real-time control

**2. BASE Module (ESP32, I2C 0x11)**
- **Hardware:** Two-wheel inverted pendulum, hoverboard BLDC motors, ODrive v3.6 (UART), BNO085 9-axis AHRS (200Hz on-chip sensor fusion)
- **Control:** 200Hz PID balancing loop (hard real-time, Linux can't guarantee this)
- **Why ESP32:** Self-balancing requires hard real-time guarantees

**3. NECK Module (Pi-controlled, USB Serial)**
- **Hardware:** 3× Feetech STS3215 servos (pan/tilt/roll), Waveshare Bus Servo Adapter
- **Personality:** Head orientation, expressive gestures (tilts, nods, shakes) with organic easing
- **Why Pi Direct:** USB serial via Waveshare adapter — no soldering, plug-and-play

**4. EARS Module (Pi-controlled, USB Serial)**
- **Hardware:** 4× Feetech SCS0009 servos (2-DOF × 2 ears), Waveshare Bus Servo Adapter
- **Personality:** Chappie-inspired ears — perked = alert, drooping = sad, swiveling = curious
- **Why Pi Direct:** USB serial via Waveshare adapter — no soldering, plug-and-play

**5. INDICATOR Module (Pi-controlled, Fusion HAT WS2812)**
- **Hardware:** 3× 8-LED WS2812 strips (24 LEDs, daisy-chained)
- **Functions:** interaction state (listening/working/speaking) · module health · PID visualization
- **Why Pi Direct:** Single data line via Fusion HAT WS2812 port

**Additional Pi-controlled hardware:** 4" heart display (animated beating heart) · kickstand (2× landing-gear servos via HAT PWM) · I2S speaker via HAT.

### Why This Architecture?

- **Hybrid control topology** — ESP32s for the two real-time-critical jobs (60 FPS eyes, 200Hz balancing); Pi direct (via Fusion HAT) for everything else. Minimal soldering, fewer firmware projects, maximum reliability.
- **Modular MECE** — each module completable in 1-2 weekend sprints, independently testable, contribution-friendly.
- **Hybrid intelligence** — local AI (Hailo) for fast/private sensing; cloud agents for sophisticated reasoning.
- **ROS2 foundation** — industry-standard pub/sub on ROS2 Jazzy / Ubuntu 24.04, with a rich ecosystem (SLAM, navigation, sensor fusion).

---

## How Expression Works

The expression engine is a ROS2 node that turns "what the companion is feeling/doing" into coordinated motion + light across every channel — in real time, from a single data-driven map.

```
companion topics ─▶ activity base ─▶ + mood bias ─▶ + speech_emotion ─▶ + vocalization
(mood/activity/      (posture)        (LED tint /     (12 authored        (transient
 speech/voc)                           idle drift)     emotions)            gestures)
                                                              │
                                                              ▼
                                          eyes (OLED) · ears · neck · WS2812 LEDs
```

- **Single source of truth:** `ros2/src/expression_engine/config/expression_map.yaml` (+ `neck_motion.yaml`, `ears_motion.yaml`). See the [config index](ros2/src/expression_engine/config/README.md).
- **Additive composition:** later layers ride on top of earlier ones, so a vocalization gesture never wipes the underlying emotional pose.
- **Hand-authored, hardware-tuned:** every emotion is authored, then frozen after a pass on the real avatar.

---

## Current Status

**🔄 Active build-in-public (Phase 2 — Expression Engine).** This is a work-in-progress; here's what's real today.

**✅ Working on real hardware:**
- **Head / eyes** — 2× OLED eyes animated by ESP32 firmware; a full set of distinct expressions (the video above)
- **Ears** — 4× SCS0009 servos: perk / droop / swivel, hardware-calibrated
- **Neck** — 3× STS3215 servos: pan / tilt / roll with organic easing
- **Expression engine (ROS2 Jazzy)** — subscribes to the companion's 4 canonical topics and drives eyes + ears + neck + LEDs in sync
- **Idle behaviour** — head drifts to sleep when idle, stirs awake on activity
- **Status LEDs** — WS2812 strips reflect listening / working / speaking

**🔨 In progress / next:**
- Expression authoring finalization (Epic 7 — in review: speech emotions, vocalizations, activity postures, mood→LED tint)
- Heart display animation (Epic 8)
- Self-balancing base — PID tuning (experimental)
- SLAM navigation (deferred)

**🔗 Companion brain:** the AI/intelligence layer lives in the sibling repo **`olaf_companion`**, which publishes the 4 topics this body renders.

**Realistic timeline:** a complex build done in spare time while working full-time. Progress is incremental, iterative, and documented as it happens.

---

## 📹 Demo Videos

### May 2026
- **OLAF expression cycling** — *2026-05-20* — the expression engine cycling through OLAF's authored emotions on real hardware (OLED eyes, articulated ears, 3-DOF neck). [▶ Watch on YouTube](https://youtube.com/shorts/9D4Txb85EnM)

---

## Repository Organization

**Module-first structure.** Everything for a module lives in one place.

```
modules/
├── head/             # OLED eyes (ESP32, I2C 0x10)
├── base/             # Self-balancing (ESP32, I2C 0x11)
├── neck/             # 3-DOF servos (Pi USB serial)
├── ears/             # Articulated ears (Pi USB serial)
├── indicator/        # WS2812 LED strips (Pi Fusion HAT)
└── shared/           # Shared ESP32 libraries
```

**ESP32 modules** (head, base): `firmware/` (PlatformIO) · `hardware/` (3D models + BOM) · `tests/` · `wiring.md`.
**Pi-controlled modules** (neck, ears, indicator): `hardware/` · `tests/` · `assembly.md`.

**ROS2 lives separately:**
```
ros2/src/
├── olaf_drivers/       # Driver logic (head_ears, neck, base, torso)
├── expression_engine/  # Phase 2: renders olaf_companion's 4 canonical topics
└── olaf_ai/            # (deferred) STT/agents now in olaf_companion
```

**Full details:** see [repository-structure.md](docs/architecture/repository-structure.md).

---

## License & Credits

**License:** Open Source (TBD: MIT or Apache 2.0)

**Builder:** [Kamal Singh](https://www.linkedin.com/in/kamal-singh)
- 15 years in tech (5 at Amazon) · 1 year intentional AI deep-dive · MBA + full-stack (hardware + software + AI)

**Inspiration:** R2D2 (non-verbal personality) · JARVIS (AI partner, not servant) · Chappie (expressive ears) · Wall-E (friendly retro-futurism).

**Built with:** AI coding assistants (Claude, GPT-4) · Raspberry Pi 5 16GB / Ubuntu 24.04 · Sunfounder Fusion HAT+ · Hailo AI Kit (26 TOPS) · ROS2 Jazzy · ESP32-S3 · Waveshare Bus Servo Adapters · OnShape · PlatformIO.

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
