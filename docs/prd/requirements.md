# Requirements

## Functional

**Architecture Foundation**
1. **FR1:** Each hardware module (Head, Ears, Neck, Projector, Base) shall operate as an independent smart peripheral with its own ESP32 controller acting as an I2C slave device (I2C addresses 0x08-0x0C)
2. **FR2:** ROS2 nodes shall run exclusively on Raspberry Pi orchestrator; driver nodes shall communicate with ESP32 modules via I2C bus (400kHz-1MHz) for <100ms latency
3. **FR3:** Each ESP32 module shall contain embedded intelligence (hardware drivers, animation engines, sensor processing, real-time control loops) executing high-level semantic commands from orchestrator (e.g., "express happy level 3", "move forward 0.5 m/s")
4. **FR4:** Each module shall be testable in standalone mode without requiring other modules to be connected
5. **FR5:** The system shall provide a test harness framework for standalone module validation supporting automated and manual testing
6. **FR6:** The system shall gracefully degrade functionality when optional modules are disconnected according to the following matrix:

| Missing Module | System Behavior |
|---------------|-----------------|
| Projector | Conversation continues; visual info delivered via beeps/speech only |
| Ears | Expression uses eyes + neck + beeps; reduced emotional range |
| Neck | Expression uses eyes + ears + beeps; stationary head orientation |
| RGBD Camera | Navigation disabled; stationary mode only; conversation functional |
| Base | Stationary companion mode; all other functions operational |

**Personality Expression System**
7. **FR7:** The OLED eye displays shall be driven via SPI interface (not I2C) at 10-20 MHz to achieve 30-60 FPS smooth animation rendering on ESP32 Head module
8. **FR8:** The OLED eye displays shall support a two-dimensional expression system with emotion types (happy, curious, thinking, confused, sad, excited, neutral) and intensity levels (1-5 scale where 1=subtle, 5=extreme)
9. **FR9:** Eye expressions shall be validated through user testing showing >80% correct emotion type identification and >70% correct intensity assessment (±1 level tolerance)
10. **FR10:** The articulated ears shall support 2 degrees of freedom each (vertical tilt, horizontal rotation) using Feetech SCS0009 serial bus servos, enabling independent directional movement and emotional gestures with intensity mapping (1=minimal movement, 5=exaggerated movement)
11. **FR11:** The articulated neck shall support 3 degrees of freedom (pan ±90°, tilt ±45°, roll ±30°) using Feetech STS3215 serial bus servos for head orientation and expressive movements with intensity-scaled animation speed (see FAQ for pan/tilt/roll definitions)
12. **FR12:** The system shall generate R2D2-style beeping sounds with tonal variations (pitch, duration, rhythm) mapped to emotion type and intensity level
13. **FR13:** The orchestrator shall coordinate eyes, ears, neck, and beeps into unified emotional states (e.g., "express excitement level 4" triggers synchronized high-energy ear wiggle + wide-eye animation + rapid happy beep sequence) with <500ms synchronization tolerance between expression channels
14. **FR14:** The system shall support dynamic emotion intensity escalation during conversations (e.g., enthusiasm builds from level 1→5 across multi-turn interaction)

**Conversational AI Integration**
15. **FR15:** The system shall capture voice input via microphone array for voice command detection
16. **FR16:** The system shall use Hailo-8L AI accelerator (13 TOPS) for local Whisper speech-to-text inference achieving <200ms latency (eliminating 1-1.5s cloud STT delay)
17. **FR17:** The system shall integrate with cloud AI API (Claude API for V1) to process natural language with conversational understanding
18. **FR18:** The system shall maintain conversation history and context across multiple interactions with persistence across power cycles in SQLite database
19. **FR19:** The AI layer shall route user requests to appropriate module functions based on intent analysis (e.g., "show me a recipe" → activate projector + fetch content)
20. **FR20:** The system shall implement retry logic with exponential backoff (initial 1s, max 10s, 3 attempts) for cloud AI API failures before gracefully falling back to "thinking" expression and error acknowledgment
21. **FR21:** The system shall log interaction events (timestamp, type, duration, user satisfaction signal if available) to enable usage analysis and daily interaction tracking

**Mobility & Navigation**
22. **FR22:** The mobility base shall implement two-wheel self-balancing using MPU6050 IMU with 200Hz PID control loop running on ESP32 Base module (Linux orchestrator cannot guarantee real-time control)
23. **FR23:** The Base module shall implement a state machine with states: RELAXED (kickstand down, motors idle), TRANSITIONING (retracting kickstand), BALANCING (active PID control), EMERGENCY_STOP (fall detected: |pitch| > 45°)
24. **FR24:** The mobility base shall support differential drive velocity commands via ODrive v3.6 motor controller interfaced through UART from ESP32 Base module
25. **FR25:** The Base module shall provide closed-loop odometry data (wheel encoder feedback via ODrive) at 10Hz for SLAM navigation with ±10cm accuracy requirement
26. **FR26:** The system shall implement SLAM using Google Cartographer library for autonomous navigation within mapped indoor environments using RGBD camera depth data
27. **FR27:** The system shall detect obstacles using RGBD camera depth sensing during navigation with collision avoidance behaviors via Nav2 stack

**Information Display**
28. **FR28:** The floor projector shall display text, images, charts, and dynamic content on floor surfaces
29. **FR29:** The projection system shall support AI-generated visual elements (emojis, thinking indicators, status information) to complement personality expression
30. **FR30:** The system shall detect human presence using mmWave sensor (V1 implementation) managed by Head ESP32 module to enable context-aware behaviors and proactive interaction initiation

**Over-The-Air Updates**
31. **FR31:** All ESP32 modules shall support OTA (Over-The-Air) firmware updates in V1 to enable rapid iteration without robot disassembly
32. **FR32:** The Raspberry Pi orchestrator shall host an HTTP server (Flask) serving firmware binaries with module-specific partition management and rollback capability

## Non-Functional

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

## FAQ: Technical Terminology

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
