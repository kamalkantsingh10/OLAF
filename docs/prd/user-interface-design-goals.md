# User Interface Design Goals

Based on the project context, Olaf's "UI" is fundamentally different from traditional screen-based interfaces—it's a **multi-modal physical interaction system** combining R2D2-style non-verbal communication with floor projection for information display.

## Olaf Physical Design

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

## Overall UX Vision

Olaf's user experience prioritizes **emotional connection over efficiency**. Interactions should feel like communicating with a companion rather than operating a device. The interface design emphasizes:

- **Non-verbal communication first**: R2D2-style beeps, expressive movements, and animated eyes create personality without relying on human speech
- **Physical presence**: Olaf occupies space, moves, and orients toward users—leveraging embodiment for natural interaction
- **Ambient awareness**: Context-sensitive behaviors (mmWave presence detection, conversation history) make interactions feel thoughtful rather than transactional
- **Progressive disclosure**: Information starts with simple beeps/gestures, escalates to floor projection only when detailed content is needed
- **Playful engagement**: Personality intensity escalation, dynamic expressions, and unpredictable micro-behaviors keep interactions fresh

**Design Philosophy**: "Feel alive first, be useful second"—users should emotionally connect with Olaf before evaluating utility.

## Key Interaction Paradigms

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

## Core Screens and Views

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

## Accessibility

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

## Branding

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

## Target Device and Platforms

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
