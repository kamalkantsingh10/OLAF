# Olaf Branding Guide

**Version:** 1.0
**Last Updated:** 2025-10-10
**Purpose:** Visual identity, sound design, and brand guidelines for the Olaf open-source robotics project

---

## Brand Identity

### Aesthetic Philosophy

**"Retro-Futurism Meets Friendly Companion"**

Olaf's brand combines 1970s-80s sci-fi nostalgia (R2D2, Wall-E, TRON) with modern minimalism. The aesthetic embraces the DIY maker culture—celebrating visible craftsmanship over corporate polish.

**Design Principles:**
- **Approachable over intimidating**: Rounded forms, friendly colors, playful personality
- **Authentic over perfect**: Show the build process, embrace imperfections
- **Melodic over mechanical**: Musical beeps, smooth movements, organic behaviors
- **Transparent over mysterious**: Open-source everything, document failures and successes

---

## Color Palette

### Primary Chassis Colors

**3D Printed Parts:**
```
Base White:    RAL 9003 (Signal White) / Pantone Cool Gray 1C
               Hex: #F4F4F4
               RGB: 244, 244, 244
               Use: Main body panels, head housing

Accent Gray:   RAL 7035 (Light Gray)
               Hex: #D7D7D7
               RGB: 215, 215, 215
               Use: Joints, module connectors, secondary structures

Finish:        Matte (non-glossy)
               Rationale: Reduces glare, hides 3D print layer lines,
                         photographs well under various lighting
```

### OLED Eye Colors (Cyan Spectrum)

**Primary Eye Color: Cyan**
```
Cyan Base:     #00FFFF
               RGB: 0, 255, 255
               Use: Default eye color, alert states

Emotion Variations:
├─ Idle/Neutral:    #00B3B3 (Cyan 70% opacity)
├─ Alert/Curious:   #00FFFF (Cyan 100% full brightness)
├─ Happy:           #00FFFF + #FFFFFF highlights (white sparkles)
├─ Sad:             #006666 (Cyan 40% opacity, dimmed)
├─ Thinking:        #00B3B3 → #00FFFF pulsing (50-90% animated)
├─ Confused:        #00E6E6 → #B3B300 flickering (cyan to yellow-green)
└─ Excited:         #00FFFF ↔ #FFFFFF rapid alternation

Rationale:
- High contrast against white chassis
- Cyan conveys intelligence + friendliness (vs. red=danger, green=utilitarian)
- Avoids uncanny valley (not trying to replicate human eyes)
- Aligns with 1980s computer terminal aesthetic
```

### Status LED Color Language

**RGB Addressable LEDs (WS2812B or similar):**

```
System States:
🟢 Green (#00FF00):      Ready / Idle
🟡 Amber (#FFA500):      Processing / Thinking
🔴 Red (#FF0000):        Error / Critical Alert
🔵 Blue (#0066FF):       Quiet Mode Active
🟣 Purple (#9933FF):     Updating / Maintenance
⚪ White Pulse:          Listening / Voice Capture Active
🌈 Rainbow Cycle:        Boot Sequence / Self-Test

Breathing Pattern (Idle State):
- 3-second fade cycle: 30% → 100% → 30% brightness
- Sinusoidal easing (smooth, organic)
- Conveys "aliveness" without distraction

Quiet Mode:
- Solid Blue LED at 20% brightness
- Located on body front panel
- Indicates reduced audio/navigation mode
```

### Projection Colors

**DLP Projector Output:**
```
Default:       White light (#FFFFFF) on neutral floor surfaces
Text:          High contrast black text on white background
               OR white text on dark/color background (adaptive)
Accents:       Cyan highlights (#00FFFF) to match eye color
Emojis:        Full color when appropriate for emotional context

Floor Surface Adaptation:
- Light floors (wood, tile): Black text preferred
- Dark floors (dark carpet): White text + border
- Patterned surfaces: Solid background box behind text
```

---

## Sound Design

### Philosophy

**Musical over Mechanical:**
- Use musical intervals (major thirds, perfect fifths) for pleasant tones
- Layer 2-3 sine waves at harmonic frequencies for richness
- R2D2 as primary inspiration—conversational beep phrases

**Timing Guidelines:**
- Acknowledgments: <200ms (quick, responsive)
- Emotional expressions: 500-1500ms (sustained, recognizable)
- Thinking states: 800-2000ms (contemplative, patient)

### Emotion-to-Sound Mapping

**HAPPY (Major chord intervals - C Major)**
```
Intensity 1: C4→E4 (300ms)           [Simple ascending]
Intensity 2: C4→E4→G4 (200ms each)   [Two-note chirp]
Intensity 3: C5→E5 (3x, 150ms each)  [Triple bounce]
Intensity 4: C4→E4→G4→C5 (100ms)     [Rising arpeggio]
Intensity 5: C5↔E5 warble (1000ms)   [Excited rapid alternation]

Frequency References:
C4 = 261.63 Hz | E4 = 329.63 Hz | G4 = 392.00 Hz | C5 = 523.25 Hz
```

**CURIOUS (Ascending questions)**
```
Intensity 1: A3→A4 (500ms, slight bend)          [Slow questioning rise]
Intensity 2: A3→D4→A4 (250ms each)               [Two-step inquiry]
Intensity 3: A3→D4 (3x pattern, 200ms)           [Persistent questioning]
Intensity 4: A3→B3→C#4→E4 (100ms)                [Rapid scanning]
Intensity 5: Repeat Intensity 4 pattern 3x fast  [Intense curiosity]

Frequency References:
A3 = 220.00 Hz | D4 = 293.66 Hz | B3 = 246.94 Hz | C#4 = 277.18 Hz
```

**THINKING (Low contemplative tones)**
```
Intensity 1: G2 sustained (800ms)                 [Single low hum]
Intensity 2: G2→D3→G2 (400ms each)                [Two-note pondering]
Intensity 3: G2 with 5Hz vibrato (1000ms)         [Wobbling consideration]
Intensity 4: D3→C3→A2→G2 (250ms each)             [Descending thought]
Intensity 5: G2↔D3 irregular alternation (2000ms) [Deep processing]

Frequency References:
G2 = 98.00 Hz | A2 = 110.00 Hz | C3 = 130.81 Hz | D3 = 146.83 Hz
```

**CONFUSED (Discordant intervals - tritones)**
```
Intensity 1: C4→F#4 (300ms, unresolved)           [Slight confusion]
Intensity 2: C4 with ±50 cent drift (500ms)       [Pitch wobble]
Intensity 3: C4+F#4 simultaneous (400ms)          [Dissonant chord]
Intensity 4: C4→F#4→B3→E4 random timing           [Chaotic sequence]
Intensity 5: C4↔F#4 rapid alternation (1500ms)    [Bewildered warble]

Frequency References:
C4 = 261.63 Hz | F#4 = 369.99 Hz | B3 = 246.94 Hz
```

**SAD (Minor intervals, descending - A minor)**
```
Intensity 1: E4→C4 (500ms, slow)                  [Gentle fall]
Intensity 2: E4→C4→A3 (400ms each)                [Two-note sigh]
Intensity 3: E4→D4→C4→A3 (300ms each)             [Melancholy phrase]
Intensity 4: Repeat Int. 3 slower (500ms each)    [Drooping pattern]
Intensity 5: E4→D4→C4→B3→A3→G3 (400ms each)       [Mournful descent]

Frequency References:
E4 = 329.63 Hz | D4 = 293.66 Hz | A3 = 220.00 Hz | G3 = 196.00 Hz
```

**EXCITED (High-energy, rapid)**
```
Intensity 1: C5→E5 (150ms)                        [Quick chirp]
Intensity 2: C5→G5 (2x, 100ms each)               [Double bounce]
Intensity 3: C5→E5→G5 (3x repeat, 80ms)           [Triple burst]
Intensity 4: C5→E5→G5→C6 (50ms, rapid)            [Frenetic arpeggio]
Intensity 5: C5→E5→G5 cycling (2000ms, very fast) [Ecstatic trill]

Frequency References:
C5 = 523.25 Hz | E5 = 659.25 Hz | G5 = 783.99 Hz | C6 = 1046.50 Hz
```

**NEUTRAL (Simple confirmation tones)**
```
Intensity 1: A4 (200ms)                           [Single beep]
Intensity 2: A4 (2x, 150ms gap)                   [Double confirmation]
Intensity 3: A4 (3x, 200ms, 100ms gaps)           [Triple beep]

Frequency Reference:
A4 = 440.00 Hz (concert pitch)
```

### Quiet Mode Sound Profile

**Sub-Bass Rumbles (tactile feedback, minimal audible disturbance):**
```
Acknowledgment:  60Hz pulse, 100ms
Thinking:        40Hz sustained, 500ms
Confirmation:    80Hz→60Hz→40Hz descending, 300ms total
Error:           60Hz irregular pulse, 800ms

Volume Adjustments:
- Standard beeps: 30% volume (significant reduction)
- High frequencies (>2kHz): Further attenuated (less piercing)
- Preferred range: 200-800Hz mid-tones in Quiet Mode
- Sub-bass: Felt vibration more than heard sound
```

### Sound Implementation Notes

**Waveform Generation:**
- Primary: Sine waves (pure tones, least harsh)
- Harmonics: Add 2nd and 3rd harmonics at -12dB and -18dB for richness
- Envelope: 10ms attack, 20ms release (prevents clicks)

**Speaker Requirements:**
- Frequency response: 80Hz - 8kHz minimum
- Small piezo buzzer sufficient for beeps
- Upgrade option: 3W speaker for richer tones
- Quiet Mode: Requires speaker with good low-frequency response (<100Hz)

---

## Typography

### Documentation & Code

**Headers:**
```
Font:      JetBrains Mono Bold
Weight:    700
Usage:     Markdown headers, documentation titles
Rationale: Monospace reinforces "maker/hacker" aesthetic
```

**Body Text:**
```
Font:      Inter or Source Sans Pro
Weight:    400 (regular), 600 (semibold for emphasis)
Usage:     Documentation paragraphs, README content
Rationale: Clean, highly readable, excellent web/print rendering
```

**Code Blocks:**
```
Font:      Fira Code
Features:  Enable ligatures (==, ->, !=, etc.)
Usage:     Code examples, configuration files
Rationale: Programming ligatures improve code readability
```

**Diagrams:**
```
Style:     ASCII art using Unicode box-drawing characters
Chars:     ┌ ┐ └ ┘ ─ │ ├ ┤ ┬ ┴ ┼ ═ ║ ╔ ╗ ╚ ╝
Usage:     Architecture diagrams, flow charts in markdown
Rationale: Universal compatibility, version-control friendly
```

---

## Visual Identity

### Logo Concept

**ASCII Logo (for terminal/GitHub):**
```
   ___
  /o o\    ← Simplified eye representation (OLED displays)
 |  ^  |   ← Head module
  \___/
   |||     ← Neck articulation
  /   \    ← Body (Raspberry Pi housing)
 ●─────●   ← Differential drive wheels

"OLAF"
Open-Source Personal AI Companion
```

**Usage:**
- GitHub repository banner
- README header
- Terminal splash screen on boot
- 3D printable logo plate for body panel

### Photography Style Guide

**Build Progress Photos:**
- ✅ Natural workshop lighting (warm, authentic, not studio-sterile)
- ✅ Show wires, tools, work-in-progress (embrace DIY aesthetic)
- ✅ Include hands/tools in frame (human element, relatability)
- ✅ Mix close-ups (component details) with wide shots (full robot context)
- ✅ Capture failures and iterations (authentic learning journey)
- ❌ Avoid over-editing/Photoshop (keep realistic and achievable)
- ❌ No plain white backgrounds (feels commercial, not maker-space)
- ❌ Don't hide mistakes (they're part of the story)

**Demo & Interaction Videos:**
- ✅ Home environment (living room, kitchen) not sterile studio
- ✅ Emphasize personality (expressions, beeps, movements) in action
- ✅ Capture human reactions (smiles, surprise, emotional engagement)
- ✅ B-roll: Component assembly intercut with final functionality
- ✅ Show Olaf's perspective (camera following Olaf, mounted shots)
- ❌ Avoid scripted dialogue (keep conversational, natural)
- ❌ No generic EDM/stock music (use quirky retro synth or just Olaf's beeps)
- ❌ Don't over-produce (raw footage > polished corporate video)

**Lighting Recommendations:**
- Soft natural light (near windows) or warm LED panels
- Avoid harsh overhead fluorescents (creates unflattering shadows)
- For projection demos: Dim ambient light to show DLP output clearly
- Eye close-ups: Backlight the OLED displays for dramatic effect

---

## Platform-Specific Guidelines

### GitHub Repository

**Banner Image:**
- ASCII architecture diagram from PRD (Intelligence → Orchestration → Module layers)
- Cyan accent color for module highlights
- Monospace font, dark background (#1E1E1E or #0D1117)

**README Structure:**
```markdown
# 🤖 Olaf - Open-Source Personal AI Companion

[ASCII Logo Here]

**Tagline:** Build your own personality-first AI robot with modular ESP32 architecture

## 🔧 Features
- [Emoji bullets for key features]

## 📚 Documentation
- [Links to build guides, BOM, wiring diagrams]

## ⚡ Quick Start
- [Step-by-step getting started]
```

**Emoji Usage:**
- 🤖 Robot/Olaf references
- 🔧 Build/hardware topics
- 📚 Documentation
- ⚡ Features/capabilities
- 🎯 Goals/objectives
- 🧪 Experimental features
- 🐛 Known issues

**Color Scheme:**
- Dark theme default (GitHub dark mode)
- Cyan (#00FFFF) for code highlights, links
- Code blocks: Monokai or Dracula syntax theme

### LinkedIn Posts

**Content Strategy:**
- **Frequency:** 2-3 posts per week
  - 1x progress update (what was built this week)
  - 1-2x insights/learnings (technical deep-dives, failures, ah-ha moments)

**Post Structure:**
```
[Eye-catching opening line - ask question or bold statement]

[2-3 paragraphs: context → what was done → learning/result]

[Optional: Photo carousel showing process]

[Call-to-action: "What would you add to Olaf?" or "Link to full build in comments"]

#PhysicalAI #ROS2 #Robotics #MakerMovement #OpenSource #BuildInPublic
```

**Photo Guidelines:**
- Carousel format: 3-5 images showing progression
- First image: Most visually striking (hooks scrollers)
- Last image: Final result or teaser for next step
- Include failures/mistakes (authenticity > perfection)

**Tone:**
- Enthusiastic but technical (avoid hype, focus on substance)
- Educational (teach while sharing progress)
- Humble (acknowledge challenges, invite collaboration)
- Conversational (write like talking to fellow makers)

### YouTube Videos

**Thumbnail Style:**
```
Layout:
├─ Left 60%: Close-up of Olaf (eyes prominent, expressive pose)
├─ Right 40%: Bold text (3-5 words max)
└─ Background: Blurred workshop or solid color with cyan accent

Text Style:
- Font: Impact or Bebas Neue (bold, readable at small size)
- Color: White with black stroke (high contrast)
- Size: Large (25-30% of thumbnail height)

Examples:
- "Building Olaf's Brain" + close-up of Raspberry Pi
- "Personality System Works!" + excited eye expression
- "SLAM Navigation Test" + Olaf moving (motion blur)
```

**Video Structure:**
```
00:00 - Intro (5-10 sec): Olaf animation (eyes light up, beep, logo)
00:10 - Hook (15 sec): Show most interesting moment ("Watch what happens...")
00:25 - Context (30 sec): What we're building today and why
00:55 - Main Content (8-15 min): Build process with chapter markers
        ├─ Chapter 1: Parts Overview
        ├─ Chapter 2: Assembly
        ├─ Chapter 3: Wiring
        ├─ Chapter 4: Code
        └─ Chapter 5: Testing & Demo
End - Outro (30 sec): Results summary, next video tease, CTAs

Total Length: 10-20 minutes (in-depth, not rushed)
```

**Audio:**
- Voiceover: Conversational, not scripted (like explaining to friend)
- Background: Subtle retro synth or ambient (low volume, non-distracting)
- Olaf's beeps: Keep audible in demos (they're part of the personality)
- No loud intros/outros (respect viewer ears)

**End Screen:**
- Left: Link to GitHub repository
- Center: Subscribe button
- Right: Next video in series OR community Discord/forum

---

## Materials & Finishes

### 3D Printed Components

**Recommended Settings:**
```
Material:      PLA (beginner-friendly) or PETG (more durable)
Color:         White or Natural (unpigmented) - avoid bright colors
Layer Height:  0.2mm (balance of speed and quality)
Infill:        20% (sufficient strength, minimizes weight/material cost)
Wall Thickness: 3 perimeters (strong enough for mounting points)
Supports:      Minimal (design parts to minimize support needs)

Post-Processing:
- Optional: Light sanding on visible surfaces (120→220 grit)
- Optional: Matte clear coat spray (hides layer lines, protects surface)
- Not required: Parts functional and acceptable without post-processing
```

**Rationale:**
- White PLA: Easiest to print, great surface finish, photographs well
- PETG alternative: Better durability for parts with stress (neck joints)
- Unpigmented filament: Shows print quality, easier to source consistently
- 20% infill: Sweet spot for strength-to-weight (saves filament cost)

### Hardware & Fasteners

**Fastener Standards:**
```
Primary:       M3 (most common size for electronics enclosures)
Secondary:     M2.5 (smaller components), M4 (heavy-duty joints)
Material:      Stainless steel (corrosion-resistant, clean appearance)
               OR black oxide steel (matches dark electronics)
Avoid:         Brass (looks cheap, less durable)

Heat-Set Inserts:
- M3 x 5mm brass inserts for 3D printed parts
- Allows repeated assembly/disassembly without thread wear
```

**Wiring Standards:**
```
Wire Type:     Silicone-insulated stranded wire (flexible, durable)
Colors:
  - Red: Power (+)
  - Black: Ground (-)
  - Yellow: Signal/Data
  - Custom: Follow module-specific color codes in wiring diagrams

Gauge:
  - Power (motors, Pi): 18-20 AWG
  - Signals (I2C, servos): 22-24 AWG

Connectors:
  - JST-XH series (reliable, polarized, common)
  - Dupont (breadboard-compatible for prototyping)

Management:
  - Avoid "rainbow spaghetti" (use black with colored heat-shrink labels)
  - Cable ties or braided sleeving for harness organization
  - Leave 10-15cm service loops at modules (easier maintenance)
```

**Enclosures & Windows:**
```
Raspberry Pi:    Clear acrylic case (show the tech, educational)
Electronics:     Transparent or translucent covers where possible
Battery:         Enclosed (safety) but with charge LED visible

Rationale: "Transparent tech" aesthetic celebrates the engineering,
           invites learning and understanding
```

---

## Anti-Patterns (What to Avoid)

### Visual Anti-Patterns
- ❌ **Sleek Black Monolith:** Intimidating, corporate, loses friendly approachability
- ❌ **Humanoid Facial Features:** Risks uncanny valley, conflicts with personality-first beep communication
- ❌ **Overly Polished:** Makes DIY builders feel their version "isn't good enough"
- ❌ **RGB Everywhere:** Tacky gamer aesthetic, conflicts with retro-futurism simplicity
- ❌ **Glossy Finishes:** Shows fingerprints, highlights imperfections, photographs poorly

### Sound Anti-Patterns
- ❌ **Synthetic Speech:** Cheap TTS ruins personality (beeps > robotic voice)
- ❌ **Harsh Beeps:** Square waves, alarm sounds = annoying not endearing
- ❌ **Random Sounds:** Every beep should have meaning/context
- ❌ **Too Loud:** Should be audible but not startling (adjustable volume)

### Content Anti-Patterns
- ❌ **Hiding Failures:** Sanitized tutorials feel fake, discourage learners
- ❌ **Hype Over Substance:** Exaggerated claims damage credibility
- ❌ **Gatekeeping Language:** Assume knowledge without explaining (be inclusive)
- ❌ **Commercial Aesthetics:** Pristine studio shots alienate maker audience

---

## Brand Voice & Messaging

### Core Messages

**What Olaf Is:**
- Open-source modular robotics framework for physical AI
- Personality-first companion (emotional engagement before utility)
- Maker-accessible platform ($400-$1000, weekend-buildable modules)
- Community-driven ecosystem (build, share, extend)

**What Olaf Is Not:**
- Production-ready commercial product (it's a platform, a starting point)
- Competitor to Boston Dynamics or expensive robots (different tier, different goals)
- Educational toy (serious AI/robotics integration, but approachable)
- One-size-fits-all (modular = configurable to your needs/budget)

### Tone Guidelines

**Writing Style:**
- **Technical but accessible:** Explain concepts, define acronyms on first use
- **Enthusiastic but realistic:** Celebrate progress, acknowledge challenges
- **Inclusive:** "We" not "I" (community effort, not solo hero journey)
- **Humble:** Share mistakes, ask for help, credit contributors
- **Actionable:** Provide next steps, links, resources (enable others to build)

**Example Phrases:**
- ✅ "Here's what worked (and what didn't) this week..."
- ✅ "Huge thanks to @contributor for solving the servo jitter issue!"
- ✅ "If you're stuck on this step, here's a troubleshooting guide..."
- ✅ "Next up: tackling SLAM integration—any ROS2 experts want to collaborate?"
- ❌ "I'm the best roboticist, watch me dominate this build..."
- ❌ "This is the future of AI, it will change everything..."
- ❌ "It's so simple, anyone can do this in an afternoon..."

---

## Version History

| Version | Date       | Changes                                      | Author          |
|---------|------------|----------------------------------------------|-----------------|
| 1.0     | 2025-10-10 | Initial branding guide creation              | John (PM Agent) |

---

## Contributing to Brand Guidelines

This branding guide evolves with the project. Contributions welcome:

1. **Propose changes:** Open GitHub issue with "Branding:" prefix
2. **Submit assets:** Logo variations, sound files, design templates
3. **Share examples:** Photos/videos following (or improving) these guidelines
4. **Discuss:** Community feedback shapes brand evolution

**Guiding Principle:** Branding serves the project, not the other way around. If a guideline hinders building, sharing, or learning—change it.
