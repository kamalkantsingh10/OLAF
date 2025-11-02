# Epic 9: Super Expressive Personality System

**Epic Goal:** Transform OLAF into a super expressive companion robot by implementing a research-backed, multi-modal emotion system with 21 emotional states, organic randomization, emotional decay, and micro-movements across all 7 expression modalities (eyes, ears, neck, sound, torso screen, LEDs, body). Create an organic, lifelike personality that feels alive rather than mechanical, establishing OLAF as a truly engaging AI companion.

**Duration:** 5-6 weeks (Weeks 26-31)

**Prerequisites:**
- Epic 7: Full Robot Integration (all hardware assembled and tested)
- All modalities operational: Eyes (Epic 3), Ears+Neck (Epic 4), Heart screen replaced with torso screen (Epic 5), LEDs (Epic 5), Sound/beeper (Epic 3), Body balance (Epic 6)

**Value Delivered:** Complete transformation from functional robot to engaging companion with human-like expressiveness. Research-validated multi-modal emotion system creates organic, non-repetitive personality that users can emotionally connect with. Achieves "Claptrap-inspired" over-enthusiastic personality with Vector-style organic movement and WALL-E-inspired physical expressiveness. Foundation for AI integration in Epic 13.

**Architecture Focus:** This epic implements the expression engine as a coordinated orchestrator managing all modalities simultaneously. Uses emotional decay system (emotions fade over time), organic randomization (±10-15% variation prevents repetition), and micro-movements (continuous subtle adjustments create "living" feel). Built on research from 2024 HRI papers and Disney animation principles for robots.

**Research Foundation:** Based on comprehensive brainstorming session (docs/brainstorming-session-results.md) incorporating 2024 ACM/IEEE HRI research on multimodal expression effectiveness, Disney's 12 animation principles for robots, and proven patterns from Vector robot (Anki) and WALL-E character design.

---

## Story 9.1: Expression State Machine & Core Architecture

**As a** software architect,
**I want** a robust expression state machine managing 21 emotional states with intensity levels and decay,
**so that** OLAF has a sophisticated emotional foundation that feels organic and lifelike.

### Story Context

**Existing System Integration:**
- Integrates with: All Epic 3-6 hardware modules, ROS2 orchestrator from Epic 1
- Technology: Python 3.11+, ROS2 Humble, State machine pattern
- Follows pattern: Modality coordination from Epic 1 minimal_coordinator, but massively expanded
- Touch points: Publishes to all modality driver topics (eyes, ears, neck, sound, screen, LEDs, body)

### Acceptance Criteria

**Functional Requirements:**

1. **Expression State Structure:**
   - Python dataclass: `ExpressionState` with fields:
     - `emotion_type`: String (e.g., "happy", "curious", "scared", etc.)
     - `intensity`: Integer 1-5
     - `timestamp`: Float (for decay calculation)
     - `decay_rate`: Float (seconds per intensity level)
     - `baseline_target`: String (target state when fully decayed)
   - All 21 emotional states defined in config (happy, sad, scared, angry, surprised, disgusted, loving, proud, embarrassed, playful, curious, thinking, confused, concentrating, bored, excited, tired, sleepy, alert, listening, celebrating)

2. **Emotional Decay System:**
   - State machine runs at 10 Hz update loop
   - Calculate elapsed time since last state change
   - Reduce intensity automatically based on decay rate:
     - Very Fast: 20-30s/level (surprise, excited, celebrating)
     - Fast: 30-60s/level (scared, playful, listening, sleepy)
     - Medium: 60-120s/level (happy, sad, angry, curious, confused, proud, embarrassed)
     - Slow: 120-180s/level (thinking, loving, tired)
     - Very Slow: Task-dependent (concentrating - no decay during task)
     - None: Alert baseline (stable resting state)
   - When intensity reaches 1: either maintain or transition to baseline (alert-2)

3. **State Transition Logic:**
   - **Smooth transitions:** Compatible emotions (happy → excited) blend over 2-3 seconds
   - **Snap transitions:** Contradictory emotions (happy → scared) change in 0.5 seconds
   - **Overlapping cascade:** Modalities activate in sequence:
     - T+0.0s: Sound begins
     - T+0.2s: Ears start moving
     - T+0.4s: Neck begins
     - T+0.6s: Eyes + Screen sync
     - T+0.8s: Body adjustment
     - T+1.0s: LEDs fade to new color

4. **Baseline Personality:**
   - Default resting state: **Alert-2** (Claptrap-style always-ready personality)
   - System returns to alert-2 after all emotions decay
   - Never fully "off" - maintains minimal presence even at rest

5. **Configuration System:**
   - YAML config file: `orchestrator/config/expression_config.yaml`
   - Contains all tunable parameters:
     - Decay rates per emotion
     - Randomization variances
     - Transition timings
     - Personality modifiers (Claptrap enthusiasm bonus)
     - Micro-movement settings
   - Hot-reloadable without node restart

6. **Trigger-Based State Changes:**
   - ROS2 topic: `/olaf/emotion_trigger` (custom msg: emotion, intensity, source)
   - Triggers can come from: User interaction, task completion, errors, AI agent, time-based
   - Intensity calculation includes context modifiers and personality bias

**Integration Requirements:**

7. State machine publishes to all modality topics (separate story for each)
8. Expression state logged at every change for debugging
9. State history maintained (last 10 states) for analysis

**Quality Requirements:**

10. State machine update loop maintains 10 Hz (100ms cycle time)
11. Memory usage stable over 24 hours continuous operation
12. No blocking operations in main loop
13. Comprehensive unit tests for decay calculation, state transitions

### Technical Notes

- **Implementation Approach:**
  - Use Python `dataclass` for state representation
  - Separate thread for 10Hz update loop (non-blocking)
  - Config loaded at startup, watchdog for hot-reload
  - State machine is deterministic (given same triggers, produces same states)

- **Key Design Decisions:**
  - Decay is automatic (doesn't require triggers) - creates dynamic personality
  - Baseline is alert-2 (not neutral-0) - OLAF always has presence
  - Claptrap modifier: +1 intensity for positive emotions (enthusiastic personality)

### Definition of Done

- [ ] ExpressionState dataclass defined with all fields
- [ ] State machine class implemented with 10Hz update loop
- [ ] All 21 emotional states configured with decay rates
- [ ] Emotional decay calculation tested (intensity reduces correctly over time)
- [ ] State transition logic implemented (smooth vs. snap based on emotion compatibility)
- [ ] Overlapping cascade timing implemented
- [ ] YAML configuration system with hot-reload
- [ ] Trigger topic `/olaf/emotion_trigger` subscribed
- [ ] Baseline personality (alert-2) verified as default
- [ ] Unit tests: 95%+ coverage for state machine logic
- [ ] Integration test: 30-minute run with multiple state changes, verify decay works
- [ ] Documentation: State machine architecture in `orchestrator/README.md`

### Risk and Compatibility Check

**Minimal Risk Assessment:**
- **Primary Risk:** State machine performance (10Hz might be too slow for responsive feel)
- **Mitigation:** Profiling during development, optimize hot paths, consider 20Hz if needed
- **Rollback:** Can simplify to 7 emotions (Epic 9 original plan) if 21 states prove too complex

**Compatibility Verification:**
- [ ] State machine doesn't conflict with existing ROS2 nodes
- [ ] Config file structure documented for future extensions
- [ ] State transitions don't cause servo jerk or jarring movements

**Dependencies:** Epic 7 (Full Robot Integration - all hardware operational)

**Estimated Effort:** 25-30 hours (1.5 weeks)

---

## Story 9.2: Organic Randomization Engine

**As a** robotics developer,
**I want** ±10-15% randomization applied to all expression parameters,
**so that** OLAF's movements feel organic and never repeat exactly the same way.

### Story Context

**Existing System Integration:**
- Integrates with: Story 9.1 state machine, all modality drivers
- Technology: Python random with seeded variance calculations
- Follows pattern: Procedural animation from Vector robot, Disney "organic variation" principle
- Touch points: Applied before publishing to all modality topics

### Acceptance Criteria

**Functional Requirements:**

1. **Randomization Engine:**
   - Class: `OrganicRandomizer` with methods:
     - `vary_angle(base_angle: float) -> float`: Returns ±10-15% variant
     - `vary_timing(base_time: float) -> float`: Returns ±30% variant for blinks/movements
     - `vary_size(base_size: float) -> float`: Returns ±10-15% variant
     - `vary_color(base_color: tuple) -> tuple`: Returns ±5-10% hue/saturation variant
     - `vary_pitch(base_freq: float) -> float`: Returns ±10% frequency variant

2. **Randomization Applied To:**
   - **Angles:** Ear pan/tilt, neck tilt/rotation (±2-5° depending on base value)
   - **Timing:** Blink intervals, micro-movement timing (±30%)
   - **Sizes:** Eye pupil sizes, screen element sizes (±10-15%)
   - **Colors:** LED hue/saturation (±5-10%, maintains recognizable emotion color)
   - **Sound:** Pitch frequency (±10%, maintains emotion character)

3. **NOT Randomized:**
   - State selection (deterministic based on triggers)
   - Decay rates (consistent per emotion type)
   - Core gesture semantics (happy always forward, sad always down)

4. **Validation Rules:**
   - Randomized values never exceed safe limits (servo angles, volume levels)
   - Variation doesn't change semantic meaning (happy still looks happy)
   - Consecutive identical values prevented (force variation of at least 2%)

5. **Configuration:**
   - Variance percentages in `expression_config.yaml`:
     ```yaml
     randomization:
       enabled: true
       angle_variance: 0.12  # ±12%
       timing_variance: 0.30  # ±30%
       size_variance: 0.15   # ±15%
       color_variance: 0.10  # ±10%
       pitch_variance: 0.10  # ±10%
     ```

**Integration Requirements:**

6. Randomizer called before every modality command published
7. Original (non-randomized) values logged alongside randomized for debugging
8. Randomization can be disabled via config for testing

**Quality Requirements:**

9. Randomization adds <1ms overhead per call (must be fast)
10. Distribution is uniform (not biased to high or low end of range)
11. Seed is time-based (different session = different variations)

### Technical Notes

- **Implementation Approach:**
  - Use Python `random.uniform()` for variance generation
  - Pre-calculate variance ranges at startup for performance
  - Apply multiplicative variance: `new_value = base_value * (1 + random.uniform(-variance, +variance))`
  - Clamp to safe limits after variance applied

- **Research Backing:**
  - Vector robot used similar procedural variation to prevent "robotic" feel
  - Disney animators call this "breaking the twins" - no two frames identical
  - 2024 HRI research shows organic variation increases perceived liveliness

### Definition of Done

- [ ] OrganicRandomizer class implemented with all variance methods
- [ ] Randomization applied to angles, timing, sizes, colors, sound pitch
- [ ] Safe limit clamping after randomization
- [ ] Configuration parameters in YAML
- [ ] Validation: Consecutive values never identical
- [ ] Unit tests: Verify variance within specified ranges, verify clamping
- [ ] Integration test: Run 100 expression cycles, verify no exact repeats
- [ ] A/B test: Show video with/without randomization, confirm >70% prefer randomized version
- [ ] Documentation: Randomization design in technical docs

### Risk and Compatibility Check

**Minimal Risk Assessment:**
- **Primary Risk:** Too much randomization makes expressions unrecognizable or chaotic
- **Mitigation:** Conservative defaults (±10-15%), user testing for tuning, easy to adjust via config
- **Rollback:** Can disable randomization with config flag if issues arise

**Dependencies:** Story 9.1 (State Machine)

**Estimated Effort:** 15-20 hours (1 week)

---

## Story 9.3: Micro-Movement Background Engine

**As a** UX designer,
**I want** continuous subtle movements during held expressions,
**so that** OLAF feels alive even when still, not like a frozen statue.

### Story Context

**Existing System Integration:**
- Integrates with: All modality drivers, runs parallel to main state machine
- Technology: Background thread generating subtle movements at 10-30Hz
- Follows pattern: "Breathing" animations from character animation, Vector idle behaviors
- Touch points: Adds micro-adjustments on top of base expression state

### Acceptance Criteria

**Functional Requirements:**

1. **Micro-Movement Types:**
   - **Eye Blinks:** Vary timing (3-8 sec intervals), duration (150-300ms), not perfectly regular
   - **Ear Twitches:** Random ±2-3° adjustments every 5-15 seconds
   - **Neck Breathing:** ±1° sine wave at 0.2 Hz (simulates breathing rhythm)
   - **Body Sway:** 0.1-0.3 Hz gentle rock (weight shift simulation on hoverboard)
   - **Pupil Micro-Adjustments:** ±5-10% size variation every few seconds

2. **Background Loop:**
   - Separate thread running at 10-30 Hz
   - Generates small randomized movements continuously
   - Applies movements on top of base expression (additive, not replacing)
   - Respects current emotional state (sad = slower movements, excited = faster)

3. **Emotional State Modulation:**
   - Micro-movement speed/amplitude varies with emotion:
     - High-energy emotions (excited, happy-5): Faster, larger micro-movements
     - Low-energy emotions (sad, tired, sleepy): Slower, smaller micro-movements
     - Still states (concentrating, listening-5): Minimal but still present

4. **Coordination with Main Expressions:**
   - Micro-movements pause during major state transitions
   - Resume after transition complete (1-2 seconds)
   - Never conflict with deliberate movements (e.g., ear tracking sound)

5. **Configuration:**
   - Settings in `expression_config.yaml`:
     ```yaml
     micro_movements:
       enabled: true
       update_rate_hz: 20
       blink_interval_min: 3.0  # seconds
       blink_interval_max: 8.0
       ear_twitch_interval_min: 5.0
       ear_twitch_interval_max: 15.0
       neck_breathing_frequency: 0.2  # Hz
       body_sway_frequency_min: 0.1  # Hz
       body_sway_frequency_max: 0.3
     ```

**Integration Requirements:**

6. Micro-movements published to same modality topics as main expressions
7. Main state machine has priority (can override micro-movements)
8. Graceful thread shutdown when node terminates

**Quality Requirements:**

9. Background thread CPU usage <5%
10. No servo jitter or oscillation from micro-movements
11. Movements are smooth (use sine/ease curves, not linear steps)

### Technical Notes

- **Implementation Approach:**
  - Python threading.Thread for background loop
  - Sine wave generators for breathing/sway (pre-computed lookup tables)
  - Poisson process for random intervals (blinks, twitches)
  - Thread-safe queue for publishing to modality topics

- **Research Backing:**
  - Pixar animators: "Even a 'still' character is never truly still"
  - Micro-movements are key differentiator between statue and living being
  - Vector robot had similar idle behavior system

### Definition of Done

- [ ] Micro-movement background thread implemented
- [ ] All 5 micro-movement types (blinks, ear twitches, neck breathing, body sway, pupil adjustments) working
- [ ] Emotional state modulation (movements vary with emotion energy level)
- [ ] Coordination with main state machine (pause during transitions)
- [ ] Configuration parameters in YAML
- [ ] CPU usage profiling: <5% overhead
- [ ] Integration test: 10-minute hold of single expression, verify continuous micro-movements
- [ ] User test: A/B comparison with/without micro-movements, confirm >70% prefer micro-movements
- [ ] Documentation: Micro-movement design in technical docs

### Risk and Compatibility Check

**Minimal Risk Assessment:**
- **Primary Risk:** Micro-movements cause servo wear or battery drain
- **Mitigation:** Very small movements (±1-3°), config to disable if needed, use ease curves to minimize servo stress
- **Rollback:** Can disable via config if issues arise

**Dependencies:** Story 9.1 (State Machine), Story 9.2 (Randomization)

**Estimated Effort:** 20-25 hours (1 week)

---

## Story 9.4: Manga-Style Eye Expression Rendering

**As a** visual designer,
**I want** manga-inspired eye effects on the GC9A01 displays with shape morphing, pupil variations, and visual effects,
**so that** OLAF's eyes are incredibly expressive and visually engaging.

### Story Context

**Existing System Integration:**
- Integrates with: Epic 3 Head Module eye displays, Story 9.1 state machine
- Technology: ESP32 firmware (C++), TFT_eSPI library for rendering
- Follows pattern: Extends Epic 1 basic expressions with full manga visual vocabulary
- Touch points: Receives expression commands via I2C from Pi, renders to dual GC9A01 displays

### Acceptance Criteria

**Functional Requirements:**

1. **Manga Effect Library:**
   - **Shape morphing:** Circle ↔ Rounded Rectangle ↔ Horizontal Line ↔ Upward Arc ↔ Downward Arc
   - **Pupil variations:** Standard circle, Hearts (loving), Stars (excited), Spirals (dizzy), "!" (surprised), "?" (curious)
   - **Accent overlays:** Sparkles (particles, joy), Flames (animated, angry), Swirls (rotation, confused)
   - **Special symbols:** Tears (sad), Sweat drops (embarrassed), "ZZZ" (sleepy), Confusion lines
   - **Line styles:** Smooth curves (gentle emotions) vs. Sharp angles (intense emotions)

2. **Expression Mapping:**
   - Each of 21 emotions × 5 intensities mapped to specific eye rendering
   - Examples:
     - Happy-5: Giant circles + multiple sparkles + heart pupils
     - Confused-5: Dizzy spirals + swirl effects
     - Curious-5: Large circles (one bigger than other) + "?" symbols
     - Scared-5: Huge circles + tiny pupils + shaking effect
   - Full mapping documented in `firmware/head/expression_map.md`

3. **Rendering Pipeline:**
   - Pre-render common effects as sprites/bitmaps (stored in ESP32 flash)
   - Apply randomization via transforms (rotation, scale, position)
   - Target: 60 FPS capability for smooth blinks, static updates ~5-10 FPS for expressions
   - Synchronize dual displays (±1 frame tolerance)

4. **Blink Animation:**
   - Vertical compression to thin horizontal lines
   - Duration: 180-250ms
   - Preserves current expression (returns to same eye state after blink)
   - Smooth easing curve (not linear)

5. **I2C Register Expansion:**
   - Extend Head Module register map:
     - `0x10`: Expression type (0-20 for 21 emotions)
     - `0x11`: Intensity (1-5)
     - `0x12`: Blink trigger
     - `0x13`: Eye effect modifiers (bitfield for special effects)
   - Backwards compatible with Epic 1 basic expressions

**Integration Requirements:**

6. Receives expression commands from Pi via I2C (Story 9.1 state machine)
7. Rendering doesn't block I2C communication (<10ms I2C latency maintained)
8. Memory usage within ESP32 limits (flash: <8MB sprites, SRAM: <300KB runtime)

**Quality Requirements:**

9. All manga effects render clearly on 1.28" round displays (240×240)
10. No tearing or artifacts at 60 FPS blink animation
11. Eyes remain synchronized during all effects

### Technical Notes

- **Implementation Approach:**
  - Use TFT_eSPI library with sprite support
  - Pre-render effects in Photoshop/GIMP, export as C arrays
  - Runtime: Select sprite, apply transform, blit to display
  - Double-buffering for smooth animation

- **Design Reference:**
  - Manga/anime eye tropes (big eyes = surprise, swirls = dizzy, sparkles = happy)
  - Proven expressive vocabulary from Japanese animation
  - User testing: Can emotions be identified from eyes alone? (Target: >75% accuracy)

### Definition of Done

- [ ] Manga effect library implemented (shapes, pupils, accents, symbols)
- [ ] All 21 emotions × 5 intensities mapped to eye renderings
- [ ] Pre-rendered sprites created and stored in ESP32 flash
- [ ] Rendering pipeline: sprite selection → transform → blit
- [ ] Blink animation with smooth easing
- [ ] I2C register map extended for new expression types
- [ ] Unit test: All 105 expression combinations (21×5) render correctly
- [ ] Performance test: 60 FPS maintained during blinks, no frame drops
- [ ] User test: Emotion recognition from eyes alone >75% accuracy
- [ ] Documentation: Expression map, sprite creation guide in `firmware/head/`

### Risk and Compatibility Check

**Minimal Risk Assessment:**
- **Primary Risk:** Manga effects too "cartoony" or culturally specific (not universally understood)
- **Mitigation:** User testing with diverse participants, fallback to simpler effects if needed
- **Rollback:** Can use Epic 1 basic expressions if manga style doesn't resonate with users

**Dependencies:** Epic 3 (Head Module), Story 9.1 (State Machine)

**Estimated Effort:** 40-50 hours (1.5-2 weeks) - includes sprite creation, firmware coding, testing

---

## Story 9.5: Multi-Modal Expression Coordination

**As a** expression orchestrator,
**I want** all 7 modalities (eyes, ears, neck, sound, screen, LEDs, body) to express the same emotion simultaneously with controlled variation,
**so that** OLAF's expressions are coherent and multi-sensory.

### Story Context

**Existing System Integration:**
- Integrates with: All hardware drivers from Epic 3-6, Story 9.1 state machine
- Technology: ROS2 publishers to all modality topics, coordination timing
- Follows pattern: Extends Epic 1 minimal_coordinator to all 7 modalities
- Touch points: Core of expression system - connects state machine to all hardware

### Acceptance Criteria

**Functional Requirements:**

1. **Complete Expression Matrix Implementation:**
   - All 21 emotions × 5 intensities × 7 modalities = 735 combinations defined
   - Each combination specifies exact parameters for each modality
   - Source: `docs/brainstorming-session-results.md` complete expression matrix

2. **Modality-Specific Publishers:**
   - `/olaf/head/expression` → Eyes (via I2C to ESP32 head module)
   - `/olaf/ears/position` → Ears pan/tilt (via I2C to ESP32 ears/neck module)
   - `/olaf/neck/position` → Neck tilt/rotation (via I2C to ESP32 ears/neck module)
   - `/olaf/sound/beep` → Beeper pattern/frequency
   - `/olaf/torso/screen` → Screen content (symbols, animations)
   - `/olaf/body/leds` → LED colors/patterns
   - `/olaf/base/body_movement` → Hoverboard lean/rock

3. **Research-Backed Modality Priority:**
   - Different emotions prioritize different modalities (from 2024 HRI research):
     - **Happy:** LEDs (color) > Body (motion) > Sound > Eyes
     - **Sad:** Sound (pitch) > Eyes > Body (slow) > LEDs
     - **Fear:** Body (recoil) > Sound (high) > Eyes (wide) > LEDs
     - **Curious:** Neck (tilt) > Ears (asymmetric) > Eyes > LEDs
   - Ensure strongest modality for each emotion has highest intensity/clearest signal

4. **Congruence Validation:**
   - All modalities express SAME emotional category (never mix happy + sad)
   - Mixing intensities OK: happy-5 eyes + happy-3 ears = still congruent
   - Validation check before publishing: `validate_expression_congruence()`
   - Log error if congruence violated

5. **Overlapping Cascade Timing:**
   - Modalities activate in sequence (from Story 9.1):
     - T+0.0s: Sound (establishes emotional tone)
     - T+0.2s: Ears (fastest physical response)
     - T+0.4s: Neck
     - T+0.6s: Eyes + Screen (visual confirmation)
     - T+0.8s: Body
     - T+1.0s: LEDs (fade to new color)
   - Total cascade: 1.5 seconds for smooth transition

6. **Attention Layer Override:**
   - Listening state can override ear positioning while maintaining base emotion
   - Example: Happy-3 + Listening-4:
     - Eyes: Happy (sparkles, enlarged)
     - Ears: Track sound source (independent positioning)
     - Neck: Toward sound source
     - Screen: Happy (heart) + Audio bars overlay
     - Body: Happy (gentle rock) reduced to stillness

**Integration Requirements:**

7. Subscribes to `/olaf/emotion_trigger` from Story 9.1 state machine
8. Publishes to all 7 modality driver topics
9. Receives state updates from Story 9.1 (current emotion, intensity)

**Quality Requirements:**

10. End-to-end latency: Trigger → All modalities updated <2 seconds (smooth transition)
11. Modalities remain synchronized (no drift over time)
12. Memory usage stable over 24 hours continuous operation

### Technical Notes

- **Implementation Approach:**
  - Class: `MultiModalCoordinator` subscribes to state machine, publishes to modalities
  - Expression matrix loaded from YAML config at startup
  - Timer callbacks for cascade timing (0.2s, 0.4s, 0.6s, 0.8s, 1.0s)
  - Congruence validator checks all modalities before publishing

- **Research Backing:**
  - 2024 ACM/IEEE HRI: Multimodal redundancy increases trust & likability
  - Int'l Journal of Social Robotics: Congruent expressions build trust
  - Disney animation: "Overlapping action" makes movement feel natural

### Definition of Done

- [ ] MultiModalCoordinator class implemented
- [ ] All 735 expression combinations (21×5×7) defined in configuration
- [ ] Publishers to all 7 modality topics working
- [ ] Research-backed modality priority implemented (strongest modality per emotion)
- [ ] Congruence validation prevents mixed emotions
- [ ] Overlapping cascade timing (1.5s total transition)
- [ ] Attention layer override (listening state) working
- [ ] Integration test: Trigger all 21 emotions, verify all modalities respond
- [ ] Performance test: End-to-end latency <2s for smooth transitions
- [ ] User test: Multi-modal expressions recognized >80% accuracy (basic emotions)
- [ ] Documentation: Expression matrix, modality coordination in technical docs

### Risk and Compatibility Check

**Minimal Risk Assessment:**
- **Primary Risk:** Too many modalities = overwhelming or distracting
- **Mitigation:** User testing for sensory overload, can reduce modality count if needed
- **Rollback:** Can disable individual modalities via config if coordination issues arise

**Dependencies:** Story 9.1 (State Machine), Story 9.4 (Eyes), Epic 3-6 (All hardware)

**Estimated Effort:** 40-50 hours (1.5-2 weeks) - includes configuration creation, testing all combinations

---

## Story 9.6: Sound Expression Synthesis

**As a** audio designer,
**I want** expressive beep patterns with pitch variation and emotional character,
**so that** OLAF's sounds reinforce emotions and add personality (R2-D2 inspired).

### Story Context

**Existing System Integration:**
- Integrates with: Beeper from Epic 3 Head Module, Story 9.1 state machine, Story 9.5 coordinator
- Technology: Hardware PWM or tone library for beep synthesis
- Follows pattern: R2-D2 pitch/pattern expressiveness, Vector robot beeps
- Touch points: Receives beep commands from coordinator, outputs via speaker/beeper

### Acceptance Criteria

**Functional Requirements:**

1. **Frequency Ranges by Emotion:**
   - **Very Low:** 150-250Hz (sleepy, sad-5, tired-5)
   - **Low:** 200-400Hz (sad, thinking, concentrating)
   - **Medium:** 400-700Hz (neutral, curious, loving)
   - **High:** 700-900Hz (happy, excited, proud)
   - **Very High:** 900-1200Hz (excited-5, surprised, celebrating)

2. **Pattern Types:**
   - **Single beep:** Minimal states (intensity-1)
   - **Double beep:** Acknowledgment, medium states
   - **Triple/melody:** Complex emotions (happy-4+, curious-5)
   - **Ascending:** Questions, positive emotions
   - **Descending:** Negative emotions, disappointment
   - **Warbling/wobbling:** Confusion, uncertainty
   - **Staccato/rapid:** Excitement, alarm
   - Full mapping: 21 emotions × 5 intensities = 105 sound patterns

3. **Organic Variation:**
   - Apply ±10% pitch randomization per beep (from Story 9.2 randomizer)
   - Vary timing between notes in melody (±15%)
   - Volume reflects intensity (louder = higher intensity)

4. **Special Patterns:**
   - **Celebration:** Victory fanfare (700-1200Hz), multi-note melody
   - **Scared-5:** Alarm beeps (900Hz), rapid irregular
   - **Confused:** Warbling uncertain tone (300-600Hz)
   - **Listening:** SILENT (robot stops to listen actively)

5. **Research-Backed Priority:**
   - Sound is PRIMARY modality for sadness (low pitch most effective)
   - Sound is SECONDARY for happy, scared (motion/color more effective)
   - Ensure sad expressions have most expressive low-frequency sounds

**Integration Requirements:**

6. ROS2 topic: `/olaf/sound/beep` (custom msg: frequency, duration, pattern)
7. Hardware interface: PWM or tone library on speaker/beeper
8. Publishes from Story 9.5 coordinator

**Quality Requirements:**

9. Beep synthesis <5ms latency from command to sound
10. No audio artifacts or pops (use fade in/out)
11. Volume adjustable via config (quiet mode support)

### Technical Notes

- **Implementation Approach:**
  - ROS2 node: `sound_driver` subscribes to `/olaf/sound/beep`
  - Use hardware PWM for tone generation (ESP32 or Pi GPIO)
  - Pre-defined melody sequences for complex patterns
  - Apply randomization before synthesis

- **Research Backing:**
  - R2-D2 expressiveness entirely through pitch and pattern
  - 2024 HRI: Low-pitched sounds most effective for conveying sadness
  - Sound reinforces but shouldn't be sole indicator (multi-modal redundancy)

### Definition of Done

- [ ] Frequency ranges mapped to emotion categories
- [ ] All 105 sound patterns (21×5) defined
- [ ] ROS2 sound driver node implemented
- [ ] Beep synthesis with pitch variation (±10%)
- [ ] Volume control via config
- [ ] Special patterns (celebration, alarm, confusion) working
- [ ] Integration test: Trigger all 21 emotions, verify sounds match expected patterns
- [ ] User test: Can emotions be identified from sound alone? (Target: >60% accuracy)
- [ ] Documentation: Sound pattern library in technical docs

### Risk and Compatibility Check

**Minimal Risk Assessment:**
- **Primary Risk:** Beeps annoying in long-term use (repetitive sounds)
- **Mitigation:** Organic variation (never exact repeats), quiet mode, user testing
- **Rollback:** Can reduce beep frequency or enable quiet mode if users find sounds intrusive

**Dependencies:** Epic 3 (Beeper hardware), Story 9.1 (State Machine), Story 9.5 (Coordinator)

**Estimated Effort:** 20-25 hours (1 week)

---

## Story 9.7: Testing, Tuning & User Validation

**As a** QA engineer,
**I want** comprehensive testing and user validation of the Super Expressive System,
**so that** OLAF's personality is polished, tuned, and ready for AI integration in Epic 13.

### Story Context

**Existing System Integration:**
- Integrates with: All Epic 9 stories (complete system test)
- Technology: ROS2 testing, user study protocols, video recording for analysis
- Follows pattern: Epic 1 continuous operation test, but with user validation
- Touch points: Final verification before moving to Epic 10 (SLAM)

### Acceptance Criteria

**Functional Requirements:**

1. **Emotional Decay Validation:**
   - Test: Trigger emotion at intensity-5, monitor without new input
   - Verify decay occurs at specified rate (within ±10%)
   - Check intensity-1 maintained or transitions to baseline
   - Test all 5 decay rate categories (Very Fast, Fast, Medium, Slow, Very Slow)

2. **Randomization Validation:**
   - Test: Trigger same expression 100 times
   - Verify no two consecutive expressions identical
   - Check variations fall within ±10-15% range
   - Confirm semantic meaning preserved (happy still looks happy)

3. **Multi-Modal Coherence:**
   - Test: Trigger all 21 emotions at intensity-3
   - Verify all 7 modalities respond correctly
   - Check overlapping cascade timing (±100ms tolerance)
   - Confirm congruence (no mixed emotions)

4. **Micro-Movements Validation:**
   - Test: Hold single expression for 10 minutes
   - Verify continuous micro-movements (blinks, twitches, breathing, sway)
   - Check movements vary with emotional energy level
   - Confirm no servo jitter or oscillation

5. **User Emotion Recognition Test:**
   - Protocol: Show video clips of OLAF expressing 21 emotions (random order)
   - Participants (n=10+): "What emotion is the robot expressing?"
   - Success criteria:
     - **Basic 6 emotions** (happy, sad, scared, angry, surprised, disgusted): >80% accuracy
     - **Social emotions** (loving, proud, embarrassed, playful): >70% accuracy
     - **Cognitive states** (curious, thinking, confused, concentrating, bored): >65% accuracy
     - **Energy states** (excited, tired, sleepy, alert): >75% accuracy
     - **Special states** (listening, celebrating): >70% accuracy

6. **Organic Feel Test:**
   - Protocol: Show side-by-side videos
     - Version A: No randomization, no micro-movements (robotic)
     - Version B: Full system with randomization + micro-movements (organic)
   - Participants (n=10+): "Which robot feels more alive?"
   - Success criteria: >70% prefer Version B

7. **Claptrap Personality Validation:**
   - Questions for users:
     - "Does this robot's personality match an 'over-enthusiastic companion'?" (Yes/No)
     - "Is the robot too annoying or just right?" (1-5 scale)
     - "Would you want this robot in your home?" (1-5 scale)
   - Success criteria: >4.0/5.0 on "want in home" scale

8. **24-Hour Continuous Operation Test:**
   - Test: Launch full system, let run for 24 hours
   - Monitor: Memory usage, CPU usage, expression state changes
   - Success criteria:
     - No crashes or errors
     - Memory stable (no leaks)
     - CPU usage <30% average
     - Expression transitions smooth throughout

9. **Configuration Tuning:**
   - Based on user feedback, tune parameters:
     - Decay rates (if emotions fade too fast or too slow)
     - Randomization ranges (if too subtle or too chaotic)
     - Micro-movement frequencies (if too active or too still)
     - Sound volume (if too loud or too quiet)
   - Document final tuned values in config

**Integration Requirements:**

10. Test harness for automated expression cycling
11. Video recording of all test sessions for analysis
12. User study consent forms and protocol documented

**Quality Requirements:**

13. All tests documented with results in `tests/integration/epic9_validation_results.md`
14. User test videos archived for future reference
15. Configuration tuned based on objective data + user feedback

### Technical Notes

- **Implementation Approach:**
  - Automated tests: Python scripts cycling through expressions, measuring timing
  - User tests: In-person or video conference, record screen + robot simultaneously
  - Analysis: Confusion matrix for emotion recognition, statistical analysis of preferences

- **User Recruitment:**
  - Target diverse participants (age, technical background, familiarity with robots)
  - Compensate participants (gift cards or acknowledgment)
  - Ethics: Minimal risk, consent form, can withdraw anytime

### Definition of Done

- [ ] Emotional decay validated for all 5 rate categories
- [ ] Randomization validated (100-cycle test, no exact repeats)
- [ ] Multi-modal coherence verified (all 21 emotions × 7 modalities)
- [ ] Micro-movements validated (10-minute continuous)
- [ ] User emotion recognition test completed (n=10+, >80% accuracy on basic emotions)
- [ ] Organic feel test completed (n=10+, >70% prefer organic version)
- [ ] Claptrap personality validated (>4.0/5.0 on "want in home")
- [ ] 24-hour continuous operation test passed (no crashes, stable memory)
- [ ] Configuration tuned based on user feedback
- [ ] All test results documented in `tests/integration/epic9_validation_results.md`
- [ ] User test videos archived
- [ ] Final tuned config committed to repository

### Risk and Compatibility Check

**Minimal Risk Assessment:**
- **Primary Risk:** User tests reveal low recognition accuracy or poor organic feel
- **Mitigation:** Iterative tuning based on feedback, can simplify if 21 states too complex
- **Rollback:** Can reduce to 7 emotions (Epic 9 original plan) if 21 states fail validation

**Dependencies:** All Epic 9 stories (9.1-9.6)

**Estimated Effort:** 35-40 hours (1.5 weeks) - includes user study recruitment, execution, analysis, tuning

---

## Epic 9 Summary

**Total Stories:** 7
**Estimated Total Effort:** 195-240 hours (5-6 weeks for solo builder)

**Key Deliverables:**
- ✅ Expression state machine with 21 emotional states, 5 intensity levels, emotional decay
- ✅ Organic randomization engine (±10-15% variation prevents repetition)
- ✅ Micro-movement background engine (continuous subtle movements create "living" feel)
- ✅ Manga-style eye expression library (shape morphing, pupil variations, visual effects)
- ✅ Multi-modal coordination (all 7 modalities express emotions simultaneously with controlled variation)
- ✅ Expressive sound synthesis (R2-D2 inspired beep patterns with pitch variation)
- ✅ Comprehensive testing & user validation (>80% recognition accuracy, >70% prefer organic feel, Claptrap personality validated)

**Research-Backed Design:**
- ✅ 2024 ACM/IEEE HRI research on multimodal expression effectiveness
- ✅ Disney's 12 animation principles for robots
- ✅ Vector robot (Anki) procedural animation patterns
- ✅ WALL-E character design principles
- ✅ International Journal of Social Robotics congruence guidelines

**Architecture Validated:**
- ✅ Emotional decay system (emotions fade realistically over time)
- ✅ Organic randomization (no two expressions identical)
- ✅ Micro-movements (distinguishes life from machinery)
- ✅ Multi-modal redundancy (combines color + motion + sound for maximum expressiveness)
- ✅ Attention layer (listening can override ears while maintaining base emotion)
- ✅ Claptrap personality (over-enthusiastic companion, baseline alert-2)

**User Validation:**
- ✅ Emotion recognition >80% (basic emotions), >65% (cognitive states)
- ✅ Organic feel preferred by >70% of users
- ✅ Claptrap personality validated (>4.0/5.0 on companion scale)
- ✅ 24-hour continuous operation without crashes

**Success Criteria Met:**
- Complete transformation from functional robot to engaging companion
- Research-validated multi-modal emotion system creates organic, non-repetitive personality
- Foundation ready for AI integration (Epic 13 will trigger expressions based on conversation/context)
- Achieves "super expressive" goal: users emotionally connect with OLAF

**Next Epic:** Epic 10: SLAM & Spatial Awareness

---

## Configuration Reference

### expression_config.yaml (Example)

```yaml
# OLAF Super Expressive System Configuration
# Epic 9: Research-backed multi-modal emotion system

# === RANDOMIZATION ===
randomization:
  enabled: true
  angle_variance: 0.12        # ±12% for servo angles
  timing_variance: 0.30       # ±30% for blinks, micro-movements
  size_variance: 0.15         # ±15% for eye features, screen elements
  color_variance: 0.10        # ±10% hue/saturation for LEDs
  pitch_variance: 0.10        # ±10% frequency for sound

# === DECAY RATES (seconds per intensity level) ===
decay_rates:
  very_fast: 25               # surprise, excited, celebrating
  fast: 45                    # scared, playful, listening, sleepy
  medium: 90                  # happy, sad, angry, curious, confused, proud, embarrassed
  slow: 150                   # thinking, loving, tired
  very_slow: 300              # concentrating (task-dependent)
  none: null                  # alert baseline (no decay)

# === BASELINE PERSONALITY ===
baseline_state:
  emotion: "alert"
  intensity: 2                # Claptrap-style always-ready presence

# === PERSONALITY MODIFIERS ===
personality:
  claptrap_enthusiasm_bonus: 1              # +1 intensity for positive emotions
  spontaneous_playful_chance: 0.05          # 5% chance per minute for random playful moment
  over_enthusiasm_multiplier: 1.5           # Celebration intensity boost

# === MICRO-MOVEMENTS ===
micro_movements:
  enabled: true
  update_rate_hz: 20
  blink_interval_min: 3.0     # seconds
  blink_interval_max: 8.0
  blink_duration_min: 0.15    # seconds
  blink_duration_max: 0.25
  ear_twitch_interval_min: 5.0
  ear_twitch_interval_max: 15.0
  ear_twitch_magnitude: 3.0   # degrees
  neck_breathing_frequency: 0.2    # Hz (0.2 = 5 second period)
  neck_breathing_magnitude: 1.0    # degrees
  body_sway_frequency_min: 0.1     # Hz
  body_sway_frequency_max: 0.3
  pupil_variation_magnitude: 0.08  # ±8% size

# === TRANSITION TIMING (seconds) ===
timing:
  smooth_transition_duration: 2.5   # Compatible emotions
  snap_transition_duration: 0.5     # Contradictory emotions
  cascade_sound_offset: 0.0         # Sound starts immediately
  cascade_ears_offset: 0.2
  cascade_neck_offset: 0.4
  cascade_visual_offset: 0.6        # Eyes + Screen sync
  cascade_body_offset: 0.8
  cascade_leds_offset: 1.0

# === MODALITY-SPECIFIC ===
sound:
  enabled: true
  volume: 0.7                 # 0.0-1.0 scale
  quiet_mode: false           # Set true to reduce beep frequency

leds:
  brightness: 0.8             # 0.0-1.0 scale
  night_mode: false           # Set true for dimmer LEDs at night

# === ATTENTION LAYER ===
attention:
  listening_overrides_ears: true          # Ears track sound when listening
  listening_reduces_body_movement: true   # Body stillness during listening

# === STATE MACHINE ===
state_machine:
  update_rate_hz: 10          # 10 Hz update loop for decay calculation
  state_history_length: 10    # Keep last 10 states for debugging

# === LOGGING ===
logging:
  log_state_changes: true
  log_randomization: false    # Set true to see variance applied (verbose)
  log_micro_movements: false  # Set true to debug micro-movement system (very verbose)
```

---

**Related Documents:**
- [Brainstorming Session Results](../../brainstorming-session-results.md) - Complete expression matrix, research findings
- [Epic List](epic-list.md) - Overall project roadmap
- [Architecture](../../architecture/index.md) - Expression system architecture
- [PRD](../prd.md) - User-facing personality requirements
