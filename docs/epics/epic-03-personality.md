# Epic 3: Complete Personality Expression System

**Epic Goal:** Implement the full two-dimensional expression system (7 emotion types × 5 intensity levels) with coordinated multimodal output across eyes, beeper, ears, and neck to create rich, nuanced personality expressions.

**Dependencies:**
- Epic 1 (Foundation + Minimal Personality) must be complete
- Epic 2 (3D Design & Physical Structure) must be complete

**Estimated Effort:** 60-80 hours (2-3 weeks)

---

## User Stories

### Story 3.1: Expression State Manager

**As a** developer implementing personality,
**I want** a centralized expression state manager in the orchestrator,
**so that** all modules coordinate to display the same emotional state simultaneously.

#### Acceptance Criteria:
1. Expression state manager node created in orchestrator (Python/ROS2)
2. Supports 7 emotion types: HAPPY, SAD, CURIOUS, SURPRISED, CONTENT, THINKING, SLEEPY
3. Supports 5 intensity levels: 1 (minimal) → 5 (maximum)
4. Publishes `/olaf/expression/current_state` topic with emotion type + intensity
5. Provides service `/olaf/expression/set_state` to change expression
6. State transitions support optional fade duration (0-2 seconds)
7. Default expression is CONTENT at intensity 2
8. Unit tests verify all 35 expression combinations

**Technical Notes:**
- Use ROS2 custom message type: `ExpressionState.msg` with fields: `emotion_type`, `intensity`, `timestamp`
- State manager validates emotion type and intensity range
- Fade duration interpolates between states for smooth transitions

---

### Story 3.2: Eye Expression Library

**As a** user interacting with Olaf,
**I want** the eyes to display distinct patterns for each emotion and intensity,
**so that** I can read Olaf's emotional state at a glance.

#### Acceptance Criteria:
1. Head module subscribes to `/olaf/expression/current_state`
2. Eye patterns implemented for all 35 expression combinations
3. Patterns use cyan (#00FFFF) with varying brightness/animation:
   - HAPPY: Wide circles, brightness increases with intensity, blink rate increases
   - SAD: Downturned curves, dimmer at higher intensity, slow blink
   - CURIOUS: Wide open, alternating blink, intensity = pupil dilation
   - SURPRISED: Maximum dilation, static (no blink), intensity = how long held
   - CONTENT: Gentle pulse (0.5Hz), consistent across intensities
   - THINKING: Asymmetric (one eye squinted), intensity = squint degree
   - SLEEPY: Half-closed lids, slow blink (3s), intensity = how closed
4. Smooth transitions between patterns (fade duration respected)
5. Eye patterns documented with ASCII diagrams in wiki
6. Demo mode cycles through all 35 expressions (2 seconds each)

**Technical Notes:**
- Use OLED frame buffer for smooth animation
- Pre-render common patterns to reduce CPU load
- Brightness range: 30% (intensity 1) → 100% (intensity 5)

---

### Story 3.3: Audio Beeper Expression Library

**As a** user interacting with Olaf,
**I want** distinct beep patterns for each emotion and intensity,
**so that** I can hear Olaf's emotional state even when not looking at it.

#### Acceptance Criteria:
1. Head module audio component subscribes to `/olaf/expression/current_state`
2. Beep patterns implemented for all 35 expression combinations (per branding guide)
3. Sound design follows emotion-specific frequency patterns:
   - HAPPY: Major chord intervals (C4→E4→G4), faster at higher intensity
   - SAD: Minor chord intervals (A3→C4→E4), slower/lower at higher intensity
   - CURIOUS: Rising arpeggio (C4→E4→G4→C5), speed increases with intensity
   - SURPRISED: Sharp staccato burst (C5, 50ms), repetitions increase with intensity
   - CONTENT: Gentle pulse (C4, 300ms), consistent across intensities
   - THINKING: Intermittent blips (G4, 100ms every 2s), frequency increases with intensity
   - SLEEPY: Descending tone (G4→C4), slower at higher intensity
4. Volume range: 50% (intensity 1) → 100% (intensity 5)
5. Quiet mode override: max 30% volume
6. Audio library documented with frequency tables
7. Demo mode plays all 35 beep patterns sequentially

**Technical Notes:**
- Use passive piezo buzzer (PWM frequency control)
- Pre-calculate frequency sequences to avoid real-time computation
- Implement non-blocking audio playback (async)

---

### Story 3.4: Ear Articulation Expression Mapping

**As a** user interacting with Olaf,
**I want** the ears to move in emotion-appropriate ways,
**so that** Olaf's body language reinforces its emotional state.

#### Acceptance Criteria:
1. Ear module subscribes to `/olaf/expression/current_state`
2. Ear positions mapped for all 7 emotion types (intensity affects speed/magnitude):
   - HAPPY: Perked up, wiggle side-to-side at higher intensity
   - SAD: Drooped down, static
   - CURIOUS: One ear forward, one back (asymmetric), intensity = angle
   - SURPRISED: Both fully forward, intensity = how fast they snap forward
   - CONTENT: Neutral position (45° up), gentle sway
   - THINKING: One ear tilted, other neutral, intensity = tilt degree
   - SLEEPY: Drooped down and back, intensity = droop angle
3. Servo position ranges: Forward 0° → Back 180° (neutral = 90°)
4. Movement speed varies with intensity: 50 ms/step (intensity 1) → 10 ms/step (intensity 5)
5. Smooth interpolation between positions during state transitions
6. Emergency stop if servo current exceeds safe threshold
7. Demo mode cycles through all ear positions

**Technical Notes:**
- Use Feetech SCS0009 servo position feedback for closed-loop control
- Implement ramping to avoid sudden jerks
- Ear position library stored as lookup table (emotion → servo angles)

---

### Story 3.5: Neck Gesture Expression Mapping

**As a** user interacting with Olaf,
**I want** the neck to perform emotion-appropriate gestures,
**so that** Olaf's head movements feel natural and expressive.

#### Acceptance Criteria:
1. Neck module subscribes to `/olaf/expression/current_state`
2. Neck gestures mapped for all 7 emotion types:
   - HAPPY: Gentle nod (tilt 0° → +15° → 0°), faster at higher intensity
   - SAD: Head down (tilt -20° to -40° based on intensity), static
   - CURIOUS: Head tilt to side (roll ±15° to ±30°), intensity = tilt angle
   - SURPRISED: Quick tilt back (tilt +30°), snap speed based on intensity
   - CONTENT: Slow pan left-right (±20°), smooth consistent motion
   - THINKING: Head tilt up slightly (tilt +10°), pan side-to-side slowly
   - SLEEPY: Head droop forward (tilt -15° to -35°), slow drift
3. Gestures loop continuously while expression is active
4. Smooth transitions when expression changes
5. Emergency stop if servo current exceeds safe threshold
6. Demo mode cycles through all neck gestures
7. Gesture library documented with range-of-motion diagrams

**Technical Notes:**
- Use STS3215 servos (3-DOF: pan, tilt, roll)
- Pan range: ±90°, Tilt range: ±45°, Roll range: ±30°
- Implement inverse kinematics for smooth multi-axis motion
- Gesture library stored as keyframe sequences

---

### Story 3.6: Coordinated Multimodal Expression Test

**As a** QA tester,
**I want** to verify all modules respond simultaneously to expression changes,
**so that** Olaf's personality feels cohesive and synchronized.

#### Acceptance Criteria:
1. Integration test suite verifies coordination:
   - Set expression state via service call
   - Measure latency: state published → eyes update (< 50ms)
   - Measure latency: state published → beeper sounds (< 100ms)
   - Measure latency: state published → ears move (< 150ms)
   - Measure latency: state published → neck moves (< 150ms)
2. Test all 35 expression combinations for synchronization
3. Test expression transitions (e.g., HAPPY-5 → SAD-3) with 1-second fade
4. Test rapid expression changes (< 500ms apart) for queue handling
5. Test expression priority (e.g., SURPRISED overrides current state)
6. System remains synchronized for 30 minutes of random expression changes
7. CPU usage remains < 40% during continuous expression changes

**Technical Notes:**
- Use ROS2 bag recording to verify timing
- Create automated test script that cycles through all combinations
- Log timestamps for each module's response

---

### Story 3.7: Expression Trigger API

**As an** AI orchestrator component,
**I want** a simple API to trigger expressions based on conversation context,
**so that** Olaf responds emotionally to user interactions.

#### Acceptance Criteria:
1. Python API created: `olaf_expression_api.py`
2. Methods provided:
   - `set_expression(emotion_type, intensity, fade_duration=0.5)`
   - `get_current_expression()` → returns emotion type + intensity
   - `express_for_duration(emotion_type, intensity, duration_sec)` → auto-revert after duration
   - `queue_expression_sequence([(emotion, intensity, duration), ...])` → plays sequence
3. API validates emotion types and intensity ranges (raises exceptions)
4. API is thread-safe for concurrent calls
5. Example scripts demonstrate usage:
   - Simple expression change
   - Timed expression (surprise for 2 seconds, then revert to content)
   - Expression sequence (thinking → surprised → happy)
6. API documented with docstrings and examples
7. Unit tests cover all methods and edge cases

**Technical Notes:**
- API wraps ROS2 service calls for ease of use
- Implements asynchronous queue for expression sequences
- Revert behavior uses stack to restore previous state

---

### Story 3.8: Context-Aware Expression Rules

**As a** user having a conversation with Olaf,
**I want** Olaf to automatically express emotions based on conversation topics,
**so that** interactions feel natural without manual expression triggering.

#### Acceptance Criteria:
1. Expression rules engine created in orchestrator
2. Rules map keywords/sentiment to expressions:
   - Positive sentiment → HAPPY (intensity based on sentiment score 0.0-1.0)
   - Negative sentiment → SAD (intensity based on sentiment score)
   - Questions from user → CURIOUS (intensity 3)
   - Unexpected input/errors → SURPRISED (intensity 4)
   - Long pauses in conversation → SLEEPY (intensity increases over time)
   - Processing AI response → THINKING (intensity 3)
   - Default idle state → CONTENT (intensity 2)
3. Rules engine subscribes to `/olaf/conversation/user_input` topic
4. Sentiment analysis performed on user input (uses lightweight model)
5. Expression changes automatically based on conversation flow
6. Manual expression overrides (via API) take priority over automatic rules
7. Rules configurable via YAML file: `expression_rules.yaml`
8. Rules engine tested with 20 sample conversation scenarios

**Technical Notes:**
- Use VADER sentiment analysis (lightweight, no GPU required)
- Implement hysteresis to avoid flickering between expressions
- Timer-based transitions (e.g., THINKING → CONTENT after 5s if no new input)

---

### Story 3.9: Expression Logging & Analytics

**As a** developer debugging personality issues,
**I want** detailed logs of expression changes and timing,
**so that** I can identify coordination problems and optimize performance.

#### Acceptance Criteria:
1. Expression logger node publishes to `/olaf/expression/log` topic
2. Logs include:
   - Timestamp (ms precision)
   - Expression type + intensity
   - Trigger source (API call, rule engine, manual)
   - Module response times (eyes, beeper, ears, neck)
   - Any errors or warnings
3. Logs stored to file: `~/.olaf/logs/expression_YYYYMMDD.log` (daily rotation)
4. Log viewer CLI tool: `olaf-expression-log --tail` (live view)
5. Analytics dashboard shows:
   - Most common expressions (histogram)
   - Average module response times
   - Expression transition frequency
   - Error rate by module
6. Dashboard accessible via web browser: `http://localhost:8080/expression-analytics`
7. Log retention: 30 days, auto-cleanup of old logs

**Technical Notes:**
- Use ROS2 logging framework + custom handlers
- Dashboard built with Flask + Chart.js
- Log format: JSON for easy parsing

---

### Story 3.10: Power-Saving Expression Modes

**As a** user concerned about battery life,
**I want** expression intensity to automatically reduce when battery is low,
**so that** Olaf extends runtime without fully shutting down.

#### Acceptance Criteria:
1. Power-saving expression mode activates when battery < 20%
2. In power-saving mode:
   - Maximum expression intensity capped at 3 (even if requested 4-5)
   - Eye brightness reduced to 50%
   - Beeper volume reduced to 50%
   - Ear movements slowed by 2× (double the time per step)
   - Neck movements slowed by 2×
3. Warning expression triggered when entering power-saving mode:
   - SLEEPY at intensity 3 for 5 seconds
   - Single low beep (C3, 500ms)
4. Power-saving mode overridden by manual API call with `force=True` parameter
5. Normal mode resumes when battery > 25% (hysteresis prevents flapping)
6. Power-saving status published to `/olaf/system/power_saving_active`
7. Tested with simulated battery discharge scenario

**Technical Notes:**
- Subscribe to `/olaf/battery/state` topic for battery percentage
- Implement middleware layer that intercepts expression commands
- Adjusts intensity/brightness before forwarding to modules

---

### Story 3.11: Documentation & Content Creation

**As a** community member learning about Olaf's personality system,
**I want** comprehensive documentation and demo videos,
**so that** I can understand and extend the expression system.

#### Acceptance Criteria:
1. Wiki pages created:
   - Expression system architecture diagram
   - Expression library reference (all 35 combinations with photos)
   - API usage guide with code examples
   - Rules engine configuration guide
   - Troubleshooting common expression issues
2. README.md updated with Epic 3 completion status
3. Code comments added to all expression-related modules
4. YouTube video recorded (8-12 minutes):
   - Demo of all 7 emotion types at varying intensities
   - Coordinated multimodal expression showcase
   - API usage walkthrough
   - Rules engine demo (conversation-triggered expressions)
5. LinkedIn post with key highlights and video link
6. GitHub discussion thread for community feedback on expressions
7. All Epic 3 code merged to main branch with passing CI/CD

**Technical Notes:**
- Record video with good lighting to capture eye expressions clearly
- Use demo mode to cycle through expressions automatically
- Include B-roll of individual modules (close-ups of ears, neck, eyes)

---

## Definition of Done

- [ ] All 11 user stories completed with passing acceptance criteria
- [ ] 35 expression combinations (7 emotions × 5 intensities) implemented and tested
- [ ] Multimodal coordination verified (< 150ms latency across all modules)
- [ ] Expression API functional and documented
- [ ] Context-aware rules engine triggers expressions automatically
- [ ] Power-saving mode reduces expression intensity when battery < 20%
- [ ] Expression logging and analytics dashboard operational
- [ ] 30-minute continuous operation test passed with no coordination errors
- [ ] Documentation complete with expression library reference
- [ ] YouTube video published showcasing personality system
- [ ] All code reviewed, commented, and merged to main branch

---

## Success Metrics

- **Coordination Latency:** < 150ms for all modules to respond to expression changes
- **Expression Coverage:** All 35 combinations implemented and distinct
- **System Uptime:** 30 minutes continuous operation without errors
- **CPU Usage:** < 40% during active expression changes
- **Battery Impact:** Power-saving mode extends runtime by ≥ 20%
- **Community Engagement:** ≥ 100 YouTube views, ≥ 20 LinkedIn reactions within first week

---

## Technical Dependencies

- Epic 1: ROS2 foundation, minimal head module, orchestrator
- Epic 2: Physical structure with ears, neck, eyes, beeper assembled
- Python libraries: `rclpy`, `flask`, `pyyaml`, `vaderSentiment`
- Hardware: Feetech SCS0009 (ears), STS3215 (neck), dual OLEDs (eyes), piezo buzzer
