---
stepsCompleted: ["step-01-validate-prerequisites", "step-02-design-epics", "step-03-create-stories"]
inputDocuments:
  - docs/planning-artifacts/prd/phase2-prd.md
  - docs/planning-artifacts/architecture/phase2-expression-engine.md
---

# OLAF Phase 2: Expression Engine - Epic Breakdown

## Overview

This document provides the complete epic and story breakdown for OLAF Phase 2 (Expression Engine), decomposing the requirements from the Phase 2 PRD and the Phase 2 Architecture (`phase2-expression-engine.md`) into implementable stories. There is no UX Design document — Phase 2 has no UI surface (the only candidate, a live expression slider tuner, was explicitly cut during SCP elicitation).

## Requirements Inventory

### Functional Requirements

FR1: The engine MUST subscribe to four ROS 2 topics — `mood`, `activity`, `speech_emotion`, `vocalization` — with topic names and DDS domain read from `expression_engine.toml` (defaults `/olaf/{name}`, domain `0`), matching the companion's `[publisher]` config.
FR2: Each topic carries `std_msgs/String` whose body is the full `EventEnvelope` JSON. The engine MUST deserialize and validate against the schema-3 payload models.
FR3: The engine MUST be subscribe-only. It MUST NOT publish to any of the four topics under any circumstance (single-writer-per-topic invariant).
FR4: On any received event with `schema_version != 3`, the engine MUST raise, log to journald, and exit non-zero (fail-fast). No silent truncation or version coercion.
FR5: The engine MUST load `expression_map.yaml` at startup and render each canonical name to body output (pose, LED, eye-state, heart) per that mapping. The mapping is the only place rendering decisions live.
FR6: The engine MUST cover, in `expression_map.yaml`, the full canonical set of the pinned `olaf_companion` release: all `Mood` values, all `ActivityState` values (with both `working_submode` values), all first-class `speech_emotion` names, and all `vocalization` tags.
FR7: For `vocalization` tags flagged `tts_supported: false` AND classified as gesture cues (`nod`, `shake`), the engine MUST render a silent head gesture and MUST NOT play any audio (`visible_only` invariant, enforced at map-load time).
FR8: The engine MUST treat `mood` as a slow base layer, `speech_emotion` as a per-utterance overlay on that base, `activity` as the held base posture, and `vocalization` as punctual gestures composed on top.
FR9: The engine MUST drive existing Phase 1 drivers (neck, ears, head/eye) in-process through Protocol-shaped adapters, importing the driver Python modules directly — not via intermediate ROS topics.
FR10: The engine MUST exhibit idle behaviour: when `activity=listening` with no `speech_emotion` for >3s, hold the mood-tinted neutral pose with micro-movement and slow breath-LED; when `activity=sleeping`, hold eyes-closed posture with slow LED breathe. The body MUST NOT go statue-still.
FR11: The engine MUST drive the WS2812 LED strip via an engine-owned adapter (no separate driver package).
FR12: The engine MUST drive the 4" Pi heart display via an engine-owned adapter, animating it from `mood` + `activity` (replaces the retired `HeartRate.msg`).
FR13: An unknown canonical name (engine map lags the companion) MUST log `expression.unmapped_<topic>` at WARN and fall back to a `default_*` pose — never freeze, never crash at runtime (distinct from FR4's schema-version fail-fast).
FR14: A mock companion publisher MUST exist that drives all four topics with schema-3 events for development and CI without the live pipeline.

### NonFunctional Requirements

NFR1 (Wake latency): When `activity` transitions `sleeping → waking`, the body MUST begin the head-lift / eyes-open within 100ms of the engine receiving the event.
NFR2 (Anticipatory window): For `speech_emotion`, the engine's pose target MUST reach the body within the (audio-anchor − 30ms) to (audio-anchor − 80ms) window — the body moves just before the matching audio.
NFR3 (Mood as slow drift): Mood transitions MUST ease in over 2–4 seconds and never snap.
NFR4 (Gesture crispness): `[nod]` / `[shake]` MUST land within ~150ms of receipt with a quick attack and fast settle — punctuation, not a held pose.
NFR5 (Extensible vocabulary): Adding a new mood / activity / emotion / vocalization MUST be a pure `expression_map.yaml` data edit with no engine code change. Startup validation is strict against the pinned companion release's canonical set; runtime is graceful per FR13; the pinned companion tag is bumped in lockstep with map extensions.
NFR6 (Hardware swap): Replacing a hardware element (servo bus, eye display, LED strip, heart display) MUST be a single new Protocol-adapter implementation — no change to the mapping, DDS layer, or animation loop.
NFR7 (Fail-fast on missing deps): Missing DDS connection, missing/invalid `expression_map.yaml`, or an offline hardware adapter at startup MUST be fatal (exit non-zero; systemd restarts). v1 has no graceful-degradation layer.
NFR8 (Long-running service): The engine MUST run as a systemd-managed long-running process with structured JSON logging.
NFR9 (Single-host default): The v1 deployment target is co-located with the pipeline on one Pi 5 over loopback DDS. Multi-host over LAN is supported by DDS but is not the v1 timing-validated scenario.
NFR10 (Regression-locked expressions): Finalized expressions MUST be covered by a replay-based regression harness that asserts pose/LED/eye/heart output, so finalized expressions do not drift as the vocabulary grows.
NFR11 (Phase 1 non-regression): All Phase 1 component test scripts MUST continue to pass unchanged — proof the reused components were not regressed by the re-scope.

### Additional Requirements

(Derived from `docs/planning-artifacts/architecture/phase2-expression-engine.md` — technical decisions that constrain story scope and acceptance criteria.)

- **AR1 — Three adapter kinds (§2, §4):** Adapters split into *Continuous* (`neck_adapter`→`NeckServoDriver`, `ears_adapter`→`EarsServoDriver`; engine interpolates per tick), *Delegating* (`eye_adapter`→`HeadI2CClient`; engine sends a semantic event, ESP32 owns 60 FPS animation), and *Engine-owned* (`led_adapter` WS2812, `heart_adapter` 4" display; no Phase 1 driver). The render loop interpolates neck+ears every tick, pushes eye on transition, ticks LED for breathe, pushes heart at low rate.
- **AR2 — Adapter Protocols frozen in Story 6.4 (§4):** `adapters/base.py` defines `ContinuousAdapter`, `DelegatingAdapter`, `SurfaceAdapter` structural `typing.Protocol`s with the exact signatures grounded in Phase 1 driver APIs. These are a freeze point — change only via documented amendment.
- **AR3 — Concurrency model (§3):** One `rclpy` executor thread handles subscriptions and writes to a mutex-protected `state` snapshot (last-write-wins per topic); a separate render thread runs the fixed-tick loop, copies state per tick, never blocks on DDS.
- **AR4 — Module decomposition (§3):** `node.py`, `schema.py`, `subscribers.py`, `state.py`, `map_loader.py`, `render_loop.py`, `idle.py`, `adapters/{base,neck,ears,eye,led,heart}_adapter.py`, plus `config/expression_map.yaml` and `config/expression_engine.toml`.
- **AR5 — `expression_map.yaml` schema frozen in Story 6.4 (§5):** `schema_version`, `pinned_companion_tag`, `defaults`, and topic-keyed `mood`/`activity`/`speech_emotion`/`vocalization` blocks. Disambiguation is by topic, not name (`mood.happy` ≠ `speech_emotion.happy`). Epic 7 authors against this with no schema changes.
- **AR6 — Composition order (§5.1):** `target = compose(activity_base, mood_bias, speech_overlay, active_vocalization)`; partial pose dicts merge, missing joints inherit from the layer below.
- **AR7 — Animation/timing model (§6):** Critically-damped easing; mood long time-constant (2–4s), speech short, activity medium; anticipatory window biases easing-start for continuous adapters and sends `set_expression` early for the delegating eye adapter; wake short-circuits easing.
- **AR8 — Startup validation sequence (§9):** Strict ordered, every step fatal: toml → map → vocab completeness → `visible_only` assertion → `adapter.connect()` each → DDS subscribe → start render thread → RUNNING.
- **AR9 — Config & calibration single-source (§8):** `expression_engine.toml` shape per companion brief Appendix B.1; hardware ports/addresses cross-checked against existing Phase 1 `config/servo-ids.yaml` — the engine does NOT duplicate calibration.
- **AR10 — Locked decision: eye vocabulary translation (§12.1):** A canonical-name→ESP32-expression-string table lives as data inside `eye_adapter.py`; `expression_map.yaml` stays purely canonical.
- **AR11 — Locked decision: `expressions.py` migrate-then-retire (§12.2):** Story 7.1 lifts the existing emotion→ear-pose preset data into `expression_map.yaml` and deletes `head_ears_driver/expressions.py`; add explicit AC "expressions.py removed; no module imports it".
- **AR12 — Locked decision: gestures parametric in engine code (§12.3):** Engine ships a tested library of parametric gesture trajectories (attack/settle from config); the map only *names* a gesture. A new gesture *shape* is a code change.
- **AR13 — Locked decision: schema re-derived, zero cross-repo coupling (§12.4):** `schema.py` is our own pydantic models matching the companion's documented JSON (Appendix A); `pinned_companion_tag` recorded in `expression_map.yaml` and cross-checked at startup; companion bumps update `schema.py` + re-pin in lockstep.
- **AR14 — Testing strategy (§10):** `test/mock_publisher.py` (FR14), schema tests, map-loader tests (incl. synthetic-entry-no-code-change proving NFR5), `test/test_locked_expressions.py` replay regression harness (NFR10), Phase-1 component scripts run unchanged in CI (NFR11).
- **AR15 — Deployment (§11):** Single Pi 5, loopback DDS, systemd unit `Restart=always`, structured JSON logs to journald.
- **AR16 — Epic 5 hard gate & strict sequencing (PRD Epic List, §13):** 5.1/5.2 recorded Done (executed during SCP approval); Story 5.3 (per-driver hardware verification) is the GATE — Epics 6–8 MUST NOT start until 5.1+5.2+5.3 are green. Sequence is strictly 5 → 6 → 7 → 8.
- **AR17 — Seed content:** `head_ears_driver/expressions.py` (~12 emotion→ear-pose presets) is the authoring seed for the `speech_emotion` block of `expression_map.yaml` (Epic 7.1) — not authored from blank.

### UX Design Requirements

Not applicable — Phase 2 has no UI surface. The only candidate (a live expression slider tuner) was explicitly cut during Sprint Change Proposal elicitation. No UX Design document exists.

### FR Coverage Map

FR1: Epic 6 - Subscribe to the four canonical topics from config.
FR2: Epic 6 - Deserialize topic payloads, validate against schema-3 models.
FR3: Epic 6 - Enforce subscribe-only invariant (never publish).
FR4: Epic 6 - schema_version != 3 fail-fast (raise/journald/exit).
FR5: Epic 6 (mechanism) / Epic 7 (content) - Load expression_map.yaml and render canonical names.
FR6: Epic 6 (startup completeness validation) / Epic 7 (author the full canonical set).
FR7: Epic 6 (visible_only enforcement at map-load) / Epic 7 (author nod/shake gesture cues).
FR8: Epic 6 (composition in render loop) / Epic 7 (tune layered overlays).
FR9: Epic 5 (per-driver hardware verification) / Epic 6 (Protocol-shaped in-process adapters).
FR10: Epic 6 - Idle behaviour; body never statue-still.
FR11: Epic 6 - Engine-owned WS2812 LED adapter.
FR12: Epic 8 - Engine-owned 4" heart display adapter.
FR13: Epic 6 - Unknown canonical name -> WARN + default_* fallback.
FR14: Epic 6 - Mock companion publisher for dev/CI.

## Epic List

### Epic 5: Restructure & Driver Verification (hard gate)

Land the architectural restructure and prove every reused Phase 1 driver moves real hardware, so nothing downstream is built on an unverified foundation. Stories 5.1 and 5.2 were executed during SCP approval (2026-05-15) and are recorded Done for traceability; Story 5.3 is the actual gate. Epics 6-8 MUST NOT start until 5.1 + 5.2 + 5.3 are all green (AR16).
**FRs covered:** FR9 (verification half); NFR11.

### Epic 6: Expression Engine

A long-running engine that turns canonical events into correctly-timed body expression and never looks dead. Includes the adapter-Protocol and expression_map.yaml schema freeze (Story 6.4) that unblocks Epic 7.
**FRs covered:** FR1, FR2, FR3, FR4, FR5, FR6, FR7, FR8, FR9, FR10, FR11, FR13, FR14.

### Epic 7: Expression Authoring & Finalization

Populate and lock the full emotional vocabulary against the frozen schema, seeded from existing presets; retire head_ears_driver/expressions.py (AR11); regression-lock finalized expressions.
**FRs covered:** FR5, FR6, FR7, FR8; NFR10.

### Epic 8: Heart Display Animation

Add the secondary heart surface once core body language is finalized: mood/activity-driven heart on the 4" Pi display.
**FRs covered:** FR12.

### Epic 9: Hardening

Cross-cutting robustness epic. Collects reliability defects surfaced during execution that are out of scope for the linear feature epics. First entry: the Pi/ESP32 I2C boot-order race discovered during Story 5.3 (the head ESP32 is absent from the bus until it finishes its independent boot, so any command sent before then is silently lost).
**FRs covered:** none (robustness); relates NFR11.

### Sequencing & Dependencies

Strictly linear: 5 -> 6 -> 7 -> 8. Story 5.3 is a hard gate blocking Epics 6-8. No dependency on the companion's implementation — develop against schema-3 with the FR14 mock publisher. **Epic 9 (Hardening) is NOT on the linear critical path** — it is a non-blocking backlog epic; its stories are scheduled independently and recommended before production but do not gate feature epics.

## Epic 5: Restructure & Driver Verification (hard gate)

Land the architectural restructure and prove every reused Phase 1 driver moves real hardware, so nothing downstream is built on an unverified foundation. Epic 5 is declared green only when 5.1, 5.2, and all of 5.3 pass; Epics 6-8 MUST NOT start before that (AR16).

### Story 5.1: Execute restructure (Done 2026-05-15)

As the maintainer,
I want the legacy expression surface archived and the `expression_engine` package scaffolded,
So that the repo reflects the subscribe-only architecture.

**Acceptance Criteria:**

**Given** the legacy per-module expression message types
**When** the restructure is executed
**Then** `Expression/Gesture/EarsPose/HeartRate.msg` are moved to `archive/ros2/olaf_interfaces/msg/` via `git mv` with a supersession README
**And** `ModuleStatus.msg` is kept.

**Given** the legacy ROS-node wrappers and launch files
**When** the restructure is executed
**Then** `ears_node.py`, `head_node.py`, and `bringup/launch/ears.launch.py` are archived
**And** `head_ears_driver` `console_scripts` are emptied
**And** the empty `ros2/src/olaf_personality/` package is deleted.

**Given** `olaf_interfaces` CMakeLists/package.xml
**When** the restructure is executed
**Then** they reference only `ModuleStatus.msg`
**And** the dead `geometry_msgs` dependency is removed.

**Given** the new subscribe-only architecture
**When** `ros2/src/expression_engine/` is scaffolded (ament_python)
**Then** it builds and the `expression_engine_node` entry point resolves to a stub
**And** all 7 ROS packages build clean via `colcon build`.

### Story 5.2: Documentation redirect (Done 2026-05-15)

As a future contributor,
I want the architecture/PRD/README docs to reflect the subscribe-only direction,
So that I am not misled by stale Phase 1 claims.

**Acceptance Criteria:**

**Given** the architecture shards
**When** the documentation redirect is applied
**Then** `technical-summary.md`, `high-level-architecture-diagram.md`, `architectural-patterns.md`, `repository-structure.md`, and `source-tree.md` are annotated with the Phase 2 redirect.

**Given** `phase1-prd.md`
**When** the documentation redirect is applied
**Then** it carries a v2.1 change-log row, an NFR3 caveat, and a "Phase 2 Preview" superseded note.

**Given** the repo `README.md`
**When** the documentation redirect is applied
**Then** it has a repo-scope callout plus a corrected diagram, roadmap, and tree
**And** no un-annotated "personality coordination on Pi" or `olaf_personality` claims remain in any edited doc.

### Story 5.3: Per-driver hardware verification (GATE)

As the maintainer,
I want each reused driver demonstrably moving real hardware via a standalone test script,
So that the in-process-adapter assumption is proven before Epic 6.

**Acceptance Criteria:**

**Given** the neck driver and a standalone script following the Phase 1 component-test convention
**When** the script is run via the documented `PYTHONPATH … poetry run python` pattern on the robot
**Then** the neck produces observable, correct motion
**And** the script exits 0 on success and is committed under the appropriate `scripts/` / module test location.

**Given** the ears driver (`ears_servo_driver.py`) and its standalone script
**When** the script is run on the robot
**Then** the ears produce observable, correct motion
**And** the script exits 0 and is committed.

**Given** the head/eye path (`head_i2c_client.py`) and its standalone script
**When** the script is run on the robot
**Then** observable eye/display output is produced
**And** the script exits 0 and is committed.

**Given** Stories 5.1, 5.2, and all of 5.3
**When** all pass
**Then** Epic 5 is declared "green"
**And** Epics 6-8 are unblocked (and MUST NOT have started before this).

## Epic 6: Expression Engine

A long-running engine that turns canonical events into correctly-timed body expression and never looks dead. Story 6.4 freezes the adapter Protocols (AR2) and the `expression_map.yaml` schema (AR5) for Epic 7.

### Story 6.1: Subscriber + schema-3 envelope validation

As the avatar,
I want incoming events validated fail-fast,
So that a contract mismatch surfaces loudly instead of corrupting expression.

**Acceptance Criteria:**

**Given** the configured DDS domain and topic names from `expression_engine.toml`
**When** the engine starts
**Then** it subscribes to all four topics — `mood`, `activity`, `speech_emotion`, `vocalization` (FR1).

**Given** a valid schema-3 `std_msgs/String` payload on any topic
**When** it is received
**Then** the `EventEnvelope` plus the four payload models deserialize it into typed objects (FR2, AR13).

**Given** an event with `schema_version != 3`
**When** it is received
**Then** the engine raises, logs to journald, and exits non-zero — no coercion or truncation (FR4).

**Given** the engine running
**When** its publishers are inspected by a test
**Then** no publisher exists on any of the four topics (FR3, subscribe-only invariant asserted).

### Story 6.2: Map loader + vocabulary completeness

As the maintainer,
I want `expression_map.yaml` loaded and validated against the pinned vocabulary at startup,
So that an incomplete map fails before the engine pretends to run.

**Acceptance Criteria:**

**Given** the package `config/` directory
**When** the engine starts
**Then** the loader reads `expression_map.yaml` and `expression_engine.toml` (FR5, AR9).

**Given** the loaded map and the pinned `olaf_companion` canonical set (`pinned_companion_tag`)
**When** completeness is asserted
**Then** any missing `Mood`/`ActivityState`(incl. both `working_submode`s)/`speech_emotion`/`vocalization` entry is fatal (FR6, NFR7, AR8).

**Given** the `vocalization` block
**When** the map is loaded
**Then** `visible_only: true` is asserted present on `nod` and `shake`; missing is fatal (FR7).

**Given** a synthetic new map entry added with no engine code change
**When** a test loads the map
**Then** it is accepted, proving NFR5 (AR5).

**Given** an unknown canonical name at runtime
**When** it is received on a topic
**Then** the engine logs `expression.unmapped_<topic>` at WARN and falls back to the `default_*` pose — never freezes, never crashes (FR13).

### Story 6.3: Animation loop

As the avatar,
I want smooth, correctly-timed motion toward target poses,
So that expression reads as alive and lands with the audio.

**Acceptance Criteria:**

**Given** the render thread
**When** the engine runs
**Then** a fixed-tick loop runs at `servo_tick_hz` (default 100Hz, configurable), reading a mutex-protected state snapshot per tick (AR3).

**Given** a changing target pose
**When** the loop ticks
**Then** continuous adapters (neck, ears) are driven by engine-side critically-damped interpolation; mood eases over 2-4s and never snaps (NFR3, AR1, AR7).

**Given** a `speech_emotion` event carrying `audio_frame_id`
**When** the pose target is computed
**Then** for continuous adapters the easing-start is biased and for the delegating eye adapter `set_expression` is sent early, so the body reaches target within the (audio-anchor − 30ms)..(−80ms) window (NFR2, AR7).

**Given** a `[nod]` or `[shake]` vocalization
**When** it is received
**Then** the gesture lands within ~150ms with configurable attack/settle (NFR4, AR12).

### Story 6.4: Reference expression end-to-end (freezes the schema)

As the maintainer,
I want one hand-authored expression rendered end-to-end,
So that the map schema and adapter Protocols are proven and frozen for Epic 7.

**Acceptance Criteria:**

**Given** one reference `speech_emotion` (e.g. `happy`) authored in `expression_map.yaml`
**When** the FR14 mock publisher emits it
**Then** neck + ears + eyes visibly render it within the anticipatory window (FR5, FR8, NFR2).

**Given** the proven end-to-end render
**When** Story 6.4 completes
**Then** the `expression_map.yaml` schema (pose/LED/eye/heart key structure, layering order) is documented and declared frozen (AR5)
**And** the adapter Protocols in `adapters/base.py` are declared frozen (AR2)
**And** Epic 7 authors against them with no schema/Protocol change.

### Story 6.5: Idle behaviour

As the avatar,
I want to look alive between events,
So that the body is never statue-still.

**Acceptance Criteria:**

**Given** the idle FSM
**When** the engine runs
**Then** it distinguishes `sleeping` (DORMANT) from `listening`-and-silent (AMBIENT) (AR-idle §7).

**Given** `activity=listening` and no `speech_emotion` for >3s
**When** the FSM evaluates
**Then** the body holds a mood-tinted neutral pose with sub-degree micro-movement and slow breath-LED (FR10).

**Given** `activity=sleeping`
**When** the FSM evaluates
**Then** the body holds an eyes-closed posture with slow LED breathe (FR10).

**Given** either ambient state
**When** observed
**Then** the body is never statue-still and never twitchy
**And** any qualifying new event exits the idle state immediately.

### Story 6.6: Real LED adapter + breath-LED

As the avatar,
I want real ambient LED behaviour,
So that mood/idle is visible on the strip.

**Acceptance Criteria:**

**Given** the WS2812 strip
**When** the engine drives LEDs
**Then** an engine-owned adapter behind the `SurfaceAdapter` Protocol drives it, with no separate driver package (FR11, AR1).

**Given** the engine in an idle state
**When** the loop ticks
**Then** breath-LED renders (ties to Story 6.5).

**Given** a need to swap the LED hardware
**When** assessed
**Then** the change is a single new Protocol implementation only (NFR6).

### Story 6.7: Service hardening

As the maintainer,
I want the engine to run as a robust long-lived service,
So that it recovers from failure and never silently degrades.

**Acceptance Criteria:**

**Given** the deployment target
**When** the engine is installed
**Then** it runs under a systemd unit (`Restart=always`) with structured JSON logging to journald (NFR8, AR15).

**Given** startup
**When** the engine initializes
**Then** validation runs strictly ordered — config → map → vocab completeness → `visible_only` → adapters connect → DDS subscribe → RUNNING — and any failure is fatal and journald-visible (NFR7, AR8).

**Given** the re-scoped repo
**When** CI runs
**Then** all Phase 1 component test scripts still pass unchanged (NFR11, AR14).

## Epic 7: Expression Authoring & Finalization

Populate and lock the full emotional vocabulary against the schema frozen in Story 6.4, seeded from existing presets.

### Story 7.1: Author + finalize the speech-emotions

As the maintainer,
I want all first-class speech-emotions authored and finalized in the map,
So that the body renders the full emotional vocabulary correctly.

**Acceptance Criteria:**

**Given** the existing `head_ears_driver/expressions.py` presets as seed (AR17)
**When** the `speech_emotion` block is authored in `expression_map.yaml`
**Then** all first-class `speech_emotion` names are present and seeded/adapted from `expressions.py` (FR6).

**Given** each authored speech-emotion
**When** the FR14 mock publisher emits it
**Then** it renders correctly on hardware
**And** it overlays correctly on a mood base (FR8).

**Given** the migration is complete
**When** the repo is inspected
**Then** `head_ears_driver/expressions.py` is deleted
**And** no module imports it — the map is the single source of expression data (AR11, NFR5).

### Story 7.2: Author + finalize the vocalizations + layered actions

As the maintainer,
I want all vocalizations authored with the audio/gesture split honoured,
So that gesture cues are silent and compose without destroying the base pose.

**Acceptance Criteria:**

**Given** the `vocalization` block
**When** all tags are authored
**Then** the audio-burst vs gesture-cue split is honoured (FR7).

**Given** an active base pose (e.g. `mood=happy`)
**When** `[nod]` or `[shake]` fires
**Then** the gesture composes on top and releases without destroying the base (FR8, AR6, AR12).

**Given** `nod` and `shake`
**When** the map is validated
**Then** the `visible_only` invariant is verified — silent, no audio asset (FR7).

### Story 7.3: Activity to posture mapping

As the maintainer,
I want every activity mapped to a base posture,
So that the held body posture always reflects the current activity.

**Acceptance Criteria:**

**Given** all `ActivityState` values, including both `working_submode`s
**When** the `activity` block is authored
**Then** each maps to exactly one base posture — simple, no elaborate transition choreography (FR8).

**Given** `activity` transition `sleeping → waking`
**When** received
**Then** the head-lift/eyes-open begins within 100ms of receipt (NFR1, AR7).

### Story 7.4: Mood to subtle ambient modifier

As the avatar,
I want mood to bias expression only subtly,
So that mood tints the body without dominating it.

**Acceptance Criteria:**

**Given** all `Mood` values
**When** the `mood` block is authored
**Then** each is a low-impact modifier (small LED/lean bias only) — mood does not strongly drive expression (FR8, AR6).

**Given** a mood change
**When** rendered
**Then** the transition eases over 2-4s and never snaps (NFR3, AR7).

### Story 7.5: Locked-expression regression harness

As the maintainer,
I want finalized expressions regression-locked,
So that they do not drift as the vocabulary grows.

**Acceptance Criteria:**

**Given** recorded event sequences
**When** the replay harness (`test/test_locked_expressions.py`) runs
**Then** it asserts composed pose/LED/eye/heart output against locked fixtures (NFR10, AR14).

**Given** CI
**When** it runs
**Then** the harness runs against the FR14 mock publisher.

**Given** a newly finalized expression
**When** it is added and re-locked
**Then** the workflow is documented and repeatable
**And** existing locks still pass.

## Epic 8: Heart Display Animation

Add the secondary heart surface once core body language is finalized.

### Story 8.1: Mood/activity-driven heart

As the avatar,
I want a mood/activity-driven heart on the 4" display,
So that the secondary surface reinforces the body's emotional state.

**Acceptance Criteria:**

**Given** the 4" Pi display
**When** the engine drives the heart
**Then** an engine-owned adapter behind the `SurfaceAdapter` Protocol drives it (FR12, AR1).

**Given** `mood` and `activity` state
**When** the heart renders
**Then** the animation is driven by `mood` + `activity` (replacing the retired `HeartRate.msg`).

**Given** a need to swap the heart hardware
**When** assessed
**Then** the change is a single new Protocol implementation only (NFR6).

**Given** finalized heart states
**When** the regression harness runs
**Then** heart output is asserted alongside pose/LED/eye (NFR10, AR14).

## Epic 9: Hardening

Cross-cutting robustness epic for reliability defects discovered during execution that fall outside the linear feature epics. Non-blocking backlog — does not gate Epics 6-8. Stories are added here as systemic issues are surfaced and scheduled independently.

### Story 9.1: Harmonize boot of all interfaces and ESP32s

As the maintainer,
I want the engine/drivers to wait for every ESP32 and I2C interface to be ready before commanding it,
So that the avatar does not silently lose commands when the Pi and an ESP32 boot independently.

**Context:** Discovered in Story 5.3. The Pi and each ESP32 power on and boot independently. If the Pi opens the I2C bus and sends commands before an ESP32 has registered as a slave, the device is absent from the bus (e.g. head `0x08` missing from `i2cdetect`; writes return Errno 121) and every command is silently lost with no error surfaced to the engine. The 5.3 workaround was a manual re-scan after the ESP32 finished booting — not acceptable for an always-on avatar.

**Acceptance Criteria:**

**Given** the Pi has booted but an ESP32 has not yet registered on the I2C bus
**When** a driver/adapter attempts its first command
**Then** it waits for and detects device readiness (presence/handshake) rather than silently losing the write
**And** it surfaces a clear, logged error if a device never appears within a bounded timeout.

**Given** all ESP32s and I2C interfaces (head, and any future ESP32 such as base)
**When** the engine starts
**Then** a single reusable readiness/handshake mechanism gates commanding for every interface, not a head-only special case.

**Given** an ESP32 that reboots or drops off the bus mid-session
**When** it returns
**Then** the driver re-establishes readiness and resumes commanding without a full engine restart.

**Given** the boot-handshake mechanism
**When** the head/eye, neck, and ears paths are exercised from a cold, simultaneous Pi+ESP32 power-on
**Then** observable correct output is produced with no manual re-scan step.
