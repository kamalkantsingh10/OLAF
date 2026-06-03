# OLAF Phase 2: Expression Engine — Product Requirements Document (PRD)

## Goals and Background Context

### Goals

Phase 2 makes the OLAF **avatar** visibly alive by rendering the companion AI's intent on the body. It delivers:

- **A subscribe-only expression engine** (`expression_engine` ROS 2 package) consuming the sibling `olaf_companion` pipeline's 4 canonical topics — never publishing back.
- **Body-side rendering as data** — an `expression_map.yaml` keyed on canonical names that maps every mood / activity / speech-emotion / vocalization to pose, LED, eye, and heart output.
- **An animation loop** that interpolates toward target poses, honours the 30–80ms audio-anticipatory window, and snaps gestures crisply.
- **Aliveness** — the body is never statue-still: idle micro-movement, breath-LED, mood-tinted neutral return.
- **A finalized, regression-locked expression vocabulary** that is extensible without code changes as the companion's vocabulary grows.
- **Clean reuse of every proven Phase 1 component** — driver hardware logic, servo SDK, calibration, configs, firmware — behind Protocol-shaped adapters.

### Background Context

The sibling project `olaf_companion` finalized its publisher contract at `schema_version=3` on 2026-05-10, moving all renderer mapping out of the publisher and onto the body side, and collapsing the AI→body interface to four typed ROS 2 topics (`mood`, `activity`, `speech_emotion`, `vocalization`). OLAF's pre-existing per-module expression surface predated that contract and was retired via the approved **Sprint Change Proposal 2026-05-15** (`docs/planning-artifacts/sprint-change-proposal-2026-05-15.md`).

Phase 1 (hardware foundation, drivers, firmware, calibration) is **complete and unchanged**. Phase 2 is the body's consumer side of the wire. The architectural posture is **producer-agnostic and subscribe-only**: the engine consumes typed events on a configurable DDS domain, runs entirely on its own hardware loop, and never reaches back into the pipeline. The line between the two projects is the wire (`schema_version=3`, `EventEnvelope` + four typed payloads), authoritatively documented in the companion's `olaf-embodiment-brief.md` (Appendix A).

**Key Phase 2 decisions (carried from SCP §9):**

1. Component named **expression engine**; package `expression_engine` (no `olaf_` prefix). The robot/project as a whole is the "avatar". The term "embodiment" is not used in this repo.
2. Rebuilt **inside this repo**; all four legacy expression msg types retired.
3. Engine imports driver Python modules **in-process** via Protocol-shaped adapters — no intermediate ROS topics for low-level control.
4. **Epic 5 is a hard gate** (restructure complete + every reused driver verified on hardware) blocking Epics 6–8.
5. Vocabulary is **extensible** — strict against the pinned companion release, graceful at runtime, growth is a pure data edit.
6. Base-module balancing remains **out of scope** on its own ESP32 firmware track.

### Change Log

| Date | Version | Description | Author |
|------|---------|-------------|--------|
| 2026-05-15 | v1.0 | Initial Phase 2 PRD, derived from approved Sprint Change Proposal 2026-05-15 | Winston (Architect) |

---

## Requirements

### Functional Requirements

**FR1**: The expression engine MUST subscribe to four ROS 2 topics — `mood`, `activity`, `speech_emotion`, `vocalization` — with topic names and DDS domain read from `expression_engine.toml` (defaults `/olaf/{name}`, domain `0`), matching the companion's `[publisher]` config.

**FR2**: Each topic carries `std_msgs/String` whose body is the full `EventEnvelope` JSON. The engine MUST deserialize and validate against the schema-3 payload models.

**FR3**: The engine MUST be **subscribe-only**. It MUST NOT publish to any of the four topics under any circumstance (single-writer-per-topic invariant).

**FR4**: On any received event with `schema_version != 3`, the engine MUST raise, log to journald, and exit non-zero (fail-fast — matches the companion's posture). No silent truncation or version coercion.

**FR5**: The engine MUST load `expression_map.yaml` at startup and render each canonical name to body output (pose, LED, eye-state, heart) per that mapping. The mapping is the only place rendering decisions live.

**FR6**: The engine MUST cover, in `expression_map.yaml`, the **full canonical set of the pinned `olaf_companion` release**: all `Mood` values, all `ActivityState` values (with both `working_submode` values), all first-class `speech_emotion` names, and all `vocalization` tags.

**FR7**: For `vocalization` tags flagged `tts_supported: false` AND classified as gesture cues (`nod`, `shake`), the engine MUST render a silent head gesture and MUST NOT play any audio (`visible_only` invariant, enforced at map-load time).

**FR8**: The engine MUST treat `mood` as a slow base layer, `speech_emotion` as a per-utterance overlay on that base, `activity` as the held base posture, and `vocalization` as punctual gestures composed on top.

**FR9**: The engine MUST drive existing Phase 1 drivers (neck, ears, head/eye) **in-process** through Protocol-shaped adapters, importing the driver Python modules directly — not via intermediate ROS topics.

**FR10**: The engine MUST exhibit idle behaviour: when `activity=listening` with no `speech_emotion` for >3s, hold the mood-tinted neutral pose with micro-movement and slow breath-LED; when `activity=sleeping`, hold eyes-closed posture with slow LED breathe. The body MUST NOT go statue-still.

**FR11**: The engine MUST drive the WS2812 LED strip via an engine-owned adapter (no separate driver package).

**FR12**: The engine MUST drive the chest heart surface via a **delegating adapter**: it composes the heart state (`bpm`/`intensity`/`color`) from `expression_map.yaml` — exactly as for neck/ears/eyes — across the **mood** (slow base), **activity** (held base, incl. `starting`/`sleeping`/`working`), and **speech_emotion** (per-utterance overlay) layers, **eased in lockstep with the body's emotion easing** (the heart eases out as the emotion eases out, never jumps independently), and **forwards** the eased state over local IPC to the standalone chest-display renderer, which owns the heartbeat animation (mirrors the eye-adapter / ESP32 split). **Vocalizations do NOT affect the heart** — punctual gestures (`nod`/`shake`) touch pose only. This replaces the retired `HeartRate.msg`. _(Re-scoped 2026-06-03 — see sprint-change-proposal-2026-06-03.md.)_

**FR13**: An unknown canonical name (engine map lags the companion) MUST log `expression.unmapped_<topic>` at WARN and fall back to a `default_*` pose — never freeze, never crash at runtime (distinct from FR4's schema-version fail-fast).

**FR14**: A mock companion publisher MUST exist that drives all four topics with schema-3 events for development and CI without the live pipeline.

### Non-Functional Requirements

**NFR1 (Wake latency)**: When `activity` transitions `sleeping → waking`, the body MUST begin the head-lift / eyes-open within 100ms of the engine receiving the event.

**NFR2 (Anticipatory window)**: For `speech_emotion`, the engine's pose target MUST reach the body within the (audio-anchor − 30ms) to (audio-anchor − 80ms) window — the body moves just before the matching audio (consumer side of companion NFR5).

**NFR3 (Mood as slow drift)**: Mood transitions MUST ease in over 2–4 seconds and never snap.

**NFR4 (Gesture crispness)**: `[nod]` / `[shake]` MUST land within ~150ms of receipt with a quick attack and fast settle — punctuation, not a held pose.

**NFR5 (Extensible vocabulary)**: Adding a new mood / activity / emotion / vocalization MUST be a pure `expression_map.yaml` data edit with **no engine code change**. Startup validation is strict against the pinned companion release's canonical set (complete-for-this-version, not complete-forever); runtime is graceful per FR13; the pinned companion tag is bumped in lockstep with map extensions.

**NFR6 (Hardware swap)**: Replacing a hardware element (servo bus, eye display, LED strip, chest heart renderer) MUST be a single new Protocol-adapter implementation (for the heart, swapping the renderer behind the same forwarding `heart_adapter`) — no change to the mapping, DDS layer, or animation loop.

**NFR7 (Fail-fast on missing deps)**: Missing DDS connection, missing/invalid `expression_map.yaml`, or an offline hardware adapter at startup MUST be fatal (exit non-zero; systemd restarts). v1 has no graceful-degradation layer.

**NFR8 (Long-running service)**: The engine MUST run as a systemd-managed long-running process with structured JSON logging.

**NFR9 (Single-host default)**: The v1 deployment target is co-located with the pipeline on one Pi 5 over loopback DDS. Multi-host over LAN is supported by DDS but is not the v1 timing-validated scenario.

**NFR10 (Regression-locked expressions)**: Finalized expressions MUST be covered by a replay-based regression harness that asserts pose/LED/eye/heart output, so finalized expressions do not drift as the vocabulary grows.

**NFR11 (Phase 1 non-regression)**: All Phase 1 component test scripts MUST continue to pass unchanged — proof the reused components were not regressed by the re-scope.

---

## Technical Assumptions

- **Transport**: ROS 2 Jazzy / DDS, RELIABLE QoS, DDS multicast discovery, no broker. Domain configurable both sides.
- **Schema pinning**: schema imports / canonical-vocabulary assumptions pinned to a tagged `olaf_companion` release, not `main`. Bumped in lockstep with map extensions (NFR5).
- **In-process drivers**: Phase 1 driver Python packages are colcon-built and importable as libraries; the engine depends on them and calls their hardware-logic classes (`ears_servo_driver.py`, `head_i2c_client.py`, neck driver) through adapters. The retired ROS-node wrappers are archived.
- **Expression seed content**: `head_ears_driver/expressions.py` (existing emotion→ear-pose presets, ~12 emotional states) is the **starting seed** for `expression_map.yaml` authoring — not authored from blank.
- **Config shape**: mirrors the companion brief Appendix B — `expression_engine.toml` (service config) + `expression_map.yaml` (vocabulary→render), both in `ros2/src/expression_engine/config/`.
- **No reasoning on the body**: no STT, no LLM, no Cartesia, no cloud round-trips for any rendering decision. LAN/loopback only.
- **Out of scope for Phase 2**: base-module balancing/locomotion (separate firmware track); anything that publishes to the pipeline (future touch/camera input would be its own project); cross-restart pose persistence; reconnect-with-backoff resilience (companion brief defers these to its v2).

---

## Epic List

| Epic | Title | Goal |
|------|-------|------|
| **Epic 5** | Restructure & Driver Verification *(gate)* | Clean foundation; every reused driver proven on real hardware. Blocks Epics 6–8. |
| **Epic 6** | Expression Engine | A working engine: canonical events in → expression out, never statue-still. |
| **Epic 7** | Expression Authoring & Finalization | The full emotional vocabulary, tuned and regression-locked. |
| **Epic 8** | Chest Display & Animated Heart | Standalone portrait 2×2 chest dashboard (animated heart + 3 log panels) on the 4.3" DSI panel; heart driven by the engine via a delegating adapter from `expression_map.yaml`. After core body language is locked. |

Sequencing is strict: 5 → 6 → 7 → 8. No dependency on the companion's *implementation* — develop against the schema with the FR14 mock publisher.

---

## Epic 5 — Restructure & Driver Verification *(gate)*

**Goal:** Land the architectural restructure and prove every reused Phase 1 driver moves real hardware, so nothing downstream is built on an unverified foundation.

> **Status:** Stories 5.1 and 5.2 were executed during SCP approval (2026-05-15) and are recorded **Done** here for traceability. Story 5.3 remains and is the actual gate.

### Story 5.1 — Execute restructure *(Done 2026-05-15)*
As the maintainer, I want the legacy expression surface archived and the `expression_engine` package scaffolded, so the repo reflects the subscribe-only architecture.
**Acceptance Criteria**
1. `Expression/Gesture/EarsPose/HeartRate.msg` moved to `archive/ros2/olaf_interfaces/msg/` via `git mv`, with a supersession README; `ModuleStatus.msg` kept.
2. `ears_node.py`, `head_node.py`, `bringup/launch/ears.launch.py` archived; `head_ears_driver` `console_scripts` emptied.
3. Empty `ros2/src/olaf_personality/` deleted.
4. `olaf_interfaces` CMakeLists/package.xml reference only `ModuleStatus.msg`; dead `geometry_msgs` dep removed.
5. `ros2/src/expression_engine/` scaffolded (ament_python, builds, `expression_engine_node` entry point resolves to a stub).
6. All 7 ROS packages build clean via `colcon build`.

### Story 5.2 — Documentation redirect *(Done 2026-05-15)*
As a future contributor, I want the architecture/PRD/README docs to reflect the subscribe-only direction, so I am not misled.
**Acceptance Criteria**
1. `technical-summary.md`, `high-level-architecture-diagram.md`, `architectural-patterns.md`, `repository-structure.md`, `source-tree.md` annotated with the Phase 2 redirect.
2. `phase1-prd.md` gets a v2.1 change-log row, NFR3 caveat, and "Phase 2 Preview" superseded note.
3. `README.md` repo-scope callout + corrected diagram/roadmap/tree.
4. No un-annotated "personality coordination on Pi" / `olaf_personality` claims remain in edited docs.

### Story 5.3 — Per-driver hardware verification *(GATE)*
As the maintainer, I want each reused driver demonstrably moving real hardware via a standalone test script, so the in-process-adapter assumption is proven before Epic 6.
**Acceptance Criteria**
1. A standalone script (Phase 1 component-test convention; runnable via the documented `PYTHONPATH … poetry run python` pattern) drives the **neck** driver and produces observable, correct motion on the robot.
2. Same for the **ears** driver (`ears_servo_driver.py`).
3. Same for the **head/eye** path (`head_i2c_client.py`) — observable eye/display output.
4. Each script exits 0 on success and is committed under the appropriate `scripts/` / module test location.
5. Epic 5 is declared "green" only when 5.1, 5.2, and all of 5.3 pass. Epics 6–8 MUST NOT start before this.

---

## Epic 6 — Expression Engine

**Goal:** A long-running engine that turns canonical events into correctly-timed body expression and never looks dead.

### Story 6.1 — Subscriber + schema-3 envelope validation
As the avatar, I want incoming events validated fail-fast, so a contract mismatch surfaces loudly.
**Acceptance Criteria**
1. Engine subscribes to all four topics on the configured domain/names (FR1).
2. `EventEnvelope` + the four payload models implemented; valid schema-3 events parse into typed objects (FR2).
3. `schema_version != 3` → raise, journald log, exit non-zero (FR4).
4. Engine never creates a publisher on the four topics (FR3) — asserted by a test.

### Story 6.2 — Map loader + vocabulary completeness
As the maintainer, I want `expression_map.yaml` loaded and validated against the pinned vocabulary at startup.
**Acceptance Criteria**
1. Loader reads `expression_map.yaml` + `expression_engine.toml` from the package `config/`.
2. Startup asserts coverage of the full pinned canonical set (FR6); a gap is fatal (NFR7).
3. `visible_only: true` asserted present on `nod` and `shake`; missing → fatal (FR7).
4. Adding a new map entry needs no code change (NFR5) — verified by a test adding a synthetic entry.
5. Unknown name at runtime → WARN `expression.unmapped_<topic>` + `default_*` fallback (FR13).

### Story 6.3 — Animation loop
As the avatar, I want smooth, correctly-timed motion toward target poses.
**Acceptance Criteria**
1. Fixed-tick render loop (servo tick configurable, default 100Hz).
2. Easing/interpolation toward target pose; mood eases over 2–4s and never snaps (NFR3).
3. `speech_emotion` pose target hits the body within the 30–80ms anticipatory window relative to `audio_frame_id` (NFR2).
4. Gesture attack/settle configurable; `[nod]`/`[shake]` land within ~150ms (NFR4).

### Story 6.4 — Reference expression end-to-end (freezes the schema)
As the maintainer, I want one hand-authored expression rendered end-to-end, so the map schema is proven and frozen for Epic 7.
**Acceptance Criteria**
1. One reference `speech_emotion` (e.g. `happy`) authored in `expression_map.yaml`.
2. FR14 mock publisher emits it; neck+ears+eyes visibly render it within the anticipatory window.
3. The `expression_map.yaml` **schema** (key structure for pose/LED/eye/heart, layering) is documented and declared frozen — Epic 7 authors against it without schema changes.

### Story 6.5 — Idle behaviour
As the avatar, I want to look alive between events.
**Acceptance Criteria**
1. Idle FSM distinguishes `sleeping` vs `listening`-and-silent.
2. `listening` + no `speech_emotion` >3s → mood-tinted neutral pose + micro-movement (FR10).
3. `sleeping` → eyes-closed posture + slow LED breathe (FR10).
4. Observably never statue-still and never twitchy in either state.

### Story 6.6 — Real LED adapter + breath-LED
As the avatar, I want real ambient LED behaviour.
**Acceptance Criteria**
1. Engine-owned WS2812 adapter behind the LED Protocol (FR11); no separate driver package.
2. Breath-LED renders during idle (ties to 6.5).
3. LED adapter swap is one Protocol implementation (NFR6).

### Story 6.7 — Service hardening
As the maintainer, I want the engine to run as a robust long-lived service.
**Acceptance Criteria**
1. systemd unit; structured JSON logging (NFR8).
2. Startup validation order: config → map → DDS → adapters → running; any failure fatal & journald-visible (NFR7).
3. All Phase 1 component test scripts still pass (NFR11).

---

## Epic 7 — Expression Authoring & Finalization

**Goal:** Populate and lock the full emotional vocabulary against the frozen schema, seeded from existing presets.

### Story 7.1 — Author + finalize the 12 speech-emotions
**Acceptance Criteria**
1. All first-class `speech_emotion` names authored in `expression_map.yaml`, seeded/adapted from `expressions.py`.
2. Each renders correctly on hardware via the mock publisher.
3. Each overlays correctly on a mood base (FR8).
4. `head_ears_driver/expressions.py` is deleted and no module imports it — the map is the single source of expression data (arch decision §12.2, NFR5).

### Story 7.2 — Author + finalize the 6 vocalizations + layered actions
**Acceptance Criteria**
1. All `vocalization` tags authored; audio-burst vs gesture-cue split honoured (FR7).
2. `nod`/`shake` compose on top of an active base pose (e.g. `[nod]` while `mood=happy`) without destroying it.
3. `visible_only` invariant verified for `nod`/`shake`.

### Story 7.3 — Activity → posture mapping
**Acceptance Criteria**
1. All `ActivityState` values (incl. both `working_submode`s) mapped to one posture each — simple, no elaborate transition choreography.
2. `sleeping → waking` begins within 100ms (NFR1).

### Story 7.4 — Mood → subtle ambient modifier
**Acceptance Criteria**
1. All `Mood` values mapped as a **low-impact** modifier (small LED/lean bias only) — mood does not strongly drive expression.
2. Mood transitions ease over 2–4s (NFR3).

### Story 7.5 — Locked-expression regression harness
**Acceptance Criteria**
1. Replay-based harness drives recorded event sequences and asserts pose/LED/eye/heart output (NFR10).
2. Runs in CI against the mock publisher.
3. Adding a new expression + re-locking is a documented, repeatable workflow; existing locks still pass.

---

## Epic 8 — Chest Display & Animated Heart

**Goal:** Turn the chest DSI panel into a managed display surface — a standalone portrait 2×2 dashboard (animated anatomical heart + three log panels) that boots straight to the app, with a view-manager for full-screen takeover. The heart's emotional input is composed by the engine from `expression_map.yaml` and forwarded to the chest renderer via a **delegating** adapter (mirrors the eye/ESP32 split). _(Re-scoped 2026-06-03 — see sprint-change-proposal-2026-06-03.md. Full BDD detail in `epics.md`.)_

### Story 8.1 — Chest display foundation
Boot-to-app (kill `getty@tty1`), fullscreen KMSDRM portrait 480×800, 2×2 grid (240×400 cells), view-manager (`DASHBOARD` + stubbed `FULLSCREEN_TAKEOVER`), systemd `Restart=always`.

### Story 8.2 — Animated anatomical heart widget
Warm, painterly (not clinical) heart top-left, glow-from-within, asymmetric lub-dub, always-alive resting beat + jitter, wake animation, ≥30 fps; exposes `set_beat_rate`/`set_intensity`/`set_tint` for later reactive wiring.

### Story 8.3 — Log panels
Three reusable `LogPanel` widgets with dummy data, scrolling/bounded/wrapped; `push()`/`set_lines()` wiring hook; heart stays the focal point.

**Deferred (not MVP):** 8.4 reactive heart (engine delegating `heart_adapter` forwards eased `heart:` state composed from `mood`+`activity`+`speech_emotion`, **not** `vocalization`, eased in lockstep with the body; FR12, NFR6, NFR10); 8.5 fullscreen-takeover views; 8.6 capacitive touch ("pet"); 8.7 regression-lock heart/dashboard; 8.8 boot/runtime hardening.

---

## Out of Scope (Phase 2)

- Base-module balancing/locomotion (separate ESP32 firmware track).
- Any publishing back to the pipeline; future touch/camera input (own project).
- STT, reasoning, Cartesia, cloud round-trips, belief state (owned by `olaf_companion`).
- Cross-restart pose persistence; reconnect-with-backoff / graceful degradation (deferred, mirrors companion v2).
- Live slider tuner for expressions (explicitly cut during SCP elicitation).
- Hailo acceleration for on-body inference (only if a measured workload later needs it).

---

## Next Steps / Handoff

1. **Architecture doc (Architect):** `docs/planning-artifacts/architecture/phase2-expression-engine.md` — adapter Protocol shapes, animation tick & easing model, idle FSM, map schema spec, startup-validation sequence, layering/composition model. Incorporates companion brief Appendix B.
2. **Story drafting (SM):** expand Epics 6–8 stories via `create-next-story`; Story 5.3 first (it gates everything).
3. **Implementation (Dev):** Epic 5.3 gate → 6 → 7 → 8.
4. **Schema pin:** record the exact `olaf_companion` tag this PRD's canonical-vocabulary assumptions are pinned to before Epic 6.2.

*Authoritative inputs: `docs/planning-artifacts/sprint-change-proposal-2026-05-15.md` and the companion's `olaf-embodiment-brief.md` (wire contract, Appendix A/B).*
