# Sprint Change Proposal — Expression Engine Re-scope

**Date:** 2026-05-15
**Author:** Kamal Singh (with PM agent — John)
**Status:** Draft — awaiting approval
**Trigger type:** Necessary pivot based on new external contract
**Change-checklist:** `.bmad-core/checklists/change-checklist.md` (Sections 1–6, worked incrementally)

> **Terminology.** "Avatar" = the whole OLAF robot/project (OLAF *is* an avatar). The component built here is the **expression engine** — ROS package `expression_engine`. We do not use "embodiment" as our vocabulary; the companion's external doc keeps its own filename `olaf-embodiment-brief.md` (their file), quoted literally only.

---

## 1. Identified Issue Summary

The sibling project `olaf_companion` finalized its publisher contract at `schema_version=3` on 2026-05-10. That work (the `olaf-embodiment-brief.md` plus the companion's own sprint-change-proposal-2026-05-10) **moved the renderer mapping out of the publisher and onto the body's side**, and collapsed everything the AI agent communicates to the body into **four typed ROS 2 topics**:

- `mood` — slow disposition (≤4/hour, latched)
- `activity` — 7-state FSM (latched)
- `speech_emotion` — per-segment, audio-anchored ~30–80ms anticipatory
- `vocalization` — punctual audio-bursts + gesture cues (`nod`/`shake`)

The OLAF body repo's pre-existing Phase 2-shaped scaffolding was sketched **before** this contract existed and is now misaligned:

- `olaf_interfaces/msg/Expression.msg` — a 7-emotion enum incompatible with the companion's 12 canonical first-class emotions, 8 moods, 7 activity states, 6 vocalizations.
- `Gesture.msg`, `EarsPose.msg`, `HeartRate.msg` — raw-angle / per-module public surfaces, superseded by a body-side `expression_map.yaml` + in-process driver calls.
- `ros2/src/olaf_personality/` — empty directory placeholder for the rendering coordinator.
- `architecture/` docs place "Personality Coordination" and "Hybrid AI Processing" on this Pi; those responsibilities now live in `olaf_companion`. This repo is the **expression engine**: subscribe, map, render.

**Core problem stated plainly:** the body's external interface was designed per-module and pre-contract. The contract is now stable and consumer-agnostic. The body must converge on *subscribe-only to 4 canonical topics, with all rendering decisions as body-side data*.

Crucially: **Phase 1 is not the problem.** Phase 1's NFR3 explicitly deferred AI/personality to Phase 2. Everything being archived is Phase 2 scaffolding that was never wired into anything. The hardware, drivers, firmware, configs, and calibration that Phase 1 delivered are exactly the "components we have built" to reuse.

### Evidence

- `olaf_companion/.../planning-artifacts/olaf-embodiment-brief.md` (2026-05-10) — defines the wire and the consumer-agnostic boundary.
- Companion's `sprint-change-proposal-2026-05-10.md` — records the mapping moving out of the publisher (schema 2→3).
- `ros2/src/olaf_interfaces/msg/Expression.msg` — 7-enum, demonstrably incompatible with the 12 canonical names.
- `ros2/src/olaf_personality/` — empty (not even a package skeleton); confirms zero sunk code, low pivot cost.

---

## 2. Epic Impact Summary

### Phase 1 — all epics survive untouched

| Epic (implied by story prefix) | Stories | Impact |
|---|---|---|
| Epic 0 — Dev environment | 0.1–0.7 | None |
| Epic 1 — Head module (eyes) | 1.3, 1.4 | None — driver reused in-process |
| Epic 2 — Base module (locomotion/balancing) | 2.4–2.8 | **Out of scope** — orthogonal ESP32 firmware track, stays as-is |
| Epic 3 — Neck module | 3.1, 3.4 | None — driver reused |
| Epic 4 — Ears module | 4.1–4.4 | None — driver reused |

### Phase 2 — abandoned-as-sketched, replaced by four new epics

The implicit per-module Phase 2 surface (the four msgs + empty `olaf_personality` + "personality coordination on Pi") is **abandoned in its current form**. Replacement structure, all under a new `docs/prd/phase2-prd.md`:

| Epic | Value delivered | Blocks |
|---|---|---|
| **Epic 5 — Restructure & driver verification** *(gate)* | Clean foundation; every reused driver proven on real hardware. Nothing downstream starts until green. | 6, 7, 8 |
| **Epic 6 — Expression engine** | A working engine: canonical events in → expression out, and never statue-still when idle. | — |
| **Epic 7 — Expression authoring & finalization** | The full emotional vocabulary, tuned and regression-locked. | — |
| **Epic 8 — Heart display animation** | Secondary surface adds to aliveness, after core body language is locked. | — |

**Sequencing:** Epic 5 (gate) → Epic 6 → Epic 7 → Epic 8, strictly in order. No dependency on companion *implementation* — develop against the schema with a mock publisher.

#### Epic 5 — Restructure & driver verification (gate)

| Story | Scope |
|---|---|
| 5.1 | Execute archival/delete/rename/scaffold per §4 (move 4 msgs to `archive/`, delete empty `olaf_personality/`, scaffold `expression_engine`, edit CMakeLists). |
| 5.2 | Doc edits per §4 (architecture diagram, patterns, technical-summary, repository-structure, Phase 1 PRD change-log, README). |
| 5.3 | Per-driver hardware verification: neck, ears, head/eye drivers each move real hardware via a standalone test script (Phase 1 component-test convention). Gate is green only when all pass. |

#### Epic 6 — Expression engine

| Story | Scope |
|---|---|
| 6.1 | Subscriber on the 4 topics + schema-3 `EventEnvelope` validation; fail-fast (raise, exit non-zero, journald) on `schema_version != 3`. |
| 6.2 | `expression_map.yaml` schema + loader + vocabulary-completeness check (strict against the pinned companion release; runtime-graceful — see §3). |
| 6.3 | Animation loop: fixed render tick, easing curves, the 30–80ms anticipatory window, gesture attack/settle. |
| 6.4 | One hand-authored **reference expression** rendered end-to-end (e.g. `speech_emotion=happy` → servos+eyes) — proves and **freezes the schema** for Epic 7. |
| 6.5 | Idle behavior: idle FSM (sleeping vs listening-and-silent), micro-movement, return-to-mood-neutral after 3s of `speech_emotion` silence. |
| 6.6 | Real WS2812 LED adapter + breath-LED (first real LED use; infrastructure reused by Epic 7). |
| 6.7 | systemd unit, structured JSON logging, startup validation order (config → map → DDS → adapters → running). |

#### Epic 7 — Expression authoring & finalization

| Story | Scope |
|---|---|
| 7.1 | Author + finalize the 12 **speech-emotions** — the primary expression layer, the bulk of the work. |
| 7.2 | Author + finalize the 6 **vocalizations**, including **actions layered on top of base poses** (`[nod]`/`[shake]` composed over a base pose). |
| 7.3 | **Activity → posture mapping** — simple, one posture per state. |
| 7.4 | **Mood → subtle ambient modifier** — lightweight; low direct impact on expression (small LED/lean bias only). |
| 7.5 | Locked-expression **regression harness** — replay events, assert pose/LED/eye output, prevent drift as the vocabulary grows. |

#### Epic 8 — Heart display animation

| Story | Scope |
|---|---|
| 8.1 | Mood/activity-driven heart rendering on the 4" Pi display, engine-owned (replaces retired `HeartRate.msg`). |

---

## 3. Extensible Vocabulary — Design Principle

The expression vocabulary **will grow over time**. The companion brief makes vocabulary completeness a startup blocker; this principle reconciles strictness with growth and becomes a Phase 2 NFR:

1. **Startup-strict against the pinned companion release.** The loader validates `expression_map.yaml` covers the canonical set *of the pinned `olaf_companion` tag* — complete-for-this-version, not complete-forever.
2. **Growth is a pure data edit.** Adding an expression is a new block in `expression_map.yaml`; the engine is vocabulary-agnostic and never needs a code change to learn a new name.
3. **Runtime grace.** An unknown name (companion ran ahead of our map) logs `expression.unmapped_<topic>` at WARN and falls back to a `default_*` pose — the body never freezes (companion brief §Risks).
4. **Lockstep for known growth.** When companion adds expressions and bumps schema, we extend the map and re-pin the companion tag in the *same* change.

---

## 4. Artifact Adjustment Needs

### Archive (move to `archive/`, removed from active build, preserved in git)

| Artifact | Reason |
|---|---|
| `ros2/src/olaf_interfaces/msg/Expression.msg` | Replaced by canonical 12-emotion / 8-mood vocabulary, body-side mapping |
| `ros2/src/olaf_interfaces/msg/Gesture.msg` | Raw-angle public surface; engine uses driver Python APIs in-process |
| `ros2/src/olaf_interfaces/msg/EarsPose.msg` | Same as Gesture |
| `ros2/src/olaf_interfaces/msg/HeartRate.msg` | Replaced by mood/activity-driven heart animation (Epic 8) |

Moved to `archive/ros2/olaf_interfaces/msg/` with a `README.md` explaining the supersession (points to this proposal). `ModuleStatus.msg` is **kept** — health monitoring is orthogonal to expression.

Two ROS-node wrappers were coupled to these msgs and are archived alongside (they *are* the retired per-module surface — see the three-layer split below):

| Artifact | Reason |
|---|---|
| `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/ears_node.py` | Subscribed to `/olaf/ears/expression`+`/olaf/ears/pose` (archived msgs) |
| `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/head_node.py` | Same per-module subscriber pattern |
| `ros2/src/olaf_bringup/launch/ears.launch.py` | Launched the now-removed `ears_node` executable |

Moved to `archive/ros2/olaf_drivers/head_ears_driver/` and `archive/ros2/olaf_bringup/launch/` with a `README.md`. `head_ears_driver`'s `console_scripts` entry points were emptied accordingly.

### Delete

`ros2/src/olaf_personality/` — empty directory, superseded by the new `expression_engine` package.

### Edit (substantive doc/code changes)

| Artifact | Change |
|---|---|
| `docs/prd/phase1-prd.md` | Add change-log entry (v2.1): "Phase 2 redirected to consume canonical 4-topic contract from `olaf_companion`; see `docs/prd/phase2-prd.md`." Update NFR3 to point to the new Phase 2 PRD by name. No body deletions. |
| `docs/architecture/high-level-architecture-diagram.md` | In the Pi block, replace "Personality Coordination / AI Integration" bullets with "Expression Engine (subscribes to `olaf_companion`)" + "Driver Nodes". Add a publisher block above showing `olaf_companion` and the 4 typed topics as the wire. |
| `docs/architecture/architectural-patterns.md` | Replace "Hybrid AI Processing (Local Fast-Path + Cloud Reasoning)" with an "Expression Engine Pattern (Subscribe-only renderer of canonical events)" entry. Add a caveat to "Hardware Abstraction via Driver Nodes": drivers are consumed in-process by the engine; ROS topics retained as low-level inspection surface only. |
| `docs/architecture/technical-summary.md` | Same redirection (personality/AI now external). |
| `docs/architecture/repository-structure.md` | Document the new `expression_engine` package; remove `olaf_personality`. |
| `ros2/src/olaf_interfaces/CMakeLists.txt` | Remove the four archived msg entries; keep `ModuleStatus.msg`. |
| `ros2/src/olaf_bringup/launch/` | Add `expression_engine.launch.py` (drivers + `expression_engine_node`). |
| `README.md` (top-level) | Trim any "personality coordination / AI on Pi" framing so new contributors aren't misled. |

### New (authored after this proposal is approved)

- `docs/prd/phase2-prd.md` — the Phase 2 PRD (PM/Architect).
- Four epics (5/6/7/8) + their stories (SM).
- `ros2/src/expression_engine/` package (see §5 structure).
- `expression_map.yaml`, `expression_engine.toml` — starting points in companion brief Appendix B, living in `ros2/src/expression_engine/config/`.

### Driver packages — three-layer split (refinement)

The original "drivers untouched" line was too coarse. Inspecting `head_ears_driver` during Epic 5.1 showed each driver package mixes three layers; only one is retired:

| Layer | Files (head_ears_driver example) | Disposition |
|---|---|---|
| **Hardware logic** | `ears_servo_driver.py`, `head_i2c_client.py` | **Kept, untouched.** Imported in-process by the engine's adapters (Epic 6). |
| **Expression content** | `expressions.py` (emotion → ear-pose presets; `ears_demo.py` notes "12 Emotional States") | **Kept, becomes Epic 7 seed** for `expression_map.yaml` — Epic 7 adapts proven poses rather than authoring from blank. |
| **Retired skin** | `ears_node.py`, `head_node.py` (subscribe to archived msgs), `bringup/launch/ears.launch.py` | **Archived** — this is the per-module topic surface the pivot eliminates. |

### Untouched

All other `olaf_drivers/*` code (neck/base/torso drivers, calibration tools), all `modules/*/firmware` (incl. base balancing), all `config/*`, `libs/scservo_sdk`, Phase 1 PRD body, `phase1-tracking.md`, all completed stories 0.x–4.x, `scripts/`, `tools/`.

---

## 5. Target File Structure

```
ros2/src/expression_engine/      # NEW package (no olaf_ prefix — project is OLAF)
├── package.xml                  # deps: rclpy, olaf_interfaces, driver pkgs
├── setup.py                     # entry_point: expression_engine_node = expression_engine.node:main
├── setup.cfg
├── resource/expression_engine
├── config/
│   ├── expression_map.yaml      # mood/activity/emotion/vocalization → pose/LED/eye/heart
│   └── expression_engine.toml   # DDS domain, topic names, anim timings, idle params
├── expression_engine/
│   ├── __init__.py
│   ├── node.py                  # ROS2 node: wires subscribers → render loop
│   ├── schema.py                # schema-3 EventEnvelope + 4 payload models, fail-fast
│   ├── subscribers.py           # 4 topic subs
│   ├── map_loader.py            # loads + validates expression_map.yaml vs pinned vocab
│   ├── render_loop.py           # fixed-tick loop, easing, anticipatory window
│   ├── idle.py                  # idle FSM: micro-move, breath-LED, return-to-neutral
│   └── adapters/
│       ├── base.py              # Adapter Protocols (servo/eye/led/heart)
│       ├── neck_adapter.py      # imports neck_driver         [real, Epic 6]
│       ├── ears_adapter.py      # imports head_ears_driver     [real, Epic 6]
│       ├── eye_adapter.py       # imports head_ears_driver     [real, Epic 6]
│       ├── led_adapter.py       # WS2812 direct, engine-owned   [real, Epic 6.6]
│       └── heart_adapter.py     # 4" Pi display, engine-owned   [stub→real, Epic 8]
└── test/
    ├── test_schema.py
    ├── test_map_loader.py
    ├── test_locked_expressions.py   # regression harness (Epic 7.5)
    └── mock_publisher.py            # fake companion driving all 4 topics for dev/CI

archive/ros2/olaf_interfaces/msg/
├── Expression.msg  Gesture.msg  EarsPose.msg  HeartRate.msg
└── README.md                    # supersession note → this proposal

ros2/src/olaf_interfaces/msg/ModuleStatus.msg     # KEPT
ros2/src/olaf_drivers/*                           # UNTOUCHED, imported in-process
ros2/src/olaf_personality/                        # DELETED (empty)
docs/prd/phase2-prd.md                            # NEW
```

---

## 6. Recommended Path Forward

**Option 3 — Re-scope** (selected over Direct-Adjustment and Rollback).

- **Direct Adjustment** rejected: carrying zombie msg types between the wire and the actuators dilutes the brief's strongest invariant ("mapping is data, body-side, keyed on canonical names") and confuses future contributors.
- **Rollback** rejected: the issue is design-level, not commit-level; `git revert` adds history noise without resolving the conceptual ambiguity.
- **Re-scope** chosen: matches the companion brief's intent, reuses every Phase 1 artifact, throws away ~4 unused files + 1 empty dir, produces a tractable four-epic Phase 2 plan. Lowest *delivery* risk because we are cutting paper, not code.

---

## 7. PRD MVP Impact

- **Phase 1 MVP:** unchanged.
- **Phase 2 MVP:** defined for the first time, not cut. Target (fleshed out in `phase2-prd.md`):
  - Subscribe + schema-3 validation + fail-fast on bad envelopes / wrong schema version.
  - `expression_map.yaml` covers the full canonical set of the pinned companion release (extensible per §3).
  - Servo/eye loop hits the 30–80ms anticipatory window (companion NFR5).
  - `[nod]`/`[shake]` land within ~150ms; crisp attack.
  - Idle return-to-mood-base after 3s of `speech_emotion` silence; never statue-still.
  - Hardware swap = one new Protocol adapter (brief Success Criteria).
  - Finalized expressions are regression-locked (Epic 7.5).

---

## 8. High-Level Action Plan & Agent Handoff

| Step | Owner | Action |
|---|---|---|
| 1 | **You** | Approve this proposal. |
| 2 | **PM/Architect** | Author `docs/prd/phase2-prd.md` anchored on the companion brief + this proposal. |
| 3 | **Architect** | Phase 2 architecture: engine node internals, Protocol adapter shapes, animation tick model, idle FSM. Incorporate brief Appendix B. |
| 4 | **SM** | Draft stories for Epics 5/6/7/8 from `phase2-prd.md`. Epic 5 first (it gates the rest). |
| 5 | **Dev** | Implement Epic 5 (restructure + driver verification gate) — must be green before Epic 6 starts. Then 6 → 7 → 8. |

### Validation — how we'll know the change worked

- Epic 5 gate green: every reused driver moves real hardware via its standalone test script.
- `ros2 topic list` on a running stack shows the expression engine subscribing to the 4 canonical topics and **no** legacy expression topics.
- The mock companion publisher driving schema-3 events produces visible, correctly-timed body expression.
- Startup fails loudly on an incomplete `expression_map.yaml` or a `schema_version != 3` event.
- Adding a new expression to `expression_map.yaml` requires **no code change** and the regression harness still passes.
- All Phase 1 component test scripts still pass unchanged (proof reused components were not regressed).

---

## 9. Decisions Captured (this session)

1. Naming: robot/project = "avatar"; the component = **expression engine**; package = `expression_engine` (no `olaf_` prefix).
2. Location: rebuild **inside this repo**.
3. Existing module topic surfaces: **retire all four**; engine imports driver Python modules directly, no intermediate ROS topics.
4. Archive scope: **architecture skin only** — keep all proven Phase 1 components.
5. Base-module balancing: **out of scope**, stays on its own ESP32 firmware track.
6. Epic structure: **four epics (5/6/7/8)**. Idle/aliveness and the LED adapter are *stories* inside Epic 6, not their own epic. Heart display is its own (last) epic.
7. Hard gate: Epic 5 (restructure complete + all drivers verified on hardware) **blocks** Epics 6–8.
8. Overlap resolution: Epic 6 includes one reference expression that **freezes the schema**; Epic 7 mass-authors against the frozen schema — no double work.
9. Missing LED/heart drivers: engine owns them directly; LED becomes real in Epic 6.6, heart in Epic 8 (stub before that).
10. Config location: inside the package — `ros2/src/expression_engine/config/`.
11. Vocabulary is **extensible** — strict against the pinned companion release, graceful at runtime, growth is a data edit (§3).
12. Authoring scope: no live tuner; mood is a low-impact subtle modifier; activity is a simple posture mapping.
13. Phase 2 PRD: `docs/prd/phase2-prd.md`.
14. Path: **Option 3 — Re-scope**.
15. Driver three-layer split (found during Epic 5.1 execution): hardware logic kept & imported in-process; `expressions.py` presets kept as Epic 7 seed; ROS-node wrappers (`ears_node.py`, `head_node.py`) + `ears.launch.py` archived as retired skin. All 7 ROS packages verified colcon-buildable post-restructure.

---

*This proposal lives in the OLAF body repo. Its companion-side counterpart is `olaf_companion/.../sprint-change-proposal-2026-05-10.md`. Keep schema-version assumptions pinned to a tagged release of `olaf_companion` — bump in lockstep.*
