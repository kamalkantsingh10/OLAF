# Story 7.3: Activity → posture mapping

Status: in-progress

<!-- Created 2026-05-20 after 7.1a/b/c froze the per-emotion look.
Finalises the `activity.*` block in expression_map.yaml so every
ActivityState (starting / sleeping / waking / listening /
working.{thinking,delegating} / speaking / going_to_sleep) drives a
distinct, complete body posture (neck + ears + eye) — not just the
current minimal stubs. -->

## Story

As the avatar,
I want every **ActivityState** to drive a complete, distinct body
posture (neck pose + ears pose + eye expression), so that "what
OLAF is currently doing" is legible from the body alone — even
without any active speech_emotion or vocalization.

## Background

- `expression_map.yaml` `activity.*` is currently a sparse stub —
  each state carries just one or two pose hints (e.g.
  `sleeping: { pose: { neck: { tilt: -25 } }, eye: { expression:
  closed }, led: { pattern: breathe_slow } }`). Ears + roll + pan
  are mostly empty.
- The full ActivityState set (FR-side) is:
  `starting / sleeping / waking / listening /
  working.thinking / working.delegating / speaking /
  going_to_sleep` — **8 leaf states** (working is nested).
- Story 6.5 ("idle behaviour") is `ready-for-dev` and overlaps:
  it owns the **idle FSM** (ACTIVE / AMBIENT / DORMANT / BOOT)
  and the micro-drift in AMBIENT / DORMANT. **7.3 owns the
  per-state POSE TARGET** that the FSM steers toward — they are
  complementary, not redundant.
- The render-loop already consumes `activity.*` via the
  expression_map (`_compose` reads activity → resolves pose).
  This story authors the values, not the dispatch.
- Eyes (7.1a) defines `EXPR_NEUTRAL/SLEEPY/WINK` and the 12
  canonical speech-emotions; activity-state eyes (`boot/closed/
  waking/open/focused/distant/animated/closing`) currently squash
  to those via `_CANONICAL_TO_ESP32` in `eye_adapter.py`.

## Acceptance Criteria

1. **Completeness** — every leaf ActivityState in
   `expression_map.yaml` `activity.*` has a full `pose.neck`
   (pan/tilt/roll) and `pose.ears` (LP/LT/RP/RT) block.
2. **Distinctness** — visibly distinct postures at L1 on hardware
   for the 8 leaf states (Kamal-confirmed).
3. **Bounds** — values lie within mechanical envelopes
   (neck pan ±80°, tilt ±20°, roll ±15°; ears pan ±50°, tilt
   −60..+90°). Right_pan ≥65° MUST be avoided (binds on SCS0009).
4. **Eye targets** — each activity references an existing device
   EXPR (via `eye_adapter._CANONICAL_TO_ESP32`); if a new device
   eye is wanted (e.g. a richer "focused" look), record the new
   target + author it; otherwise reuse what's there.
5. **Compatibility** — no regression to the 15 frozen speech-
   emotion poses (Stories 7.1a/b/c). Render loop should compose
   activity (base) + speech_emotion (additive bias) cleanly.
6. **Tests** — host tests validating every leaf ActivityState
   has full neck + ears blocks, bounds respected, no `key:value`
   no-space YAML traps. Existing suites stay green.
7. **Hardware verify** — Kamal walks each ActivityState (via an
   activity-stepper helper or by tickling `system_status` from
   the Pi) and confirms the posture reads.

## Tasks / Subtasks

- [x] Task 1: **Author activity postures** — extend
  `expression_map.yaml` `activity.*` blocks with full neck + ears
  poses. Seed from bear convention (sleeping = drooped neck/ears;
  listening = perked ears + neutral neck; thinking = head pan
  left + cocked ears; delegating = head pan right + perked ears;
  speaking = forward + perked; going_to_sleep = neck dip + ears
  droop).
- [x] Task 2: **Eye-target review** — confirm each activity's
  `eye.expression` resolves cleanly via the AR10 table. If a new
  activity-only eye is wanted, add it as a device-extra (precedent:
  `flirty` = EXPR 14 in 7.1a).
- [x] Task 3: **Tests** — full-block coverage assertion, bounds
  check (incl. right_pan ≤ 50), YAML integrity (no `:value`
  no-space). Existing suites stay green.
- [ ] Task 4: **Hardware verify** — small helper to step through
  ActivityStates from the Pi (drive `system_status` register +
  the engine's activity topic). Kamal-locks each.
  - [x] Harness built: `test/e2e_activity_run.py` (walks all 8 leaf
    states one at a time, holds each, prints authored vs eased pose).
  - [ ] On-robot walk + Kamal-lock of each state — **PENDING** (needs
    the avatar; run command in the harness docstring).

## Dev Notes

### Source of truth

- **Activity list:** `expression_map.yaml` `activity` block;
  enumeration of leaves above (8).
- **Render-loop consumer:** `render_loop._compose` resolves
  activity via `map.resolve("activity", …)`. Posture goes to the
  `act_neck` / `act_ears` channels (additive base layer).
- **Mechanical envelopes:** neck `_LIMITS` in `neck_adapter.py`
  (±80 / ±20 / ±15); ears `_LIMITS` in `ears_adapter.py`
  (±50 / −60..+90).
- **Idle FSM (Story 6.5):** orthogonal — 6.5 owns the
  transition logic; 7.3 owns the per-state target.

### Constraints / guards

- Do NOT change the 15 speech-emotion poses (7.1a/b/c frozen).
- Do NOT add a per-emotion 3-level model to activity — activities
  are state targets, not transients. Idle drift is 6.5's domain.
- `right_pan ≥65°` BINDS on SCS0009 — keep ≤50°
  ([[reference-head-esp32-gotchas]] sibling).

### References

- [Source: `ros2/src/expression_engine/config/expression_map.yaml` § `activity`]
- [Source: `ros2/src/expression_engine/expression_engine/render_loop.py` `_compose`]
- [Source: Story 6.5 — `docs/implementation-artifacts/6-5-idle-behaviour.md`]
- Memory: [[project-expression-design-dna]], [[project-epic7-idle-motion]],
  [[project-epic6-7-sequencing]]

## Dev Agent Record

### Progress

- 2026-05-22: Tasks 1–3 complete; Task 4 harness built. Awaiting the
  on-robot walk (AC#7) before flipping to `review`.

### Agent Model Used

claude-opus-4-7 (1M context)

### Debug Log References

- `PYTHONPATH="ros2/src/expression_engine:${PYTHONPATH}" poetry run
  python -m pytest ros2/src/expression_engine/test/ -p no:cacheprovider`
  → 278 passed (host suite; includes the new 44 activity-posture cases).

### Implementation Plan

- Authored the 8 leaf `activity.*` postures as the absolute BASE layer
  (full neck pan/tilt/roll + ears LP/LT/RP/RT each), seeded from the
  bear convention in Task 1. Kept every value inside the adapter
  mechanical envelopes and right_pan ≤50 (SCS0009 binding guard).
- Eye-target review (Task 2): every activity reuses an existing device
  eye already in `eye_adapter._CANONICAL_TO_ESP32` (boot/closed/waking/
  open/focused/distant/animated/closing). No NEW device eye authored —
  the body posture carries the per-state distinctness; a richer
  `focused`/`distant` device eye is left as a deferred follow-up
  (recorded below).
- Added explicit `eye.intensity` per state (sleeping/going_to_sleep = 1
  dim; speaking = 4 animated; others 2–3) for extra legibility.
- TDD: wrote `test_activity_postures.py` first (red against the sparse
  stubs), then authored the YAML to green.

### Completion Notes List

- ✅ AC#1 Completeness — all 8 leaf states carry full neck + ears
  blocks (`test_leaf_has_full_neck_and_ears_block`).
- ✅ AC#3 Bounds — neck within ±80/±20/±15, ears within ±50/−60..+90,
  right_pan ≤50 (`test_neck_within_envelope`, `test_ears_within_envelope`,
  `test_right_pan_safe_for_scs0009`).
- ✅ AC#4 Eye targets — every `eye.expression` resolves via the AR10
  table (`test_eye_expression_resolves_via_ar10`).
- ✅ AC#5 Compatibility — whole map still loads; the 12 speech-emotion
  poses untouched (`test_speech_emotions.py` green); render-loop
  activity+speech composition test still green.
- ✅ AC#6 Tests — 44 new cases; full host suite 278 passed. Fixed one
  stale SANITY threshold in `test_render_loop.py`
  (`TestWakeShortCircuit`): sleeping→waking tilt gap shrank 25°→14°
  because authored sleeping tilt is now −18 (was an out-of-envelope −25
  that the adapter silently clamped to −20). The real NFR1 assertion
  (`moved >= 0.25 * gap`) is unchanged.
- ⏳ AC#2 Distinctness / AC#7 Hardware verify — host-side pairwise
  distinctness asserted; the on-robot human confirmation is PENDING
  (run `test/e2e_activity_run.py` on the avatar).
- 📝 Deferred follow-up: richer activity-only device eyes for
  `focused`/`distant` (currently both → `neutral` on the device). Body
  asymmetry already differentiates thinking vs delegating, so this is a
  nice-to-have, not a blocker.
- 📝 Note: the sleeping `eye: closed` maps to device `sleepy`, NOT a
  true lids-shut render — the device eye set has no dedicated "closed".
  Flagged for the same deferred device-eye follow-up if a real shut-eye
  is wanted for sleep.

### File List

- `ros2/src/expression_engine/config/expression_map.yaml` (modified —
  authored the 8 `activity.*` leaf postures; `starting` eye → sleepy L3;
  `listening` eye → curious L2; added `led_overlay` tint to all 8 moods)
- `ros2/src/expression_engine/expression_engine/adapters/eye_adapter.py`
  (modified — `_CANONICAL_TO_ESP32["boot"]` neutral → sleepy; +
  `_ACTIVITY_TO_STATUS` table + `status_for_activity()` +
  `set_system_status()` / `set_led_overlay()` passthrough)
- `ros2/src/expression_engine/test/test_activity_postures.py` (new —
  44 host tests: completeness, bounds, distinctness, eye-target, YAML)
- `ros2/src/expression_engine/test/e2e_activity_run.py` (new —
  on-robot activity-stepper harness)
- `ros2/src/expression_engine/test/test_render_loop.py` (modified —
  stale wake-gap sanity threshold 15→10; comment refresh)
- `modules/head/firmware/src/animation_engine.cpp` (modified — boot
  default expression EXPR_NEUTRAL@2 → EXPR_SLEEPY@3)
- `modules/head/firmware/src/main.cpp` (modified — removed setup()
  auto-wake + demo-on-boot; removed the SPEAKING-only expression gate
  so the engine owns the eye in every state)
- `modules/head/firmware/src/led_strip.cpp` (modified — strip lit ONLY
  for listening/processing/speaking; SPEAKING rendered white so the
  mood overlay tints it; overlay gated to lit states so dark states
  stay off)
- ⚡ Head firmware COMPILED + OTA-FLASHED to olaf-head.local
  (192.168.118.81) 2026-05-22; device rebooted OK.
- `ros2/src/expression_engine/expression_engine/render_loop.py`
  (modified — `_handle_events` now drives head `system_status` from
  activity + LED `overlay` from mood, both fire-on-change)
- `ros2/src/expression_engine/expression_engine/adapters/_testing.py`
  (modified — Recording/Null delegating doubles record/no-op the new
  set_system_status / set_led_overlay calls)
- `ros2/src/expression_engine/test/test_status_led_wiring.py` (new —
  25 host tests: activity→status + mood→overlay mappings, fire-on-change,
  eye_adapter passthrough)
- `docs/implementation-artifacts/7-3-activity-to-posture-mapping.md`
  (modified — status, tasks, Dev Agent Record)
- `docs/implementation-artifacts/sprint-status.yaml` (modified —
  7-3 → in-progress)

### Change Log

- 2026-05-22: Story 7.3 — authored full activity→posture base layer
  (8 leaf states, neck+ears+eye), added host test suite + on-robot
  stepper harness. Host suite green (278). On-robot walk pending.
- 2026-05-22: Firmware boot default eye → EXPR_SLEEPY L3 + engine
  `boot`→sleepy (Kamal request). ⚠️ Surfaced two firmware interactions
  that block visibility: `setup()` auto-wakes ~200ms after boot, and
  I2C expression is gated to `system_status == SPEAKING` only — see
  Completion Notes "Firmware findings". Awaiting Kamal direction.
- 2026-05-22: `listening` eye → `curious` @ L2 (owner call). Body kept
  neutral-attentive (curious head-cock optional, deferred).
- 2026-05-22: Engine-side status/LED wiring built (host-tested, 303
  passed). Render loop now drives head `system_status` from activity
  (fire-on-change; starting/sleeping→idle, waking→woke_up, listening→
  listening, working→processing, speaking→speaking, going_to_sleep→
  going_idle) and LED `overlay` tint from mood (warm/cool/hot/bright).
  Activity stepper harness prints the expected `system_status` + strip
  lit/dark per state.
- 2026-05-22: Firmware deltas built + OTA-flashed (olaf-head.local):
  boot sleepy L3, removed auto-wake + demo-on-boot, removed SPEAKING
  expression gate, strip lit-only-for-3-states + SPEAKING white +
  overlay gated to lit states. `pio run` SUCCESS (flash 13%); OTA
  upload OK; device rebooted + reachable. Hardware walk (postures +
  eyes + LEDs end-to-end) now possible — pending Kamal.

### Follow-up design — eye / LED / status integration (agreed 2026-05-22)

This grew out of 7.3 but architecturally spans firmware + 6.6 (LED
adapter) + 6.5 (idle). Captured here so it isn't lost; NOT yet built.

Model:
- **Eyes** are engine-driven in ALL states. Speaking → `speech_emotion`;
  otherwise → the activity resting-pose eye, steered by 6.5 idle
  behaviour. Requires REMOVING the firmware SPEAKING-only expression
  gate (`main.cpp:119`).
- **Servos (neck/ears)** = activity resting pose (7.3) + 6.5 drift. The
  "resting position" (servos + eye) is OWNED by 6.5; 7.3 only supplies
  the per-activity pose targets.
- **WS2812 strip** driven by `activity → system_status`; lit ONLY for
  LISTENING / PROCESSING / SPEAKING, dark otherwise. Symmetrical
  center-out animation (8-LED array `[0..3]|[4..7]`), Alexa-style,
  pattern matched to intent (replace random-rainbow SPEAKING). Colour =
  current mood mapped to nearest of the 5 `led_overlay` tints
  (happy→WARM, excited→BRIGHT, playful→WARM, calm/curious/thoughtful/
  sleepy→COOL, grumpy→HOT).

Firmware deltas: boot SLEEPY L3 (done) · remove `setup()` auto-wake
(boot stays IDLE until first I2C) · remove SPEAKING expression gate ·
LED lit-only-for-3-states + mood tint + symmetrical patterns.

Engine deltas: map `ActivityState → system_status` (send on activity
change — the missing wire) · map `mood → led_overlay` tint (send on
mood change). Both are new and host-testable.
