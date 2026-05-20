# Story 7.3: Activity → posture mapping

Status: ready-for-dev

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

- [ ] Task 1: **Author activity postures** — extend
  `expression_map.yaml` `activity.*` blocks with full neck + ears
  poses. Seed from bear convention (sleeping = drooped neck/ears;
  listening = perked ears + neutral neck; thinking = head pan
  left + cocked ears; delegating = head pan right + perked ears;
  speaking = forward + perked; going_to_sleep = neck dip + ears
  droop).
- [ ] Task 2: **Eye-target review** — confirm each activity's
  `eye.expression` resolves cleanly via the AR10 table. If a new
  activity-only eye is wanted, add it as a device-extra (precedent:
  `flirty` = EXPR 14 in 7.1a).
- [ ] Task 3: **Tests** — full-block coverage assertion, bounds
  check (incl. right_pan ≤ 50), YAML integrity (no `:value`
  no-space). Existing suites stay green.
- [ ] Task 4: **Hardware verify** — small helper to step through
  ActivityStates from the Pi (drive `system_status` register +
  the engine's activity topic). Kamal-locks each.

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

(none yet)

### Agent Model Used

### Debug Log References

### Completion Notes List

### File List
