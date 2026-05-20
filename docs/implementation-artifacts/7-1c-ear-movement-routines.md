# Story 7.1c: Per-emotion ear movement routines (3-level intensity)

Status: review

<!-- Owner-directed follow-on to 7.1b (2026-05-20) — same per-
expression iteration loop applied to ears. Parallel structure to
7.1a (eyes) + 7.1b (neck); 3-level intensity model
([[project-story-71bc-plan]]). -->

## Story

As the avatar,
I want each speech-emotion to drive a distinct **ear pose +
optional micro-drift + optional peak gesture** at intensity levels
1 / 2 / 3,
so that OLAF's *ears* communicate the emotion alongside the eyes
and neck — perked when alert, drooped when sad, flattened when
afraid, asymmetric when curious.

## Background

- Story 7.1a froze the eyes and 7.1b froze the neck. Same iteration
  loop applies to the **ears**.
- `expression_map.yaml` already carries `pose.ears` (`left_pan,
  left_tilt, right_pan, right_tilt`) per emotion. These are the
  **L1 starting points** for this story; iterated on hardware in
  Task 5.
- Ears are **SCS0009 ×4** driven by
  `ros2/src/olaf_drivers/head_ears_driver/ears_servo_driver.py`.
  Servo SDK vendored at `libs/scservo_sdk/` (`scscl` class).

## Acceptance Criteria

1. **L1** — Each of the 12 canonical speech-emotions + sleepy /
   wink / flirty drives a **distinct static ear pose**
   (left_pan, left_tilt, right_pan, right_tilt), Kamal-confirmed
   on hardware.
2. **L2** — Adds **idle drift / micro-movement** (sub-degree
   oscillation, per-ear independent phase).
3. **L3** — Adds a **brief peak gesture** (perk_up / droop /
   flatten / flick / one_cock / swivel_in / swivel_out / twitch)
   at expression onset, then settles back to L1.
4. **Safety** — per-joint mechanical envelope honoured: `left_pan`
   ±50°, `right_pan` ±50° (right_pan ≥65° BINDS on the SCS0009
   — already in `EarsAdapter._LIMITS`), `left_tilt` / `right_tilt`
   −60° to +90°. The player clamps after summing L1+L2+L3.
5. **Verification harness** — `eye_stepper.py` extended to drive
   eyes + neck + ears in lockstep, same `1/2/3` keys + `a` auto.
6. **Tests** — `expression_engine` + `head_ears_driver` +
   `neck_driver` suites stay green; new tests cover the ears
   player, gesture-registry invariants, safety clamp, YAML
   integrity (no `key:value` no-space trap recurrence).
7. **Owner verification** — per-emotion iteration loop with
   Kamal locks each. Status flips to `review` only on full freeze.

## Tasks / Subtasks

- [x] Task 1: **Author per-emotion ears spec** — `ears_motion.yaml`
  with all 15 emotions; per-emotion L1 4-joint pose, L2 drift,
  L3 gesture (token + amp + dur). Bear-inspired
  ([[reference-neck-bear-ref]]).
- [x] Task 2: **Ears gesture registry + motion player** —
  `ears_gestures.py` (8 tokens: perk_up / droop / flatten /
  flick / twitch / swivel_in / swivel_out / one_cock) +
  `ears_motion_player.py` (composes L1 + L2 drift + L3 gesture;
  ~30 Hz background driver thread; per-joint safety clamp from
  YAML; independent phases per joint so the two ears don't mirror
  perfectly).
- [x] Task 3: **Extend `eye_stepper.py`** — drives eyes (I2C) +
  neck (serial) + ears (serial) in lockstep; any subsystem
  degrades gracefully when its deps aren't on the box.
- [x] Task 4: **Tests** — `test_ears_motion_player.py` (34 tests):
  intensity→level mapping, L1 static, L2 4-joint bounded drift,
  L3 fire/decay/clear, safety clamp (incl. extreme amp), fallbacks,
  YAML integrity (every emotion has full L1; every L3 token
  resolves), gesture-registry invariants (zero at u=0/u=1, bounded
  peak), re-fire restarts gesture. Suites green end-to-end:
  expression_engine **201** · head_ears_driver **31** · neck_driver
  **27**.
- [x] Task 5: **Hardware verify** — owner walked the stepper
  through all 15 expressions × 3 levels with eyes + neck + ears
  running together; froze on the bear-inspired seed without a
  per-emotion iteration ("all good .. lets freeze", 2026-05-20).

## Dev Notes

### Source of truth

- **L1 seed:** `expression_map.yaml` `speech_emotion.*.pose.ears`
  values (Story 7.1).
- **Mechanical envelope:** `ears_adapter._LIMITS` — pan ±50°,
  tilt −60..+90°. Right_pan ≥65° binds (project memory).
- **SCS0009 quirks:** `right_pan` direction = -1, center=241
  ([[reference_head_esp32_gotchas]] sibling). EEPROM offset
  doesn't take effect — use software center in config.
- **Inspiration:** same BEARS sheet as 7.1b
  ([[reference_neck_bear_ref]], `.ai/neck_ref_sheet.jpg`) — bears
  have visible ear poses: **perked** (happy / curious / alert),
  **drooped** (sad / sleepy), **flattened** (anger / fear),
  **asymmetric** (curious / wink).

### Constraints / guards

- Do NOT change canonical schema or the `EXPR_*` enum.
- Eyes remain frozen (Story 7.1a). Neck remains frozen (Story
  7.1b).
- Render-loop integration of the 3-level model is deferred (same
  as 7.1b) — the loop's existing layered targets cover the
  production path; the 3-level player drives the verification
  harness.
- Intensity model parallel to 7.1a/7.1b: 1→L1, 2→L2, 3+→L3.

### References

- [Source: `ros2/src/olaf_drivers/head_ears_driver/`]
- [Source: `ros2/src/expression_engine/expression_engine/adapters/ears_adapter.py`]
- Memory: [[project-story-71bc-plan]], [[reference-neck-bear-ref]],
  [[feedback-eye-rendering]], [[project-epic7-interactive-mode]],
  [[reference-head-esp32-gotchas]]

## Dev Agent Record

### Progress

**FROZEN 2026-05-20** on first-pass bear-inspired seed (no
per-emotion hardware iteration was needed before owner approval —
"all good .. lets freeze"). Same 3-level intensity model as
Stories 7.1a (eyes) and 7.1b (neck); same `eye_stepper.py`
harness now drives all three subsystems together.

### Agent Model Used

claude-opus-4-7[1m]

### Debug Log References

(none — hardware lock landed on the seed pass)

### Completion Notes List

- Owner accepted the bear-inspired seed without iteration: ears
  drooped on sad/melancholic/sleepy, flattened on angry/scared/
  frustrated, perked on happy/excited/surprised, asymmetric on
  curious/wink/flirty. The +30% blanket exaggeration applied to
  neck in 7.1b was NOT requested for ears — the L1 amplitudes here
  are already large (sad LT/RT = −25°; angry/scared at −28°/−35°).
- The L2 drift uses INDEPENDENT phases per joint (not mirrored
  pairs), so the two ears appear "alive" independently — a
  deliberate departure from neck's three-axis single-phase drift.
- Render-loop integration of the 3-level model is deferred (same
  as 7.1b); the existing `EarsAdapter` ContinuousAdapter still
  carries the production path. The player is the verification
  harness.
- The mechanical `right_pan ≥65° binds` warning (project memory)
  is honoured by the safety clamp (≤ 50°).

### File List

Modified (committed to repo):
- `docs/implementation-artifacts/7-1c-ear-movement-routines.md` — story spec, Status, Tasks, Dev Agent Record, File List.
- `docs/implementation-artifacts/sprint-status.yaml` — `7-1c` → `review`.

New (committed):
- `ros2/src/expression_engine/config/ears_motion.yaml` — per-emotion 3-level ears spec for all 15.
- `ros2/src/expression_engine/expression_engine/adapters/ears_gestures.py` — 8-token registry.
- `ros2/src/expression_engine/expression_engine/adapters/ears_motion_player.py` — composer + ~30 Hz driver thread + safety clamp.
- `ros2/src/expression_engine/test/test_ears_motion_player.py` — 34 tests.

Modified (committed):
- `modules/head/firmware/tools/eye_stepper.py` — opens BOTH neck + ears players; lockstep drive for eyes + neck + ears; both degrade independently.
