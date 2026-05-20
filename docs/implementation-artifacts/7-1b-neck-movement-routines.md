# Story 7.1b: Per-emotion neck movement routines (3-level intensity)

Status: review

<!-- Owner-directed follow-on to 7.1a (2026-05-20) — "we need to
create similar routines for head movements too". Applies the same
per-expression iteration loop, on the same 3-level intensity model
([[project-story-71bc-plan]]). Ears come next in 7.1c. -->

## Story

As the avatar,
I want each speech-emotion to also drive a distinct **neck pose +
optional micro-drift + optional peak gesture** at intensity levels 1 / 2 / 3,
so that OLAF's *head* communicates the emotion alongside the eyes —
not statue-still on the body while the LCDs do all the work.

## Background

- Story 7.1a froze the eyes — 15 expressions, 3-level intensity
  (L1 base / L2 colour + static mark / L3 anime FX), Kamal-locked
  one-by-one on hardware (2026-05-20). The same iteration loop now
  applies to the **neck**.
- `ros2/src/expression_engine/config/expression_map.yaml` already
  carries a `pose.neck` block for each speech-emotion (pan/tilt/roll
  in degrees). These are the **L1 starting point** for this story;
  they may be re-authored on hardware during the per-emotion loop.
- The neck is **STS3215 ×3** (pan ID11 + left/right linkages ID12/13),
  driven by `ros2/src/olaf_drivers/neck_driver/`. Servo SDK vendored
  at `libs/scservo_sdk/` (`sms_sts` class). Level-head zero
  recalibrated 2026-05-17 — *do not* re-zero this story.

## Acceptance Criteria

1. **Given** each of the 12 canonical speech-emotions + `sleepy`/
   `wink`/`flirty`, **When** the engine drives intensity 1, **Then**
   the neck reaches a **distinct per-emotion static target pose**
   (pan, tilt, roll) Kamal-confirmed on hardware (no two emotions
   visibly identical at L1).
2. **Given** intensity 2 (firmware-strict: 2 → L2; 3,4,5 → L3),
   **When** the engine drives it, **Then** the neck holds the L1
   pose **plus a small idle drift / micro-movement** (sub-degree
   oscillation; spec from [[project-epic7-idle-motion]]).
3. **Given** intensity 3, **When** the engine drives it, **Then** a
   **brief peak gesture** (nod / tilt-flick / look-away …) plays
   once at expression onset, then the neck settles back to the L1
   pose. Gesture finite, ≤ 1 s, reusable token.
4. **Given** safety, **When** any target is computed, **Then** every
   joint stays within **±55° Disney-natural** ([[project-expression-design-dna]])
   and the neck **torque-relaxes when idle** ([[project-epic5-closed]]).
5. **Given** the verification harness, **When** the stepper runs,
   **Then** `modules/head/firmware/tools/eye_stepper.py` is **extended
   to drive the neck in lockstep with the eyes** — same `1/2/3`
   level keys, same `a` auto-sweep, all 15 expressions.
6. **Given** the suites, **When** they run, **Then** `expression_engine`
   + `head_ears_driver` + `neck_driver` test suites stay green;
   the integration adapter has new tests for the L1 pose table and
   the L2/L3 gesture dispatch.
7. **Given** owner verification, **When** each emotion is driven at
   L1 → L2 → L3 in the loop, **Then** Kamal locks each before the
   story closes. Status flips to `review` only after all 15 locks.

## Tasks / Subtasks

- [x] Task 1: **Author per-emotion neck spec** — new file
  `ros2/src/expression_engine/config/neck_motion.yaml` keyed by all
  15 emotions (12 canonical + sleepy/wink/flirty). Per-emotion L1
  pan/tilt/roll, L2 drift {amp_deg, period_s}, L3 gesture {token,
  amp_deg, dur_s}. L1 seeded bear-inspired ([[reference-neck-bear-ref]]).
- [x] Task 2: **Neck controller** — `neck_motion_player.py`
  composes L1 + L2 drift + L3 gesture; background ~30 Hz thread
  ticks the driver. Gesture token registry in `neck_gestures.py`:
  `nod`, `dip`, `shake`, `tilt_left`, `tilt_right`, `look_away`,
  `peek` (each `f(u, amp_deg) → (pan_off, tilt_off, roll_off)`).
- [x] Task 3: **Engine wiring** — existing `NeckAdapter`
  (ContinuousAdapter) preserved for the render-loop path; new
  `NeckMotionPlayer` is the per-emotion 3-level orchestrator used
  by the verification harness. Render-loop integration of the
  3-level model itself is deferred (out of scope; the loop's own
  layered targets already cover the production path).
- [x] Task 4: **Extend `eye_stepper.py`** — runs eyes (I2C) +
  neck (serial via `NeckMotionPlayer`) in lockstep. Degrades to
  eyes-only when neck dependencies are missing (e.g. dev PC).
  Self-bootstraps `sys.path` so it can be invoked by absolute path.
- [x] Task 5: **Tests** — `test_neck_motion_player.py` (34 tests):
  intensity→level mapping, L1 static, L2 drift bounds, L3 gesture
  fire/decay, safety clamp, fallbacks, gesture-registry shape
  invariants (zero at u=0/u=1, bounded peak), YAML integrity (every
  emotion has full L1; every L3 token resolves). Three suites green
  end-to-end: expression_engine **167** · head_ears_driver **31** ·
  neck_driver **27**.
- [x] Task 6: **Hardware verify** — owner ran the stepper on
  hardware; iteration produced two passes (one fixed a YAML parser
  trap at `sad` where `tilt:-16` lost its key) followed by a
  blanket **+30% exaggeration** for final lock. Status →
  `review` 2026-05-20 ("can you further exaggerate the movements
  by 30 % and lock it").

## Dev Notes

### Source of truth

- **L1 starting point:** `ros2/src/expression_engine/config/expression_map.yaml`
  `speech_emotion.*.pose.neck` (the values already there from Story 7.1).
- **Safety bounds:** ±55° Disney-natural per joint
  ([[project-expression-design-dna]]).
- **Torque-relax-when-idle:** [[project-epic5-closed]] design note.
- **Idle-drift conventions:** [[project-epic7-idle-motion]] (3 poses /
  expression, continuous procedural micro-drift, calm = amp 0).
- **Servo IDs / centres:** `config/servo-ids.yaml` — pan=2164,
  left_linkage=3096, right_linkage=370 (recal 2026-05-17, commit 9d7e251).

### Constraints / guards

- **Do NOT** re-calibrate the neck zero this story (committed
  2026-05-17). `neck_calibration.py set-center` strips YAML comments —
  if hardware tuning needs a centre nudge, apply values **surgically**.
- **Do NOT** edit the AR1 smart-peripheral contract or the I2C wire
  to the head; this story is ROS-side + neck-driver-side only.
- The `pose.neck` block in `expression_map.yaml` is owner-frozen as
  L1 input but **may be re-authored on hardware** during this
  story's iteration loop (same as eye geometry in 7.1a).
- Intensity model parallel to 7.1a: firmware-strict 1→L1, 2→L2, 3+→L3.
  Map currently sends 2-5; Kamal re-authors later (same as 7.1a).

### References

- [Source: docs/planning-artifacts/ux-design-specification.md — neck section]
- [Source: ros2/src/olaf_drivers/neck_driver/ — driver]
- [Source: libs/scservo_sdk/ — sms_sts class for STS3215]
- Memory: [[project-story-71bc-plan]], [[project-story71-state]],
  [[project-expression-design-dna]], [[project-epic7-idle-motion]],
  [[project-epic5-closed]], [[feedback-eye-rendering]],
  [[project-epic7-interactive-mode]]

## Dev Agent Record

### Progress

**FROZEN 2026-05-20.** Three iteration passes on hardware:

1. **Bear-inspired seed pass** — L1 pan/tilt/roll authored from the
   BEARS expression sheet ([[reference-neck-bear-ref]]); per-emotion
   table sign-off pre-build; controller + player + stepper-extension
   built; suites green; synced to Pi.
2. **Sad-crash fix + exaggeration pass** — owner reported a crash on
   `sad` (`KeyError: 'tilt'`). Cause: YAML flow tokens like
   `tilt:-16` (no space after colon) parse as a single scalar key
   without value, silently dropping the joint. Every joint colon now
   has a trailing space; regression test added
   (`test_bundled_yaml_every_emotion_has_full_l1`). Same pass
   exaggerated L1/L2/L3 amplitudes toward the mechanical envelope.
3. **+30% blanket exaggeration & lock** — every L1 axis, every L2
   drift amp, every L3 gesture amp scaled ×1.30 (clamped to the
   STS3215 envelope of pan ±80° / tilt ±20° / roll ±15°). Owner
   freeze.

### Agent Model Used

claude-opus-4-7[1m]

### Debug Log References

- Hardware crash 2026-05-20: `KeyError: 'tilt'` at sad → YAML flow
  spacing bug. Fix: `tilt:-16` → `tilt: -16` on every row. Locked
  by new test.

### Completion Notes List

- Three-level model (1→L1, 2→L2, 3+→L3) mirrors Story 7.1a;
  intensity values from the map are firmware-strict clamped in the
  player.
- Render-loop integration of the 3-level model deferred — the
  loop's own layered model already exists for the production path;
  the per-emotion 3-level player drives the verification harness.
  A future story can promote the player's drift/gesture composition
  into the render loop if desired.
- All 15 emotion L1 values lie inside the STS3215 mechanical
  envelope. A few axes (e.g. excited tilt = 20°, curious roll = 15°,
  sleepy tilt = -20°) sit AT the envelope by design (Kamal's
  exaggeration ask) — anything past it is clamped silently by the
  player.

### File List

Modified (committed to repo):
- `docs/implementation-artifacts/7-1b-neck-movement-routines.md` — story spec, Status, Tasks, Dev Agent Record, File List.
- `docs/implementation-artifacts/sprint-status.yaml` — `7-1b` → `review`.

New (committed):
- `ros2/src/expression_engine/config/neck_motion.yaml` — per-emotion 3-level neck spec for all 15.
- `ros2/src/expression_engine/expression_engine/adapters/neck_gestures.py` — 7-token registry (nod, dip, shake, tilt_left, tilt_right, look_away, peek).
- `ros2/src/expression_engine/expression_engine/adapters/neck_motion_player.py` — composer + ~30 Hz driver thread + safety clamp.
- `ros2/src/expression_engine/test/test_neck_motion_player.py` — 34 tests (intensity mapping, L1/L2/L3 behaviour, safety clamp, gesture registry invariants, YAML integrity).

Modified (committed):
- `modules/head/firmware/tools/eye_stepper.py` — drives eyes + neck in lockstep; self-bootstraps sys.path; degrades to eyes-only without neck deps.

New (git-ignored, local-only):
- `.ai/neck_ref_sheet.jpg` — BEARS expression sheet (Pinterest / DeviantArt thagirion).
