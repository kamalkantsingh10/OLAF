# Story 7.2: Author / finalize the vocalizations (layered actions)

Status: review

<!-- Created 2026-05-20 after Story 7.1a/b/c froze the per-emotion
look on eyes/neck/ears. 7.2 finalises the 6 `vocalization.*` entries
in expression_map.yaml — what each does on the body (neck + ears +
eye accent). RESHAPED 2026-05-20 (owner direction during dev):
LED dispatch is DROPPED from this story (Story 6.6 owns continuous
LED rendering); every numeric param is RANDOMIZED per fire so the
same vocalization never looks twice the same; nod/shake derive their
eye accent from the current mood (new mood.<mood>.eye block).
Vocalization was deferred from 7.1 ([[project-expression-map-schema]]
§5.2). -->

## Story

As the avatar,
I want each of the 6 `vocalization` events (`laughter`, `sigh`,
`gasp`, `clears_throat`, `nod`, `shake`) to fire a **layered**,
crisp-transient action on the body — a quick neck/ears gesture
plus a brief eye-expression accent — whose parameters are RANDOMLY
SAMPLED per fire (amplitudes, durations, eye intensity, attack/settle
all drawn from per-tag ranges), with total gesture window between
500 ms and 1500 ms, that peaks then settles back to whatever
speech_emotion / mood / activity is already running, without
disturbing the base layer (AR12),
so that "she laughs while saying that" reads visibly AND never
looks twice the same, even though the underlying expression
doesn't change.

## Background

- `ros2/src/expression_engine/config/expression_map.yaml`
  `vocalization.*` is currently a stub — six entries each carrying
  only `visible_only` (laughter/sigh/gasp/clears_throat = false;
  nod/shake = true).
- `render_loop.py` has the layering infrastructure: a `_Gesture`
  class fired from `_handle_events` when a `vocalization` snapshot
  changes, mapped via the hard-coded `_Gesture.SHAPES` table
  (`nod → neck tilt 14°`, `shake → neck pan 16°`). **Only `nod` and
  `shake` are wired**; the audible-only four have no body layer yet.
  Story 7.2 REPLACES `_Gesture.SHAPES` with map-driven dispatch
  (`_VocalizationAction`); the map becomes the single source.
- AR12 (architecture): vocalization is a **transient** layer above
  speech_emotion / mood / activity. It **does not** displace them;
  the loop sums offsets for neck+ears and overrides the delegating
  eye for the duration of the window, then releases. Each
  vocalization completes within 500–1500 ms (per-tag random).
- Story 7.1b authored a neck gesture library
  (`neck_gestures.py` — nod/dip/shake/tilt_left/tilt_right/
  look_away/peek). Story 7.1c authored an ears gesture library
  (`ears_gestures.py` — perk_up/droop/flatten/flick/twitch/
  swivel_in/swivel_out/one_cock). 7.2 composes from these.
- DROPPED FROM SCOPE (owner direction 2026-05-20): LED overlay
  dispatch on vocalization. The `REG_LED_OVERLAY` register stays
  reserved for speech_emotion / mood rendering (Story 6.6).

## Acceptance Criteria

1. **Schema (vocalization)** — `vocalization.*` in
   `expression_map.yaml` extended per entry with `layered_action`
   carrying:
   - `attack_ms: [min, max]` (perceptual onset window, sampled int)
   - `settle_ms: [min, max]` (sampled int; total window
     `attack_ms + settle_ms ∈ [500, 1500]`)
   - `eye: { expression: <str>, intensity: [min, max] }`  *(omit
     `expression` for nod/shake — taken from mood — but
     `intensity` range MUST be present)*
   - `neck_gesture: { token: <str>, amp_deg: [min, max], dur_s: [min, max] }`
     *(optional component)*
   - `ears_gesture: { token: <str>, amp_deg: [min, max], dur_s: [min, max] }`
     *(optional component)*
   Range fields accept a 2-list `[min, max]` (sampled per fire) or
   a single scalar (fixed). Pinned `companion v3.0.0` schema
   compatibility is preserved — `layered_action` is pure
   consumer-side enrichment; publisher untouched.

2. **Schema (mood eye accent)** — `mood.<mood>` in
   `expression_map.yaml` gains an optional `eye: { expression: <str>,
   intensity: [min, max] }` block. All 8 mood entries author it
   (the source for nod/shake's eye accent).

3. **Loop wiring** — `render_loop._Gesture.SHAPES` is REPLACED by
   a `_VocalizationAction` class that reads `layered_action` from
   the map, samples ranges via an injectable `random.Random`,
   composes the neck/ears offset over time, and overrides the eye
   for the gesture window. All 6 vocalizations dispatch end-to-end
   (laughter/sigh/gasp/clears_throat newly wired; nod/shake migrated
   off the hard-coded table). nod/shake eye = current mood's `eye`
   block (or the map `defaults.eye` when no mood is active).

4. **Distinct, varied renders** — each vocalization is visibly
   distinct on hardware (Kamal-confirmed) AND never identical twice
   in a row at the joint level: laughter = brisk multi-cycle nod +
   ears flick + happy eye accent; sigh = slow head dip + ears droop
   + content eye; gasp = sharp peek + ears perk_up + surprised eye;
   clears_throat = small dip + ears twitch + content eye; nod =
   neck nod + mood-derived eye; shake = neck pan + mood-derived eye.

5. **AR12** — vocalization layer is ADDITIVE on neck + ears; the
   underlying speech_emotion / mood / activity neck/ears targets
   are preserved and resume cleanly after the gesture settles (no
   snap-back). The eye fires the vocalization expression on entry
   and re-fires the composed eye on expiry via the existing
   fire-on-change path (no leak past the window).

6. **Suites** — `expression_engine` + `head_ears_driver` +
   `neck_driver` stay green; new tests cover:
   - Map validation of every new field shape (range + scalar forms)
   - Per-tag dispatch (each vocalization fires the right gesture
     tokens + eye)
   - AR12 additivity invariant (composed neck/ears with vs without
     vocalization differ ONLY during the gesture window)
   - Eye fire-on-change pattern: vocalization eye fires, then the
     composed eye re-fires after expiry, exactly once each
   - Determinism via injected `random.Random` (seeded sampler →
     exact sampled values)

7. **Hardware verify** — Kamal-confirmed on real OLAF, one
   vocalization at a time, via an ad-hoc CLI harness
   ([[project-epic7-interactive-mode]]). Confirm: distinct body
   behaviour per tag, the base layer (speech_emotion + activity)
   resumes unchanged after settle, two fires of the same tag
   visibly differ.

## Tasks / Subtasks

- [x] Task 1: **Vocalization spec extension (YAML)** — extend
  `expression_map.yaml` `vocalization.*` with `layered_action` per
  AC#1. Seed ranges (all durations in [500, 1500] ms total window):
  - laughter: attack [60,120] / settle [440,1380]; eye happy
    intensity [1,3]; neck nod amp [8,14]° dur [0.5,1.0]s;
    ears flick amp [10,16]° dur [0.5,0.9]s
  - sigh: attack [120,200] / settle [380,1300]; eye content
    intensity [1,3]; neck dip amp [6,10]° dur [0.7,1.4]s; ears
    droop amp [6,10]° dur [0.7,1.3]s
  - gasp: attack [40,80] / settle [460,1420]; eye surprised
    intensity [3,5]; neck peek amp [10,16]° dur [0.5,1.0]s; ears
    perk_up amp [12,18]° dur [0.5,1.0]s
  - clears_throat: attack [80,150] / settle [420,1350]; eye content
    intensity [1,3]; neck dip amp [3,6]° dur [0.5,1.0]s; ears
    twitch amp [3,6]° dur [0.5,0.9]s
  - nod: attack [60,120] / settle [440,1380]; eye intensity
    [2,3] *(expression comes from mood)*; neck nod amp [12,18]°
    dur [0.5,1.0]s
  - shake: attack [60,120] / settle [440,1380]; eye intensity
    [2,3] *(expression comes from mood)*; neck shake amp [14,20]°
    dur [0.5,1.0]s

- [x] Task 2: **Mood eye-accent (YAML)** — add `eye:
  { expression, intensity: [min,max] }` to all 8 mood entries.
  Suggested seeds (Kamal-tunable):
  calm→content [1,2], happy→happy [2,3], playful→excited [2,3],
  curious→curious [2,3], thoughtful→neutral [1,2], sleepy→sleepy
  [1,2], grumpy→angry [1,2], excited→excited [3,4].

- [x] Task 3: **Map loader validation** — `map_loader.py` validates
  every new field shape (range list-of-2 numerics OR scalar
  numeric; eye.expression string; gesture.token in the relevant
  GESTURES dict). Fail-fast at startup (NFR5/NFR7) consistent with
  the existing posture.

- [x] Task 4: **Render-loop dispatch (replace `_Gesture`)** — add
  `_VocalizationAction` class that reads the map's `layered_action`,
  samples ranges via injected `random.Random` at construction, and
  exposes `neck_offset(now)`, `ears_offset(now)`,
  `eye_target(now, composed_eye)`, `expired(now)`. Extend
  `RenderLoop.__init__` to accept `rng: random.Random | None`
  (default `random.Random()`) and a mood-eye lookup. Wire
  `_handle_events` to dispatch any of the 6 vocalizations through
  `_VocalizationAction`. The eye fire-on-change tracks the
  vocalization eye while active and re-fires the composed eye on
  expiry. Delete `_Gesture.SHAPES` (map is single source).

- [x] Task 5: **Tests** — see AC#6 list. Use a seeded `random.Random`
  so sampled values are exact. Keep existing suites green
  (`test_render_loop.TestGestures` adapts to the new dispatch with
  the seeded sampler).

- [x] Task 6: **Hardware verify** — interactive stepper
  `modules/head/firmware/tools/vocalization_stepper.py` (eye_stepper
  pattern; standalone, no ROS) drives real neck + ears + eye and
  fires one vocalization per ENTER. Prints sampled window / amp /
  dur / eye intensity per fire. Run on the Pi:

      ssh olaf.local
      cd ~/olaf
      ~/.local/bin/poetry run python modules/head/firmware/tools/vocalization_stepper.py

  Controls inside the stepper:
      ENTER     fire current tag             n / b   next / prev tag
      m         cycle baseline mood          s       cycle baseline speech_emotion
      q         quit

  Observe (Kamal-confirms): (a) each of the 6 tags is visibly
  distinct on the body; (b) two consecutive fires of the SAME tag
  visibly differ (random sampling); (c) body returns cleanly to
  the held speech_emotion baseline pose after each settle; (d) for
  nod / shake, cycling `m` changes the eye accent (mood-derived).

## Dev Notes

### Source of truth

- **Schema:** `ros2/src/expression_engine/expression_engine/schema.py`
  `VocalizationPayload` (companion v3.0.0, pinned) — unchanged.
- **Loop hook:** `render_loop.py` `_handle_events` "vocalization"
  branch (line ~356). The old `_Gesture` class + `SHAPES` table is
  REMOVED in this story.
- **Gesture libraries:** `adapters/neck_gestures.py` (7 tokens),
  `adapters/ears_gestures.py` (8 tokens) — both story-frozen.
- **Mood vocabulary:** 8 names from
  `schema.Mood` Literal (calm/happy/playful/curious/thoughtful/
  sleepy/grumpy/excited).

### Constraints / guards

- Vocalization is a **transient layer**, not a state. It must not
  be confused with speech_emotion (which is a state-style field
  with hold).
- The audible-only four (laughter / sigh / gasp / clears_throat)
  currently have `visible_only: false` — they are intended to
  carry audio too. This story is the BODY + EYE layer only;
  audio rendering is out of scope (future story).
- LED overlay on vocalization is OUT OF SCOPE (owner-dropped
  2026-05-20). The `REG_LED_OVERLAY` register stays reserved for
  speech_emotion + mood (Story 6.6).
- Mood intensity factor — Kamal floated multiplying vocalization
  amp/dur by a per-mood factor (Story 7.4 cross-cut). DEFERRED
  to Story 7.4 to keep this story scoped.
- Do NOT regress: eyes (Story 7.1a), neck (Story 7.1b), ears
  (Story 7.1c), speech_emotion authoring (Story 7.1) — none of
  those configs / players change here.
- Determinism: the only `random.Random` used by the loop is the
  injected one. No `random` module-level calls. Tests pass a
  seeded RNG to assert exact sampled values; production wiring
  uses an unseeded default.

### References

- [Source: `ros2/src/expression_engine/config/expression_map.yaml` § `vocalization`, § `mood`]
- [Source: `ros2/src/expression_engine/expression_engine/render_loop.py` `_Gesture` (to be replaced)]
- [Source: `ros2/src/expression_engine/expression_engine/map_loader.py` `_check_entries`]
- [Source: `ros2/src/expression_engine/expression_engine/adapters/neck_gestures.py` `GESTURES`]
- [Source: `ros2/src/expression_engine/expression_engine/adapters/ears_gestures.py` `GESTURES`]
- Memory: [[project-expression-map-schema]],
  [[project-epic6-7-sequencing]], [[project-epic7-interactive-mode]],
  [[reference-head-esp32-gotchas]]

## Dev Agent Record

### Progress

- 2026-05-20 — Story implemented under owner-direction reshape (mid-dev
  conversation). Original LED + fixed-value path was replaced with
  randomized-range + eye-on-vocalization + mood-derived eye for
  nod/shake. Mood factor (Kamal floated) DEFERRED to Story 7.4.
- Code complete + all suites green (272 tests: 214 expression_engine,
  31 head_ears_driver, 27 neck_driver). Awaiting Kamal's stepper run
  for hardware confirmation (Task 6).

### Agent Model Used

Claude Opus 4.7 (1M context).

### Debug Log References

None — implementation was clean-first-try; the only churn was the
upfront reshape conversation (LED dropped, ranges + eye + mood-derived
nod/shake added) BEFORE writing code.

### Completion Notes List

- **Schema reshape (mid-dev):** Original AC#1/3/6 + Task 1/3 mandated
  per-vocalization LED pulse and fixed amp/dur values. Owner directed
  (a) drop LED from vocalization scope (Story 6.6 owns continuous
  LED), (b) randomize every numeric param within 500..1500ms total
  window, (c) add eye expression on each vocalization, (d) for
  nod/shake derive eye expression from current mood via new
  `mood.<mood>.eye` block. ACs and Tasks rewritten to match before
  any code landed.
- **`_VocalizationAction`** in `render_loop.py` replaces the old
  hard-coded `_Gesture.SHAPES` table. Reads
  `vocalization.<tag>.layered_action` from the map, samples ranges at
  construction via an injected `random.Random` (seeded in tests,
  unseeded in production). Exposes `neck_offset`, `ears_offset`,
  `eye_target`, `expired`. Two fires of the same tag now sample
  different sub-1.5s windows + amplitudes.
- **Eye override** during the gesture window uses the existing
  fire-on-change path — on expiry, next tick's composed eye differs
  from `_last_eye` and re-fires the speech_emotion eye automatically
  (no extra plumbing).
- **Mood eye accent** lives at `mood.<mood>.eye:
  {expression, intensity:[lo,hi]}`. All 8 mood entries authored.
  nod/shake `layered_action.eye` omits `expression` (validator rejects
  if it IS authored, to prevent silent mood-derivation bypass).
- **Map loader validation:** new helpers `_check_range_numeric`,
  `_check_vocalization_eye`, `_check_gesture_block`,
  `_check_layered_action`. Gesture tokens validated against the
  GESTURES dicts in `neck_gestures.py` / `ears_gestures.py`. Unknown
  fields (e.g. a stray `led_pulse` from a copy-paste) are fatal at
  startup.
- **Hardware harness:** `modules/head/firmware/tools/vocalization_stepper.py`
  mirrors `eye_stepper.py` (interactive, no ROS, no engine
  subscribers — directly instantiates `_VocalizationAction` so the
  on-robot behaviour is exactly what the engine will render). Fires
  one tag per ENTER; cycles speech/mood baselines via `s` / `m` keys.

### File List

- `docs/implementation-artifacts/7-2-author-finalize-the-vocalizations-layered-actions.md` (modified — reshape + dev notes)
- `docs/implementation-artifacts/sprint-status.yaml` (modified — 7-2 → in-progress)
- `ros2/src/expression_engine/config/expression_map.yaml` (modified — mood.eye on all 8 moods; vocalization.layered_action on all 6 tags)
- `ros2/src/expression_engine/expression_engine/map_loader.py` (modified — layered_action + mood.eye validation)
- `ros2/src/expression_engine/expression_engine/render_loop.py` (modified — `_VocalizationAction` replaces `_Gesture`; RNG injection; eye override; ears gesture composition)
- `ros2/src/expression_engine/test/test_render_loop.py` (modified — `TestVocalizationDispatch` + `TestVocalizationEye` + `TestVocalizationAdditivity` + `TestDeterministicSampling`; old `TestGestures` removed)
- `ros2/src/expression_engine/test/test_map_loader.py` (modified — `TestStory72LayeredActionValidation`)
- `ros2/src/expression_engine/test/test_review_hardening.py` (modified — P6 `TestShakeSettles` retargeted to `neck_gestures.shake` + `_VocalizationAction`)
- `ros2/src/expression_engine/test/e2e_vocalization_run.py` (new — batch end-to-end harness driving the full engine + mock publisher)
- `modules/head/firmware/tools/vocalization_stepper.py` (new — eye_stepper-style interactive review CLI; Kamal-driven hardware verify)

### Change Log

- 2026-05-20 — Story implemented (LED dropped, ranges + eye + mood-eye, deterministic RNG); all suites green; hardware verify pending Kamal's stepper run.
- 2026-05-20 — Hardware review iter #1: laughter neck amp `[14,20]` + `cycles:[2,4]`; sigh eye → `[content, frustrated]` list; gasp eye → `sympathetic`; clears_throat dropped ears + neck → small `shake`; nod gained `cycles:[1,3]`. Schema: `eye.expression` accepts list, gesture blocks accept extra kwargs (e.g. `cycles`). `neck_gestures.nod` extended with `cycles=1` default (back-compat). 218 tests green.
- 2026-05-20 — Hardware review iter #2: sigh eye reverted to single `content`; eye block made OPTIONAL in `layered_action`; clears_throat / nod / shake DROPPED eye blocks entirely (composed speech_emotion eye shows through). Validator updated. 219 tests green.
- 2026-05-20 — Hardware review iter #3: stepper eye-write switched from `HeadI2CClient.set_expression` (TYPE→INTENSITY, no verify) to `eye_stepper.py`'s INTENSITY→TYPE-then-verify-read-back protocol (up to 5 retries × 0.2s) — earlier "sigh stuck on happy" was the I2C write-swap quirk masking the lost write. Gasp eye → `sad` intensity `1` (was `sympathetic` range).
- 2026-05-20 — Hardware review iter #4: sigh eye → `neutral` (oval-steady). The `content` render is firmware-tagged "crescent-up shallow" — visually indistinguishable from `happy`'s "crescent-up". `neutral` is the clean baseline contrast for a sigh beat. AC#7 satisfied — Kamal-confirmed across all 6 vocalizations on real OLAF.
