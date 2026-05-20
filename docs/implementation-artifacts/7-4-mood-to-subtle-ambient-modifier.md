# Story 7.4: Mood → subtle ambient modifier

Status: ready-for-dev

<!-- Created 2026-05-20 after 7.1a/b/c froze the per-emotion look.
Validates that the 8 `mood.*` entries in expression_map.yaml act as
a SUBTLE long-time-constant bias (low-impact LED tint + small neck
lean), never as an overt overlay that competes with speech_emotion.
Per 7.1a AC#7 the eye is ALWAYS muted-cyan / kawaii — mood is NOT an
eye recolour. -->

## Story

As the avatar,
I want each `mood` (`calm / happy / playful / curious / thoughtful
/ sleepy / grumpy / excited`) to act as a **subtle, slow ambient
modifier** on top of whatever speech_emotion / activity is running
— a small neck lean bias + a low-intensity LED tint, **never an
eye recolour and never a body-dominating displacement** — so that
the long-arc mood is felt without competing with the moment-to-
moment expression.

## Background

- `expression_map.yaml` `mood.*` already carries `lean_bias` (a
  neck-tilt offset, degrees) and `led_bias` (`color, intensity`)
  for all 8 moods.
- The render-loop's `_compose` resolves mood and adds it to the
  `mood_neck` channel (eased on a **long time-constant**, see
  `_EaseChannel` `smooth_time` for mood) — already in place from
  Stories 6.1–6.3.
- 7.1a AC#7 froze the rule "mood stays a low-impact LED tint
  (NOT eye recolour)". This story re-confirms + tunes the
  values on hardware.

## Acceptance Criteria

1. **Subtlety** — each mood's `lean_bias` ≤ 8° (neck tilt) and
   `led_bias.intensity` ≤ 0.7 — verifiable in the YAML and at
   runtime; the loop must NOT route mood to eyes (already true
   by construction in 7.1a's renderer).
2. **Distinctness** — the 8 moods read distinctly in a hardware
   walk (Kamal-confirmed) when applied **on top of `neutral`
   speech_emotion + `listening` activity** (so the only visible
   variable is mood).
3. **Slow ease** — the mood layer eases with the longest time-
   constant of the three layers (mood > activity > speech in
   `render_loop._EaseChannel`); spec is preserved (no regression
   to 6.3's tuning).
4. **No eye recolour** — `mood.*` payloads MUST NOT carry any
   `eye.*` field; if any does, that's a violation. Test guards.
5. **Suites** — `expression_engine` + `head_ears_driver` +
   `neck_driver` stay green; new tests cover the subtlety bounds,
   slow-ease invariant (mood smooth-time > activity > speech),
   and the no-eye-recolour rule.
6. **Hardware verify** — Kamal walks the 8 moods; locks values
   (likely small per-mood lean_bias / intensity tweaks).

## Tasks / Subtasks

- [ ] Task 1: **Spec audit** — confirm every `mood.*` has only
  `lean_bias` + `led_bias`. Add a per-mood comment cross-
  referencing intent (calm = cool blue, happy = warm gold, …).
- [ ] Task 2: **Subtlety invariants in code** — add a
  module-level constants/asserts (or a load-time validation) for
  `lean_bias_max=8.0`, `led_intensity_max=0.7`, `no_eye_field`.
- [ ] Task 3: **Tests** — invariant tests (bounds, slow ease >
  activity/speech, no-eye-field). Existing suites stay green.
- [ ] Task 4: **Hardware verify** — small helper / one-liner to
  set `mood` from the Pi while `neutral` + `listening` is
  running; Kamal walks all 8 and locks values.

## Dev Notes

### Source of truth

- **Spec:** `expression_map.yaml` `mood` block — 8 entries.
- **Loop consumer:** `render_loop._compose` "mood" branch (line
  ~268); routes `lean_bias` → `mood_neck.tilt`, `led_bias` →
  the LED channel.
- **Subtlety guardrail (existing):** 7.1a AC#7 ("mood stays a
  low-impact LED tint, NOT eye recolour"); the kawaii renderer
  has no per-emotion-tint hook for mood (only the L2/L3 levels
  of speech-emotions).
- **Ease constants:** `_EaseChannel` smooth-times — mood (long,
  ~2-3 s) > activity (~1 s) > speech (~0.3 s); set in 6.3.

### Constraints / guards

- Do NOT route mood to eyes. Do NOT introduce mood-driven eye
  recolour (would re-open the cyan-silhouette debate frozen by
  7.1a). Mood is body + LED only.
- The LED bias colours are intentionally pastel (#a0c0ff,
  #ffd060, …) — they MUST stay at low intensity (0.2-0.6
  current values) to avoid overpowering the speech-emotion
  reading.
- Mood values are LONG-time-constant. Anything that needs to
  pop fast belongs in speech_emotion or vocalization.

### References

- [Source: `ros2/src/expression_engine/config/expression_map.yaml` § `mood`]
- [Source: `ros2/src/expression_engine/expression_engine/render_loop.py` `_compose`]
- [Story 7.1a AC#7 — `7-1a-eye-visual-language-rewrite.md`]
- Memory: [[project-epic6-7-sequencing]],
  [[project-expression-design-dna]],
  [[feedback-expression-tuning]]

## Dev Agent Record

### Progress

(none yet)

### Agent Model Used

### Debug Log References

### Completion Notes List

### File List
