# Story 7.4: Mood → subtle ambient modifier

Status: review

> **RE-SCOPED 2026-05-22 (Kamal):** mood is NO LONGER a head/body
> modifier. The head already has activity + speech_emotion +
> vocalization competing for it, so a mood neck-lean added little and
> actively interfered (the happy lean was reducing the sleep droop).
> **Decision:** mood's emotional display moves to the **HEART (Epic
> 8.1)**; on the head it keeps only the **WS2812 colour tint**
> (`led_overlay`, already wired in 7.3) and, later, **idle-drift
> modulation (Story 6.5)**. `mood.eye` stays (consumed only by nod/shake
> vocalizations, Story 7.2). The `lean_bias` field + the render-loop
> mood→neck channel were REMOVED. The original ACs below (mood as a
> subtle *neck/LED* modifier) are superseded by this de-scope; what
> shipped is the removal + the LED-tint/heart/idle handoff.

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

- [x] Task 1: **Spec audit** — DONE. Found `mood.*` carried
  `lean_bias` + `led_bias` + `led_overlay` (7.3) + `eye` (7.2). The
  AC#4 "no eye field" conflicts with 7.2's nod/shake mood-eye → led to
  the re-scope decision above.
- [x] Task 2: ~~Subtlety invariants~~ → SUPERSEDED by de-scope.
  `lean_bias` removed entirely (mood doesn't touch the body), so the
  ≤8°/≤0.7 bounds are moot. `mood.eye` kept for nod/shake; mood never
  drives the ambient eye (guaranteed in `_compose`, which no longer has
  a mood branch).
- [x] Task 3: **Tests** — DONE. Removed the mood-neck-ease test;
  repurposed the snapshot + namespacing tests to the mood→`led_overlay`
  path (mood drives the LED tint only, never the neck). Full host suite
  green (302).
- [x] Task 4: ~~Hardware verify (walk 8 moods on the head)~~ →
  SUPERSEDED. Mood no longer changes the head body; the LED tint was
  already verified on hardware during the 7.3 walk (baseline mood=happy
  → warm strip). Mood's hardware verification now belongs to the HEART
  (Epic 8.1) and idle drift (Story 6.5).

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

- 2026-05-22: Re-scoped (see banner). Removed mood→neck layer; mood is
  now LED-tint (head) + heart (8.1) + idle (6.5). Host suite green (302).

### Agent Model Used

claude-opus-4-7 (1M context)

### Completion Notes List

- Removed the render-loop mood→neck channel: `_EaseChannel self._m`,
  `_ComposedTarget.mood_neck`, the `_compose` mood branch, the tick
  `self._m.step(...)`, and the `_m.value` term in `neck_out`.
- Removed `lean_bias` from all 8 `mood.*` entries in
  `expression_map.yaml`; kept `led_bias` (heart), `led_overlay` (head
  tint), `eye` (nod/shake). Map-loader still ALLOWS an optional
  `lean_bias` (NFR5 extensibility) — just unused.
- `mood_ease_seconds` config retained (unused by the render loop now;
  may serve the heart/idle later).
- Tests: removed `test_mood_eases_over_seconds_never_steps`; repurposed
  `test_snapshot_per_tick_last_write_wins` and
  `test_topic_namespacing_mood_vs_speech_happy` to the mood→`led_overlay`
  path; updated `test_map_loader` namespacing assertion (lean_bias →
  led_bias). AC#4 conflict resolved by re-scope (mood.eye kept for
  nod/shake; mood never drives the ambient eye).

### File List

- `ros2/src/expression_engine/expression_engine/render_loop.py`
  (modified — removed the mood→neck channel)
- `ros2/src/expression_engine/config/expression_map.yaml` (modified —
  dropped `lean_bias` from the 8 moods; re-scope comment)
- `ros2/src/expression_engine/test/test_render_loop.py` (modified —
  removed mood-ease test; repurposed snapshot + namespacing tests)
- `ros2/src/expression_engine/test/test_map_loader.py` (modified —
  namespacing assertion lean_bias → led_bias)
- `ros2/src/expression_engine/test/e2e_activity_run.py` (modified —
  baseline-mood comment: mood no longer touches the body)
- `docs/implementation-artifacts/7-4-mood-to-subtle-ambient-modifier.md`
  (modified — re-scope banner, tasks, this record)
- `docs/implementation-artifacts/sprint-status.yaml` (modified — 7-4 → review)

### Change Log

- 2026-05-22: Story 7.4 RE-SCOPED — mood is no longer a head/body
  modifier (removed neck-lean layer + lean_bias). Mood → heart (8.1) +
  head LED tint (7.3) + idle drift (6.5). Host suite green (302).
