# Story 6.5: Idle behaviour

Status: review

> **RE-SCOPED 2026-05-22 (Kamal):** idle is a **drift-to-sleep decay of
> the WHOLE HEAD**, not a two-state ambient. Left alone, the avatar winds
> down through a path of expressions/levels to deep sleep, at an organic
> random pace, and keeps stirring once asleep. The original AMBIENT/
> DORMANT + breath-LED ACs below are superseded by the decay model. (LED
> behaviour follows the 7.3 decision: idle/decay = strip OFF, NOT a
> breathe — supersedes the old "slow LED breathe".)

## Story

As the avatar,
I want to gradually drift off to sleep when nobody is interacting,
so that I feel alive — winding down naturally and stirring in my sleep,
never statue-still.

## Acceptance Criteria (re-scoped)

1. **Decay path** — when idle, the WHOLE HEAD (neck + ears + eye) decays
   along: `current expr (down to L1) → neutral → content → sleepy L1 →
   sleepy L2 → sleepy L3`, the body drooping progressively toward the
   `sleeping` pose; the eye steps the canonical expressions/levels.
2. **Random pace** — each decay step occurs at a random interval in
   `[step_min_seconds, step_max_seconds]` (default 10–30s); never a fixed
   metronome.
3. **Stir loop** — at `sleepy L3` it does NOT freeze: it loops back up
   (neutral → content → sleepy L1→L2→L3) so it's never statue-still.
4. **When it runs** — decay runs in ALL activity states EXCEPT `speaking`
   and `working`(processing). `listening` is gated: decay starts only
   after `idle_after_seconds` (default 30s) with nothing heard.
5. **Reset** — any new `speech_emotion`, `activity`, or `vocalization`
   event exits decay immediately (next tick) → back to ACTIVE; the event
   drives the head.
6. **LEDs off** — the WS2812 strip is dark throughout the decay (idle =
   dark, per Story 7.3); the engine drives `system_status=idle` while
   decaying even if the activity was `listening`.
7. **Mood drift** — subtle sub-degree neck micro-movement throughout,
   amplitude modulated by mood (calmer mood = gentler).
8. **Suites + hardware** — host tests (stage progression, seeded random
   pacing, suppression in speaking/processing, listening threshold,
   reset, LED off, never-statue-still); existing suites green; Kamal
   walks it on the robot.

## Tasks / Subtasks

- [ ] Task 1: `[idle]` config (toml + `config.py`, validated) —
  `idle_after_seconds`, `step_min_seconds`, `step_max_seconds`,
  `drift_amplitude_deg`, `drift_period_seconds` (most params live here).
- [ ] Task 2: `idle.py` decay controller — time-since-event, suppressed
  in speaking/processing, decay-stage sequence + random pacing (injected
  clock + rng), whole-head droop fraction (0→1 toward the `sleeping`
  pose), sleepy-L3 stir loop, reset on qualifying event.
- [ ] Task 3: Render-loop integration — consult `idle.py` at the existing
  seam; when decaying, OVERRIDE the composed target with the decay pose
  (body droop toward sleep + eye stage) + mood-modulated micro-drift;
  force `system_status=idle` (LED off); immediate exit on event.
- [ ] Task 4: (removed — old DORMANT breathe-LED; superseded by 7.3 dark
  strip + the sleepy eye/blink already shipped in 7.3).
- [ ] Task 5: Tests (AC: #1–#4)
  - [ ] `listening` + silence >3s → AMBIENT; micro-movement amplitude is sub-degree and non-zero (never statue-still, never twitchy)
  - [ ] `sleeping` → DORMANT; eyes-closed sent once on entry, LED breathe continuous
  - [ ] Any qualifying event → immediate exit (assert transition on next tick)
  - [ ] Simulated-clock: breath/micro-movement periodic, not jittery

## Dev Notes

### Idle FSM (architecture §7 — authoritative)

```
   [ACTIVE] ── no speech_emotion >3s (activity=listening) ──► [AMBIENT]
        ▲                                                       │
        └────────────── any new qualifying event ───────────────┘
   activity=sleeping  ─► [DORMANT]  eyes-closed + slow LED breathe
   activity=starting  ─► [BOOT]     boot sequence, then ACTIVE
```
- AMBIENT and DORMANT are **never statue-still** (FR10): low-amplitude sinusoidal micro-movement (sub-degree neck) + `breath_period_seconds` LED breathe.
- Transitions out of AMBIENT/DORMANT are **immediate** on any qualifying event. [Source: phase2-expression-engine.md#7]

### "Never statue-still AND never twitchy" — both fail conditions

AC #4 has two failure modes the dev must defend against: (a) zero motion = statue (fails FR10), (b) high-frequency/high-amplitude jitter = twitchy (fails the quality bar). Micro-movement is **sub-degree** and **slow** (period on the order of seconds, tied to breath). Tune amplitude/period via config, not hardcoded. [Source: phase2-prd.md#FR10; phase2-expression-engine.md#7]

### Integration seam (AR3 + Story 6.3)

`idle.py` is **consulted by** `render_loop` (Story 6.3 left a seam for this — use it; do not fork the loop). The FSM decides live-vs-ambient; the loop still owns interpolation, ticking, and adapter calls. Mood-tinted neutral = the `defaults`/mood-bias compose path from 6.2/6.3, not a new pose system. Eyes-closed uses the delegating eye adapter's semantic set (fire-on-change), not per-tick. [Source: phase2-expression-engine.md#2, #3, #7]

### Config (architecture §8)

`[idle] return_to_neutral_after_seconds=3.0  breath_period_seconds=4.0`. Both tunable, no code change.

### Anti-patterns

- Do NOT implement the real WS2812 driver here — that's Story 6.6. Drive LED breathe through the existing LED path/test adapter; keep the FSM hardware-agnostic so 6.6 swaps the adapter only (NFR6).
- Do NOT add a separate timer thread for micro-movement — it must ride the existing render tick (AR3, jitter-free).
- Do NOT treat idle as a mode that bypasses the loop — it substitutes the *target*, the loop still runs.

### Previous Story Intelligence (6.1–6.4)

- 6.3 `render_loop` exposes the idle seam and owns easing/tick; 6.2 supplies `defaults` + mood bias; 6.4 froze the schema/Protocols and proved real adapters. Reuse all — this story adds only the FSM + ambient targets.
- Eye-closed expression must map through `eye_adapter`'s canonical→ESP32 table (only 7 ESP32 strings; `sleepy` is the closest closed-eyes string — confirm against the table established in 6.4).

### Project Structure Notes

```
ros2/src/expression_engine/expression_engine/
  idle.py              # NEW
  render_loop.py       # UPDATE — consult idle FSM at the existing seam
  test/test_idle.py    # NEW
```
Matches architecture §3. No variance.

### References

- [Source: docs/planning-artifacts/epics.md#Story-6.5]
- [Source: docs/planning-artifacts/prd/phase2-prd.md — FR10]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#2, #3, #7, #8]

## Dev Agent Record

### Agent Model Used

claude-opus-4-7 (1M context)

### Completion Notes List

- `[idle]` config in toml + `IdleConfig` in `config.py` (validated
  bounds; step_min ≤ step_max). Most tuning lives here.
- `idle.py` `IdleController`: pure/clock-injected. Decay path = wind the
  current eye down to L1, then the repeating tail (neutral → content →
  sleepy L1→L2→L3). Random per-step pace via its OWN `random.Random`
  (never perturbs the vocalization RNG). Loops the tail at sleepy L3
  (the "stir"). Suppressed in `speaking`/`working`. `notify_event`
  resets. `drift_offset` = sub-degree mood-scaled breath sway.
- `render_loop` integration: consults idle each tick; when decaying the
  whole head eases toward the `sleeping` pose by the stage droop, the eye
  shows the stage, `system_status` is forced `idle` (strip OFF), and the
  micro-drift is added to the neck. Status driving moved into `tick`
  (idle-aware) and made fire-on-change. `notify_event` hooked on
  speech_emotion / activity / vocalization changes in `_handle_events`.
- DORMANT breathe-LED (old AC#3) intentionally NOT implemented — Story
  7.3 made idle/sleep = strip OFF; the sleepy eye + drowsy blink (7.3)
  carry "asleep".
- Tests: `test_idle.py` (14 — threshold, suppression, path, wind-down,
  loop, reset, drift) + 4 render-loop integration tests (engage + LED
  off, speech reset, no-idle while speaking/working). Stable-order suite
  green (317). NOTE: a rare random-order flake (~1/60 runs) is the
  pre-existing real-time threaded test, not idle (idle is clock-injected
  deterministic).
- Hardware harness `test/e2e_idle_run.py` (`just idle-run`) — fast demo
  timings, prints idle transitions, fires a speech_emotion to show reset.

### Hardware verify

- [x] DONE 2026-05-22 — Kamal walked it on the robot ("it works.. cool").
  Tuned on hardware: eyes go drowsy (sleepy half-lid, was a flat line —
  the `sleepy` canonical was untranslated), neck droops, strip off, the
  stir is shallow (content↔sleepy, not a full wake to neutral) and the
  transitions ease gently (`ease_seconds` 2.0). idle_after_seconds → 60
  (drift off only after a minute). Suppressed in speaking/working; resets
  on any new activity/speech/vocalization.

### File List

- `ros2/src/expression_engine/expression_engine/idle.py` (new — decay controller)
- `ros2/src/expression_engine/expression_engine/config.py` (modified — IdleConfig + _parse_idle + bounds)
- `ros2/src/expression_engine/config/expression_engine.toml` (modified — [idle] section)
- `ros2/src/expression_engine/expression_engine/render_loop.py` (modified — idle integration, idle-aware status, micro-drift, _mood_energy)
- `ros2/src/expression_engine/expression_engine/node.py` (modified — pass config.idle to RenderLoop)
- `ros2/src/expression_engine/test/test_idle.py` (new — idle unit tests)
- `ros2/src/expression_engine/test/test_render_loop.py` (modified — TestIdleIntegration)
- `ros2/src/expression_engine/test/e2e_idle_run.py` (new — on-robot idle harness)
- `justfile` (modified — `idle-run` recipe)
- `docs/implementation-artifacts/6-5-idle-behaviour.md` (modified — re-scope, this record)
- `docs/implementation-artifacts/sprint-status.yaml` (modified — 6-5 → in-progress)

### Change Log

- 2026-05-22: Story 6.5 RE-SCOPED + implemented — idle drift-to-sleep
  decay of the whole head (config-tunable, random pace, stir loop, mood
  drift, LED off, resets on event). Host suite green (317). On-robot
  walk pending.
