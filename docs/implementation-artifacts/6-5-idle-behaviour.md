# Story 6.5: Idle behaviour

Status: ready-for-dev

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the avatar,
I want to look alive between events,
so that the body is never statue-still.

> Prerequisite: Stories 6.1–6.4 `done` (render loop + frozen schema/Protocols). LED breath here renders through whatever LED path exists; the *real* WS2812 adapter is Story 6.6 — design so 6.6 drops in without touching the FSM.

## Acceptance Criteria

1. **Given** the idle FSM, **When** the engine runs, **Then** it distinguishes `sleeping` (DORMANT) from `listening`-and-silent (AMBIENT).
2. **Given** `activity=listening` and no `speech_emotion` for >3s, **When** the FSM evaluates, **Then** the body holds a mood-tinted neutral pose with sub-degree micro-movement and slow breath-LED (FR10).
3. **Given** `activity=sleeping`, **When** the FSM evaluates, **Then** the body holds an eyes-closed posture with slow LED breathe (FR10).
4. **Given** either ambient state, **When** observed, **Then** the body is never statue-still and never twitchy **And** any qualifying new event exits the idle state immediately.

## Tasks / Subtasks

- [ ] Task 1: `idle.py` FSM (AC: #1)
  - [ ] States: `ACTIVE`, `AMBIENT`, `DORMANT`, `BOOT` per architecture §7
  - [ ] Transitions: `ACTIVE → AMBIENT` when `activity=listening` and no `speech_emotion` for `return_to_neutral_after_seconds` (default 3.0); `AMBIENT → ACTIVE` on any qualifying event (immediate); `activity=sleeping → DORMANT`; `activity=starting → BOOT → ACTIVE`
- [ ] Task 2: Render-loop integration (AC: #4)
  - [ ] `render_loop` consults `idle.py` each tick to decide whether the composed target is "live" or "ambient" — substitute an ambient target, do not bypass the loop
  - [ ] Exit from AMBIENT/DORMANT on any qualifying event is immediate (next tick)
- [ ] Task 3: AMBIENT behaviour (AC: #2)
  - [ ] Mood-tinted neutral pose + low-amplitude sinusoidal sub-degree neck micro-movement + slow breath-LED at `breath_period_seconds` (default 4.0)
- [ ] Task 4: DORMANT behaviour (AC: #3)
  - [ ] Eyes-closed posture (delegating eye adapter — semantic, fire-on-change) + slow LED breathe
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

### Debug Log References

### Completion Notes List

### File List
