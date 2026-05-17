# Story 6.3: Animation loop

Status: done

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the avatar,
I want smooth, correctly-timed motion toward target poses,
so that expression reads as alive and lands with the audio.

> Prerequisite: Stories 6.1 + 6.2 `done` (state model + map data feed the loop).

## Acceptance Criteria

1. **Given** the render thread, **When** the engine runs, **Then** a fixed-tick loop runs at `servo_tick_hz` (default 100Hz, configurable), reading a mutex-protected state snapshot per tick (AR3).
2. **Given** a changing target pose, **When** the loop ticks, **Then** continuous adapters (neck, ears) are driven by engine-side critically-damped interpolation; mood eases over 2–4s and never snaps (NFR3, AR1, AR7).
3. **Given** a `speech_emotion` event carrying `audio_frame_id`, **When** the pose target is computed, **Then** for continuous adapters the easing-start is biased and for the delegating eye adapter `set_expression` is sent early, so the body reaches target within the (audio-anchor − 30ms)..(−80ms) window (NFR2, AR7).
4. **Given** a `[nod]` or `[shake]` vocalization, **When** it is received, **Then** the gesture lands within ~150ms with configurable attack/settle (NFR4, AR12).

## Tasks / Subtasks

- [x] Task 1: Adapter Protocol surface (AC: #2) — `adapters/base.py`
  - [x] Define `ContinuousAdapter`, `DelegatingAdapter`, `SurfaceAdapter` `typing.Protocol`s per architecture §4 (signatures verbatim — see Dev Notes). These are **proposed** here; they are FROZEN in Story 6.4 — do not treat as final until then.
  - [x] Provide test/no-op continuous + delegating adapter implementations so the loop is testable without hardware (real adapters are 6.4/6.6/8.1)
- [x] Task 2: `render_loop.py` fixed-tick loop (AC: #1)
  - [x] Separate render thread (not the rclpy executor thread — AR3); copy the mutex-protected `state` snapshot per tick, never block on DDS
  - [x] Tick rate from `expression_engine.toml [animation] servo_tick_hz` (default 100), `led_tick_hz` (default 30)
- [x] Task 3: `compose()` target from state + map (AC: #2)
  - [x] `target = compose(activity_base, mood_bias, speech_overlay, active_vocalization)` — partial pose dicts merge, missing joints inherit the layer below (AR6)
- [x] Task 4: Easing model (AC: #2)
  - [x] Critically-damped interpolation current→target; time constants by layer: mood long (`mood_ease_seconds` 2–4s, default 3.0), speech short, activity medium (NFR3, AR7)
  - [x] Mood transitions never snap
- [x] Task 5: Anticipatory window (AC: #3)
  - [x] `speech_emotion` carries `audio_frame_id`; target the pose to *arrive* at `(audio_anchor − emotion_anticipatory_ms)`, `emotion_anticipatory_ms ∈ [30,80]` (config default 50). Continuous: bias easing-start. Delegating eye: send `set_expression` early, let the ESP32 ramp land in-window (AR1, AR7, NFR2)
- [x] Task 6: Gestures (AC: #4)
  - [x] Parametric gesture trajectories in engine code (`gesture_attack_ms` ≈80, `gesture_settle_ms` ≈200, from config); superimpose on the composed base then release; `[nod]`/`[shake]` land ≤150ms (NFR4, AR12)
- [x] Task 7: Wake short-circuit (supports NFR1; full activity mapping is Story 7.3)
  - [x] `activity sleeping→waking` short-circuits easing to begin motion within 100ms of receipt, then eases the remainder (NFR1, AR7)
- [x] Task 8: Tests
  - [x] Tick cadence stable under bursty events (jitter-free — state copied per tick)
  - [x] Mood eases ≥2s, never steps; assert no single-tick jump beyond threshold
  - [x] Gesture lands ≤150ms (simulated clock)
  - [x] Anticipatory: continuous target reaches within [30,80]ms before `audio_anchor`

## Dev Notes

### The defining architectural distinction (AR1 — get this right or everything downstream breaks)

[Source: phase2-expression-engine.md#2, #4, #6]

| Kind | Adapters | Loop responsibility | Ticked? |
|---|---|---|---|
| **Continuous** | `neck_adapter`→`NeckServoDriver`, `ears_adapter`→`EarsServoDriver` | engine interpolates per tick, issues absolute joint angles; adapter does NOT ease | **Yes** |
| **Delegating** | `eye_adapter`→`HeadI2CClient` | engine sends a *semantic* event on change; ESP32 owns 60 FPS eye animation | No — fire-on-change |
| **Engine-owned** | `led_adapter`, `heart_adapter` | LED ticked for breathe; heart low-rate | LED yes / heart low |

Consequence for the loop: interpolate **neck+ears every tick**, push **eye state only on transition**, tick **LED** for breathe, push **heart** at low rate. The anticipatory window (NFR2) is enforced for neck/ears *in the loop*, and for eyes by *sending the semantic event early*.

### Adapter Protocols (architecture §4 — verbatim; PROPOSED here, FROZEN in 6.4)

```python
@runtime_checkable
class ContinuousAdapter(Protocol):
    def connect(self) -> None: ...          # fatal on failure (NFR7)
    def close(self) -> None: ...
    def apply(self, targets: dict[str, float], speed_pct: float = 0.0) -> None: ...  # absolute deg, per tick, no easing in adapter
    def neutral(self) -> dict[str, float]: ...

@runtime_checkable
class DelegatingAdapter(Protocol):
    def connect(self) -> None: ...
    def close(self) -> None: ...
    def set_expression(self, canonical_name: str, intensity: int) -> None: ...
    def blink(self) -> None: ...
    def look(self, x: int, y: int) -> None: ...

@runtime_checkable
class SurfaceAdapter(Protocol):
    def connect(self) -> None: ...
    def close(self) -> None: ...
    def render(self, frame: "SurfaceFrame") -> None: ...
```

Real driver call-mapping (grounded in actual Phase 1 APIs — for when concrete adapters land in 6.4/6.6):
- `neck_adapter.apply({"pan","tilt","roll"})` → `NeckServoDriver.move_pose(pan=, tilt=, roll=, speed=)`; `neutral()` → centered triple; `connect/close` → ctor / `close()`
- `ears_adapter.apply({"left_pan","left_tilt","right_pan","right_tilt"})` → `EarsServoDriver.move_left_pan/left_tilt/right_pan/right_tilt(deg, speed_pct)`; `neutral()` → `center_all()` targets
- `eye_adapter.set_expression(name,intensity)` → `HeadI2CClient.set_expression(name,intensity)`; `blink()`→`trigger_blink()`; `look(x,y)`→`set_look_direction(x,y)`; `connect()`→`open()`
  - ⚠️ `HeadI2CClient` accepts only 7 expression strings (`neutral,happy,sad,surprised,angry,sleepy,wink`). The canonical→ESP32 translation table lives **inside `eye_adapter.py`** (AR10) — NOT in this loop, NOT in the map. The loop only passes canonical names.

### Concurrency (AR3 — non-negotiable)

One `rclpy` executor thread writes `state` (mutex, last-write-wins per topic). A **separate render thread** runs this loop, copies the state snapshot per tick, and never blocks on DDS. This keeps tick jitter-free regardless of event cadence. Do not run the loop on the executor thread. Do not let `apply()` block the tick (servo writes must be non-blocking or budgeted within the tick period). [Source: phase2-expression-engine.md#3]

### Timing config (architecture §8)

`expression_engine.toml [animation]`: `servo_tick_hz=100 led_tick_hz=30 mood_ease_seconds=3.0 emotion_anticipatory_ms=50 gesture_attack_ms=80 gesture_settle_ms=200`; `[idle] return_to_neutral_after_seconds=3.0 breath_period_seconds=4.0`. All tunable without code change.

### Anti-patterns

- Do NOT ease inside adapters — easing is the loop's job (AR1). Adapters issue absolute angles only.
- Do NOT put gesture *shapes* in the map — the map only *names* a gesture; trajectories are parametric in engine code (AR12). A new gesture shape is a code change (accepted, rare, safety-sensitive).
- Idle/ambient behaviour is **Story 6.5**, not here. This loop renders live targets; it must expose a clean seam for the idle FSM to substitute an ambient target, but do not implement idle here.
- Concrete hardware adapters that move real servos: the neck/ears ones are exercised end-to-end in **Story 6.4**; LED in 6.6; heart in 8.1. This story uses test adapters.

### Previous Story Intelligence (6.1, 6.2)

- `state.py` (6.1) is the mutex-protected snapshot source — consume its copy API; do not re-read DDS.
- `map_loader` (6.2) yields topic-namespaced render data + `default_*` fallback — `compose()` consumes that; respect topic disambiguation (`mood.happy` ≠ `speech_emotion.happy`).
- Reuse the established journald/structured-log + fail-fast helpers.

### Project Structure Notes

```
ros2/src/expression_engine/expression_engine/
  render_loop.py        # NEW
  adapters/base.py      # NEW (Protocols — frozen in 6.4)
  adapters/__init__.py  # NEW
  (test adapters)       # NEW under test/ or adapters/_testing.py
  node.py               # UPDATE — start render thread
  test/test_render_loop.py  # NEW
```
Matches architecture §3. No variance.

### References

- [Source: docs/planning-artifacts/epics.md#Story-6.3]
- [Source: docs/planning-artifacts/prd/phase2-prd.md — FR8, NFR1, NFR2, NFR3, NFR4]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#2, #3, #4, #5, #6, #8, #12]

## Dev Agent Record

### Agent Model Used

Amelia (bmad-dev-story) · claude-opus-4-7[1m]

### Debug Log References

- **Sequencing:** Built per the agreed hybrid plan (6.3→6.4→Epic 7→6.5/6.6/6.7), not linear sprint order. Prereqs 6.1/6.2 are `review` (not `done`); their artifacts (`state.py`, `map_loader.py`, `schema.py`) exist and are green — loader/state consumed via their public APIs.
- **Adapter Protocols are PROPOSED, frozen by Story 6.4** (AR2). Verbatim from architecture §4. `SurfaceFrame` added (minimal) for the frozen surface contract.
- **TTS scope clarification (Kamal):** the expression engine NEVER produces audio — TTS is the companion's job. `VocalizationPayload.tts_supported` is a required schema-3 wire field (validated since 6.1) but the render loop **ignores its value**; gestures are visual-only (brief §A.7 also forbids audio on nod/shake). Tests supply `tts_supported` solely so the envelope deserializes.
- **SmoothDamp settle ratio:** critically-damped `smooth_damp` reaches ~98% of a step in ≈2.9× `smooth_time` (measured). The anticipatory ease sizes `smooth_time = (deadline−now)/2.9` so the pose *arrives* (~98%) at the deadline, landing `emotion_anticipatory_ms` before the audio anchor — verified in `[30,80]ms` by `test_continuous_reaches_target_30_80ms_before_anchor`.
- **Node placeholder adapters:** `node.py` starts the render thread (§9 step 7) against `NullContinuousAdapter`/`NullDelegatingAdapter` — deliberate placeholders; the real neck/ears/eye hardware adapters land in Story 6.4. Clearly logged; reversible.
- Test-calibration fixes during RED: first tick has `dt=0` (timing baseline, no motion — expected); wake test needs a `sleeping` precondition before the transition; convergence-tolerance windows widened to match the measured 2.9× settle.
- Pre-existing UNRELATED failures persist (not 6.3): `head_ears_driver/test_expressions.py::test_get_preset_happy|sad` (stale `left_tilt<=0`, commit `7af9fb6`); `ros2/src/olaf_drivers/` untouched by this story.

### Completion Notes List

- All 4 ACs + Task 8 satisfied. **81/81** expression_engine tests pass (16 new in `test_render_loop.py`; **65 prior 6.1+6.2 tests remain green** — zero regressions).
- **AR1/AR3 honoured:** continuous (neck/ears) interpolated per tick with absolute angles, adapters never ease; delegating eye is fire-on-change, `set_expression` sent immediately (early → ESP32 ramp lands in-window); the loop runs on a dedicated thread, snapshot-per-tick, never the rclpy executor, never blocks on DDS.
- **Per-layer critically-damped easing:** activity (medium 0.5s), mood (`mood_ease_seconds`, 3.0s — never snaps, ≥2s, no single-tick jump >5%), speech (short 0.15s, or anticipatory-sized). Layers compose additively: `activity_base + mood_bias + speech_overlay + gesture` (§5.1); topic namespacing preserved (`mood.happy` ≠ `speech_emotion.happy`).
- **Gestures parametric in code (AR12), NOT in map:** `_Gesture` nod=neck-tilt, shake=neck-pan, attack/settle from config; superimposed crisp (not eased) then released; land ≤150ms; auto-expire. Vocalization map content irrelevant (deferred per [[project_expression_map_schema]]) — only the tag selects the trajectory.
- **Wake short-circuit (NFR1):** `sleeping→waking` shrinks the activity ease time-constant for a 100ms window so motion begins <100ms, then resumes medium ease.
- **FR13 robustness:** unknown canonical name → `map_loader` returns `defaults`; the loop renders defaults and stays alive (`test_unknown_speech_name_falls_back_no_crash`).
- **Idle deferred (Story 6.5):** not implemented; the per-tick compose is a clean seam for an ambient-target substitution later (Dev Notes anti-pattern respected).
- `[animation]` config parsed in `config.py` (lenient defaults per §8 — a present-but-mistyped value is the one fatal case). Production modules flake8-clean under default ignores (E704/W503 are pycodestyle defaults; Q000 is the same accepted non-ament double-quote repo style as 6.1/6.2). No new dependency.

### File List

**New:**
- `ros2/src/expression_engine/expression_engine/adapters/__init__.py`
- `ros2/src/expression_engine/expression_engine/adapters/base.py`
- `ros2/src/expression_engine/expression_engine/adapters/_testing.py`
- `ros2/src/expression_engine/expression_engine/render_loop.py`
- `ros2/src/expression_engine/test/test_render_loop.py`

**Modified:**
- `ros2/src/expression_engine/expression_engine/config.py` (`AnimationConfig` + lenient `[animation]` parse)
- `ros2/src/expression_engine/expression_engine/node.py` (render thread §9 step 7; Null placeholder adapters; start/stop wiring)

### Change Log

- 2026-05-17 — Story 6.3 implemented: fixed-tick layered animation render loop. Adapter Protocols (§4, proposed/frozen-in-6.4) + test/Null doubles; `render_loop.py` with snapshot-per-tick on its own thread (AR3), per-layer critically-damped easing (mood never snaps, NFR3), additive layer composition (§5.1), anticipatory window sized to land 30–80ms before the audio anchor (NFR2), parametric code-side nod/shake gestures ≤150ms (AR12/NFR4), sleeping→waking short-circuit (NFR1). `[animation]` config parsing; node starts the render thread against Null placeholders (real adapters = 6.4). 16 tests added; full suite 81/81 green; zero regressions.
