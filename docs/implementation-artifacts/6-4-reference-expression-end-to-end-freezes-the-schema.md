# Story 6.4: Reference expression end-to-end (freezes the schema)

Status: review

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the maintainer,
I want one hand-authored expression rendered end-to-end,
so that the map schema and adapter Protocols are proven on real hardware and frozen for Epic 7.

> Prerequisite: Stories 6.1, 6.2, 6.3 `done`. **This story is a freeze point** — its completion locks the `expression_map.yaml` schema and the `adapters/base.py` Protocols. Epic 7 authors against them with no schema/Protocol change. Treat post-freeze schema changes as a documented amendment only. [Source: phase2-expression-engine.md#5.2, end-note; epics.md#Story-6.4]

## Acceptance Criteria

1. **Given** one reference `speech_emotion` (e.g. `happy`) authored in `expression_map.yaml`, **When** the FR14 mock publisher emits it, **Then** neck + ears + eyes visibly render it within the anticipatory window (FR5, FR8, NFR2).
2. **Given** the proven end-to-end render, **When** Story 6.4 completes, **Then** the `expression_map.yaml` schema (pose/LED/eye/heart key structure, layering order) is documented and declared frozen (AR5) **And** the adapter Protocols in `adapters/base.py` are declared frozen (AR2) **And** Epic 7 authors against them with no schema/Protocol change.

## Tasks / Subtasks

- [x] Task 1: FR14 mock companion publisher
  - [x] `ros2/src/expression_engine/test/mock_publisher.py` — publishes schema-3 `EventEnvelope` JSON on the four topics; scriptable sequences; the dev/CI substitute for the live pipeline (FR14, architecture §10)
  - [x] Includes a sequence that emits `speech_emotion=happy` with a valid `audio_frame_id`
- [x] Task 2: Concrete continuous adapters (real hardware)
  - [x] `adapters/neck_adapter.py` wrapping `NeckServoDriver`; `apply({"pan","tilt","roll"})` → `move_pose(...)`; `neutral()` → centered; `connect/close` → ctor/`close()`
  - [x] `adapters/ears_adapter.py` wrapping `EarsServoDriver`; `apply({"left_pan","left_tilt","right_pan","right_tilt"})` → `move_left_pan/left_tilt/right_pan/right_tilt(deg, speed_pct)`; `neutral()` → `center_all()` targets
- [x] Task 3: Concrete delegating eye adapter
  - [x] `adapters/eye_adapter.py` wrapping `HeadI2CClient`; `set_expression(canonical,intensity)`/`blink()`/`look(x,y)`/`connect()→open()` (+ ESP32 wake on connect — boots-asleep gotcha)
  - [x] **Canonical→ESP32 expression-string translation table lives inside this adapter** (AR10). `happy`→`"happy"`; documented; 7 ESP32 strings; full 12-emotion + activity-eye-state table with safe `neutral` default
- [x] Task 4: Author the reference expression
  - [x] Added complete `speech_emotion: happy` to `expression_map.yaml` (neck+ears pose + eye), ears seeded verbatim from `EMOTION_HAPPY` preset (no invented angles)
- [x] Task 5: End-to-end hardware run (AC: #1)
  - [x] Mock publisher → engine → real neck+ears+eyes; **Kamal-confirmed on OLAF**: neck + ears + eyes all rendered `happy`, returned to neutral. NFR2 timing sub-clause: anticipatory *mechanism* proven in 6.3 unit tests [30,80]ms; no live audio clock in the FR14 mock by design — real audio-anchor timing is a later integration story
  - [x] Ran on the robot via `source ROS + PYTHONPATH … poetry run`; engine evidence captured (ears = `EMOTION_HAPPY` exactly; neck = layered sum 16°; eye `happy`→ESP32 `happy`)
- [x] Task 6: FREEZE + document (AC: #2)
  - [x] §5.2 schema FROZEN (as amended) in `phase2-expression-engine.md`, date 2026-05-17 + hardware-proven commit `0f43dfa`
  - [x] `adapters/base.py` Protocols + `SurfaceFrame` marked FROZEN (§4 banner + module docstring + end-note)
  - [x] Stated explicitly: Epic 7 authors content only — no schema/Protocol edits; toml timing values NOT frozen (tunable, NFR3)

## Dev Notes

### Why this is the most important story in Epic 6

Epic 7 authors the entire emotional vocabulary against this schema. If the schema isn't proven on real hardware first, Epic 7 builds dozens of entries against an unvalidated structure. This story de-risks Epic 7 by rendering exactly one expression all the way to servos+eyes, then freezing. [Source: phase2-expression-engine.md end-note; epics.md#Story-6.4]

### Real driver contracts (verbatim — adapters wrap these exactly)

**NeckServoDriver** (`neck_driver.neck_servo_driver`): `move_pose(pan,tilt,roll,speed=None)->bool`, `center_all(speed=None)`, `close()`. pan + = looks left; tilt + = chin up; roll + = tilts right.
**EarsServoDriver** (`head_ears_driver.ears_servo_driver`): `move_left_pan/left_tilt/right_pan/right_tilt(degrees, speed_pct=0.0)->bool`, `center_all()`, `close()`. `speed_pct` ≈ -0.3..+0.3 fractional on base_speed. ⚠️ `right_pan` binds ≥65° — keep ≤50° (project memory).
**HeadI2CClient** (`head_ears_driver.head_i2c_client`): `open()`, `set_expression(expr,intensity=3)->bool` (7 strings only), `trigger_blink()`, `set_look_direction(x,y)` (x,y −100..100), `close()`. I2C bus 1, addr 0x08.

`head_ears_driver/expressions.py` provides `PRESETS: dict[int,dict[str,float]]` + `get_preset_by_name(name, intensity=1.0)` over 10 emotions incl. `EMOTION_HAPPY` — **use this as the seed for the reference `happy` pose** (do not invent angles). [Source: Explore report; phase2-prd.md Technical-Assumptions — expressions.py is the seed]

### Adapter Protocols being frozen (architecture §4)

`ContinuousAdapter` (`connect/close/apply(targets,speed_pct)/neutral`), `DelegatingAdapter` (`connect/close/set_expression/blink/look`), `SurfaceAdapter` (`connect/close/render`). Defined (proposed) in Story 6.3; this story instantiates concrete ones, proves them, and FREEZES the Protocol signatures. NFR6 (one-adapter hardware swap) holds by construction. [Source: phase2-expression-engine.md#4, #12.1]

### In-process, no ROS topics for control (FR9)

Adapters import driver classes directly and call them in-process. There is NO intermediate ROS topic between engine and drivers. The mock publisher is the ONLY ROS traffic, and it stands in for the companion on the four canonical topics. [Source: phase2-prd.md#FR9; phase2-expression-engine.md#1]

### Hardware-run discipline (same as Story 5.3)

`ssh olaf.local`; `cd ~/olaf`; `PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver:ros2/src/olaf_drivers/neck_driver:libs ~/.local/bin/poetry run …`. Vendored SDK at `libs/scservo_sdk/` (don't pip-install feetech). Stay inside `config/servo-ids.yaml` angle limits. `close()` adapters in `finally`. Story 5.3 must already have proven these drivers move hardware — if 5.3 evidence shows a driver issue, fix that gate first.

### Anti-patterns

- Do NOT put the ESP32 expression vocabulary in `expression_map.yaml` — it stays canonical; the hardware-specific table is hidden in `eye_adapter.py` (AR10). Swapping the eye display = rewrite that one table only.
- Do NOT broaden scope to author more than one reference expression — full vocabulary is Epic 7. One expression, end-to-end, then freeze.
- Do NOT change the `schema.py` envelope or Protocols casually — after this story they are frozen; pre-freeze changes must be deliberate and documented.

### Previous Story Intelligence (6.1–6.3)

- 6.1 `schema.py`/`subscribers.py`; 6.2 `map_loader` (topic-namespaced + `default_*`); 6.3 `render_loop` + `compose()` + easing + anticipatory + Protocol definitions and **test** adapters. This story swaps test adapters for **real** ones and proves the chain end-to-end.
- Reuse 6.3's `compose()` and anticipatory logic unchanged — this story validates them on hardware, it does not reimplement them.

### Project Structure Notes

```
ros2/src/expression_engine/
  expression_engine/adapters/{neck,ears,eye}_adapter.py  # NEW
  config/expression_map.yaml                              # UPDATE — add reference `happy`
  test/mock_publisher.py                                  # NEW (FR14)
docs/planning-artifacts/architecture/phase2-expression-engine.md  # UPDATE — FROZEN markers
```
Matches architecture §3. No variance.

### References

- [Source: docs/planning-artifacts/epics.md#Story-6.4]
- [Source: docs/planning-artifacts/prd/phase2-prd.md — FR5, FR8, FR9, FR14, NFR2, NFR6; Technical Assumptions]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#1, #4, #5, #6, #10, #12.1, end-note]
- Drivers: `ros2/src/olaf_drivers/{neck_driver,head_ears_driver}/...`; seed `head_ears_driver/expressions.py`

## Dev Agent Record

### Agent Model Used

Amelia (bmad-dev-story) · claude-opus-4-7[1m] — with Winston (architect) for the pre-freeze §5.2/§4 reconciliation.

### Debug Log References

- **Hardware run is the source of truth.** Two real defects were found *by* the on-robot run (this is exactly why 6.4 exists):
  1. `mock_publisher.run_sequence()` called `rclpy.init()` from its publisher thread while the harness already owned the context → `Context.init() must only be called once` → publisher died, robot stayed neutral. Fix: `manage_rclpy` flag (commit `9fd49d5`).
  2. Eyes stayed **closed** — the Head ESP32 boots asleep and ignores `set_expression` until `set_system_status("woke_up")` (documented Pi/ESP32 gotcha). `EyeAdapter.connect()` only did `open()`. Fix: wake on connect (commit `0f43dfa`).
- Pi enablement: ROS Jazzy installed but not sourced; Pi poetry venv has `pydantic` but not `rclpy`. Solved with **zero Pi changes**: `source /opt/ros/jazzy/setup.bash` then `PYTHONPATH=…:$PYTHONPATH` (prepend — clobbering it drops the ROS overlay; same bug seen locally). Pi-local `config/servo-ids.yaml` was byte-identical to committed `9d7e251` (neck recalibration) so the `git stash`→`pull` lost no calibration (stash retained as safety net).
- Code reached the Pi via commit→push→pull (Kamal-authorized): branch `phase2/expression-engine-rescope` @ `0f43dfa`.
- Reference run engine evidence (commit `0f43dfa`): ears `{left_pan:20, left_tilt:18, right_pan:20, right_tilt:18}` = `EMOTION_HAPPY` preset exactly; neck `tilt≈15.97°` = layered sum (activity `listening` 5 + mood `happy` lean 5 + speech `happy` 6 = 16, eased); eye `happy`→ESP32 `"happy"` via AR10 table; 1400 render ticks; safe neutral + close. **Kamal visually confirmed all three surfaces rendered happy.**
- AC#1 timing nuance (honest scope): the FR14 mock carries no live Pipecat audio clock, so no `audio_anchor_resolver` was injected (engine eased immediately). The anticipatory-window *mechanism* was already proven within [30,80]ms in Story 6.3 unit tests against a simulated anchor; real audio-anchor timing on hardware is a later integration story (no live pipeline by design). The freeze's purpose — proving the schema + Protocols render correctly end-to-end on hardware — is met.
- Tuning feedback noted (not a 6.4 change): Kamal finds the default easing slow; dominated by the deliberate 3 s mood ease (NFR3). Timing lives in `expression_engine.toml` — tunable, NOT part of the freeze. Captured for Epic 7 / Story 7.4.
- Pre-existing UNRELATED failures persist (not 6.4): `head_ears_driver/test_expressions.py::test_get_preset_happy|sad`.

### Completion Notes List

- **All 6 tasks + both ACs satisfied.** AC#1: reference `happy` rendered end-to-end on real neck + ears + eyes (Kamal-confirmed). AC#2: `expression_map.yaml` schema (as amended) **and** `adapters/base.py` Protocols **declared FROZEN** (2026-05-17, hardware-proven @ `0f43dfa`) with explicit "Epic 7 = content only" in `phase2-expression-engine.md` §4, §5.2, end-note + `base.py` docstring.
- **101 expression_engine tests pass** (7 mock-publisher + 13 real-adapter unit tests added; 6.1–6.3's 81 still green — zero regressions). Real adapters unit-tested hardware-free via injected fake drivers; the actual hardware proof is the e2e run.
- FR9 honoured: adapters call driver classes in-process; the only ROS traffic is the FR14 mock on the four canonical topics. AR10 honoured: the canonical→ESP32 vocabulary table lives solely in `eye_adapter.py`. AR1: continuous adapters issue absolute angles, never ease. NFR7: `node.connect_adapters()` is fatal-on-failure in the §9 sequence; `close_adapters()` + neutral in `finally`.
- `node.py` refactored for **injectable adapters** (default `Null*` = hardware-safe; harness injects real) + §9 adapter connect/close — keeps a plain `main()` safe and made the e2e harness clean.
- Production modules flake8-clean (E/F/W). No new dependency. The Story 6.4 software was committed before the freeze; the freeze docs land in the final commit.

### File List

**New:**
- `ros2/src/expression_engine/test/mock_publisher.py`
- `ros2/src/expression_engine/test/e2e_reference_run.py`
- `ros2/src/expression_engine/test/test_mock_publisher.py`
- `ros2/src/expression_engine/test/test_real_adapters.py`
- `ros2/src/expression_engine/expression_engine/adapters/neck_adapter.py`
- `ros2/src/expression_engine/expression_engine/adapters/ears_adapter.py`
- `ros2/src/expression_engine/expression_engine/adapters/eye_adapter.py`

**Modified:**
- `ros2/src/expression_engine/config/expression_map.yaml` (reference `speech_emotion: happy` reseeded from `EMOTION_HAPPY`)
- `ros2/src/expression_engine/expression_engine/adapters/base.py` (FROZEN stamp)
- `ros2/src/expression_engine/expression_engine/node.py` (injectable adapters; §9 connect/close)
- `docs/planning-artifacts/architecture/phase2-expression-engine.md` (§4 + §5.2 + end-note FROZEN stamps; Winston's §5.2-Amendment / SurfaceFrame from the prior reconciliation)
- `docs/implementation-artifacts/sprint-status.yaml`

### Change Log

- 2026-05-17 — Story 6.4: reference `happy` expression proven end-to-end on real hardware (neck+ears+eyes); `expression_map.yaml` schema (as amended) and `adapters/base.py` Protocols **FROZEN** @ commit `0f43dfa`. Added FR14 mock publisher, real neck/ears/eye adapters (+AR10 table, +ESP32 wake), e2e harness; node refactored for injectable adapters + §9 connect/close. Two defects found & fixed via the hardware run (rclpy double-init; ESP32 boots-asleep). 101 tests green. Epic 7 may now author content against the frozen schema.

## Review Findings

_Adversarial code review 2026-05-17 (Blind Hunter + Edge Case Hunter + Acceptance Auditor, all Opus) of commits `dce46a7^..b62fbec`, Stories 6.1–6.4. Core happy-path is sound (101 tests + hardware proof); findings are robustness/safety hardening + one scope decision._

### Decision-needed (resolved)

- [x] [Review][Defer→6.5] **Speech-overlay / mood TTL decay not implemented** — §3/§5.1 specify the overlay "decays to base after 3s silence"; `state.py` last-write-wins holds the last emotion's posture through silence. **Resolved 2026-05-17: deferred to Story 6.5 (idle).** Reason: this is the idle/return-to-neutral concern — `[idle] return_to_neutral_after_seconds=3.0` is literally Story 6.5's config, and Story 6.3 deliberately left an ambient-target seam for the idle FSM to substitute; architecturally coherent there, not a 6.3 gap. [render_loop.py `_compose`; state.py] (src: blind+auditor+edge)

### Patch

- [x] [Review][Patch] **connect_adapters() partial failure leaks open serial/I2C handles** — neck connects (serial port open in ctor) then ears.connect() raises → main() except branch does destroy_node()+exit(1) but never close_adapters() → leaked port, systemd restart loop. The expected NFR7 fatal path is the one that leaks. [node.py:~119,~177-189] (blind+edge, High)
- [x] [Review][Patch] **map_loader validates key presence only, not value types/shapes** — non-numeric pose/`eye.intensity`, non-dict nested `pose`/`eye`, non-dict mood/speech/working entries, duplicate YAML keys (silent last-wins) all pass startup, then crash every 100Hz tick. Critical before Epic 7 hand-authors this file. [map_loader.py:~135-191] (edge, High)
- [x] [Review][Patch] **render tick: persistent exception → unbounded 100Hz log flood, zombie engine** — `_run` catches per-tick but a deterministic failure repeats forever with no circuit-breaker; journald rate-limits away the diagnostic. Add consecutive-failure escalation→fatal (NFR7). [render_loop.py:~411-419] (blind+edge, Med)
- [x] [Review][Patch] **Neck adapter has no range clamp (ears does)** — composed activity+mood+speech + stacked gestures (nod±14/shake±16) can command unsafe neck angles; ears clamp, neck forwards raw. Add defensive clamp + warn. [neck_adapter.py:~54-63] (edge, Med/High safety)
- [x] [Review][Patch] **Anticipatory: past/stale audio anchor → 1ms smooth_time → snap** — `(deadline-now)` negative → clamp 1e-3 → neck/ears snap (NFR3 "never snap" violated), no missed-window log. Floor to `_SPEECH_EASE_S` + WARN. [render_loop.py:~297-305] (blind+edge, Med)
- [x] [Review][Patch] **Gesture `shake` does not settle to zero → discontinuous neck-pan release** — `amp *= sin(e/attack_s·π)` keeps oscillating through the settle window; terminal offset ≠ 0, vanishes on expiry → visible neck snap. [render_loop.py:~618-632] (blind, Med)
- [x] [Review][Patch] **assert_schema_version: missing `schema_version` key accepted as v3** — unversioned/legacy publisher silently treated as schema-3, defeating the FR4 fail-fast intent for that contract-breach. Require field present. [schema.py:~251] (blind+edge, Med)
- [x] [Review][Patch] **eye_adapter: failed/false ESP32 wake is non-fatal** — `set_system_status` returning False or absent → connect "succeeds", eyes stay closed, only an INFO line; NFR7 says connect failure is fatal. Escalate failed wake. [eye_adapter.py:~77-81] (edge, Med)
- [x] [Review][Patch] **§9 startup order inverted** — subscriptions created in `ExpressionEngineNode.__init__` before `connect_adapters()`; frozen-doc §9 orders adapter.connect (step 5) before DDS subscribe (step 6). Reorder. [node.py:~119,~330,~410] (auditor, Med)
- [x] [Review][Patch] **_changed identity check → spurious eye re-fire + wake re-arm** — latched re-delivery makes a new EventEnvelope instance; `is not` treats it as change. Compare by value (frozen models support `==`). [render_loop.py:~738-743] (blind, Low)
- [x] [Review][Patch] **config NFR7 robustness** — `domain_id` accepts negative/>232; `servo_tick_hz` tiny→engine dead / huge→busy-spin (only `<=0` rejected); mistyped `[section]` silently all-defaults. Add bounds + warn. [config.py:~91-150] (edge, Low/Med)
- [x] [Review][Patch] **schema_version `"3"` string → fatal exit-loop; `3.0` float silently OK** — inconsistent; a stringly-typed version crashes the engine. Normalize numeric compare / treat type-mismatch as malformed not version-fatal. [schema.py:~252] (edge, Low)
- [x] [Review][Patch] **`working` submode path bypasses resolve() → no `expression.unmapped_activity` WARN** — FR13 observability gap for the working path only. [render_loop.py:~233-238] (edge, Low)
- [x] [Review][Patch] **_packaged_config swallows all exceptions → misleading FileNotFound** — broken/unreadable colcon install falls back to source-tree path; operator sees wrong path. Narrow except / better message. [node.py:~58-65] (blind+edge, Low)

### Deferred

- [x] [Review][Defer] **mock_publisher manage_rclpy=False teardown on dead context** [test/mock_publisher.py] — deferred: manual e2e hardware harness, not a production path; teardown edge only when engine hit FR4 fatal mid-run.
- [x] [Review][Defer] **mock_publisher latched depth-1 late-join misses `waking`** [test/mock_publisher.py] — deferred: DDS semantics of the test harness; engine subscribes before publish in practice; relevant to future live-pipeline integration, not the FR14 substitute.

### Dismissed (noise / false-positive / already covered)

- `smooth_damp` overshoot guard "broken" — it IS the canonical Unity/Game-Programming-Gems guard; `test_smooth_damp` (no-overshoot, monotonic, 2000 iters) passes; reviewer self-acknowledged "accidentally correct".
- SchemaVersionError propagation through `SingleThreadedExecutor.spin()` "unverified" — empirically covered: `test_subscribe_only.py::test_bad_version_message_exits_process_nonzero` is a subprocess test asserting exit 1 via the real executor on Jazzy, and it passes.
- Blind self-withdrawn items (#2 EngineState trust, #8 os._exit ordering, #13 base envelope, #14 led_bias dead-for-now, #16 first-tick dt) — by-design / not live bugs.
- `neutral()` all-zero vs "center_all()" — auditor explicitly confirmed functionally equivalent (drivers define centre as 0° calibrated), not a violation.

### Change Log (review)

- 2026-05-17 — Adversarial code review (Blind + Edge + Acceptance, Opus) of `dce46a7^..b62fbec`. 14 patch findings applied (commit `7f7939e`), 1 decision-needed deferred to Story 6.5 (speech-overlay/mood decay), 2 deferred (test-harness), ~6 dismissed. 116 tests green (15 new hardening regressions), production E/F/W clean. **Status held at `review` (NOT auto-`done`): the patches modified the hardware-proven path (§9 order, neck clamp, eye-wake-now-fatal, anticipatory floor, gesture release) — Story 6.4's AC#1 was proven at `0f43dfa`, before these. A short hardware re-verify of the reference `happy` run is required before this story is `done`, to keep the freeze honest.**
