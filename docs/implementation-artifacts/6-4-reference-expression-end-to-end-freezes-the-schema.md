# Story 6.4: Reference expression end-to-end (freezes the schema)

Status: ready-for-dev

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

- [ ] Task 1: FR14 mock companion publisher
  - [ ] `ros2/src/expression_engine/test/mock_publisher.py` — publishes schema-3 `EventEnvelope` JSON on the four topics; scriptable sequences; the dev/CI substitute for the live pipeline (FR14, architecture §10)
  - [ ] Includes a sequence that emits `speech_emotion=happy` with a valid `audio_frame_id`
- [ ] Task 2: Concrete continuous adapters (real hardware)
  - [ ] `adapters/neck_adapter.py` wrapping `NeckServoDriver`; `apply({"pan","tilt","roll"})` → `move_pose(...)`; `neutral()` → centered; `connect/close` → ctor/`close()`
  - [ ] `adapters/ears_adapter.py` wrapping `EarsServoDriver`; `apply({"left_pan","left_tilt","right_pan","right_tilt"})` → `move_left_pan/left_tilt/right_pan/right_tilt(deg, speed_pct)`; `neutral()` → `center_all()` targets
- [ ] Task 3: Concrete delegating eye adapter
  - [ ] `adapters/eye_adapter.py` wrapping `HeadI2CClient`; `set_expression(canonical,intensity)`/`blink()`/`look(x,y)`/`connect()→open()`
  - [ ] **Canonical→ESP32 expression-string translation table lives inside this adapter** (AR10). Map `happy`→`"happy"`; document the table; only 7 ESP32 strings exist (`neutral,happy,sad,surprised,angry,sleepy,wink`) — pick the closest for the reference emotion
- [ ] Task 4: Author the reference expression
  - [ ] Add a complete `speech_emotion: happy` entry to `expression_map.yaml` exercising pose (neck+ears) + eye, in the §5.2 structure, seeded from `head_ears_driver/expressions.py` `EMOTION_HAPPY` preset
- [ ] Task 5: End-to-end hardware run (AC: #1)
  - [ ] Mock publisher → engine → real neck+ears+eyes; visibly renders `happy`; pose reaches body within (audio_anchor−30ms)..(−80ms) (NFR2)
  - [ ] Run on the robot via the documented `PYTHONPATH … poetry run` pattern; capture observed evidence
- [ ] Task 6: FREEZE + document (AC: #2)
  - [ ] Write the frozen `expression_map.yaml` schema spec (key structure for pose/LED/eye/heart, layering/composition order) into `docs/planning-artifacts/architecture/phase2-expression-engine.md` (or a referenced freeze doc) and mark it FROZEN with date + commit
  - [ ] Mark `adapters/base.py` Protocols FROZEN (same)
  - [ ] State explicitly that Epic 7 authors content only — no schema/Protocol edits

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

### Debug Log References

### Completion Notes List

### File List
