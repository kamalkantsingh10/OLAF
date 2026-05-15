# Story 5.3: Per-driver hardware verification (GATE)

Status: in-progress

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the maintainer,
I want each reused Phase 1 driver demonstrably moving real hardware via a standalone test script,
so that the in-process-adapter assumption is proven before any Epic 6 work begins.

> **🚧 THIS STORY IS A HARD GATE.** Epic 5 is declared "green" only when Stories 5.1, 5.2, and ALL of 5.3 pass. Epics 6, 7, and 8 MUST NOT start until this story is `done`. [Source: docs/planning-artifacts/epics.md#Epic-5; docs/planning-artifacts/prd/phase2-prd.md#Story-5.3; AR16]

## Acceptance Criteria

1. **Given** the neck driver and a standalone script following the Phase 1 component-test convention, **When** the script is run via the documented `PYTHONPATH … poetry run python` pattern on the robot, **Then** the neck produces observable, correct motion **And** the script exits 0 on success and is committed under the appropriate `scripts/` / module test location.
2. **Given** the ears driver (`ears_servo_driver.py`) and its standalone script, **When** the script is run on the robot, **Then** the ears produce observable, correct motion **And** the script exits 0 and is committed.
3. **Given** the head/eye path (`head_i2c_client.py`) and its standalone script, **When** the script is run on the robot, **Then** observable eye/display output is produced **And** the script exits 0 and is committed.
4. **Given** Stories 5.1, 5.2, and all of 5.3, **When** all pass, **Then** Epic 5 is declared "green" **And** Epics 6–8 are unblocked (and MUST NOT have started before this).

## Tasks / Subtasks

- [ ] Task 1: Neck driver hardware verification script (AC: #1)
  - [x] Create `scripts/verify_neck_driver.py` (standalone, NOT a ROS 2 node)
  - [x] Import `NeckServoDriver` from `neck_driver.neck_servo_driver`
  - [x] Instantiate with default config; in `try/finally`, exercise `move_pose(pan=…, tilt=…, roll=…)` through a small safe sweep, then `center_all()`, then `close()` in `finally`
  - [x] Print human-observable step descriptions; `sys.exit(0)` on success, non-zero on exception
  - [ ] Run on the robot, confirm observable correct motion (pan = looks left/right, tilt = chin up/down, roll = head tilt) **← HARDWARE, blocked on robot access**
- [ ] Task 2: Ears driver hardware verification script (AC: #2)
  - [x] Create `scripts/verify_ears_driver.py`
  - [x] Import `EarsServoDriver` from `head_ears_driver.ears_servo_driver`
  - [x] Exercise `move_left_pan/move_left_tilt/move_right_pan/move_right_tilt`, then `center_all()`, then `close()` in `finally`
  - [x] Keep angles inside per-servo `min_angle`/`max_angle` from `config/servo-ids.yaml` (see Dev Notes — right_pan binding limit; pan capped at 45°)
  - [ ] Run on the robot, confirm observable correct motion **← HARDWARE, blocked on robot access**
- [ ] Task 3: Head/eye path hardware verification script (AC: #3)
  - [x] Create `scripts/verify_head_eye.py`
  - [x] Import `HeadI2CClient` from `head_ears_driver.head_i2c_client`; `open()`, cycle a few `set_expression(...)` values + `trigger_blink()` + `set_look_direction(x,y)`, `close()` in `finally`
  - [ ] Run on the robot, confirm observable eye/display output **← HARDWARE, blocked on robot access**
- [ ] Task 4: Gate closure (AC: #4)
  - [ ] Confirm all three scripts exit 0 and are committed under `scripts/`
  - [ ] Update `docs/implementation-artifacts/sprint-status.yaml`: `5-3-per-driver-hardware-verification: done`; when 5.1+5.2+5.3 all done, set `epic-5: done`
  - [ ] Record observed-motion evidence in Completion Notes (what moved, expected vs actual)

## Dev Notes

### Why this story exists (do not skip the hardware run)

The entire Phase 2 architecture rests on one unproven assumption: that the engine can import the Phase 1 driver Python classes **in-process** and drive real hardware through Protocol-shaped adapters (FR9, AR1). Stories 6.x build adapters directly on these classes. If any driver does not actually move hardware when imported standalone, every downstream epic is built on sand. **This story's value is the physical observation, not the code.** A script that exits 0 without a human confirming motion does NOT satisfy the ACs. [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#2; #12]

### Real driver APIs (verbatim contracts — cite these, do not guess)

**NeckServoDriver** — `ros2/src/olaf_drivers/neck_driver/neck_driver/neck_servo_driver.py`
```python
NeckServoDriver(config_path: str = CONFIG_PATH)
.move_pose(pan: float = 0.0, tilt: float = 0.0, roll: float = 0.0, speed: int | None = None) -> bool
.move_pan/move_tilt/move_roll(degrees: float, speed: int | None = None) -> bool
.center_all(speed: int | None = None) -> bool
.close() -> None
```
Servo names: `pan`, `left_linkage`, `right_linkage`. Angle semantics: pan + = OLAF looks left; tilt + = chin up; roll + = tilts to OLAF's right.

**EarsServoDriver** — `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/ears_servo_driver.py`
```python
EarsServoDriver(config_path: str = CONFIG_PATH)
.move_left_pan/move_left_tilt/move_right_pan/move_right_tilt(degrees: float, speed_pct: float = 0.0) -> bool
.move(servo_name: str, degrees: float, speed_pct: float = 0.0) -> bool
.center_all() -> bool
.close() -> None
```
Servo names: `left_pan`, `left_tilt`, `right_pan`, `right_tilt`. `speed_pct` is a fractional adjustment (≈ -0.3..+0.3) on `base_speed`. Pan 0 = center, + = outward.

**HeadI2CClient** — `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/head_i2c_client.py`
```python
HeadI2CClient(bus_number: int = 1, address: int = 0x08)
.open() -> None
.set_expression(expression: str, intensity: int = 3) -> bool   # intensity 1–5 clamped
.trigger_blink() -> bool
.set_look_direction(x: int, y: int) -> bool   # x,y in -100..+100
.set_system_status(status: str) -> bool
.close() -> None
```
Valid expression strings: `neutral, happy, sad, surprised, angry, sleepy, wink` (only 7 — this constraint drives the Epic-6 eye-adapter translation table, AR10). System status strings: `idle, woke_up, listening, processing, speaking, going_idle`.

### Hardware safety constraints (regression-critical)

- **Ears `right_pan` binding:** `right_pan` center ≈ 241 with `direction: -1`; max safe angle ≈ 55°. At ≥65° the raw position approaches ~19 and the servo **binds**. Keep the ears script's `right_pan` sweep ≤ 50°. Do NOT change the `right_pan` direction sign. [Source: project memory — Ears Config]
- Read each servo's `min_angle`/`max_angle` from `config/servo-ids.yaml` and stay inside them; never hardcode a wider range.
- Always `close()` in a `finally` block — leaving the serial bus open wedges the next run.

### Execution environment (exact, from project memory + Explore)

- Robot host: `ssh olaf.local` (passwordless). Repo on Pi: `~/olaf`. Poetry: `~/.local/bin/poetry`.
- Run pattern (matches Phase 1 convention, e.g. `scripts/olaf_demo.py`):
  ```bash
  cd ~/olaf && PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver:ros2/src/olaf_drivers/neck_driver:libs \
  ~/.local/bin/poetry run python scripts/verify_<x>.py
  ```
- Serial ports: `/dev/waveshare_neck`, `/dev/waveshare_ears` (udev-stable). Head is I2C bus 1, addr `0x08`.
- The full Feetech SDK is vendored at `libs/scservo_sdk/` (pip `feetech-servo-sdk` is stripped — do NOT `pip install` it). [Source: project memory — Servo SDK]
- `config/servo-ids.yaml` on the Pi may carry uncommitted local center calibration — `git stash`/commit Pi-side before pulling. [Source: project memory — SCS0009 Gotchas]

### Reuse, do not reinvent

- `scripts/olaf_demo.py` already imports and sequences all three drivers (a 75s demo). **Model the verification scripts on its structure** (direct import, instantiate, `time.sleep` sequencing, `close()` on exit). Consider lifting its setup boilerplate rather than re-deriving it.
- Module unit tests already exist (`ros2/src/olaf_drivers/*/test/test_*.py`, pytest + mocked serial/I2C). 5.3 is explicitly a **hardware** verification, complementary to those — do not duplicate the mocked unit tests.

### Project Structure Notes

- New scripts go in `scripts/` (Phase 1 convention; `olaf_demo.py` lives there). ACs accept "appropriate `scripts/` / module test location" — `scripts/` is the right choice for cross-driver hardware checks.
- These are standalone scripts, NOT ROS 2 nodes and NOT pytest tests (no hardware in CI). Do not place under a `test/` dir that CI collects.
- No conflict with the `expression_engine` package — 5.3 touches only `scripts/` and reads existing drivers; it does not modify driver code.

### References

- [Source: docs/planning-artifacts/epics.md#Story-5.3]
- [Source: docs/planning-artifacts/prd/phase2-prd.md#Story-5.3 — GATE definition]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#2 — continuous vs delegating adapters; #12 — locked decisions]
- [Source: project memory — SSH Access, Servo SDK, SCS0009 Gotchas, Ears Config]
- Driver files: `ros2/src/olaf_drivers/neck_driver/neck_driver/neck_servo_driver.py`, `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/{ears_servo_driver,head_i2c_client}.py`
- Convention exemplar: `scripts/olaf_demo.py`

## Dev Agent Record

### Agent Model Used

claude-opus-4-7[1m] (BMad dev-story workflow)

### Implementation Plan

Three standalone, non-ROS verification scripts under `scripts/`, modeled on the Phase 1 convention exemplar `scripts/olaf_demo.py` (direct driver import → instantiate → `time.sleep`-sequenced moves → `close()` in `finally`). Each exercises one reused driver against its real, verified API; prints human-observable step labels; `sys.exit(0)` on clean completion, non-zero on any driver/hardware exception. Angles deliberately conservative and inside `config/servo-ids.yaml` limits (ears `right_pan` capped 45° < ~55° binding limit).

### Debug Log References

- `python3 -m py_compile scripts/verify_{neck_driver,ears_driver,head_eye}.py` → PASS (syntax clean).
- Scripts marked executable (`chmod +x`).
- Driver-exercising / hardware run NOT performed in the dev environment: driver constructors open `/dev/waveshare_*` serial / I2C bus 1 on `__init__` and raise `ConnectionError` without hardware. Hardware run is required on `olaf.local` (see Completion Notes).

### Completion Notes List

- ✅ Code artifacts for Tasks 1–3 complete: `scripts/verify_neck_driver.py`, `scripts/verify_ears_driver.py`, `scripts/verify_head_eye.py`. Syntax-validated, executable, API-accurate (signatures cross-checked against `neck_servo_driver.py`, `ears_servo_driver.py`, `head_i2c_client.py`).
- ⛔ **HALT — story intentionally NOT marked done/review.** ACs #1–#4 require *observed correct motion on the physical robot*; that evidence cannot be produced from the dev environment. Per workflow anti-fabrication rule, code-creation subtasks are checked, but every "Run on the robot, confirm…" subtask and Task 4 (gate closure) remain open pending hardware.
- ▶️ **Operator hardware run (on `olaf.local`, hardware ready):**
  ```bash
  cd ~/olaf && git pull   # bring the three verify_*.py scripts onto the Pi
  PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver:ros2/src/olaf_drivers/neck_driver:libs \
  ~/.local/bin/poetry run python scripts/verify_neck_driver.py
  PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver:ros2/src/olaf_drivers/neck_driver:libs \
  ~/.local/bin/poetry run python scripts/verify_ears_driver.py
  PYTHONPATH=ros2/src/olaf_drivers/head_ears_driver:ros2/src/olaf_drivers/neck_driver:libs \
  ~/.local/bin/poetry run python scripts/verify_head_eye.py
  ```
  For each: confirm motion/eye output matched the printed steps and the exit code was 0. Paste the three outputs + a one-line motion confirmation back here; dev-story will then check the hardware subtasks + Task 4, record evidence, set Status → review, and (5.1+5.2+5.3 green) flip `epic-5: done` to unblock Epic 6.
- Note: `config/servo-ids.yaml` on the Pi may carry uncommitted local calibration — `git stash` or commit Pi-side before `git pull` to avoid a merge conflict.

### File List

- `scripts/verify_neck_driver.py` (new)
- `scripts/verify_ears_driver.py` (new)
- `scripts/verify_head_eye.py` (new)
- `docs/implementation-artifacts/5-3-per-driver-hardware-verification.md` (modified — Tasks/Status/Dev Agent Record)
- `docs/implementation-artifacts/sprint-status.yaml` (modified — 5.3 → in-progress)

## Change Log

| Date | Change |
|------|--------|
| 2026-05-16 | Created the three standalone driver hardware-verification scripts (Tasks 1–3 code). Story moved ready-for-dev → in-progress. HALT pending operator hardware run on olaf.local for AC #1–#4 verification. |
