# Story 6.7: Service hardening

Status: ready-for-dev

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the maintainer,
I want the engine to run as a robust long-lived service,
so that it recovers from failure and never silently degrades.

> Prerequisite: Stories 6.1–6.6 `done`. This is the last Epic 6 story; completing it should let `epic-6: done` once all 6.x are done, unblocking Epic 7.

## Acceptance Criteria

1. **Given** the deployment target, **When** the engine is installed, **Then** it runs under a systemd unit (`Restart=always`) with structured JSON logging to journald (NFR8, AR15).
2. **Given** startup, **When** the engine initializes, **Then** validation runs strictly ordered — config → map → vocab completeness → `visible_only` → adapters connect → DDS subscribe → RUNNING — and any failure is fatal and journald-visible (NFR7, AR8).
3. **Given** the re-scoped repo, **When** CI runs, **Then** all Phase 1 component test scripts still pass unchanged (NFR11, AR14).

## Tasks / Subtasks

- [ ] Task 1: systemd unit (AC: #1)
  - [ ] Provide a unit (e.g. `deploy/expression-engine.service`) with `Restart=always`, journald stdout/stderr, correct `WorkingDirectory`/env (`PYTHONPATH` for in-process drivers + vendored SDK), single Pi 5 target (NFR9, AR15)
  - [ ] Document install/enable steps; fold in any privilege/overlay requirement surfaced by the WS2812 adapter (Story 6.6)
- [ ] Task 2: Structured JSON logging (AC: #1)
  - [ ] All engine logs emit structured JSON to journald (NFR8); consistent fields (event, topic, level, ts). Make `expression.unmapped_<topic>` (FR13) and startup steps machine-greppable
- [ ] Task 3: Enforce the startup validation order (AC: #2)
  - [ ] `node.py` runs the §9 sequence exactly: (1) load `expression_engine.toml` → (2) load `expression_map.yaml` → (3) assert vocab completeness vs `pinned_companion_tag` → (4) assert `visible_only:true` on nod/shake → (5) `adapter.connect()` each, in sequence → (6) DDS init + subscribe 4 topics → (7) all green → start render thread → state RUNNING
  - [ ] Every step fatal: clear operator-facing message, non-zero exit, journald captures, systemd restarts (NFR7)
- [ ] Task 4: Phase 1 non-regression in CI (AC: #3)
  - [ ] Ensure all Phase 1 component/unit test scripts still pass unchanged; wire them into CI if not already; the re-scope must not have regressed Phase 1 (NFR11, AR14)
- [ ] Task 5: Tests
  - [ ] Each startup step's failure → process exits non-zero with the correct ordered message (table-driven: bad toml, missing map, incomplete vocab, missing `visible_only`, adapter connect fail, DDS fail)
  - [ ] Log output parses as JSON; required fields present
  - [ ] CI green including Phase 1 suites

## Dev Notes

### Startup validation sequence (architecture §9 — every step fatal, order is the contract)

```
1. Load expression_engine.toml
2. Load expression_map.yaml
3. Assert vocabulary completeness vs pinned_companion_tag canonical set   (FR6)
4. Assert visible_only:true on nod & shake                                (FR7)
5. adapter.connect() for every configured adapter, in sequence            (NFR7)
6. DDS init + subscribe to all 4 topics on the domain
7. All green → start render thread → state RUNNING
```
The order matters: fail before opening DDS if the map is bad; fail before RUNNING if any adapter is offline. Identical posture to the companion. v1 has **no graceful-degradation layer** (NFR7) — do not add retry/backoff (explicitly deferred to companion v2 / out of scope). [Source: phase2-expression-engine.md#9; phase2-prd.md#NFR7, Out-of-Scope]

Much of steps 1–6 already exists from Stories 6.1/6.2/6.4 — this story's job is to **enforce the exact ordering and fatality in `node.py`**, not reimplement the checks. Consolidate the scattered fail-fast helpers into one ordered startup routine.

### Logging / service (NFR8, NFR9, AR15)

Single Pi 5, co-located with the pipeline over loopback DDS (NFR9 — do not engineer for multi-host). systemd `Restart=always`; structured JSON logs to journald. Reuse the journald/fail-fast helper established in 6.1 and used since; standardize the JSON schema of log lines now (one place). [Source: phase2-expression-engine.md#11; phase2-prd.md#NFR8, NFR9]

### NFR11 — prove the re-scope didn't regress Phase 1 (AR14)

All Phase 1 component test scripts MUST pass **unchanged**. This is the evidence that reusing the drivers in-process did not break them. Existing module unit tests live at `ros2/src/olaf_drivers/*/test/test_*.py` (pytest, mocked serial/I2C). Story 5.3's hardware scripts are manual (not CI). Wire the *unit* suites into CI; do not modify Phase 1 tests to make them pass — if they fail, that's a real regression to fix. [Source: phase2-prd.md#NFR11; phase2-expression-engine.md#10; Explore report — test layout]

### Anti-patterns

- Do NOT add graceful degradation / reconnect-with-backoff (out of scope, NFR7 — v1 is fail-fast + systemd restart).
- Do NOT change Phase 1 test files to go green (NFR11 = "unchanged").
- Do NOT reorder or make any startup step non-fatal "for convenience" — the order is the architectural contract.
- Do NOT engineer multi-host/clock-sync (NFR9 — single Pi loopback is the v1 target).

### Previous Story Intelligence (6.1–6.6)

- 6.1 established journald/fail-fast + node wiring; 6.2 vocab/visible_only checks; 6.4 froze Protocols + adapter `connect()`; 6.6 may have surfaced a privilege/overlay requirement for WS2812 → fold into the systemd unit. This story unifies all of it into one ordered, observable, restartable service.

### Project Structure Notes

```
ros2/src/expression_engine/expression_engine/node.py  # UPDATE — enforce §9 ordered startup
deploy/expression-engine.service                       # NEW (systemd unit)
ros2/src/expression_engine/test/test_startup_order.py  # NEW
CI config                                              # UPDATE — include Phase 1 suites
```
Matches architecture §9/§11. No variance.

### References

- [Source: docs/planning-artifacts/epics.md#Story-6.7]
- [Source: docs/planning-artifacts/prd/phase2-prd.md — NFR7, NFR8, NFR9, NFR11; Out of Scope]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#9, #10, #11]

## Dev Agent Record

### Agent Model Used

### Debug Log References

### Completion Notes List

### File List
