# Story 6.1: Subscriber + schema-3 envelope validation

Status: review

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the avatar,
I want incoming events validated fail-fast,
so that a contract mismatch surfaces loudly instead of silently corrupting expression.

> **⛔ PREREQUISITE GATE:** Do not start this story until Story 5.3 is `done` and `epic-5: done` in `sprint-status.yaml`. Epic 5 is a hard gate. [Source: docs/planning-artifacts/epics.md#Sequencing-&-Dependencies; AR16]

## Acceptance Criteria

1. **Given** the configured DDS domain and topic names from `expression_engine.toml`, **When** the engine starts, **Then** it subscribes to all four topics — `mood`, `activity`, `speech_emotion`, `vocalization` (FR1).
2. **Given** a valid schema-3 `std_msgs/String` payload on any topic, **When** it is received, **Then** the `EventEnvelope` plus the four payload models deserialize it into typed objects (FR2, AR13).
3. **Given** an event with `schema_version != 3`, **When** it is received, **Then** the engine raises, logs to journald, and exits non-zero — no coercion or truncation (FR4).
4. **Given** the engine running, **When** its publishers are inspected by a test, **Then** no publisher exists on any of the four topics (FR3, subscribe-only invariant asserted).

## Tasks / Subtasks

- [x] Task 1: Package config plumbing (AC: #1)
  - [x] Create `ros2/src/expression_engine/config/expression_engine.toml` per architecture §8 (`[dds] domain_id`, `[topics]` four names defaulting to `/olaf/{name}`)
  - [x] Implement config load (stdlib `tomllib`, Python 3.11+; if pinned to 3.10 per `pyproject.toml`, use `tomli`) — fail-fast if file missing/invalid (NFR7)
- [x] Task 2: `schema.py` — schema-3 models (AC: #2, #3)
  - [x] Define `EventEnvelope` + 4 payload pydantic models matching the companion `olaf-embodiment-brief.md` Appendix A JSON shape (re-derived, NOT imported — AR13)
  - [x] `assert_schema_version(envelope)` → raise on `schema_version != 3`, log to journald, exit non-zero (FR4)
  - [x] Record `pinned_companion_tag` source-of-truth note (full enforcement lands in Story 6.2)
- [x] Task 3: `subscribers.py` — four subscriptions (AC: #1, #2)
  - [x] Create four `std_msgs/String` subscriptions on configured names/domain, RELIABLE QoS
  - [x] Deserialize JSON → `EventEnvelope` → typed payload; hand validated events to `state.py` (state model interface stubbed if 6.x ordering requires — keep this story self-contained)
- [x] Task 4: `node.py` — node + subscribe-only invariant (AC: #1, #4)
  - [x] `ExpressionEngineNode` (replace the Epic-5.1 stub) wires config → subscribers; single `rclpy` executor thread (AR3)
  - [x] NEVER create a publisher on the four topics (FR3)
- [x] Task 5: Tests (AC: #2, #3, #4)
  - [x] Valid schema-3 payload parses into typed objects
  - [x] `schema_version != 3` → raises + non-zero exit (assert process exit, not just exception swallowed)
  - [x] Malformed/short envelope rejected
  - [x] Assert the node holds zero publishers on the four topics (introspect `node.get_publisher_names_and_types_by_node` or equivalent)

## Dev Notes

### Architectural contract (read before coding)

This story builds the left edge of the pipeline: `subscribers → schema(validate, fail-fast) → state`. [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#1, #3]

- **Module decomposition (AR4):** `node.py` (lifecycle, executor, wiring, startup sequence), `schema.py` (`EventEnvelope` + 4 payload models, `assert_schema_version`), `subscribers.py` (4 `std_msgs/String` subs, JSON→model, hand to state), `state.py` (current mood/activity/speech_emotion/vocalization — full impl is later; this story only needs the write-side handoff point). [Source: phase2-expression-engine.md#3]
- **Concurrency (AR3):** one `rclpy` executor thread handles subscriptions and writes to a mutex-protected `state` snapshot, last-write-wins per topic (the companion already dedups `speech_emotion`). The render thread is a later story — do NOT build it here, but do not design `state` in a way that blocks it.
- **Subscribe-only is an invariant, not a preference (FR3):** single-writer-per-topic. The engine must be structurally incapable of publishing to the four topics. AC #4 asserts this with a test — treat a publisher on those topics as a build-breaking defect.
- **Fail-fast posture (FR4, NFR7):** `schema_version != 3` is fatal — raise, journald, `sys.exit(non-zero)`; systemd restarts. No version coercion, no truncation, no "best effort". This mirrors the companion's posture. Distinct from FR13's *runtime* graceful fallback (that is Story 6.2, unknown-name handling — do not conflate).

### Schema source — IMPORTANT dependency

`schema.py` models are **re-derived from the companion's `olaf-embodiment-brief.md` Appendix A** documented JSON shape — there is **zero cross-repo build coupling** (AR13). That brief lives in the sibling `olaf_companion` project, not this repo.
- If Appendix A is not available to the dev agent, STOP and surface it as a blocker (saved question below) — do not invent the envelope shape. The four topics are `mood`, `activity`, `speech_emotion`, `vocalization`; payload specifics (fields, the `audio_frame_id` on `speech_emotion`, `working_submode` on `activity`) come from Appendix A.
- Record the exact pinned `olaf_companion` tag; it gets written into `expression_map.yaml` (`pinned_companion_tag`) and cross-checked in Story 6.2. [Source: phase2-expression-engine.md#12.4; phase2-prd.md Next-Steps #4]

### Tech stack / libraries

- ROS 2 **Jazzy** / rclpy, `std_msgs/String`, RELIABLE QoS, DDS multicast discovery, no broker. [Source: phase2-prd.md#Technical-Assumptions]
- Package is `ament_python` (scaffolded in Story 5.1). Entry point `expression_engine_node = expression_engine.node:main` (already in `setup.py`). `package.xml` already declares `rclpy, std_msgs, olaf_interfaces, neck_driver, head_ears_driver`.
- Validation: pydantic (declare the version in `setup.py`/`package.xml` and pin it). Python `^3.10` per root `pyproject.toml` — confirm `tomllib` (3.11+) vs `tomli` accordingly.
- Structured JSON logging to journald is required system-wide (NFR8) — set it up minimally here; hardening is Story 6.7.

### Reuse / anti-patterns

- The Epic-5.1 `ExpressionEngineNode` is a deliberate stub that warns "real engine is Epic 6". **Replace it**, don't add alongside it.
- Do NOT introduce any ROS topic for low-level control — drivers are in-process via adapters in later stories (FR9). This story has no driver interaction at all.
- Do NOT build the render loop, map loader, or adapters here — scope is strictly subscribe + validate + state-handoff. Avoid scope creep (checklist 3.5).

### Project Structure Notes

```
ros2/src/expression_engine/
  expression_engine/
    node.py            # UPDATE — replace stub
    schema.py          # NEW
    subscribers.py     # NEW
    state.py           # NEW (write-side only this story)
  config/
    expression_engine.toml   # NEW
  test/                # NEW — schema + subscribe-only tests
```
Aligns with architecture §3 module table. No variance.

### References

- [Source: docs/planning-artifacts/epics.md#Story-6.1]
- [Source: docs/planning-artifacts/prd/phase2-prd.md — FR1, FR2, FR3, FR4, NFR7, NFR8; Technical Assumptions]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#1, #3, #8, #9, #12.4]
- Package scaffold: `ros2/src/expression_engine/` (setup.py entry point, package.xml deps)

### Saved Questions

1. Is the companion `olaf-embodiment-brief.md` Appendix A (envelope + 4 payload JSON schemas) accessible to the dev agent, and what exact `olaf_companion` git tag are we pinning to? Blocking for Task 2.

## Dev Agent Record

### Agent Model Used

Amelia (bmad-dev-story) · claude-opus-4-7[1m]

### Debug Log References

- Saved Question 1 **RESOLVED** (was blocking Task 2): companion `olaf-embodiment-brief.md` §Appendix A (lines 112–271) + source modules `src/voice_agent_pipeline/schemas/*_event.py` were made available. **Pinned `olaf_companion` tag: `v3.0.0`** (annotated, commit `321d9f8`). Models re-derived verbatim from that source (AR13 — zero cross-repo build coupling; NOT imported).
- Env: pydantic **not** installed on system Python (PEP 668 blocks pip); apt `python3-pydantic` is v1.10 (incompatible with the companion's v2 contract). Resolved per Kamal's direction by using the existing project **poetry** env (`/home/kamal/.cache/pypoetry/virtualenvs/olaf-KIMtbiLa-py3.12`, system-site-packages enabled) which already has pydantic 2.12.5 + ROS rclpy. Test invocation: `PYTHONPATH="ros2/src/expression_engine:${PYTHONPATH}" poetry run python -m pytest …` (`poetry run pytest` resolves to system pytest w/o pydantic — use `python -m pytest`; do NOT clobber the ROS overlay `PYTHONPATH`, prepend to it).
- Exit-code convention confirmed with companion side: `schema_version != 3` → `SchemaVersionError` → structured journald line → `sys.exit(1)` (symmetry with the pipeline's `__main__.py` → 1).
- Pre-existing UNRELATED failures: `head_ears_driver/test_expressions.py::test_get_preset_happy` & `::test_get_preset_sad` fail on a stale `left_tilt <= 0` assertion superseded by commit `7af9fb6` (head-ears hardware-limit fix). `ros2/src/olaf_drivers/` is untouched by this story — not a Story 6.1 regression, out of scope to fix here.

### Completion Notes List

- All 4 ACs satisfied; **41/41** expression_engine tests pass (`test_config` 8, `test_schema` 24, `test_state` 4, `test_subscribe_only` 5 incl. DDS-loopback round-trip + subprocess exit-code assertion). Zero regressions introduced (change is purely additive).
- **AR13 decision (not an oversight):** schema models are hand-re-derived in `expression_engine/schema.py`, NOT imported from `olaf_companion`. Architecture mandates zero cross-repo build coupling for this isolated `ament_python` package; Story 6.2's cross-check against `PINNED_COMPANION_TAG = "v3.0.0"` exists precisely to police hand-mirror drift. `schema.py` docstrings cite the exact companion source module + Appendix A section per mirrored type.
- **Per-topic QoS** re-derived from brief §A.1/§A.2 (story said "RELIABLE QoS" — the reliability axis): all four RELIABLE; `mood`/`activity` = TRANSIENT_LOCAL depth 1 (latched, late-join replay); `speech_emotion`/`vocalization` = VOLATILE depth 8. This honours both the story task and the canonical contract.
- **Fail-fast scoping:** `schema_version != 3` is the designated fatal contract breach (AC#3 — propagates out of the executor → `os._exit(1)`; systemd restarts). Other validation errors (malformed/short/invariant) are rejected loudly (structured `event_rejected` log) and dropped — NOT fatal. Runtime unknown-name graceful fallback (FR13) is deliberately left to Story 6.2, not conflated.
- **Minor structural additions** beyond the §3 module table: `config.py` (config loader) and `logging_setup.py` (NFR8 structured journald JSON, minimal — hardening is Story 6.7). Both are small SRP leaf modules supporting `node.py`'s startup; the §3 core modules (node/schema/subscribers/state) are intact and unchanged in role.
- **Dependency declared + pinned** per story: `pyproject.toml` (`pydantic = "^2.0"`, conditional `tomli` for <3.11), `setup.py` `install_requires`, `poetry.lock` regenerated (`poetry lock --no-update`, `poetry check` clean). `package.xml` documents why the `python3-pydantic` rosdep key is deliberately omitted (apt v1.10 incompatible) — flagged for Story 6.7 deployment hardening.

### File List

**New:**
- `ros2/src/expression_engine/config/expression_engine.toml`
- `ros2/src/expression_engine/expression_engine/config.py`
- `ros2/src/expression_engine/expression_engine/schema.py`
- `ros2/src/expression_engine/expression_engine/state.py`
- `ros2/src/expression_engine/expression_engine/subscribers.py`
- `ros2/src/expression_engine/expression_engine/logging_setup.py`
- `ros2/src/expression_engine/test/test_config.py`
- `ros2/src/expression_engine/test/test_schema.py`
- `ros2/src/expression_engine/test/test_state.py`
- `ros2/src/expression_engine/test/test_subscribe_only.py`

**Modified:**
- `ros2/src/expression_engine/expression_engine/node.py` (replaced Epic-5.1 stub)
- `ros2/src/expression_engine/setup.py` (install_requires: pydantic, tomli)
- `ros2/src/expression_engine/package.xml` (pydantic dependency note)
- `pyproject.toml` (pydantic + conditional tomli deps)
- `poetry.lock` (regenerated, no version churn)

### Change Log

- 2026-05-17 — Story 6.1 implemented: subscribe-only engine + schema-3 fail-fast envelope validation. Replaced Epic-5.1 scaffold with real `node.py`; added `config.py`, `schema.py`, `subscribers.py`, `state.py`, `logging_setup.py`, packaged `expression_engine.toml`. Schemas re-derived from `olaf_companion` @ `v3.0.0` (AR13). 41 tests added, all green. Pinned companion tag `v3.0.0` recorded in `schema.py` for the Story 6.2 cross-check.