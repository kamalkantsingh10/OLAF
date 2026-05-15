# Story 6.1: Subscriber + schema-3 envelope validation

Status: ready-for-dev

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

- [ ] Task 1: Package config plumbing (AC: #1)
  - [ ] Create `ros2/src/expression_engine/config/expression_engine.toml` per architecture §8 (`[dds] domain_id`, `[topics]` four names defaulting to `/olaf/{name}`)
  - [ ] Implement config load (stdlib `tomllib`, Python 3.11+; if pinned to 3.10 per `pyproject.toml`, use `tomli`) — fail-fast if file missing/invalid (NFR7)
- [ ] Task 2: `schema.py` — schema-3 models (AC: #2, #3)
  - [ ] Define `EventEnvelope` + 4 payload pydantic models matching the companion `olaf-embodiment-brief.md` Appendix A JSON shape (re-derived, NOT imported — AR13)
  - [ ] `assert_schema_version(envelope)` → raise on `schema_version != 3`, log to journald, exit non-zero (FR4)
  - [ ] Record `pinned_companion_tag` source-of-truth note (full enforcement lands in Story 6.2)
- [ ] Task 3: `subscribers.py` — four subscriptions (AC: #1, #2)
  - [ ] Create four `std_msgs/String` subscriptions on configured names/domain, RELIABLE QoS
  - [ ] Deserialize JSON → `EventEnvelope` → typed payload; hand validated events to `state.py` (state model interface stubbed if 6.x ordering requires — keep this story self-contained)
- [ ] Task 4: `node.py` — node + subscribe-only invariant (AC: #1, #4)
  - [ ] `ExpressionEngineNode` (replace the Epic-5.1 stub) wires config → subscribers; single `rclpy` executor thread (AR3)
  - [ ] NEVER create a publisher on the four topics (FR3)
- [ ] Task 5: Tests (AC: #2, #3, #4)
  - [ ] Valid schema-3 payload parses into typed objects
  - [ ] `schema_version != 3` → raises + non-zero exit (assert process exit, not just exception swallowed)
  - [ ] Malformed/short envelope rejected
  - [ ] Assert the node holds zero publishers on the four topics (introspect `node.get_publisher_names_and_types_by_node` or equivalent)

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

### Debug Log References

### Completion Notes List

### File List
