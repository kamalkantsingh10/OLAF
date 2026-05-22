# Story 7.6: Body-owned expression interface contract

Status: review

<!-- Created 2026-05-23. Architecture decision (Kamal, 2026-05-23):
invert contract OWNERSHIP so the expression interface is a body-owned,
versioned STANDARD that any producer publishes to — the cmd_vel pattern.
Goal: one companion (or any producer) drives MANY hardware bodies. Data
flow is unchanged (producer publishes → body subscribes); this story is
about who OWNS/VERSIONS the contract and making it a real artifact. The
producer-facing contract doc already exists at
ros2/src/expression_engine/contract/INTERFACE.md (authored alongside this
story). -->

## Story

As the **maintainer of the OLAF body**,
I want the expression interface (the 4 topics the body consumes) to be a
**body-owned, versioned, machine-checkable standard** rather than an
implicit mirror of one companion release, so that **the same companion —
or any producer — can drive multiple hardware bodies**, and the body and
producer can no longer silently drift apart.

## Background

- Today the body **subscribes** to `mood` / `activity` / `speech_emotion`
  / `vocalization` (`std_msgs/String` + JSON, `schema_version: 3`) and is
  **subscribe-only** by invariant (`subscribers.py` FR3).
- The wire is already **abstract intent** (no actuator commands) — the
  property that makes multi-hardware possible. This story protects and
  formalises that; it does **not** change the data-flow direction.
- The contract is currently **companion-owned**: the body pins
  `pinned_companion_tag: v3.0.0` (`schema.py`, `expression_map.yaml:23`)
  and hand-mirrors the companion's schemas. That couples the body to one
  producer's *release*, backwards from the body-owned model.
- The producer-facing spec is now written:
  `ros2/src/expression_engine/contract/INTERFACE.md`. This story turns it
  into an enforced, versioned artifact and flips the ownership in code.
- This is the **cmd_vel pattern**: the body advertises an input interface;
  any producer publishes to drive it. Data flow stays producer → body.

## Scope (this repo only)

| In scope (body / this repo) | Out of scope |
|---|---|
| `contract/` artifact: INTERFACE.md (done) + JSON Schemas + VERSION | Producer-side changes (tracked in the producer repo — see `contract/INTERFACE.md`) |
| `interface_version` semantics replacing `pinned_companion_tag` | Building a separate neutral `olaf_expression_interface` repo (defer to 2nd-body trigger) |
| Relative topic names + node namespace (default `/olaf/*` preserved) | Authoring/expression tuning (Epic 7 stories 7.1–7.4) |
| QoS sourced from the contract spec | The locked-value regression harness (Story 7.5) |
| Optional `expression/capabilities` latched publisher | |

## Acceptance Criteria

1. **Contract artifact, self-contained.** `contract/` holds the
   normative spec (`INTERFACE.md`), a `VERSION` (semver, seeded `1.0.0`),
   and machine-readable **JSON Schemas** for the envelope + each of the 4
   payloads (`contract/schemas/*.json`), reverse-engineered from the
   current `schema.py` models. The directory is laid out so it can later
   be lifted into a neutral package with `git mv` (no in-repo imports
   leaking into it).
2. **Schema validation is single-source.** `schema.py` validates incoming
   envelopes against the JSON Schemas (or is cross-checked against them by
   a test), so the Python models and the published schemas can never
   diverge. No canonical vocabulary list is duplicated outside the
   contract (the `schema.py` tuples become derived from / checked against
   the schemas).
3. **`interface_version` replaces the companion-release pin.** The body
   accepts a **supported range** of interface versions: a **major**
   mismatch is fatal (same posture as today's `schema_version != 3`); a
   **minor** bump is forward-compatible (additive payload fields are
   tolerated, not fatal — relax envelope/payload `extra="forbid"` to
   "ignore-additive" *only* where the contract says a field is additive).
   `pinned_companion_tag` is removed from `expression_map.yaml` and code.
4. **Relax producer coupling.** `EventEnvelope.source` is no longer the
   `"voice_agent_pipeline"` literal — it accepts any non-empty producer
   id string. The interface must not name one producer.
5. **Topics namespaceable.** Topic names are configurable as **relative**
   names scoped by the node namespace (e.g. `expression/mood` under
   `/<robot>`), so two bodies coexist on one DDS graph. The current
   absolute `/olaf/*` names remain the **default** (no behaviour change
   out of the box). QoS profiles are read from the contract spec, not
   hard-coded in two places.
6. **(Optional, phaseable) capabilities announce.** A single latched
   `expression/capabilities` publisher — the **only** thing the body
   publishes — carrying `interface_version` + which channels/features the
   body actually drives. May be deferred to a follow-up if it risks the
   story; if deferred, leave a stub + note.
7. **Governance documented.** INTERFACE.md states (already drafted) that
   any vocabulary/envelope change is a version bump owned in `contract/`.
   The README / architecture docs link the contract as the interface
   source of truth.
8. **Tests green + contract tests added.** Existing `expression_engine`
   suite stays green. New tests: (a) every `schema.py` model validates a
   golden valid envelope and rejects the documented bad cases; (b) the
   JSON Schemas and the pydantic models agree (the AC#2 cross-check);
   (c) namespaced topic resolution works.

## Tasks / Subtasks

- [x] Task 1: **JSON Schemas** — author `contract/schemas/{envelope,
  mood,activity,speech_emotion,vocalization}.schema.json` from the
  `schema.py` v3 models; add `contract/VERSION` = `1.0.0`. (AC#1)
- [x] Task 2: **Validate against schemas** — wire `schema.py` /
  `subscribers.py` to validate against the JSON Schemas (or add the
  models-vs-schemas cross-check test). Remove duplicated vocab lists.
  Install `contract/` as package data (`setup.py`) so the schemas resolve
  at runtime. (AC#2)
- [x] Task 3: **`interface_version`** — introduce supported-range
  acceptance (major-fatal / minor-forward-compat); remove
  `pinned_companion_tag` from `expression_map.yaml` + `schema.py`. (AC#3)
- [x] Task 4: **Relax `source`** to a producer-id string. (AC#4)
- [x] Task 5: **Namespace topics** — relative names + node namespace,
  `/olaf/*` default; QoS sourced from the contract. (AC#5)
- [ ] Task 6: **(Optional) capabilities** latched publisher. (AC#6)
  — **DEFERRED** (AC#6 is optional & sanctions deferral). Not built:
  it would introduce the body's FIRST publisher, breaking the
  subscribe-only invariant — needs explicit owner go-ahead. Stub/note
  left in `contract/INTERFACE.md` ("Still planned (deferred)").
- [x] Task 7: **Docs** — link `contract/INTERFACE.md` from README +
  `docs/architecture/`; note the governance rule. (AC#7)
- [x] Task 8: **Tests** — golden + bad-case + cross-check + namespace
  resolution. (AC#8)

## Dev Notes

### Source of truth

- **Wire models:** `ros2/src/expression_engine/expression_engine/schema.py`
  (re-derived from companion `v3.0.0` — keep the lockstep note).
- **Subscriptions + QoS:** `expression_engine/subscribers.py`
  (`_qos_for`, `create_subscriptions`).
- **Topic config:** `config/expression_engine.toml` `[dds]`/`[topics]`;
  loaded by `config.py` (`REQUIRED_TOPICS`).
- **Pin to remove:** `expression_map.yaml:23` `pinned_companion_tag`,
  `schema.py:PINNED_COMPANION_TAG`.
- **Producer contract (this story's sibling artifact):**
  `contract/INTERFACE.md`.

### Constraints / guards

- **Do NOT change the data-flow direction.** Producer publishes → body
  subscribes, always (pub/sub). This story changes *ownership +
  versioning*, not direction. The subscribe-only invariant (FR3) holds —
  the only thing the body may publish is the optional `capabilities`
  topic (about itself, not state).
- **Keep `std_msgs/String` + JSON.** Do not introduce a custom `.msg`/IDL
  — loose coupling is deliberate (polyglot / non-ROS producers).
- **Backward compatible by default.** `/olaf/*` names + domain 0 keep
  working with the current companion `v3.0.0` with no producer change on
  day one.
- **`extra="forbid"` relaxation is surgical** — only additive fields the
  contract declares forward-compatible; unknown *envelope* typos should
  still fail loudly.

### Sequencing

- Independent of Story 7.5 (locked-value harness) and the Epic 7
  authoring stories (7.1–7.4). Can land any time.
- The neutral shared-interface package extraction is **explicitly
  deferred** until a second hardware body exists — keep `contract/`
  self-contained so that's a `git mv`, not a redesign.

### References

- Architecture discussion 2026-05-23 (cmd_vel-style body-owned interface;
  one companion → many hardware).
- `contract/INTERFACE.md` — the producer publishing contract.
- Memory: [[project-phase2-rescope]], [[reference-expression-engine-testing]].

## Dev Agent Record

### Progress

Tasks 1–5, 7, 8 complete; Task 6 (optional capabilities publisher)
deferred per AC#6. Full suite: **330 passed** (baseline 318 + 12 new).

### Agent Model Used

claude-opus-4-7 (Amelia / dev-story workflow)

### Debug Log References

- `just exp-test` — full suite green at each gate (318 → 322 → 326 → 330).

### Completion Notes List

- **Backward-compatible by design.** The wire `schema_version` (int `3`)
  stays the major-fatal gate; **no new required wire field** was added,
  so today's companion `v3.0.0` keeps working unchanged. Confirmed by the
  unchanged schema-version + subscribe-only integration tests.
- **AC#3 interpretation.** "Supported range" is realized as: wire
  `schema_version == 3` (major gate, fatal otherwise) + payload
  `extra="ignore"` (minor/additive forward-compat). The envelope stays
  `extra="forbid"` (typos fatal). `pinned_companion_tag` removed from
  `schema.py`, `map_loader.py`, and `expression_map.yaml`; replaced by a
  body-owned `interface_version` (`contract/VERSION` = `1.0.0`, mirrored
  as `schema.INTERFACE_VERSION`, cross-checked against the map at
  startup — the Story 6.2 lockstep safety, now body-owned).
- **AC#2 single-source.** The pydantic models are the source; the
  committed `contract/schemas/*.json` are GENERATED
  (`python -m expression_engine.contract_schemas`) and a test FAILS on
  drift — no jsonschema runtime dependency added.
- **AC#5 QoS.** Kept as the single code source (`subscribers._qos_for`),
  documented in `INTERFACE.md`, and PINNED by `test_contract_qos_profiles`
  (interpreting "not hard-coded in two places" as single-source +
  contract-documented). Namespacing needed no node code change — rclpy
  namespaces relative names natively; verified end-to-end through
  `create_subscriptions`.
- **AC#6 DEFERRED.** The `expression/capabilities` latched publisher is
  optional and would make the body publish for the first time (breaking
  the subscribe-only invariant). Left a note in `INTERFACE.md`; awaiting
  owner go-ahead.

### File List

**Added**
- `ros2/src/expression_engine/contract/INTERFACE.md` (authored with the story)
- `ros2/src/expression_engine/contract/VERSION`
- `ros2/src/expression_engine/contract/schemas/{envelope,mood,activity,speech_emotion,vocalization}.schema.json`
- `ros2/src/expression_engine/expression_engine/contract_schemas.py`
- `ros2/src/expression_engine/test/test_contract.py`
- `ros2/src/expression_engine/test/test_interface_namespace.py`

**Modified**
- `ros2/src/expression_engine/expression_engine/schema.py` (INTERFACE_VERSION; payload `extra="ignore"`; `source` → non-empty str; removed PINNED_COMPANION_TAG)
- `ros2/src/expression_engine/expression_engine/map_loader.py` (`pinned_companion_tag` → `interface_version` lockstep)
- `ros2/src/expression_engine/config/expression_map.yaml` (`pinned_companion_tag` → `interface_version`)
- `ros2/src/expression_engine/config/expression_engine.toml` (relative-name / namespace guidance)
- `ros2/src/expression_engine/setup.py` (install `contract/` as package data)
- `ros2/src/expression_engine/test/test_schema.py` (interface-version + forward-compat + source tests)
- `ros2/src/expression_engine/test/test_map_loader.py` (interface_version lockstep tests)
- `README.md` (link the producer contract)
- `ros2/src/expression_engine/config/README.md` (link the contract)

## Change Log

| Date | Change |
|------|--------|
| 2026-05-23 | Story implemented (Tasks 1–5,7,8); Task 6 deferred. Body-owned versioned interface contract — `contract/` artifact, `interface_version` replaces `pinned_companion_tag`, forward-compatible payloads, relaxed `source`, namespaceable topics. 330 tests pass. Status → review. |
