# Story 6.2: Map loader + vocabulary completeness

Status: review

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the maintainer,
I want `expression_map.yaml` loaded and validated against the pinned vocabulary at startup,
so that an incomplete map fails before the engine pretends to run.

> Prerequisite: Story 6.1 `done` (provides `schema.py` canonical payload models / vocabulary types this loader validates against).

## Acceptance Criteria

1. **Given** the package `config/` directory, **When** the engine starts, **Then** the loader reads `expression_map.yaml` and `expression_engine.toml` (FR5, AR9).
2. **Given** the loaded map and the pinned `olaf_companion` canonical set (`pinned_companion_tag`), **When** completeness is asserted, **Then** any missing `Mood`/`ActivityState` (incl. both `working_submode`s)/`speech_emotion`/`vocalization` entry is fatal (FR6, NFR7, AR8).
3. **Given** the `vocalization` block, **When** the map is loaded, **Then** `visible_only: true` is asserted present on `nod` and `shake`; missing is fatal (FR7).
4. **Given** a synthetic new map entry added with no engine code change, **When** a test loads the map, **Then** it is accepted, proving NFR5 (AR5).
5. **Given** an unknown canonical name at runtime, **When** it is received on a topic, **Then** the engine logs `expression.unmapped_<topic>` at WARN and falls back to the `default_*` pose — never freezes, never crashes (FR13).

## Tasks / Subtasks

- [x] Task 1: Seed `expression_map.yaml` skeleton (AC: #1)
  - [x] Create `ros2/src/expression_engine/config/expression_map.yaml` with the §5.2 structure: `schema_version`, `pinned_companion_tag`, `defaults` (pose/eye/led/heart), and `mood`/`activity`/`speech_emotion`/`vocalization` blocks (minimal entries OK here; full authoring is Epic 7)
- [x] Task 2: `map_loader.py` load + parse (AC: #1)
  - [x] Load YAML (`pyyaml`, already a project dep), parse into typed structures keyed **by topic** (mood/activity/speech_emotion/vocalization are distinct namespaces — AR6, brief #4)
  - [x] Resolve `defaults.*` for the FR13 fallback
- [x] Task 3: Completeness validation vs pinned canonical set (AC: #2)
  - [x] Derive the required canonical set from the pinned companion vocabulary (from `schema.py` enums / pinned tag — see Dev Notes)
  - [x] Assert every `Mood`, every `ActivityState` (including both `working_submode` values under `working`), every first-class `speech_emotion`, every `vocalization` tag has a map entry; any gap → fatal, clear operator message, non-zero exit (NFR7, AR8 step 3)
- [x] Task 4: `visible_only` invariant (AC: #3)
  - [x] Assert `nod` and `shake` have `visible_only: true`; missing/false → fatal (AR8 step 4, FR7)
- [x] Task 5: Runtime unknown-name fallback (AC: #5)
  - [x] On an unmapped canonical name at runtime: log `expression.unmapped_<topic>` at WARN, return the `default_*` render — never raise, never freeze (FR13)
- [x] Task 6: Tests (AC: #2, #3, #4, #5)
  - [x] Incomplete map (missing one canonical name) → fatal
  - [x] `nod`/`shake` missing `visible_only` → fatal
  - [x] **Synthetic added entry with NO code change loads cleanly (proves NFR5)** — this is the regression that protects extensibility
  - [x] Unknown runtime name → WARN + `default_*`, process stays alive

## Dev Notes

### The two distinct failure modes (do NOT conflate)

- **FR4 (Story 6.1):** wrong `schema_version` → fatal fail-fast.
- **FR6/NFR7 (this story, startup):** map missing a canonical name the pinned companion release defines → **fatal** at startup.
- **FR13 (this story, runtime):** an event arrives with a canonical name the map doesn't have (engine map lags companion) → **graceful** WARN + `default_*`, stay alive.

Startup is strict (complete-for-this-version); runtime is graceful. This asymmetry IS the design — getting it backwards breaks NFR5. [Source: phase2-prd.md#NFR5; phase2-expression-engine.md#9, #5]

### `expression_map.yaml` schema (architecture §5.2 — authoritative)

Disambiguation is **by topic, not by name**: `mood.happy` and `speech_emotion.happy` are different keys with different renders. The loader must preserve topic namespacing. Composition order (consumed by the render loop in Story 6.3) is `activity_base → mood_bias → speech_overlay → active_vocalization`; partial pose dicts merge, missing joints inherit the layer below. [Source: phase2-expression-engine.md#5.1, #5.2]

Required top-level keys: `schema_version` (engine map's own, independent of pipeline), `pinned_companion_tag`, `defaults` (pose/eye/led/heart — the FR13 fallback), `mood`, `activity` (note nested `working: {thinking:…, delegating:…}`), `speech_emotion`, `vocalization` (each `vocalization` gesture cue carries `visible_only`).

### Where the canonical set comes from

The "full canonical set of the pinned `olaf_companion` release" = all `Mood` values, all `ActivityState` values (both `working_submode`s), all first-class `speech_emotion` names, all `vocalization` tags — defined by the companion Appendix A and pinned via `pinned_companion_tag`. Source the enumerations from the `schema.py` models created in Story 6.1 (which were re-derived from Appendix A). The pinned tag is bumped in lockstep with map extensions (NFR5) — this loader is what enforces the lockstep. [Source: phase2-expression-engine.md#12.4; phase2-prd.md#FR6, NFR5]

### NFR5 is the load-bearing requirement here

Adding a new mood/activity/emotion/vocalization MUST be a pure `expression_map.yaml` data edit with **no engine code change**. The Task-6 synthetic-entry test is the guarantee. If satisfying completeness validation requires editing Python when the vocabulary grows, the design is wrong — revisit before proceeding. [Source: phase2-prd.md#NFR5; phase2-expression-engine.md#10]

### Reuse / scope

- `pyyaml` is already a project dependency (root `pyproject.toml`). Do not add another YAML lib.
- This story does NOT render anything and does NOT touch hardware/adapters or the render loop (Story 6.3). It produces validated, queryable map data + the fallback resolver. Keep it pure/testable.
- `expression_map.yaml` content authoring (the full vocabulary) is **Epic 7**, not here. Here the map only needs enough entries for tests; the schema/structure is what matters and it is frozen by Story 6.4.

### Project Structure Notes

```
ros2/src/expression_engine/
  expression_engine/map_loader.py   # NEW
  config/expression_map.yaml        # NEW (skeleton; Epic 7 authors content)
  test/test_map_loader.py           # NEW
```
Config lives in the package `config/` per architecture §5/§8 (mirrors companion brief Appendix B). No variance.

### Previous Story Intelligence (6.1)

- `schema.py` exposes the schema-3 payload models / canonical enums — reuse them as the source of the "required set", don't re-list canonical names by hand (single source of truth).
- Startup-fatal pattern (raise + journald + non-zero exit) was established in 6.1 — reuse the same helper for completeness/`visible_only` failures (consistency, NFR7/AR8).

### References

- [Source: docs/planning-artifacts/epics.md#Story-6.2]
- [Source: docs/planning-artifacts/prd/phase2-prd.md — FR5, FR6, FR7, FR13, NFR5, NFR7]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#5, #9, #10, #12.4]

## Dev Agent Record

### Agent Model Used

Amelia (bmad-dev-story) · claude-opus-4-7[1m]

### Debug Log References

- **Prerequisite note:** Story 6.1 is `review` (not yet `done`) when 6.2 ran — user explicitly directed proceeding. The concrete dependency (`schema.py` canonical enums) exists and is green (6.1: 41 tests). The loader derives its required set *dynamically* from `schema.py` (`get_args` + pinned constants), so it stays correct if a 6.1 review tweaks the enums.
- **Canonical-set source:** `schema.py` had `Mood`/`ActivityState`/`WorkingSubmode` Literals but `speech_emotion`/`vocalization` are `str` on the wire (forward-compat). Per Dev Notes ("single source of truth, don't re-list by hand") I added pinned tuples `SPEECH_EMOTION_CANONICAL` (12, brief §A.6), `VOCALIZATION_TAGS` (6, §A.7), `VOCALIZATION_GESTURE_CUES` (`nod`,`shake`) to `schema.py` — additive, cited to Appendix A @ `v3.0.0`. Cross-story touch of a `review`-state 6.1 file, justified as the architecture-mandated single vocab source; flag for 6.1 review awareness.
- **OWNER-DIRECTED SCHEMA DEVIATION (architecture §5.2 conflict — needs Story 6.4 / Winston reconciliation):**
  - `vocalization` entries are **deferred** — v1 vocalizations will become parametric *animations* (laugh, yes/no). The §5.2 `gesture:`/`audio_asset:` form is rejected by the owner. Story 6.2 keeps `vocalization` at the minimum the ACs require: all 6 tags present (AC#2) + `nod`/`shake` `visible_only:true` (AC#3/FR7). No pose/animation content authored — designed in a later story.
  - `speech_emotion` entries carry **both** a `neck` bias and `ears` pose (owner directive), not ears-only.
  - These diverge from architecture §5.2 (Dev Notes called it authoritative/frozen-by-6.4). Shipped `config/expression_map.yaml` is the truth; **the architecture doc and Story 6.4 must reconcile the vocalization shape.**
- **caplog gotcha:** `logging_setup` sets `propagate=False`, so pytest's root-attached `caplog` cannot capture engine WARNs (the structured line *is* emitted to stderr). Tests attach a capture handler directly to the `expression_engine` logger instead.
- Pre-existing UNRELATED failures persist (not 6.2): `head_ears_driver/test_expressions.py::test_get_preset_happy|sad` (stale `left_tilt<=0`, commit `7af9fb6`). `ros2/src/olaf_drivers/` untouched by this story.

### Completion Notes List

- All 5 ACs satisfied. **65/65** expression_engine tests pass (24 new in `test_map_loader.py`; **41 Story 6.1 tests remain green** after the node-signature change — zero regressions).
- **Strict-startup / graceful-runtime asymmetry implemented (the load-bearing design):** `load_expression_map` is fatal (`MapValidationError ⊂ ValueError`) on missing file, malformed YAML, missing top-level key, pinned-tag mismatch, incomplete canonical vocabulary (FR6), or `nod`/`shake` not `visible_only:true` (FR7). `ExpressionMap.resolve(topic,name)` is graceful — unmapped *name* → WARN `expression.unmapped_<topic>` + `defaults` render, never raises (FR13). An unknown *topic* raises `KeyError` (programming error, not a wire event — deliberately NOT the FR13 path).
- **NFR5 proven:** extra map entries beyond the pinned set are accepted (loader checks `required ⊆ present`, never forbids extras). `TestExtensibilityNFR5` adds a synthetic `speech_emotion`/`mood` entry with NO Python change and asserts it loads + is queryable.
- **Lockstep enforced:** map `pinned_companion_tag` must equal `schema.PINNED_COMPANION_TAG` (`v3.0.0`) or startup is fatal — this is the NFR5/§12.4 cross-check Story 6.1 promised.
- **Topic namespacing preserved (AR6, brief #4):** `mood.happy` ≠ `speech_emotion.happy` — distinct keyed entries, asserted by `test_topic_namespacing_preserved`.
- **§9 startup wiring:** `node.main` now runs §9 steps 1–4 (toml → map → completeness → visible_only) before `rclpy.init`; `MapValidationError` reuses the Story 6.1 fail-fast handler (structured journald `expression_map_load_failed` + `sys.exit(1)`). `ExpressionEngineNode` holds the validated map for the Story 6.3 render loop (not yet rendered). Story 6.1's `test_subscribe_only.py` updated to the new node constructor.
- Production modules flake8-clean (E/F/W @ line 99). No new dependency (pyyaml already a project dep).

### File List

**New:**
- `ros2/src/expression_engine/config/expression_map.yaml`
- `ros2/src/expression_engine/expression_engine/map_loader.py`
- `ros2/src/expression_engine/test/test_map_loader.py`

**Modified:**
- `ros2/src/expression_engine/expression_engine/schema.py` (added pinned `SPEECH_EMOTION_CANONICAL` / `VOCALIZATION_TAGS` / `VOCALIZATION_GESTURE_CUES` — single vocab source)
- `ros2/src/expression_engine/expression_engine/node.py` (§9 map-load wiring; `ExpressionEngineNode(config, expression_map)`; `_packaged_config` helper + `_default_map_path`)
- `ros2/src/expression_engine/test/test_subscribe_only.py` (updated to new node constructor — 6.1 regression fix)

### Change Log

- 2026-05-17 — Story 6.2 implemented: `expression_map.yaml` loader + vocabulary completeness. Seeded structural skeleton (all canonical names @ `v3.0.0`); `map_loader.py` with strict-startup completeness/`visible_only`/pinned-tag validation (FR6/FR7/NFR7) and graceful FR13 runtime fallback; pinned canonical tuples added to `schema.py`; §9 startup wired into `node.py`. 24 tests added; full suite 65/65 green. **Owner-directed deviation from architecture §5.2 (vocalization deferred to animations; speech_emotion gains neck bias) — flagged for Story 6.4 / architecture reconciliation.**
