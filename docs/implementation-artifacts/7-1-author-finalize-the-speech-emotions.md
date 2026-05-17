# Story 7.1: Author + finalize the speech-emotions

Status: ready-for-dev

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the maintainer,
I want all first-class speech-emotions authored and finalized in `expression_map.yaml`, defined collaboratively and proven on the real robot,
so that the body renders the full emotional vocabulary correctly and `expressions.py` can be retired as the single source of expression data.

## Acceptance Criteria

1. **Given** the existing `head_ears_driver/expressions.py` presets as seed (AR17), **When** the `speech_emotion` block is authored in `expression_map.yaml`, **Then** all 12 first-class `speech_emotion` names are present and seeded/adapted from `expressions.py` where a preset exists, finalized to the frozen §5.2 shape (FR6).
2. **Given** each authored speech-emotion, **When** the FR14 mock publisher emits it (with a mood base active), **Then** it renders correctly on real hardware (Kamal-confirmed) **and** overlays correctly on the mood base without destroying it (FR8).
3. **Given** the migration is complete, **When** the repo is inspected, **Then** `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/expressions.py` is deleted **and** no module imports it — the map is the single source of expression data (AR11, NFR5).
4. **Given** the frozen contracts, **When** this story completes, **Then** NO change was made to `expression_map.yaml`'s key structure, `adapters/base.py` Protocols, `schema.py`, `render_loop.py`, or `map_loader.py` — Story 7.1 is **content + driver-side migration only** (AR5/§5.2-FROZEN, §4-FROZEN).

## Tasks / Subtasks

> **⚠️ EXECUTION MODE — INTERACTIVE, NOT AUTONOMOUS.** This story is authored **one emotion at a time, with Kamal**. Do NOT batch-author all 12 and mark done. Per-emotion loop (Task 2):
> define together → author the single entry → render on the robot (Kamal observes) → generate a visualization image via the **imagen MCP** (Gemini "nano-banana-pro") for compare/approve → iterate → next. AC#2's "renders correctly" is **Kamal's visual call**, never self-certified.

- [ ] Task 1: Seed map + preset alignment (AC: #1, #4)
  - [ ] Build the 12-canonical → `expressions.py` preset map. Direct presets exist for: `neutral, happy, curious, sad, excited, angry, surprised`. NO direct preset (adapt from nearest + collaborative tuning): `content, scared, sympathetic, frustrated, melancholic`. Record the chosen seed/adaptation per name.
  - [ ] Keep `happy` AS-IS — it is the Story 6.4 hardware-proven reference (ears verbatim from `EMOTION_HAPPY`); do not churn it.
  - [ ] Confirm frozen entry shape per §5.2-Amendment: `{ pose: { neck:{…}, ears:{…} }, eye: { expression, intensity }, led_overlay }`. `pose` carries BOTH neck and ears (owner-directed). No schema edits.
- [ ] Task 2: Author + hardware-prove each emotion **interactively** (AC: #1, #2)
  - [ ] For EACH of the 12 (skip already-proven `happy`), with Kamal: (a) discuss intended look; (b) author that one `speech_emotion.<name>` entry; (c) render on the robot via the e2e harness with a mood base active (verify FR8 overlay-not-destroy); (d) generate a visualization image via the imagen MCP for side-by-side approval; (e) iterate to Kamal's approval before the next emotion.
  - [ ] Use `roll` (sideways tilt) expressively per emotion: noticeable on `curious`/`sympathetic`/`melancholic`/quizzical; subtle or none on `happy`. neck bias on all (owner-directed).
  - [ ] Ear angles stay within safe limits (`right_pan ≤ 50°`, project memory); the hardened neck/ears adapter clamps + warns — author within range so no clamp fires.
- [ ] Task 3: Retire `expressions.py` (AC: #3)
  - [ ] Delete `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/expressions.py`.
  - [ ] Update/retire its importers so nothing imports it: `head_ears_driver/ears_demo.py` (uses `get_preset_by_name`), `head_ears_driver/test/test_expressions.py` (delete — tests the deleted module; this also clears the long-standing pre-existing `test_get_preset_happy|sad` failures), `head_ears_driver/test/test_head_i2c_client.py` (remove any `expressions` reference). Verify `grep -rn expressions ros2/src` is clean (ignore `ros2/build|install` artifacts and unrelated `.expressions` attribute names in `_testing.py`/`test_real_adapters.py`).
- [ ] Task 4: Tests (AC: #1, #3, #4)
  - [ ] A test asserting the map's `speech_emotion` block covers exactly `schema.SPEECH_EMOTION_CANONICAL` (12) and still loads under the hardened `map_loader` value validation.
  - [ ] A test asserting `head_ears_driver/expressions.py` does not exist and no source module imports it (AR11/NFR5 guard).
  - [ ] Full `expression_engine` suite + `head_ears_driver` suite green (the 2 pre-existing `test_expressions.py` failures should now be GONE because the file+test are deleted, not skipped).
- [ ] Task 5: Hardware re-verify pass + freeze integrity (AC: #2, #4)
  - [ ] Final consolidated robot run cycling all 12 emotions over a mood base (Kamal observes); capture engine-side evidence + the imagen visualization set.
  - [ ] Confirm zero edits to frozen files (`git diff` touches only `config/expression_map.yaml`, `head_ears_driver/*` retirement, and new tests).

## Dev Notes

### Frozen contracts — content only (read first)

Story 6.4 FROZE `expression_map.yaml`'s key structure (§5.2-as-amended) and the `adapters/base.py` Protocols (§4), hardware-proven @ commit `0f43dfa`. **Story 7.1 authors entry *values* only.** Do NOT modify `schema.py`, `render_loop.py`, `map_loader.py`, `adapters/*`, or the map's key structure. `pinned_companion_tag` stays `v3.0.0`; `schema_version` stays `1`. [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md §4-FROZEN, §5.2 + §5.2-Amendment; epics.md AR5]

Frozen `speech_emotion` entry shape (owner-directed, §5.2-Amendment): `pose` carries BOTH `neck` (pan/tilt/roll) and `ears` (left_pan/left_tilt/right_pan/right_tilt) partials; `eye: { expression, intensity 1–5 }`; `led_overlay` is a token string (`none|warm|cool|hot|bright`) — not yet rendered (LED adapter is Story 6.6) but authored now. The hardened `map_loader` (code review 2026-05-17) fails startup on non-numeric pose/intensity, non-dict entries, or duplicate keys — author valid numbers.

### The 12 canonical names + seed mapping (AR17)

`schema.SPEECH_EMOTION_CANONICAL` = primary `neutral, content, excited, sad, angry, scared`; secondary `happy, curious, sympathetic, surprised, frustrated, melancholic`. `head_ears_driver/expressions.py` `PRESETS` (ear angles only, no neck) cover: `neutral, happy, curious, sad, excited, angry, surprised` (direct), plus `thinking, confused, sleepy` (NOT speech-emotions — do not map). **No preset for `content, scared, sympathetic, frustrated, melancholic`** — define collaboratively, adapting the nearest preset + a neck/roll bias. All 12 already have placeholder entries in the map (6.2 skeleton); 7.1 *finalizes* them. `happy` is already final + hardware-proven (6.4) — leave it. [Source: ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/expressions.py; config/expression_map.yaml]

### Eye expression vocabulary (AR10)

`eye.expression` should remain the canonical emotion name (e.g. `frustrated`); the `eye_adapter._CANONICAL_TO_ESP32` table (AR10, frozen-adjacent — do NOT move it into the map) maps all 12 → the ESP32's 7 strings. Keep eye intensity 1–5.

### Hardware-run discipline (same as Story 5.3 / 6.4)

`ssh olaf.local`; `cd ~/olaf`; commit→push→`git pull` (Pi servo-cal is committed history — `git stash` is safe); then:
`source /opt/ros/jazzy/setup.bash && PYTHONPATH=ros2/src/expression_engine:ros2/src/olaf_drivers/neck_driver:ros2/src/olaf_drivers/head_ears_driver:libs:$PYTHONPATH ~/.local/bin/poetry run python ros2/src/expression_engine/test/e2e_reference_run.py` — extend the harness with a selectable/cycling emotion sequence over a `mood` base (FR8 overlay check). Adapters return to neutral + close in `finally`. Tests run via `poetry run python -m pytest` (NOT `poetry run pytest`); prepend, never clobber, `PYTHONPATH`. [Source: [[reference_expression_engine_testing]]; Story 6.4 Dev Agent Record]

### Interactive + imagen visualization

Per [[project_epic7_interactive_mode]]: each emotion is defined with Kamal, authored, rendered on hardware (his visual approval is AC#2), and visualized via the **imagen MCP** (Gemini nano-banana-pro) for compare/approve. The imagen MCP must be connected before the dev run; if unavailable, the hardware render + Kamal's eyes are the source of truth and the image step is skipped (non-blocking design aid — do not HALT the story for it).

### Scope guards / anti-patterns

- Story 7.1 ≠ Story 7.5. The locked-replay regression harness (`test/test_locked_expressions.py`, NFR10) is **Story 7.5**, not here. Do not build it.
- Do NOT author `vocalization` (Story 7.2), `activity` (7.3), or `mood` ambient tuning (7.4). speech_emotion only.
- Do NOT change easing/timing code. Kamal finds default easing slow — that is config tuning for Story 7.4, not a 7.1 code change ([[feedback_expression_tuning]]).
- Do NOT re-introduce `gesture:`/`audio_asset:` or move the AR10 eye table into the map.

### Project Structure Notes

```
ros2/src/expression_engine/config/expression_map.yaml          # UPDATE — finalize 12 speech_emotion entries
ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/expressions.py   # DELETE (AR11)
ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/ears_demo.py     # UPDATE — drop expressions import
ros2/src/olaf_drivers/head_ears_driver/test/test_expressions.py          # DELETE (tests deleted module)
ros2/src/olaf_drivers/head_ears_driver/test/test_head_i2c_client.py      # UPDATE if it refs expressions
ros2/src/expression_engine/test/e2e_reference_run.py                     # UPDATE — multi-emotion cycle (harness, not frozen)
ros2/src/expression_engine/test/test_speech_emotions.py                  # NEW — coverage + retirement guard
```
No variance from architecture §3. The map and adapter Protocol/schema are FROZEN — touching them is out of scope and a freeze violation.

### Previous Story Intelligence (6.1–6.4)

- 6.4 hardware-proved the chain and the `happy` reference; its `e2e_reference_run.py` + `mock_publisher.HAPPY_REFERENCE_SEQUENCE` are the template to extend per-emotion. Engine-side evidence (eased neck/ears targets) accompanies Kamal's visual confirmation.
- The §9 startup order matters: harness must call `connect_adapters()` then `wire_subscriptions()` then `render_loop.start()` (a missed `wire_subscriptions()` silently renders neutral — caught in 6.4 re-verify, commit `68a71a7`).
- ESP32 boots asleep — `eye_adapter.connect()` now wakes it (fatal if it can't). Pi enablement: `source /opt/ros/jazzy/setup.bash` gives the poetry venv `rclpy`.
- Map-loader/render hardened by the 2026-05-17 code review — author within those validators (numeric, in-range, no duplicate keys).

### References

- [Source: docs/planning-artifacts/epics.md#Story-7.1 (lines 370–390); AR5, AR11, AR17, AR14, FR5/FR6/FR7/FR8, NFR5/NFR10]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md §4-FROZEN, §5.2 + §5.2-Amendment, §12.2 (expressions.py retire), §12.4]
- [Source: ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/expressions.py — PRESETS seed]
- [Source: ros2/src/expression_engine/config/expression_map.yaml — current skeleton + proven `happy`]
- Memory: [[project_epic7_interactive_mode]], [[project_expression_map_schema]], [[feedback_expression_tuning]], [[reference_expression_engine_testing]], [[project_epic6_7_sequencing]]

### Saved Questions

1. Connect the imagen MCP (Gemini nano-banana-pro) before the dev run — it is not currently among connected MCP tools. Non-blocking (hardware + visual approval is the real gate) but the visualization step needs it.
2. For the 5 emotions with no `expressions.py` preset (`content, scared, sympathetic, frustrated, melancholic`) — these will be defined from scratch with Kamal in the interactive loop (expected, not a blocker).

## Dev Agent Record

### Agent Model Used

### Debug Log References

### Completion Notes List

### File List
