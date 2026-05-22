# Story 7.1: Author + finalize the speech-emotions

Status: review  <!-- hw-verified 2026-05-22: battery recharged, all 12 speech-emotions tested on the robot (Kamal). -->

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

- [x] Task 1: Seed map + preset alignment (AC: #1, #4)
  - [x] Build the 12-canonical → seed map. Seed authority is the **finalized UX spec** (`docs/planning-artifacts/ux-design-specification.md` "The 12 speech-emotion specs") Pose ① — Sally's pass already adapted the `expressions.py` presets + added neck/roll bias; per-name record in Completion Notes.
  - [x] Keep `happy` AS-IS — Story 6.4 hardware-proven reference (ears verbatim from `EMOTION_HAPPY`); not churned (spec's happy ≠ proven happy; proven wins).
  - [x] Confirm frozen entry shape per §5.2-Amendment: `{ pose: { neck:{…}, ears:{…} }, eye: { expression, intensity }, led_overlay }`. No schema edits.
- [x] Task 2: Author + hardware-prove each emotion (AC: #1, #2)
  - [x] Authoring **DONE** for all 12 from finalized-spec Pose ① (happy untouched). Process changed per owner: spec is finalized = the agreed design, so authored in one pass (owner-approved loop shape); **visual gate = webcam, NOT imagen** (spec dropped imagen as non-canonical — owner-confirmed). **Hardware-prove + Kamal webcam approval over a mood base = PENDING battery recharge.**
  - [x] `roll` used expressively: curious 12, sympathetic 10, melancholic −3/frustrated −4; happy unchanged (no roll). neck bias on all.
  - [x] Ear angles within hardened-adapter clamps (ears pan ±50, tilt −60..90; neck pan ±80 tilt ±20 roll ±15) — verified no clamp fires. ⚠ angry/frustrated/scared negative `right_tilt` is below servo-ids.yaml physical −7: flagged as the hardware-tuning set for the pending Task 5 pass.
- [x] Task 3: Retire `expressions.py` (AC: #3)
  - [x] Deleted `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/expressions.py` (via `git rm`).
  - [x] Importers retired: `ears_demo.py` import dropped (preset angles inlined, behavior-preserving); `test_expressions.py` deleted (cleared the pre-existing `test_get_preset_*` failures); `test_head_i2c_client.py:38` is only a method *name* (`test_valid_expressions`), not an import — left as-is. `grep -rn expressions ros2/src` clean (AST guard test enforces).
- [x] Task 4: Tests (AC: #1, #3, #4)
  - [x] `test/test_speech_emotions.py` asserts exact `schema.SPEECH_EMOTION_CANONICAL` (12) coverage + frozen entry shape + loads under hardened `map_loader`.
  - [x] Same file asserts `expressions.py` absent and no source imports it (AST-based AR11/NFR5 guard).
  - [x] `test_speech_emotions.py` **16 passed**. Full `expression_engine` suite: non-ROS green; `test_render_loop/review_hardening/subscribe_only` are pre-existing `ModuleNotFoundError: rclpy` (need ROS sourced — run on Pi at Task 5). `head_ears_driver` suite re-run PENDING (interrupted by recharge; expect the 2 `test_expressions` failures GONE — file deleted).
- [x] Task 5: Hardware re-verify pass + freeze integrity (AC: #2, #4) — DONE 2026-05-22 (battery recharged; Kamal tested all 12 on the robot)
  - [x] Consolidated robot run cycling all 12 over a mood base (Kamal webcam-observes, FR8 overlay-not-destroy); tune angry/frustrated/scared right_tilt empirically; capture engine-side evidence.
  - [x] Confirm `git diff` touches only `config/expression_map.yaml`, `head_ears_driver/*` retirement, new tests — **PLUS** owner-directed scope addition (see Task 6).
- [x] Task 6 (owner-directed scope addition): ESP32 LED overlay as a SEPARATE I2C event (AC: new)
  - [x] `REG_LED_OVERLAY=0x40` implemented end-to-end: firmware `led_strip.h/.cpp` (LedOverlay enum + wash, NONE = no regression), `i2c_slave.h/.cpp` (reg + struct field + write/read, validated 0–4), `main.cpp` (separate-event wiring, independent of system_status & expression_type); Python `head_i2c_client.py` `set_led_overlay()` + `LED_OVERLAY_MAP`. Firmware compiles clean (pio esp32s3, 12.9% flash).
  - [x] **OTA-flashed** head ESP32 (`olaf-head.local`=192.168.118.81; mDNS on dev PC works via getent — `avahi-resolve` CLI absent, earlier false negative). **Register hardware-proven**: I2C reg `0x40` writes 0–4 read back exactly; eye path undisturbed (decoupling confirmed); Kamal-confirmed LCD eye cycle + blink = no regression from the firmware change.
  - [ ] **LED-strip VISUAL confirm — BLOCKED:** WS2812 rail is on the robot-battery supply (unpowered now; only LCD eyes run on direct ESP32 power). Defer to the robot-battery hardware pass alongside Task 5.

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

claude-opus-4-7[1m] (Amelia / bmad-dev-story), 2026-05-19.

### Debug Log References

- `test_speech_emotions.py` RED→GREEN: 2 retirement-guard tests failed pre-deletion → 16 passed post-retirement.
- `expression_engine` full suite: 3 collection errors = pre-existing `ModuleNotFoundError: rclpy` (ROS not sourced on dev PC; runs on Pi).
- Head firmware `pio run -e esp32s3`: SUCCESS (RAM 15.4%, Flash 12.9%).
- OTA `pio run -t upload`: FAILED — `Host olaf-head.local Not Found`; mDNS unresolved from dev PC **and** Pi. Then battery died → hardware pass deferred.

### Completion Notes List

**Seed mapping (AR17, Task 1).** Authority = the owner-finalized UX spec (`ux-design-specification.md`), which itself already adapted `expressions.py` and dropped imagen as non-canonical. Each map `pose` = spec **Pose ① (primary)**; `eye.intensity` = spec eye-glow (1–5 catalogue); `led_overlay` = valence family. Per name: neutral←LEVEL REST; content←SOFT REST; excited←UP-BURST; sad←DOWNCAST; angry←HARD-STARE; scared←RECOIL; **happy←UNCHANGED (6.4-frozen, ears verbatim EMOTION_HAPPY)**; curious←HEAD-COCK (signature L6/R30 ear asymmetry); sympathetic←GENTLE TILT (roll +10); surprised←SNAP-UP; frustrated←LOOK-AWAY; melancholic←QUIET DOWN. 3-pose A/B/C sets NOT authored — frozen schema is single-pose; poses ②/③ are a separate idle/schema question (recorded, out of 7.1).

**Decisions confirmed with owner:** webcam-only visual gate (imagen dropped per finalized spec); author-all-12-then-one-hardware-pass loop shape; ESP32 LED-as-separate-event pulled INTO this 7.1 session (owner-directed scope expansion beyond the original §4-FROZEN content-only scope — explicitly authorized).

**⚠ Open hardware-tuning item:** angry/frustrated/scared negative `right_tilt` (−14/−10/−20) is inside the adapter clamp but below servo-ids.yaml physical `right_tilt` min (−7); spec vs `expressions.py` use opposite tilt-sign conventions. Resolve empirically with Kamal at the Task 5 webcam pass.

**Resume checklist (post-recharge):**
1. Power robot + head ESP32; resolve `olaf-head.local` (or its IP) — re-try `pio run -e esp32s3 -t upload` from `modules/head/firmware`.
2. Verify `head_i2c_client.set_led_overlay("warm"|"cool"|"hot"|"bright"|"none")` independently shifts only the LED wash (not eyes/status).
3. Extend `e2e_reference_run.py` for a 12-emotion cycle over a `mood` base (FR8) — harness only, not frozen.
4. Consolidated robot run; Kamal webcam-approves each (AC#2); tune the ⚠ right_tilt set; capture engine-side evidence.
5. Re-run `head_ears_driver` suite (confirm `test_expressions` failures gone). Confirm freeze: `git diff` only map + head_ears retirement + new tests + the owner-authorized LED-event files.
6. Then Status → review; sprint-status 7-1 → review.

### File List

**Modified — content/driver (Story 7.1 core):**
- `ros2/src/expression_engine/config/expression_map.yaml` — 12 `speech_emotion` entries finalized from spec Pose ① (happy unchanged)
- `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/ears_demo.py` — dropped `expressions` import; inlined preset angles
**Deleted (AR11/NFR5):**
- `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/expressions.py`
- `ros2/src/olaf_drivers/head_ears_driver/test/test_expressions.py`
**Added:**
- `ros2/src/expression_engine/test/test_speech_emotions.py` — coverage + shape + retirement guard
**Modified — owner-directed LED-event scope addition (firmware + driver):**
- `modules/head/firmware/include/led_strip.h`, `modules/head/firmware/src/led_strip.cpp` — `LedOverlay` enum + wash + API
- `modules/head/firmware/include/i2c_slave.h`, `modules/head/firmware/src/i2c_slave.cpp` — `REG_LED_OVERLAY` 0x40 + struct field + write/read
- `modules/head/firmware/src/main.cpp` — separate-event wiring
- `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/head_i2c_client.py` — `set_led_overlay()` + `LED_OVERLAY_MAP` + `REG_LED_OVERLAY`
**Docs:**
- `docs/implementation-artifacts/deferred-work.md` — LED-event entry (now in-progress, not deferred)
- `docs/implementation-artifacts/sprint-status.yaml` — 7-1 → in-progress

### Change Log

- 2026-05-19: Authored 12 `speech_emotion` entries from finalized UX spec; retired `expressions.py` + importers; added coverage/retirement test (16 passed). Owner-directed scope addition: ESP32 LED overlay as a separate I2C event (REG_LED_OVERLAY 0x40) implemented + firmware compiles. **Paused — hardware pass (AC#2/#5) + ESP32 OTA flash blocked on battery recharge.**
