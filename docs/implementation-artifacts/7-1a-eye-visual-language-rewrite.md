# Story 7.1a: Rewrite the eye visual language (spec-faithful renderer)

Status: review

<!-- Split out of Story 7.1 (owner-directed, 2026-05-19). 7.1 authored
the 12 speech_emotion map entries + retired expressions.py; it does
NOT touch the ESP32 eye renderer. This story owns ALL eye-device
rendering. -->

## Story

As the maintainer,
I want the Head ESP32 eye renderer rewritten to OLAF's **finalized UX visual language** — glowing cyan **filled shapes** (no iris/pupil/highlight), the 6-shape lexicon with per-emotion size/aspect/corner/glow, anime special-motifs, and per-emotion blink character — driven by all **12** canonical emotions distinctly,
so that the emotions authored in `expression_map.yaml` (Story 7.1) actually *read* on the robot's eyes as designed, instead of the legacy iris+pupil look squashed 12→7.

## Background / Why this is its own story

- Story 7.1 authored the 12 `speech_emotion` entries from the finalized UX spec (`docs/planning-artifacts/ux-design-specification.md`). The map carries the intent; the device cannot yet show it.
- **Today's gap (measured during 7.1):** the ESP32 renders an *iris circle + black eyelid mask + dark pupil + specular highlights* keyed to **7** device expressions (`EXPR_NEUTRAL/HAPPY/SAD/SURPRISED/ANGRY/SLEEPY/WINK`). The AR10 `_CANONICAL_TO_ESP32` table squashes the 12 canonical emotions onto those 7, so `content→happy`, `curious→surprised`, `sympathetic/melancholic→sad`, `frustrated→angry`, `scared→surprised` — 5 emotions have no distinct read.
- The finalized UX spec specifies a **different visual language**: "two eyes ... glowing cyan **FILLED shapes** with a soft 2–3 px outer bloom. **No outline, no pupil**, no mouth, no brows" — shape + size + corner-tilt + glow + blink-character carry 100% of the emotion. The current renderer is fundamentally a different system; this is a rewrite, not a parameter tweak.
- This is **firmware-heavy** and modifies the **AR10 table** (architecture-frozen-adjacent) and the **I2C expression enum/wire contract** — it needs its own ACs, tests, and hardware verification, separate from 7.1.

## Acceptance Criteria

1. **Given** the finalized UX spec eye system, **When** the ESP32 renders any of the 12 canonical emotions, **Then** the eye is a **glowing cyan filled shape with soft bloom, no pupil/iris/highlight**, using the 6-shape lexicon (`oval-steady, crescent-up, arc-down, narrow-slant, wide-round, tall-alert`) with the spec's per-emotion H% (visor height), W:H, corner-tilt, vertical position, and glow level.
2. **Given** all 12 `schema.SPEECH_EMOTION_CANONICAL` names, **When** each is driven, **Then** each renders **distinctly** (no two emotions visually identical) and matches its spec row — Kamal-confirmed on the LCD eyes (battery-independent; webcam capture per [[reference_webcam_capture]]).
3. **Given** the spec's anime special-motif library, **When** an emotion that triggers a motif peaks (sad→tears, angry→cross-vein + brief red flush, frustrated→steam/blank, surprised→sparkle/star-burst, sleepy→half-lid+ZZZ; others = plain signature eyeball), **Then** that motif renders as a finite reusable token; default state is the plain cyan shape.
4. **Given** the spec's blink rules, **When** an emotion is active, **Then** blink interval/duration/close-style follow that emotion's spec values ("blink for a reason" on pose change for curious/surprised).
5. **Given** the canonical→device path, **When** the table is updated, **Then** AR10 `_CANONICAL_TO_ESP32` maps 12 canonical → 12 distinct device targets (no squash), `head_i2c_client.EXPRESSION_MAP` + the ESP32 `EXPR_*` enum/`EXPR_COUNT` are extended consistently, and the change is recorded as a **deliberate owner-authorized deviation** from the AR10/§4 freeze (with rationale) in this story + the freeze ledger.
6. **Given** the rewrite, **When** the suite runs, **Then** there are firmware-logic/host tests for the canonical→device mapping (all 12 distinct, unknown→safe neutral) and the eye-shape parameter table; existing `expression_engine` + `head_ears_driver` suites stay green; `eye_adapter.translate` unit tests updated for the 12-distinct table.
7. **Given** the colour system, **When** rendering, **Then** eyes stay near-monochrome cyan by default; the overt colour shift (e.g. anger red flush) is only the special-motif peak (<15% per spec); mood stays a low-impact LED tint (NOT eye recolour). No regression to look-direction, wake/sleep, or the Story 7.1 LED-overlay event.

## Tasks / Subtasks

- [x] Task 1: Device contract — extend `EXPR_*` enum to the 12 canonical (+ keep sleepy/wink for internal status use), `EXPR_COUNT`, I2C register-map docs; `head_i2c_client.EXPRESSION_MAP`; AR10 `_CANONICAL_TO_ESP32` → 12 distinct + docstring; record the freeze deviation (AC#5). **Also added device-only extra `EXPR_FLIRTY=14` (Kamal 2026-05-19); `EXPR_COUNT=15`.**
- [x] Task 2: New filled-shape renderer — replaced with a clean kawaii composer in `animation_engine.{h,cpp}`: per-side `EyeGeom` + 13-mode `CropMode` lexicon (none/top/bottom/topbot/top-curved/topbot-curv/left/diag-top/diag-bot/base-curved/top-curved-slant/bottom-slant/bottom-curved-slant), pupil + catchlight, soft bloom halo. Blink/wake/transition machinery preserved (AC#1, **shape system per owner re-spec 2026-05-19/20 supersedes the original cyan-silhouette spec — recorded in Dev Agent Record**).
- [x] Task 3: Per-emotion params — all 15 expressions authored (12 canonical + sleepy/wink + flirty) and Kamal-locked one-by-one on hardware over an interactive per-expression loop (AC#2).
- [x] Task 4: Special-motif library — finite reusable manpu tokens: 💢 cross-vein, sweat drop, gloom lines, white/yellow sparkles, deep-yellow star-burst, white pupil-sparkle, floating pink hearts, ZZZ, "?" "!", impact lines, steam, tear stream, tremble, glow-pulse. Used per Kamal-defined per-emotion L2/L3 placements (AC#3).
- [x] Task 5: Blink character — per-emotion interval + duration tables in `calculateNextBlinkInterval()` / `triggerBlink()` (AC#4).
- [x] Task 6: Tests — `expression_engine` (133) + `head_ears_driver` (31) suites green after every flash; AR10 12-distinct table verified (no change since T1) (AC#6).
- [x] Task 7: Hardware verify — OTA-flashed `192.168.118.81` after every iteration; per-expression LCD confirmation completed for all 15 over the iteration loop (AC#2/#7). Final freeze owner-approved 2026-05-20 ("drop demo for now.. the eye expressions frozen").

## Dev Notes

### Source of truth

- **Eye system spec:** `docs/planning-artifacts/ux-design-specification.md` — "Shared eye rendering conventions", "Disney facial-cue → OLAF shape translation key", "Eye-shape lexicon", "Eye Special-Motif Library", "The 12 speech-emotion specs", "Consolidated eye-shape catalogue (the eyes reference sheet)". This is the canonical, owner-finalized artifact (imagen dropped; tabular spec is canonical).
- **12 canonical names:** `ros2/src/expression_engine/expression_engine/schema.py` `SPEECH_EMOTION_CANONICAL` (do NOT re-list elsewhere).
- **Authored intent:** `ros2/src/expression_engine/config/expression_map.yaml` `speech_emotion.*.eye` (Story 7.1 — frozen; this story does NOT edit the map).

### Current renderer (what is being replaced)

- `modules/head/firmware/src/animation_engine.cpp` — `calculateExpressionParams`, `renderEye` (iris→eyelid-mask→pupil→highlight), `applyEyelidMask`, `EyeShape` enum in `include/animation_engine.h`. Sprite-based, 60 FPS on the ESP32 (smart peripheral, AR1/§2 — engine sends semantic `set_expression` on change only).
- `modules/head/firmware/include/i2c_slave.h` — `EXPR_*` (0–6), `EXPR_COUNT=7`, reg `0x10 EXPRESSION_TYPE` / `0x11 EXPRESSION_INTENSITY`.
- Python: `ros2/src/expression_engine/expression_engine/adapters/eye_adapter.py` (AR10 table — frozen-adjacent), `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/head_i2c_client.py` (`EXPRESSION_MAP`, reg writes).

### Constraints / guards

- AR10 freeze deviation is **owner-authorized** (Kamal, 2026-05-19) — record rationale; do NOT move the table into the map (architecture rule still holds — it stays in `eye_adapter.py`).
- Smart-peripheral contract holds: engine sends semantic name + intensity on change; ESP32 owns its animation/motifs/blink. No render-loop ticking of eyes.
- Do NOT regress: look direction (reg 0x20/0x21), wake/sleep (boots asleep — `set_system_status('woke_up')`; expressions only render in SPEAKING state per current `main.cpp` — revisit gating as part of the rewrite), and Story 7.1's `REG_LED_OVERLAY` (0x40) separate LED event.
- Pre-existing head I2C **readback** quirk (reg 0x10/0x11 read swap under interleaved writes) is out of scope — the *write* path drives rendering. Note only.
- LED strip is on the robot-battery rail (unpowered without battery); LCD eyes are directly powered → this story is LCD-verifiable without the battery.

### Sequencing

- Depends on Story 7.1's authored map (done) for the canonical eye names + intensities. 7.1 can reach `review` without this; the robot just renders the legacy squash until 7.1a lands. Recommend 7.1a immediately after 7.1.

### References

- [Source: docs/planning-artifacts/ux-design-specification.md — full eye system + 12 specs + catalogue]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md §2 (smart peripheral), AR1, AR10 (frozen-adjacent — owner-authorized deviation here)]
- [Source: modules/head/firmware (animation_engine, i2c_slave), eye_adapter.py, head_i2c_client.py]
- Memory: [[project-story71-state]], [[project-expression-design-dna]], [[project-epic7-interactive-mode]], [[reference-head-esp32-gotchas]]

## Dev Agent Record

### Progress (PARKED 2026-05-19)

Owner-directed pivot: the finalised UX-spec cyan-silhouette eye system was rejected on hardware as not expressive; the device eye is now a **kawaii cartoon model** (white eyeball/shape + black pupil + catchlight, mirrored, ~80%, blink kept) iterated per-expression on real hardware (LCD, battery-independent) with dev-PC webcam self-verify.

- **T1 device contract: DONE** — AR10 un-squashed to 12 distinct (`eye_adapter._CANONICAL_TO_ESP32` 1:1 + docstring), `head_i2c_client.EXPRESSION_MAP` 7-13, firmware `EXPR_*` 0-13 / `EXPR_COUNT=14`. Python tests green (TestEyeAdapter 7, head_i2c_client 30). Owner-authorised AR10 freeze deviation recorded. Blink interval/duration are a per-emotion firmware contract (T5 partial).
- **Renderer (T2/T3/T5): in progress** — `animation_engine.{h,cpp}` rewritten to filled cartoon shapes; per-emotion param table + per-emotion blink table built. **LOCKED: happy, sad.** Other 10 provisional after a delegated "do all" + exaggeration pass; neutral/content/curious reworked, not yet confirmed.
- **Tooling:** flash by IP `pio run -e esp32s3 -t upload --upload-port 192.168.118.81`; webcam self-verify per [[reference-webcam-capture]]; manual review via `modules/head/firmware/tools/eye_stepper.py`.
- **Remaining:** lock the 10 unlocked emotions (per-expression loop), T4 special-motifs (optional), T6 host tests, T7 final hardware verify, then Status→review + sprint-status 7-1a→review. Full state: [[feedback-eye-rendering]], [[project-story71-state]].

### Progress (re-spec 2026-05-19, session 2)

Owner handed a full per-expression re-spec (`.ai/notes review.txt`) keyed
to a 35-expression anime ref sheet (`.ai/eye_ref_sheet.jpg`, grid 5×7),
plus a strict **3-level intensity model** and a new **15th device
expression `flirty`**. Decisions captured: intensity firmware-strict
1→L1 2→L2 3+→L3 (map re-authored later by owner); "left/right eye" =
viewer POV; **L1 shapes locked, only L2 colour-top-up + L3 anime-FX
iterate** ([[feedback-eye-levels-7-1a]]).

- **Renderer rewritten** — `animation_engine.{h,cpp}` replaced the
  accreted shape-zoo with a clean kawaii composer: per-emotion
  per-side `EyeGeom` (crop modes top/bottom/both/curved/left/diagonal),
  pupil + catchlight; blink/wake/transition machinery preserved.
- **All 15 expressions authored** per the re-spec + ref-sheet cells
  (neutral/happy/content=Pleased/excited/sad=Concerned-asym/
  melancholic=Cold/sympathetic=Serious/angry=Irritated/
  frustrated=Disgusted/scared/curious/surprised/sleepy=Tired/
  wink=Devious/flirty).
- **3-level model + anime-FX motif library** — L2 evident colour tints
  (neutral the only mono) + static marks; L3 anime FX from manpu
  research (💢 cross-vein, sweat drop, gloom lines, sparkles/star-burst,
  twinkles, floating pink hearts for flirty, ZZZ, ?/!, steam, tear
  stream, tremble, glow-pulse). Expression×3-level table owner-approved
  pre-build.
- **Device contract** — `EXPR_FLIRTY=14`, `EXPR_COUNT=15`;
  `head_i2c_client.EXPRESSION_MAP['flirty']=14` (device-only extra, NOT
  a frozen speech canonical → no schema/AR10-canonical change).
- **Build + suites** — `pio run -e esp32s3` clean (13.0% flash);
  expression_engine (133) + head_ears_driver (31) suites green.
  OTA-flashed 192.168.118.81 OK.
- **Tooling** — `eye_stepper.py` extended: L1/L2/L3 level keys + `a`
  auto-sweep (every expression × 3 levels) + flirty.
- **BLOCKED on hardware verify (T7/AC#2,#7):** the fixed dev-PC webcam
  has OLAF's head half out of frame (only one eye visible) — owner must
  re-centre/reframe the camera (or OLAF) before per-expression LCD
  confirmation can proceed. Status stays `in-progress`.

### Hardware verify — per-expression locks (2026-05-20)

- **Viewer-side convention fix (2026-05-20):** `renderEye(is_right)`
  now uses `viewer_left = is_right` — the driver's `selectEye(LEFT/
  RIGHT)` is OLAF-anatomical and FLIPPED from viewer POV. Pre-fix,
  pupils diverged on neutral and asymmetric cells landed on the
  wrong LCD. See [[reference-head-lcd-sides]].
- **LOCKED:** **happy** (upper-arc smile-arch + 50%-smaller pupil
  touching base curve; L2/L3 muted yellow `#fdffdd`; L3
  right-LCD-edge deep-yellow sparkle 0.50·halfW); **content**
  (nasal pupils + Pleased crop; L2/L3 `#edf2d8` tint-only; L3
  outer glow-pulse); **excited** (big eye + big pupil + BIG black
  4-pt sparkle centred on pupil, catchlight off; L2/L3 `#fcffcd`
  hint-yellow; L3 toned-down deep-yellow background sparkles, no
  star-burst). Kamal-confirmed muted-tint perception ("changed
  slightly — keep it as it is", 2026-05-20).

### Final lock — all 15 expressions frozen (2026-05-20)

Final per-expression iterations on hardware, then owner freeze
("drop demo for now.. the eye expressions frozen"):

- **neutral** — round, −5%, pupils pulled nasal. L1 only (no tint).
- **happy** — bottom **curved + slanted** crop (smile-arch lower lid,
  outer corners higher); small centred pupil low; tint `#fdffdd` at
  L2/L3; L3 large deep-yellow sparkle on viewer-right LCD's right
  edge. White central sparkle is **excited's** signature, not happy's.
- **content** — Pleased-style flat 30% top-crop, big pupil, pupils
  pulled toward each other (nasal). L2/L3 tint `#edf2d8` (tint-only);
  L3 soft outer glow-pulse.
- **excited** — big round eye, +50% pupil, **catchlight off**, white
  4-pt sparkle that **fits inside** the pupil (0.60·pupR, static).
  L2/L3 tint `#fcffcd`; L3 deep-yellow background sparkles (toned
  down, no star-burst).
- **sad** — both eyes same curved-from-top crop (`0.20`), pupils to
  the eyeline. L2: pale-blue tint + 3 purple slashes on viewer-right
  eye (no tear). L3: animated cyan tear-stream off viewer-left eye +
  gloom lines (no purple).
- **melancholic** — Cold-style top-crop part-circles, viewer-right
  smaller; pupils to screen-right. L2 cold-blue.
- **sympathetic** — Serious-style 40% top-crop half-lid; viewer-right
  smaller. L2 soft-warm; L3 teary glisten.
- **angry** — **renamed** from Irritated to the old Disgusted shape
  (diagonal-from-top viewer-left, diagonal-from-bottom viewer-right).
  Pupil unchanged. **No additional shape / no animation at any
  level.** L2 subtle red `#fab0a0`; L3 noticeable red `#ff6040`.
- **frustrated** — visual identity now = ref-sheet **Silly** (R2C4):
  flat 28% top-crop, small pupils glancing up-outward. L2 hinted
  orange `#ffd0a0`; L3 more-defined orange `#ffa060` (not deep). No
  marks at any level.
- **scared** — plain round eye (left-edge crop removed); small pupil,
  raised. L2 pale-blue.
- **curious** — asymmetric (viewer-left bigger), pupils up.
- **surprised** — big wide raised eye, tiny pupil. L2 light-cyan tint
  + "!" + Flirty-style motion lines on viewer-left. L3 impact-flash.
- **sleepy** *(status)* — heavy droopy lid (55% top-crop), low pupil,
  catchlight off.
- **wink** *(status)* — Devious sly half-lid: both eyes use the new
  `CROP_TOP_CURVED_SLANT` (curved top + linear slant, outer edges
  higher); **viewer-right eye smaller**. Pupils glance outward.
- **flirty** *(device-only extra, EXPR=14)* — small coy eye with
  bottom-crop, pupil glances up-outward. L2 pink tint + colour
  motion lines + static hearts. L3 **floating pink hearts in the
  background** + animated lines.

### Change Log

| Date | Note |
|--|--|
| 2026-05-19 | Rewrote renderer (kawaii composer + 3-level intensity); all 15 expressions drafted; owner table approved pre-build; OTA-flashed. |
| 2026-05-20 | Iterative per-expression hardware verify loop; fixed `viewer_left = is_right` (driver LEFT/RIGHT is OLAF-anatomical, flipped from viewer POV → [[reference-head-lcd-sides]]); successive lock-ins for all 15; angry/frustrated identity-swap; happy moved to curved+slanted bottom crop. **Owner freeze.** |

### Agent Model Used

claude-opus-4-7[1m]

### Debug Log References

### Debug Log References

### Completion Notes List

- Code/build/test complete for the re-spec; story remains `in-progress`
  pending owner per-expression LCD confirmation (AC#2/#7 cannot be
  self-certified). Webcam reframe required first.

### File List

Modified (committed to repo):
- `modules/head/firmware/include/animation_engine.h` — kawaii composer header, `EyeGeom`, 13-mode `CropMode` enum, motif method declarations.
- `modules/head/firmware/src/animation_engine.cpp` — full renderer rewrite: per-emotion `buildEye()` for all 15, `drawEyeball()` column rasteriser with all crop modes, `applyEmotionFX()` per-emotion L2/L3 motif dispatch, motif library (vein/sweat/gloom/sparkles/hearts/ZZZ/?/!/impact/steam/tear-stream/tremble/glow-pulse/blush/twinkles/star-burst/background-sparkles), per-emotion blink interval+duration tables.
- `modules/head/firmware/include/i2c_slave.h` — `EXPR_FLIRTY=14`, `EXPR_COUNT=15`.
- `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/head_i2c_client.py` — `EXPRESSION_MAP['flirty']=14`.
- `docs/implementation-artifacts/7-1a-eye-visual-language-rewrite.md` — Dev Agent Record, Tasks, Status, File List, Change Log.
- `docs/implementation-artifacts/sprint-status.yaml` — `7-1a` → `review`.

New (committed):
- `modules/head/firmware/tools/eye_stepper.py` — manual stepper with `1/2/3` level keys + `a` auto-sweep all-expressions×L1-L3 + flirty.

New (git-ignored, local-only):
- `.ai/eye_ref_sheet.jpg` — 35-expression anime ref sheet (Pinterest, thagirion).
- `docs/captures/eye_demo_*` — webcam verify stills (per `docs/captures/.gitignore`).

Deferred (drafted, not finished; demo work paused 2026-05-20):
- `scripts/eye_demo_record.py` — webcam-orchestrator stub (SSH multiplex + gst record + I2C-set-per-shot + timing JSON).
- `scripts/eye_demo_compose.py` — ffmpeg composer stub (intro card + transition + per-segment text overlays).
