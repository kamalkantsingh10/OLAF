# Story 7.5: Locked-expression regression harness

Status: ready-for-dev

<!-- Created 2026-05-20. Stories 7.1a / 7.1b / 7.1c froze the eyes,
neck, and ears at carefully-iterated values. 7.5 builds a permanent
regression harness so a future change can't silently drift any of
the locked values without an explicit test failure + owner
re-approval. -->

## Story

As the maintainer,
I want a **golden-snapshot regression harness** over every
hardware-locked expression value (eyes EXPR table + per-emotion
neck pose+drift+gesture + per-emotion ears pose+drift+gesture +
mood/activity bounds), so that **any future YAML/firmware change
that would silently change the look** of a locked emotion fails a
test loudly — forcing a deliberate owner-approved bump of the
golden, not a quiet regression.

## Background

- 7.1a froze the 15 EXPR enum mappings + the per-emotion
  geometry/tint/motif tables in firmware (`animation_engine.cpp`
  is compiled; cannot be golden-snapshotted from Python, but the
  device-contract enum + Python `EXPRESSION_MAP` + AR10 table can).
- 7.1b froze `neck_motion.yaml` — 15 emotions × {L1, L2 drift,
  L3 gesture}. Frozen 2026-05-20 ("+30% & lock").
- 7.1c froze `ears_motion.yaml` — same shape with 4 joints, 8-
  token gesture registry. Frozen on the seed pass 2026-05-20.
- The locked Python sides (EXPRESSION_MAP, AR10 table, the two
  motion YAMLs, mood + activity bounds) are EASY to snapshot —
  just hash / pickle the parsed structures into a golden artefact
  the tests compare against.
- Hardware (firmware) snapshotting is harder; covered by
  flash-hash + Pi-side smoke run (out of scope for this story —
  see 9.x Hardening if pursued).

## Acceptance Criteria

1. **Golden artefact** — a versioned, owner-approved JSON file
   under `docs/implementation-artifacts/locks/expression_golden.json`
   (or sibling) carrying:
   - `EXPRESSION_MAP` (head_i2c_client) — 15 entries
   - `_CANONICAL_TO_ESP32` (eye_adapter) — 12 canonical + activity
     aliases
   - `neck_motion.yaml` parsed dict (per emotion: L1, drift, gesture)
   - `ears_motion.yaml` parsed dict
   - `mood.*` (bias bounds)
   - safety envelopes (neck ±80/±20/±15; ears ±50/−60..+90)
2. **Regression tests** — host tests that diff the live structures
   against the golden and FAIL with a human-readable summary of
   what changed (per-emotion field-by-field) when they diverge.
3. **Bump procedure** — a documented one-liner (Makefile target,
   `poetry run` script, or marker in the story) to **regenerate
   the golden** that the owner runs explicitly after a sanctioned
   change. The regeneration script MUST NOT be on the green test
   path (so no silent drift).
4. **Owner-approval audit** — each entry in
   `expression_golden.json` carries a `locked_at` ISO timestamp +
   `locked_by_story` field (e.g. `"7.1b"`, `"7.1c"`); the
   regeneration tool fills these and refuses to overwrite without
   an explicit flag.
5. **Tests** — golden-diff tests added to `expression_engine`
   suite; `head_ears_driver` + `neck_driver` suites stay green.
6. **No false positives on Float drift** — comparisons use a small
   tolerance (1e-6) where appropriate; integer fields are exact.

## Tasks / Subtasks

- [ ] Task 1: **Snapshot script** —
  `scripts/regen_expression_golden.py` that loads
  `expression_map.yaml`, `neck_motion.yaml`, `ears_motion.yaml`,
  the Python tables (`EXPRESSION_MAP`, `_CANONICAL_TO_ESP32`),
  and the adapter safety envelopes; writes
  `docs/implementation-artifacts/locks/expression_golden.json`.
  Default-refuses overwrite; `--force` to bump. Stamps
  `locked_at` / `locked_by_story`.
- [ ] Task 2: **Initial golden** — run the script and commit the
  first golden. Lock-by stamps: `7.1a` for the device contract,
  `7.1b` for neck values, `7.1c` for ears values.
- [ ] Task 3: **Regression tests** — under
  `ros2/src/expression_engine/test/test_expression_lock.py`:
  per-section diff tests (eyes contract / neck / ears / mood
  bounds / activity bounds / safety envelopes). FAIL message
  shows the diff path + the changed value.
- [ ] Task 4: **Docs** — README block (or Dev Notes here) on
  how to bump the golden after a sanctioned change (and link it
  from each story's Change Log).

## Dev Notes

### Source of truth

- **Eye device contract:** `EXPR_*` in `i2c_slave.h`,
  `EXPRESSION_MAP` in `head_i2c_client.py`,
  `_CANONICAL_TO_ESP32` in `eye_adapter.py`.
- **Neck spec:** `neck_motion.yaml`. Safety envelope:
  `neck_adapter._LIMITS`.
- **Ears spec:** `ears_motion.yaml`. Safety envelope:
  `ears_adapter._LIMITS`.
- **Mood / activity bounds:** in `expression_map.yaml` (7.3 / 7.4
  will land final values; this harness picks them up).
- **Gesture registries:** `neck_gestures.GESTURES`,
  `ears_gestures.GESTURES` — token names also snapshotted.

### Constraints / guards

- Do NOT run the regeneration on CI / pre-commit. The script is
  an OWNER ACTION only. The test suite must FAIL on drift, not
  auto-update the golden.
- The golden file is small (a few KB JSON); commit it.
- If 7.2 / 7.3 / 7.4 finalise additional values, those land in
  the golden via a sanctioned bump tagged to the responsible
  story.

### References

- [Story 7.1a — eye device contract + L1 shapes (frozen)]
- [Story 7.1b — neck values (frozen)]
- [Story 7.1c — ears values (frozen)]
- Memory: [[project-story-71bc-plan]],
  [[feedback-eye-levels-7-1a]], [[reference-head-lcd-sides]]

## Dev Agent Record

### Progress

(none yet)

### Agent Model Used

### Debug Log References

### Completion Notes List

### File List
