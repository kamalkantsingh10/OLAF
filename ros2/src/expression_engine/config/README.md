# `expression_engine/config/` — index

The three YAML files here are the **behavior-authoring** source of truth for
how OLAF emotes. They are split by **actuator subsystem**, are loaded by
**different code paths**, and were **frozen by different stories** — keep them
separate (see "Why three files", below).

> Per-machine **hardware calibration** (servo IDs, centers, directions,
> mechanical limits) lives elsewhere: repo-root [`config/servo-ids.yaml`](../../../../config/servo-ids.yaml).
> That file is Pi-written (`set-center`, calibration tools) and is a different
> lifecycle — do **not** move it here or merge it in.

## Files

| File | Owns | Loaded by | Frozen by |
|------|------|-----------|-----------|
| `expression_map.yaml` | canonical-vocabulary → body render: `defaults`, `mood` (LED tint + nod/shake eye), `activity` base postures, 12 `speech_emotion` poses, `vocalization` layered actions. Structure is **frozen** (arch §5.2, Story 6.4); Epic 7 authors content into it. | `map_loader.load_expression_map()` via `node.py` `_default_map_path()` | 6.4 (schema) / 7.1, 7.2, 7.3, 7.4 (content) |
| `neck_motion.yaml` | per-emotion neck motion: `l1` static pose, `l2` idle drift, `l3` peak gesture (15 emotions). Angles in degrees. | `adapters/neck_motion_player.py` | **7.1b** (2026-05-20) |
| `ears_motion.yaml` | per-emotion ears motion: same `l1`/`l2`/`l3` shape over 4 joints (left/right pan+tilt). Angles in degrees. | `adapters/ears_motion_player.py` | **7.1c** (2026-05-20) |

## How config is resolved at runtime

Installed as ROS2 **package share data** — `setup.py` globs `config/*` into
`share/expression_engine/config/`. At runtime the node resolves via
`get_package_share_directory("expression_engine")`, with a dev-source-tree
fallback to `<pkg>/config/` (`node.py:56-62`). The motion players resolve
relative to their module (`Path(__file__).resolve().parents[2] / "config" / ...`).

**This is why these files must stay in the package**: move them to repo-root
`/config` and the colcon-installed node can no longer find them through the
standard share-dir lookup.

## Safety envelopes (do not exceed)

Clamped in code (`adapters/neck_adapter.py`, `adapters/ears_adapter.py`); the
authoritative mechanical numbers are in `config/servo-ids.yaml`:

- **Neck:** pan ±80° / tilt ±20° / roll ±15° (STS3215 linkage)
- **Ears:** pan ±50° (right_pan ≥65° **binds** — hard limit) / tilt −60..+90°;
  right_tilt physical min is −7 (servo-ids.yaml) though the adapter clamp is wider

## Why three files (not one)

Each file is loaded by a **different adapter** and was **frozen by a different
story** on its own hardware pass. One combined file would couple three
independently-tuned subsystems, growing the merge-conflict surface during active
tuning with no runtime benefit. The only upside of merging — discoverability —
is what this README provides instead.

## Bumping frozen values

`neck_motion.yaml` / `ears_motion.yaml` and the `speech_emotion` block are
**frozen**. A change is a deliberate, owner-approved act, not a casual edit —
note it in the responsible story's Change Log. (A golden-snapshot regression
harness over these values is the subject of **Story 7.5**, currently deferred.)
