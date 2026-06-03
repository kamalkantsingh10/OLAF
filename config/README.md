# `config/` — repo-root configuration

Machine/hardware configuration that is **not** tied to a single ROS2 package.
Two files live here:

| File | Owns | Lifecycle |
|------|------|-----------|
| [`i2c/addresses.yaml`](i2c/addresses.yaml) | I2C bus settings + the address/description/component map for every module (head_ears `0x08`, neck `0x09`, torso `0x0A`, base `0x0B`). | Hand-edited; changes only when hardware changes. |
| [`servo-ids.yaml`](servo-ids.yaml) | Feetech servo assignments + per-servo calibration: IDs, `center_position`, directions, angle limits, `counts_per_degree`, speeds. Covers ears (SCS0009) and neck (STS3215). | **Pi-written** — calibration tools and `neck_calibration.py set-center` rewrite it on the robot. Pull/stash carefully to avoid conflicts. |

## Expression / personality config lives elsewhere

How OLAF *emotes* — moods, speech emotions, vocalizations, eye/neck/ear motion,
LED behaviour — is **not** here. It is package-local to the expression engine,
because the ROS2 node resolves it through the colcon package share dir:

```
ros2/src/expression_engine/config/
├── expression_map.yaml      # ⭐ canonical emotion → body render (the main authoring file)
├── neck_motion.yaml         # per-emotion neck motion (L1/L2/L3) — frozen
├── ears_motion.yaml         # per-emotion ears motion (L1/L2/L3)
└── expression_engine.toml   # engine runtime wiring: DDS domain, topics, tuning knobs
```

See [`ros2/src/expression_engine/config/README.md`](../ros2/src/expression_engine/config/README.md)
for what each file owns, which code path loads it, and why they stay split.

**To tune personality / expression, edit those files — not this directory.**

## Why these aren't merged

`servo-ids.yaml` (Pi-written calibration) and `i2c/addresses.yaml` (hand-edited
hardware map) have different owners and lifecycles, so they stay separate. The
expression files stay in their package so the installed node can find them via
`get_package_share_directory("expression_engine")` — moving them to repo-root
`/config` breaks that lookup.

## Related

- Servo calibration guide: [`docs/guides/ear-servo-calibration.md`](../docs/guides/ear-servo-calibration.md)
- Expression input contract: [`ros2/src/expression_engine/contract/INTERFACE.md`](../ros2/src/expression_engine/contract/INTERFACE.md)
