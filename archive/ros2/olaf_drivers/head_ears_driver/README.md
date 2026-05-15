# Archived ROS-node wrappers — superseded 2026-05-15

`ears_node.py` and `head_node.py` were the per-module ROS-topic subscriber
wrappers — they subscribed to `/olaf/ears/expression`, `/olaf/ears/pose`
(the now-archived `Expression` / `EarsPose` msgs). They are the
"architecture skin" the Phase 2 re-scope retires.

**What was kept** in `ros2/src/olaf_drivers/head_ears_driver/`:

| File | Why kept |
|---|---|
| `ears_servo_driver.py` | Hardware logic — imported in-process by the expression engine's adapters (Phase 2, Epic 6) |
| `head_i2c_client.py` | Head ESP32 I2C logic — same |
| `expressions.py` | Expression *content* (emotion → ear pose presets) — **seed material for `expression_map.yaml`** (Phase 2, Epic 7) |
| `ears_calibration.py`, `ears_demo.py` | Calibration / manual test tooling |

`archive/ros2/olaf_bringup/launch/ears.launch.py` was archived alongside —
it launched the now-removed `ears_node` executable.

**Rationale & full plan:** `docs/sprint-change-proposal-2026-05-15.md`
(Expression Engine Re-scope, §4 three-layer split).
