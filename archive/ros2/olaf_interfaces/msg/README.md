# Archived message types — superseded 2026-05-15

These four message definitions were the pre-contract, per-module expression
surface. They are **archived, not deleted** — preserved here for history and
in case any of the angle conventions are useful as reference during Phase 2.

| File | Why archived | Replaced by |
|---|---|---|
| `Expression.msg` | 7-emotion enum, incompatible with the companion's 12 canonical first-class emotions / 8 moods / 7 activity states / 6 vocalizations | `expression_map.yaml` keyed on canonical names (Phase 2) |
| `Gesture.msg` | Raw neck+ear angle public surface | Expression engine calls drivers in-process; no intermediate ROS topic |
| `EarsPose.msg` | Raw ear angle public surface | Same as Gesture |
| `HeartRate.msg` | Standalone heart-rate animation control | Mood/activity-driven heart animation (Phase 2, Epic 8) |

`ModuleStatus.msg` was **kept** in `ros2/src/olaf_interfaces/msg/` — health
monitoring is orthogonal to expression.

**Rationale & full plan:** `docs/sprint-change-proposal-2026-05-15.md`
(Expression Engine Re-scope).
