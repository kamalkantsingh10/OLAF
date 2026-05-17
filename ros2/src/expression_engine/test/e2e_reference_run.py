"""Story 6.4 Task 5 — reference expression end-to-end harness (AC#1).

Mock companion publisher → engine (real subscribers + render loop) →
REAL neck / ears / eye adapters. Proves the frozen schema + Protocols
on hardware by visibly rendering `speech_emotion=happy` within the
anticipatory window.

RUN ON THE ROBOT ONLY (drives real servos):

    ssh olaf.local
    cd ~/olaf
    PYTHONPATH=ros2/src/expression_engine:\
ros2/src/olaf_drivers/neck_driver:\
ros2/src/olaf_drivers/head_ears_driver:libs \
      ~/.local/bin/poetry run python \
      ros2/src/expression_engine/test/e2e_reference_run.py

Safety: adapters are driven to `neutral()` then `close()` in a
`finally` — Ctrl-C is safe. Stay within `config/servo-ids.yaml`
limits (the ears adapter also clamps right_pan ≤50°).

This is a manual hardware harness, not a CI test (no pytest
collection — filename is not `test_*`). AC#1's "visibly render" is a
human observation; the harness prints programmatic evidence to
accompany that observation.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from expression_engine.adapters.ears_adapter import EarsAdapter
from expression_engine.adapters.eye_adapter import EyeAdapter
from expression_engine.adapters.neck_adapter import NeckAdapter
from expression_engine.config import load_config
from expression_engine.map_loader import load_expression_map
from expression_engine.node import (
    ExpressionEngineNode,
    _default_config_path,
    _default_map_path,
)

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mock_publisher import HAPPY_REFERENCE_SEQUENCE, run_sequence  # noqa: E402


def main() -> None:
    config = load_config(_default_config_path())
    emap = load_expression_map(_default_map_path())
    happy = emap.speech_emotion["happy"]
    print("=== Story 6.4 reference run — expected `happy` target ===")
    print(f"  neck/ears pose : {happy['pose']}")
    print(f"  eye            : {happy['eye']} "
          f"→ ESP32 '{EyeAdapter.translate('happy')}'")
    print(f"  anticipatory ms: {config.animation.emotion_anticipatory_ms}")

    rclpy.init()
    node = ExpressionEngineNode(
        config,
        emap,
        neck=NeckAdapter(),
        ears=EarsAdapter(),
        eye=EyeAdapter(),
    )
    node.connect_adapters()
    node.render_loop.start()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Publish the reference sequence from a background thread on the
    # same DDS domain (stands in for the companion — FR14/FR9).
    pub = threading.Thread(
        target=run_sequence,
        kwargs={"sequence": HAPPY_REFERENCE_SEQUENCE,
                "gap_s": 1.0, "hold_s": 8.0},
        daemon=True,
    )
    pub.start()

    t_end = time.monotonic() + 14.0
    try:
        while time.monotonic() < t_end:
            executor.spin_once(timeout_sec=0.1)
        # Programmatic evidence to accompany the visual observation.
        rl = node.render_loop
        neck = {j: round(rl._a.value.get(j, 0.0)
                          + rl._m.value.get(j, 0.0)
                          + rl._s.value.get(j, 0.0), 2)
                for j in ("pan", "tilt", "roll")}
        ears = {j: round(rl._a.value.get(j, 0.0)
                          + rl._s.value.get(j, 0.0), 2)
                for j in ("left_pan", "left_tilt",
                          "right_pan", "right_tilt")}
        print("=== observed (engine-side eased targets) ===")
        print(f"  ticks rendered : {rl.tick_count}")
        print(f"  neck target    : {neck}")
        print(f"  ears target    : {ears}")
        print("  expected ears  ≈ left_pan20 left_tilt18 "
              "right_pan20 right_tilt18 (+speech overlay eased)")
        print("Visually confirm: head joyful chin-up, both ears perked "
              "forward, eyes 'happy'.  [AC#1 = human observation]")
    finally:
        node.render_loop.stop()
        # Safety: return to neutral before releasing the bus.
        try:
            node._neck.apply(node._neck.neutral())
            node._ears.apply(node._ears.neutral())
        except Exception:
            pass
        node.close_adapters()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
