"""OLAF expression engine — node entry point (scaffold).

This package was created by Epic 5.1 of the Phase 2 re-scope
(docs/sprint-change-proposal-2026-05-15.md). It is intentionally a
*scaffold only*: the real engine — subscriber + schema-3 validation,
expression_map.yaml loader, animation loop, idle behaviour, adapters —
is Epic 6, which is GATED behind Epic 5's driver-verification step.

Running this now logs that fact and exits cleanly rather than pretending
to be a working engine.
"""

import rclpy
from rclpy.node import Node


class ExpressionEngineNode(Node):
    def __init__(self) -> None:
        super().__init__("expression_engine")
        self.get_logger().warn(
            "expression_engine is a scaffold (Epic 5.1). The engine itself "
            "is Epic 6, gated behind Epic 5 driver verification. "
            "See docs/sprint-change-proposal-2026-05-15.md."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExpressionEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
