"""OLAF expression engine — node lifecycle + startup wiring.

Story 6.1 Task 4 (AC #1, #4; FR1, FR3, AR3). Replaces the Epic-5.1
scaffold stub: this is the real left edge of the pipeline —
``subscribers -> schema(validate, fail-fast) -> state``.

AR3: a single rclpy executor thread services all four subscriptions
and writes the mutex-protected EngineState. The render thread is a
later story and is intentionally not built here.

FR3 subscribe-only invariant: ``ExpressionEngineNode`` wires config →
subscribers → state and creates NO publishers. Treat a publisher on
any of the four topics as a build-breaking defect (asserted by
test_subscribe_only.py, AC #4).

Startup is fail-fast (NFR7): a missing/invalid config, or a wire
``schema_version != 3`` at runtime, is fatal — structured journald
line + ``sys.exit(1)``; systemd restarts (symmetry with the pipeline).
"""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from expression_engine.config import EngineConfig, load_config
from expression_engine.logging_setup import log_event, setup_logging
from expression_engine.schema import SchemaVersionError
from expression_engine.adapters._testing import (
    NullContinuousAdapter,
    NullDelegatingAdapter,
)
from expression_engine.map_loader import ExpressionMap, load_expression_map
from expression_engine.render_loop import RenderLoop
from expression_engine.state import EngineState
from expression_engine.subscribers import create_subscriptions

_NECK_JOINTS = ("pan", "tilt", "roll")
_EARS_JOINTS = ("left_pan", "left_tilt", "right_pan", "right_tilt")

_CONFIG_BASENAME = "expression_engine.toml"
_MAP_BASENAME = "expression_map.yaml"


def _packaged_config(basename: str) -> Path:
    """Resolve a packaged ``config/`` file, dev-source-tree fallback.

    Installed (colcon) layout: ``share/expression_engine/config/``.
    Source/dev layout: ``<pkg>/config/`` next to this module's parent.
    """
    try:
        share = get_package_share_directory("expression_engine")
        candidate = Path(share) / "config" / basename
        if candidate.is_file():
            return candidate
    except Exception:  # not built/installed — dev source-tree run
        pass
    return Path(__file__).resolve().parents[1] / "config" / basename


def _default_config_path() -> Path:
    return _packaged_config(_CONFIG_BASENAME)


def _default_map_path() -> Path:
    return _packaged_config(_MAP_BASENAME)


class ExpressionEngineNode(Node):
    """Subscribe-only engine node (FR1, FR3).

    Wires the loaded config to the four validated subscriptions, all
    feeding a single mutex-protected :class:`EngineState`. Creates no
    publishers — subscribe-only is a structural invariant.
    """

    def __init__(
        self,
        config: EngineConfig,
        expression_map: ExpressionMap,
        neck=None,
        ears=None,
        eye=None,
    ) -> None:
        super().__init__("expression_engine")
        self.config = config
        # Validated at startup (§9 steps 2–4); the render loop
        # (Story 6.3) consumes it. Held here, not yet rendered.
        self.expression_map = expression_map
        self.state = EngineState()
        # Retain handles so subscriptions are not GC'd.
        self._subscriptions_held = create_subscriptions(
            self, config, self.state
        )
        # Adapters are injectable (Story 6.4). Default = NULL
        # placeholders so a plain `main()` is hardware-safe; the
        # end-to-end harness injects the real neck/ears/eye adapters.
        self._neck = neck or NullContinuousAdapter(_NECK_JOINTS)
        self._ears = ears or NullContinuousAdapter(_EARS_JOINTS)
        self._eye = eye or NullDelegatingAdapter()
        # Story 6.3: render loop on its OWN thread (AR3 — never the
        # rclpy executor).
        self.render_loop = RenderLoop(
            self.state,
            expression_map,
            config.animation,
            neck=self._neck,
            ears=self._ears,
            eye=self._eye,
        )

    def connect_adapters(self) -> None:
        """§9 step 5 — connect every adapter in sequence (fatal NFR7)."""
        self._neck.connect()
        self._ears.connect()
        self._eye.connect()

    def close_adapters(self) -> None:
        """Best-effort adapter teardown (always neutral-safe in finally)."""
        for a in (self._neck, self._ears, self._eye):
            try:
                a.close()
            except Exception as exc:  # never mask the original exit path
                log_event(
                    logging.ERROR,
                    "adapter_close_error",
                    adapter=type(a).__name__,
                    detail=str(exc),
                )


def main(args=None) -> None:
    setup_logging()
    # §9 startup sequence — every step fatal (NFR7). Step 1: toml.
    try:
        config = load_config(_default_config_path())
    except (FileNotFoundError, ValueError) as exc:
        # NFR7 — never start on silent defaults.
        log_event(
            logging.CRITICAL,
            "config_load_failed",
            error=type(exc).__name__,
            detail=str(exc),
        )
        sys.exit(1)

    # §9 steps 2–4: load map + assert vocabulary completeness (FR6)
    # + nod/shake visible_only (FR7). MapValidationError is a
    # ValueError — same fail-fast posture as config (Story 6.1).
    try:
        expression_map = load_expression_map(_default_map_path())
    except (FileNotFoundError, ValueError) as exc:
        log_event(
            logging.CRITICAL,
            "expression_map_load_failed",
            error=type(exc).__name__,
            detail=str(exc),
        )
        sys.exit(1)

    rclpy.init(args=args)
    node = ExpressionEngineNode(config, expression_map)
    log_event(
        logging.INFO,
        "engine_started",
        domain_id=config.domain_id,
        topics=list(config.topics.values()),
    )
    # §9 step 5 — adapter.connect() in sequence, fatal on failure.
    try:
        node.connect_adapters()
    except Exception as exc:
        log_event(
            logging.CRITICAL,
            "adapter_connect_failed",
            error=type(exc).__name__,
            detail=str(exc),
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    node.render_loop.start()  # §9 step 7 — render thread (AR3)
    exit_code = 0
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except SchemaVersionError as exc:
        # FR4 — fatal contract breach surfaced from a callback.
        log_event(
            logging.CRITICAL,
            "fatal_schema_version",
            found=exc.found,
            supported=exc.supported,
            source=exc.source,
        )
        exit_code = 1
    finally:
        node.render_loop.stop()
        node.close_adapters()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        # os._exit so the non-zero status is not masked by interpreter
        # teardown after rclpy shutdown; systemd restarts the unit.
        os._exit(exit_code)


if __name__ == "__main__":
    main()
