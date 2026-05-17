"""Body-surface adapters for the expression engine.

`base.py` holds the structural Protocols (proposed in Story 6.3,
FROZEN in Story 6.4). Concrete hardware adapters land later: neck/ears
in 6.4, LED in 6.6, heart in 8.1. Story 6.3 ships only the Protocols
and recording test doubles so the render loop is testable without
hardware.
"""
