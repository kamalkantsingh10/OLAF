"""OLAF chest-display app (Epic 8).

Standalone pygame/KMSDRM portrait dashboard on the 4.3" DSI chest panel:
an animated anatomical heart (Story 8.2) plus three log panels (Story 8.3),
with a view-manager (Story 8.1) for full-screen takeover.

Kept import-light: this package's __init__ pulls in NO pygame, so the pure
layout/view-manager logic is unit-testable headlessly.
"""
