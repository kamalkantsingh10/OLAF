"""Portrait 2x2 grid geometry for the chest dashboard.

Pure integer math — NO pygame import — so it is unit-testable headlessly.
The panel is the 4.3" DSI (native 800x480, kernel-rotated to portrait via
`video=DSI-2:800x480,rotate=90`), so the app draws a 480x800 surface; each
of the four cells is 240x400. Heart goes top-left, log panels in the other
three (Stories 8.2 / 8.3).
"""

from dataclasses import dataclass

PORTRAIT_SIZE = (480, 800)


@dataclass(frozen=True)
class Rect:
    """An axis-aligned rectangle (pygame-free; convertible to a pygame.Rect tuple)."""

    x: int
    y: int
    w: int
    h: int

    @property
    def right(self) -> int:
        return self.x + self.w

    @property
    def bottom(self) -> int:
        return self.y + self.h

    @property
    def center(self) -> tuple[int, int]:
        return (self.x + self.w // 2, self.y + self.h // 2)

    def as_tuple(self) -> tuple[int, int, int, int]:
        """`(x, y, w, h)` — accepted directly by `pygame.Rect(...)` / `draw.rect`."""
        return (self.x, self.y, self.w, self.h)


def grid_cells(width: int, height: int, rows: int = 2, cols: int = 2) -> list[Rect]:
    """Return `rows*cols` cell rects in row-major order (TL, TR, BL, BR for 2x2)."""
    cell_w = width // cols
    cell_h = height // rows
    return [
        Rect(c * cell_w, r * cell_h, cell_w, cell_h)
        for r in range(rows)
        for c in range(cols)
    ]


HEART_BAND_FRACTION = 0.30  # heart = full-width band across the top 30%


def dashboard_slots(width: int = 480, height: int = 800) -> dict[str, Rect]:
    """Dashboard layout — 4 blocks: a full-width HEART band (top 30%), a
    full-width MIDDLE band below it, then two bottom quadrants.

        heart    -> full-width band, top 30%          (Story 8.2)
        log_mid  -> full-width band, middle row        (Story 8.3)
        log_bl   -> bottom-left quadrant               (Story 8.3)
        log_br   -> bottom-right quadrant              (Story 8.3)

    For 480x800: heart = 480x240; log_mid = 480x280; each bottom quadrant = 240x280.
    """
    band_h = int(height * HEART_BAND_FRACTION)   # 240 for 800
    rest = height - band_h                       # 560
    row_h = rest // 2                            # 280
    mid_y = band_h                               # 240
    bottom_y = band_h + row_h                    # 520
    cell_w = width // 2                          # 240
    return {
        "heart": Rect(0, 0, width, band_h),
        "log_mid": Rect(0, mid_y, width, row_h),
        "log_bl": Rect(0, bottom_y, cell_w, row_h),
        "log_br": Rect(cell_w, bottom_y, cell_w, row_h),
    }
