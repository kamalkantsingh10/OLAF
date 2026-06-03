"""Headless tests for the 2x2 portrait grid layout (no pygame needed)."""

from chest_display.layout import (
    Rect,
    grid_cells,
    dashboard_slots,
    PORTRAIT_SIZE,
    HEART_BAND_FRACTION,
)


def test_portrait_size():
    assert PORTRAIT_SIZE == (480, 800)


def test_grid_cells_count_and_size():
    cells = grid_cells(480, 800, rows=2, cols=2)
    assert len(cells) == 4
    for c in cells:
        assert (c.w, c.h) == (240, 400)


def test_grid_cells_positions_row_major():
    tl, tr, bl, br = grid_cells(480, 800, 2, 2)
    assert (tl.x, tl.y) == (0, 0)
    assert (tr.x, tr.y) == (240, 0)
    assert (bl.x, bl.y) == (0, 400)
    assert (br.x, br.y) == (240, 400)


def test_grid_cells_tile_without_overlap():
    cells = grid_cells(480, 800, 2, 2)
    # Cells cover the whole surface...
    assert sum(c.w * c.h for c in cells) == 480 * 800
    # ...and are pairwise disjoint.
    for i, a in enumerate(cells):
        for b in cells[i + 1:]:
            overlap_x = max(0, min(a.right, b.right) - max(a.x, b.x))
            overlap_y = max(0, min(a.bottom, b.bottom) - max(a.y, b.y))
            assert overlap_x * overlap_y == 0


def test_dashboard_slots_four_blocks():
    slots = dashboard_slots(480, 800)
    assert set(slots) == {"heart", "log_mid", "log_bl", "log_br"}


def test_heart_is_full_width_top_band():
    slots = dashboard_slots(480, 800)
    # heart = full width, top 30% (= 240px)
    assert slots["heart"].as_tuple() == (0, 0, 480, 240)
    assert HEART_BAND_FRACTION == 0.30


def test_middle_is_full_width_band():
    slots = dashboard_slots(480, 800)
    # merged top quadrants -> one full-width middle band
    assert slots["log_mid"].as_tuple() == (0, 240, 480, 280)


def test_bottom_two_quadrants():
    slots = dashboard_slots(480, 800)
    assert slots["log_bl"].as_tuple() == (0, 520, 240, 280)
    assert slots["log_br"].as_tuple() == (240, 520, 240, 280)


def test_dashboard_slots_tile_full_surface():
    slots = dashboard_slots(480, 800)
    # heart band + middle band + 2 quadrants cover the whole 480x800, no overlap
    assert sum(r.w * r.h for r in slots.values()) == 480 * 800


def test_rect_helpers():
    r = Rect(10, 20, 30, 40)
    assert r.right == 40
    assert r.bottom == 60
    assert r.center == (25, 40)
    assert r.as_tuple() == (10, 20, 30, 40)
