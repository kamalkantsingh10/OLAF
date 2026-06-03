"""DashboardView — the default 2x2 portrait dashboard (AC #3).

Hosts four cells: heart (top-left, Story 8.2) + three log panels (Story 8.3).
For Story 8.1 the cells show labelled placeholders so the grid is visually
verifiable before the real widgets land. Widgets are injected per slot, so
8.2/8.3 drop in with no change here.
"""

from __future__ import annotations

import pygame

from ..layout import dashboard_slots
from ..view_manager import View
from ..widgets.heart import HeartWidget
from ..widgets.log_panel import LogPanel

# Dark base so the heart's warm glow (8.2) stays the focal point; logs cool/dim.
COLOR_BG = (12, 10, 14)
COLOR_SEPARATOR = (40, 36, 44)
COLOR_PLACEHOLDER = (90, 84, 96)
COLOR_CELL_BORDER = (28, 25, 32)


def _seed_lines(title: str) -> list[str]:
    return [f"{title.lower()} log started", "waiting for data…"]


class _DummyFeeder:
    """Placeholder data so the panels visibly scroll (Story 8.3).

    Replaced by real feeds in 8.4+ (which just call panel.push(...) instead).
    """

    def __init__(self, panels, interval: float = 1.3):
        self.panels = panels
        self.interval = interval
        self._t = 0.0
        self._n = 0

    def update(self, dt: float) -> None:
        self._t += dt
        if self._t >= self.interval:
            self._t = 0.0
            self._n += 1
            stamp = self._n * self.interval
            for panel in self.panels:
                panel.push(f"t+{stamp:5.1f}s  {panel.title.lower()} evt {self._n:03d}")


class DashboardView(View):
    name = "dashboard"

    def __init__(self, size: tuple[int, int] = (480, 800), widgets: dict | None = None):
        super().__init__(self.name)
        self.size = size
        self.slots = dashboard_slots(*size)
        # slot_name -> widget exposing draw(surface, rect) [+ optional update(dt)]
        self.widgets: dict = widgets or {}
        # The heart band gets the living heart widget by default (Story 8.2).
        if "heart" not in self.widgets:
            self.widgets["heart"] = HeartWidget()
        # The three log areas get placeholder log panels (Story 8.3).
        for slot, title in (("log_mid", "SYSTEM"), ("log_bl", "SPEECH"), ("log_br", "SENSORS")):
            if slot not in self.widgets:
                panel = LogPanel(title=title)
                panel.set_lines(_seed_lines(title))
                self.widgets[slot] = panel
        self._dummy = _DummyFeeder(
            [self.widgets[s] for s in ("log_mid", "log_bl", "log_br") if s in self.widgets]
        )
        self._font: pygame.font.Font | None = None

    def _ensure_font(self) -> None:
        if self._font is None:
            self._font = pygame.font.SysFont("monospace", 16)

    def set_widget(self, slot: str, widget) -> None:
        """Attach a widget to a slot (used by 8.2 heart / 8.3 log panels)."""
        if slot not in self.slots:
            raise KeyError(f"unknown dashboard slot: {slot!r}")
        self.widgets[slot] = widget

    def update(self, dt: float) -> None:
        self._dummy.update(dt)   # placeholder data (Story 8.3); real feeds in 8.4+
        for widget in self.widgets.values():
            if hasattr(widget, "update"):
                widget.update(dt)

    def draw(self, surface) -> None:
        self._ensure_font()
        surface.fill(COLOR_BG)
        for slot, rect in self.slots.items():
            widget = self.widgets.get(slot)
            if widget is not None:
                # Clip each widget to its cell so nothing spills into neighbours.
                prev = surface.get_clip()
                surface.set_clip(pygame.Rect(rect.as_tuple()))
                widget.draw(surface, rect)
                surface.set_clip(prev)
            else:
                self._draw_placeholder(surface, slot, rect)
        self._draw_separators(surface)

    def _draw_placeholder(self, surface, slot: str, rect) -> None:
        pygame.draw.rect(surface, COLOR_CELL_BORDER, rect.as_tuple(), width=1)
        label = self._font.render(slot, True, COLOR_PLACEHOLDER)
        surface.blit(label, (rect.x + 8, rect.y + 8))

    def _draw_separators(self, surface) -> None:
        w, h = self.size
        heart_bottom = self.slots["heart"].bottom     # under the heart band
        mid_bottom = self.slots["log_mid"].bottom      # under the full-width middle band
        # full-width line under the heart band
        pygame.draw.line(surface, COLOR_SEPARATOR, (0, heart_bottom), (w, heart_bottom), 2)
        # full-width line under the middle band
        pygame.draw.line(surface, COLOR_SEPARATOR, (0, mid_bottom), (w, mid_bottom), 2)
        # vertical divider between the two bottom quadrants only
        pygame.draw.line(surface, COLOR_SEPARATOR, (w // 2, mid_bottom), (w // 2, h), 2)
