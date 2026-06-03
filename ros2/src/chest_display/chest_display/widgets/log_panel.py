"""LogPanel — a reusable titled, monospace, dim/cool log panel (Story 8.3).

`LogBuffer` is the pure, data-source-agnostic core (bounded ring of lines,
`push` / `set_lines`) — unit-testable headlessly. `LogPanel` renders it: a title
bar + a rule + the newest lines, truncated to width (terminal-tail: newest at the
bottom). Cool/dim palette so the heart stays the focal point.

Wiring real data later (Story 8.4+) = call `panel.push(line)` / `panel.set_lines(...)`;
no layout change.
"""

from __future__ import annotations

from collections import deque

import pygame

TITLE_COLOR = (110, 140, 150)   # cool, dim — subordinate to the heart
TEXT_COLOR = (120, 130, 140)
RULE_COLOR = (40, 46, 52)
ELLIPSIS = "…"


class LogBuffer:
    """Bounded line buffer (oldest -> newest)."""

    def __init__(self, capacity: int = 200):
        self.capacity = capacity
        self._lines: deque[str] = deque(maxlen=capacity)

    def push(self, line) -> None:
        self._lines.append(str(line))

    def set_lines(self, lines) -> None:
        self._lines.clear()
        self._lines.extend(str(x) for x in list(lines)[-self.capacity:])

    def lines(self) -> list[str]:
        return list(self._lines)

    def __len__(self) -> int:
        return len(self._lines)


class LogPanel:
    def __init__(self, title: str = "LOG", capacity: int = 200, font_size: int = 14):
        self.title = title
        self.buffer = LogBuffer(capacity)
        self.font_size = font_size
        self._font: pygame.font.Font | None = None
        self._title_font: pygame.font.Font | None = None

    # -- data API (real feeds call these in 8.4+) --
    def push(self, line) -> None:
        self.buffer.push(line)

    def set_lines(self, lines) -> None:
        self.buffer.set_lines(lines)

    def update(self, dt: float) -> None:   # display-only; data comes from outside
        pass

    # -- render --
    def _ensure_fonts(self) -> None:
        if self._font is None:
            self._font = pygame.font.SysFont("monospace", self.font_size)
            self._title_font = pygame.font.SysFont("monospace", max(10, self.font_size - 2), bold=True)

    def _truncate(self, text: str, max_chars: int) -> str:
        if len(text) <= max_chars:
            return text
        return text[: max(0, max_chars - 1)] + ELLIPSIS

    def draw(self, surface, rect) -> None:
        self._ensure_fonts()
        pad = 6
        x = rect.x + pad

        title = self._title_font.render(self.title, True, TITLE_COLOR)
        surface.blit(title, (x, rect.y + pad))
        rule_y = rect.y + pad + title.get_height() + 2
        pygame.draw.line(surface, RULE_COLOR, (rect.x + pad, rule_y), (rect.right - pad, rule_y), 1)

        line_h = self._font.get_height()
        area_top = rule_y + 4
        avail_h = (rect.bottom - pad) - area_top
        max_lines = max(0, avail_h // line_h)
        char_w = self._font.size("M")[0] or 8
        max_chars = max(4, (rect.w - 2 * pad) // char_w)

        visible = self.buffer.lines()[-max_lines:]   # newest tail
        for i, raw in enumerate(visible):
            text = self._truncate(raw, max_chars)
            surface.blit(self._font.render(text, True, TEXT_COLOR), (x, area_top + i * line_h))
