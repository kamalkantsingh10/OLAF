"""FullscreenTakeoverView — stub proving the full-screen takeover path (AC #4).

A single app can occupy the whole 480x800 surface; the view-manager returns to
the dashboard when done. Story 8.5 replaces this stub with real takeover apps.
"""

from __future__ import annotations

import pygame

from ..view_manager import View

COLOR_BG = (6, 18, 24)
COLOR_TEXT = (120, 200, 220)


class FullscreenTakeoverView(View):
    name = "takeover"

    def __init__(self, size: tuple[int, int] = (480, 800), label: str = "FULLSCREEN TAKEOVER"):
        super().__init__(self.name)
        self.size = size
        self.label = label
        self._font: pygame.font.Font | None = None

    def draw(self, surface) -> None:
        if self._font is None:
            self._font = pygame.font.SysFont("monospace", 26)
        surface.fill(COLOR_BG)
        text = self._font.render(self.label, True, COLOR_TEXT)
        surface.blit(text, text.get_rect(center=(self.size[0] // 2, self.size[1] // 2)))
