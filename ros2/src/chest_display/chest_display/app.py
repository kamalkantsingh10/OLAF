"""Chest-display app entrypoint (AC #1, #2, #5, #6).

Opens a fullscreen KMSDRM surface on the DSI panel, runs a fixed-rate render
loop over the view-manager, and shuts down cleanly on SIGTERM so systemd can
restart it (AC #5).

Rotation safety (Saved Question #2): the panel is kernel-rotated to portrait,
but to be robust regardless of how KMSDRM presents the mode, the app always
draws to an offscreen 480x800 PORTRAIT canvas and then blits it to the actual
device surface — rotating only if the device surface comes back landscape.
The rotation direction is `CHEST_ROTATE_DEG` (default 90) so it can be flipped
on hardware without a code change.
"""

from __future__ import annotations

import json
import os
import signal
import time

import pygame

from .view_manager import ViewManager
from .views.dashboard import DashboardView
from .views.takeover import FullscreenTakeoverView

PORTRAIT_SIZE = (480, 800)
TARGET_FPS = 30


def _log(msg: str, **fields) -> None:
    """Structured single-line JSON to stdout -> journald (NFR8 style)."""
    record = {"ts": round(time.time(), 3), "comp": "chest_display", "msg": msg}
    record.update(fields)
    print(json.dumps(record), flush=True)


class ChestDisplayApp:
    def __init__(
        self,
        portrait: tuple[int, int] = PORTRAIT_SIZE,
        fps: int = TARGET_FPS,
        driver: str = "kmsdrm",
    ):
        self.portrait = portrait
        self.fps = fps
        self._running = False
        self._rotate_deg = int(os.environ.get("CHEST_ROTATE_DEG", "90"))
        # Default to KMSDRM unless the environment already set a driver
        # (e.g. "dummy" in CI, "x11" on a desktop for eyeballing).
        os.environ.setdefault("SDL_VIDEODRIVER", driver)

    # -- lifecycle -------------------------------------------------------
    def _init_display(self) -> None:
        pygame.init()
        try:
            # (0, 0) => use the panel's current mode; robust to 480x800 vs 800x480.
            self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        except pygame.error as exc:  # connect() must be fatal (NFR7)
            _log("display_init_failed", error=str(exc),
                 driver=os.environ.get("SDL_VIDEODRIVER"))
            raise SystemExit(2)
        pygame.mouse.set_visible(False)
        self.screen_size = self.screen.get_size()
        self.frame = pygame.Surface(self.portrait)  # offscreen portrait canvas
        self.device_is_landscape = self.screen_size[0] > self.screen_size[1]
        _log(
            "display_ready",
            driver=pygame.display.get_driver(),
            screen=list(self.screen_size),
            portrait=list(self.portrait),
            device_is_landscape=self.device_is_landscape,
            rotate_deg=self._rotate_deg,
        )

    def _install_signals(self) -> None:
        signal.signal(signal.SIGTERM, self._handle_stop)
        signal.signal(signal.SIGINT, self._handle_stop)

    def _handle_stop(self, *_args) -> None:
        _log("stop_signal")
        self._running = False

    def _build_views(self) -> None:
        self.vm = ViewManager()
        self.vm.register(DashboardView(self.portrait), default=True, activate=True)
        self.vm.register(FullscreenTakeoverView(self.portrait))

    # -- frame presentation ---------------------------------------------
    def _present(self) -> None:
        """Blit the portrait canvas onto the device surface, rotating if needed."""
        if self.device_is_landscape:
            rotated = pygame.transform.rotate(self.frame, self._rotate_deg)
            dest = rotated.get_rect(
                center=(self.screen_size[0] // 2, self.screen_size[1] // 2)
            )
            self.screen.blit(rotated, dest)
        elif self.frame.get_size() == self.screen_size:
            self.screen.blit(self.frame, (0, 0))
        else:
            self.screen.blit(pygame.transform.smoothscale(self.frame, self.screen_size), (0, 0))
        pygame.display.flip()

    def _handle_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self._running = False
                elif event.key == pygame.K_t:            # demo: enter takeover
                    self.vm.switch_to("takeover")
                elif event.key == pygame.K_d:            # demo: back to dashboard
                    self.vm.return_to_dashboard()

    # -- main loop -------------------------------------------------------
    def run(self) -> None:
        self._init_display()
        self._install_signals()
        self._build_views()
        clock = pygame.time.Clock()
        self._running = True
        _log("running", fps=self.fps)
        try:
            while self._running:
                dt = clock.tick(self.fps) / 1000.0
                self._handle_events()
                self.vm.update(dt)
                self.vm.draw(self.frame)
                self._present()
        finally:
            self._shutdown()

    def _shutdown(self) -> None:
        _log("shutting_down")
        pygame.quit()


def main() -> None:
    ChestDisplayApp().run()


if __name__ == "__main__":
    main()
