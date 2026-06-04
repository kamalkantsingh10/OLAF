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

from pathlib import Path

from .emotion_source import HeartDirector, load_heart_config
from .ros_link import EmotionLink
from .view_manager import ViewManager
from .views.dashboard import DashboardView
from .views.takeover import FullscreenTakeoverView
from .widgets.heart import _ASSET_DIR


def _default_map_path() -> str:
    """The engine's expression_map.yaml (source-tree layout); CHEST_HEART_MAP overrides.

    .../ros2/src/chest_display/chest_display/app.py -> .../ros2/src/expression_engine/config/...
    """
    return str(
        Path(__file__).resolve().parents[2]
        / "expression_engine" / "config" / "expression_map.yaml"
    )

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
        self._dashboard = DashboardView(self.portrait)
        self.vm.register(self._dashboard, default=True, activate=True)
        self.vm.register(FullscreenTakeoverView(self.portrait))

    # -- heart reactivity (Story 8.4) -----------------------------------
    def _setup_emotion(self) -> None:
        """Read the map's heart config + subscribe to activity/speech_emotion.

        All graceful: if the map or ROS is unavailable, the heart stays on its
        default calm beat (8.2 behaviour preserved)."""
        self._heart_widget = self._dashboard.widgets.get("heart")
        self._director = None
        self._link = None
        self._img_cache: dict[str, str] = {}

        map_path = os.environ.get("CHEST_HEART_MAP") or _default_map_path()
        try:
            cfg = load_heart_config(map_path)
            self._director = HeartDirector(cfg)
            _log("heart_config_loaded", path=str(map_path),
                 emotions=len(cfg.speech), activities=len(cfg.activity))
        except Exception as exc:
            _log("heart_config_unavailable", error=f"{type(exc).__name__}: {exc}")

        self._link = EmotionLink()
        if self._link.available:
            _log("emotion_link_ready")
        else:
            _log("emotion_link_unavailable", error=self._link.error)

    def _heart_image_path(self, name: str) -> str:
        if name not in self._img_cache:
            p = os.path.join(_ASSET_DIR, f"heart_{name}.png")
            self._img_cache[name] = p if os.path.exists(p) else os.path.join(
                _ASSET_DIR, "heart_calm.png"
            )
        return self._img_cache[name]

    def _drive_heart(self, dt: float) -> None:
        if self._director is None or self._heart_widget is None:
            return
        act = sub = emo = None
        if self._link is not None:
            self._link.poll()
            act, sub, emo = (
                self._link.activity_state,
                self._link.working_submode,
                self._link.speech_emotion,
            )
        name, bpm, amp = self._director.update(dt, act, sub, emo)
        self._heart_widget.set_profile(
            image=self._heart_image_path(name), bpm=bpm, amplitude=amp
        )

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
        self._setup_emotion()
        clock = pygame.time.Clock()
        self._running = True
        _log("running", fps=self.fps)
        try:
            while self._running:
                dt = clock.tick(self.fps) / 1000.0
                self._handle_events()
                self._drive_heart(dt)
                self.vm.update(dt)
                self.vm.draw(self.frame)
                self._present()
        finally:
            self._shutdown()

    def _shutdown(self) -> None:
        if getattr(self, "_link", None) is not None:
            self._link.close()
        _log("shutting_down")
        pygame.quit()


def main() -> None:
    ChestDisplayApp().run()


if __name__ == "__main__":
    main()
