"""HeartWidget — renders the lub-dub HeartModel as a per-emotion heart image.

The heart's *look* is a full-colour PNG that swaps by emotion (calm/neutral/
content uses `assets/heart_calm.png`); the *feel* is the model's bpm + amplitude.
Each frame the image is scaled by the beat (lub-dub throb) and faded in on the
first show (wake). No glow, no tint — colour lives in the image.

Story 8.4 calls `set_profile(image=..., bpm=..., amplitude=...)` once per emotion
(values composed from the expression map).
"""

from __future__ import annotations

import os

import pygame

from ..heart_beat import HeartModel

_ASSET_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "assets"))
DEFAULT_IMAGE = os.path.join(_ASSET_DIR, "heart_calm.png")  # calm / neutral / content
FIT_FRACTION = 0.85   # heart fills 85% of its cell at scale 1.0 (leaves a margin)


class HeartWidget:
    def __init__(self, model: HeartModel | None = None, image: str = DEFAULT_IMAGE):
        self.model = model or HeartModel()
        self._image_path = image
        self._sprite: pygame.Surface | None = None   # lazy load (needs a display)

    # -- inputs (Story 8.4 wiring) --
    def set_beat_rate(self, bpm: float) -> None:
        self.model.set_beat_rate(bpm)

    def set_amplitude(self, amplitude: float) -> None:
        self.model.set_amplitude(amplitude)

    def set_image(self, path: str) -> None:
        if path != self._image_path:
            self._image_path = path
            self._sprite = None   # reload on next draw

    def set_profile(self, image: str | None = None, bpm: float | None = None,
                    amplitude: float | None = None) -> None:
        """Apply an emotion profile in one call (image + feel)."""
        if image is not None:
            self.set_image(image)
        if bpm is not None:
            self.set_beat_rate(bpm)
        if amplitude is not None:
            self.set_amplitude(amplitude)

    def update(self, dt: float) -> None:
        self.model.update(dt)

    # -- render --
    def _ensure_sprite(self) -> pygame.Surface:
        if self._sprite is None:
            img = pygame.image.load(self._image_path).convert_alpha()
            # Crop to the non-transparent content so every emotion image fills
            # the cell by its actual heart, not its canvas — the source PNGs have
            # different amounts of transparent padding (e.g. calm 369x350 tight vs
            # sad/sleepy 450x450 padded), which otherwise renders some hearts small.
            bounds = img.get_bounding_rect()
            if bounds.width and bounds.height and bounds.size != img.get_size():
                img = img.subsurface(bounds).copy()
            self._sprite = img
        return self._sprite

    def draw(self, surface, rect) -> None:
        img = self._ensure_sprite()
        iw, ih = img.get_size()
        beat = self.model.scale
        fit = min(rect.w * FIT_FRACTION / iw, rect.h * FIT_FRACTION / ih) * beat
        size = (max(1, int(iw * fit)), max(1, int(ih * fit)))

        sprite = pygame.transform.smoothscale(img, size)
        wake = max(0.0, min(1.0, self.model.wake))
        if wake < 1.0:                       # fade the heart in on first show
            sprite.fill((255, 255, 255, int(255 * wake)), special_flags=pygame.BLEND_RGBA_MULT)

        surface.blit(sprite, sprite.get_rect(center=rect.center))
