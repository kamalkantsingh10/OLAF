"""Real eye DelegatingAdapter — Story 6.4 Task 3 (wraps HeadI2CClient).

Implements the frozen `DelegatingAdapter` Protocol (§4). The eyes are
a *smart peripheral* (AR1/§2): the engine sends a semantic
`set_expression` on change; the Head ESP32 owns its own 60 FPS
animation. The engine never ticks the eyes.

AR10 — the canonical→ESP32 vocabulary translation table lives HERE,
not in `expression_map.yaml` and not in the render loop. The map stays
canonical; swapping the eye display = rewriting THIS table only
(NFR6).

Story 7.1a (owner-authorised AR10 freeze deviation, 2026-05-19): the
Head ESP32 firmware now renders all **12** canonical speech-emotions
distinctly (the finalised spec filled-cyan-shape visual language +
per-emotion blink contract), so this table is **1:1** for the 12
speech-emotions — the 12→7 squash is removed. Activity eye-states
still map onto the device set. Blink interval/duration/close-style is
a per-emotion firmware contract (the device owns blink timing — the
engine never sends blink timing; smart-peripheral AR1/§2); the
firmware blink table is pinned to the spec by a host test.
"""

from __future__ import annotations

import logging
from typing import Callable, Optional

from expression_engine.logging_setup import log_event

# Canonical name (speech_emotion 12 + activity eye states + defaults)
# → an ESP32 device-expression string. AR10: this is the ONLY place
# the hardware eye vocabulary appears. Story 7.1a: speech-emotions are
# 1:1 (12 distinct device renders, no squash). Unknown → neutral
# (safe) + a logged warning.
_CANONICAL_TO_ESP32: dict[str, str] = {
    # ── speech_emotion (12 first-class, brief §A.6) — 1:1, distinct ──
    "neutral": "neutral",
    "content": "content",
    "excited": "excited",
    "sad": "sad",
    "angry": "angry",
    "scared": "scared",
    "happy": "happy",          # ← reference expression (Story 6.4)
    "curious": "curious",
    "sympathetic": "sympathetic",
    "surprised": "surprised",
    "frustrated": "frustrated",
    "melancholic": "melancholic",
    # ── idle decay + mood eye (Story 6.5 / 7.2) ──
    # `sleepy` is a real device expression (EXPR_SLEEPY) emitted directly
    # as a canonical by the idle controller and by mood.sleepy.eye. It
    # was previously only a device VALUE (closed/closing → sleepy), so the
    # canonical fell back to neutral (the 2026-05-22 idle bug). 1:1 here.
    "sleepy": "sleepy",
    # ── activity eye states (expression_map.yaml `activity.*.eye`) ──
    # Map onto the device set; not required distinct.
    "boot": "sleepy",         # ← asleep at boot/startup (Story 7.3); agrees with firmware default
    "closed": "sleepy",
    "waking": "neutral",
    "open": "neutral",
    "focused": "neutral",
    "distant": "neutral",
    "animated": "happy",
    "closing": "sleepy",
}
_ESP32_FALLBACK = "neutral"

# AR10-style device translation (Story 7.3): canonical ActivityState →
# Head ESP32 system_status name (head_i2c_client.STATUS_MAP). The
# system_status drives the device's wake level (eye openness) AND the
# WS2812 strip pattern (lit only for listening/processing/speaking;
# dark for idle/woke_up/going_idle). `working` (both submodes) → the
# single PROCESSING status. The render loop sends this on activity
# change (fire-on-change); the map stays canonical (NFR6).
_ACTIVITY_TO_STATUS: dict[str, str] = {
    "starting": "idle",
    "sleeping": "idle",
    "waking": "woke_up",
    "listening": "listening",
    "working": "processing",
    "speaking": "speaking",
    "going_to_sleep": "going_idle",
}


def _default_client_factory():
    from head_ears_driver.head_i2c_client import HeadI2CClient

    return HeadI2CClient()


class EyeAdapter:
    """`DelegatingAdapter` over `HeadI2CClient` (semantic, fire-on-change)."""

    def __init__(
        self, client_factory: Optional[Callable[[], object]] = None
    ) -> None:
        self._factory = client_factory or _default_client_factory
        self._client = None

    def connect(self) -> None:
        self._client = self._factory()
        self._client.open()
        # BOOT = SLEEP MODE (Story 7.3): the Head ESP32 boots asleep and
        # we KEEP it asleep on connect — the engine drives wake via
        # activity→system_status (waking→woke_up). We send an explicit
        # 'idle' (NOT 'woke_up') so the head doesn't flash neutral/awake
        # on engine start; it stays in its sleepy boot state until the
        # first activity says otherwise.
        # The 'idle' write also doubles as the NFR7 connect check: a
        # failed/absent status write is a CONNECT FAILURE (the head isn't
        # reachable over I2C), so escalate (raises out of
        # node.connect_adapters → §9 step 5 fatal → systemd restart)
        # rather than starting blind.
        set_status = getattr(self._client, "set_system_status", None)
        if not callable(set_status):
            raise RuntimeError(
                "eye client has no set_system_status — cannot reach the "
                "Head ESP32 over I2C"
            )
        if not set_status("idle"):
            raise RuntimeError(
                "Head ESP32 not responding (set_system_status 'idle' "
                "failed) — failing fast (NFR7)"
            )
        log_event(logging.INFO, "eye_adapter_connected", boot_state="idle")

    def close(self) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None

    @staticmethod
    def translate(canonical_name: str) -> str:
        """Canonical → ESP32 string (AR10). Unknown → safe neutral."""
        esp = _CANONICAL_TO_ESP32.get(canonical_name)
        if esp is None:
            log_event(
                logging.WARNING,
                "eye.untranslated_canonical",
                canonical=canonical_name,
                fallback=_ESP32_FALLBACK,
            )
            return _ESP32_FALLBACK
        return esp

    @staticmethod
    def status_for_activity(state: str) -> Optional[str]:
        """ActivityState → Head system_status name (Story 7.3).

        Unknown state → None (the render loop simply sends nothing —
        the device holds its current status). `working` maps the same
        for both submodes (thinking/delegating → processing)."""
        return _ACTIVITY_TO_STATUS.get(state)

    def set_system_status(self, status: str) -> None:
        """Drive the Head ESP32 system_status (wake level + WS2812 strip).

        Story 7.3 — sent by the render loop on activity change. Unknown
        status names are handled (WARN + no-op) by the client."""
        if self._client is not None:
            self._client.set_system_status(status)

    def set_led_overlay(self, overlay: str) -> None:
        """Drive the WS2812 emotional colour wash (Story 7.3 / 6.6).

        Sent by the render loop on mood change — the mood's nearest tint
        bucket (none/warm/cool/hot/bright). A SEPARATE event from
        system_status; the firmware layers it on the active strip
        animation."""
        if self._client is not None:
            self._client.set_led_overlay(overlay)

    def set_expression(self, canonical_name: str, intensity: int) -> None:
        if self._client is None:
            return
        esp = self.translate(canonical_name)
        # ESP32 intensity is 1..5 (head_i2c_client clamps too).
        self._client.set_expression(esp, max(1, min(5, int(intensity))))

    def blink(self) -> None:
        if self._client is not None:
            self._client.trigger_blink()

    def look(self, x: int, y: int) -> None:
        if self._client is not None:
            self._client.set_look_direction(x, y)
