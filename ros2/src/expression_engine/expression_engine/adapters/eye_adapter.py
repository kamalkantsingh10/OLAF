"""Real eye DelegatingAdapter — Story 6.4 Task 3 (wraps HeadI2CClient).

Implements the frozen `DelegatingAdapter` Protocol (§4). The eyes are
a *smart peripheral* (AR1/§2): the engine sends a semantic
`set_expression` on change; the Head ESP32 owns its own 60 FPS
animation. The engine never ticks the eyes.

AR10 — the canonical→ESP32 vocabulary translation table lives HERE,
not in `expression_map.yaml` and not in the render loop. The map stays
canonical; swapping the eye display = rewriting THIS table only
(NFR6). The ESP32 firmware accepts exactly 7 expression strings
(`head_i2c_client.EXPRESSION_MAP`): neutral, happy, sad, surprised,
angry, sleepy, wink.
"""

from __future__ import annotations

import logging
from typing import Callable, Optional

from expression_engine.logging_setup import log_event

# Canonical name (speech_emotion 12 + activity eye states + defaults)
# → one of the ESP32's 7 strings. AR10: this is the ONLY place the
# hardware eye vocabulary appears. "Pick the closest" for the 7-string
# device; unknown → neutral (safe) + a logged warning.
_CANONICAL_TO_ESP32: dict[str, str] = {
    # ── speech_emotion (12 first-class, brief §A.6) ──
    "neutral": "neutral",
    "content": "happy",
    "excited": "happy",
    "sad": "sad",
    "angry": "angry",
    "scared": "surprised",
    "happy": "happy",          # ← reference expression (Story 6.4)
    "curious": "surprised",
    "sympathetic": "sad",
    "surprised": "surprised",
    "frustrated": "angry",
    "melancholic": "sad",
    # ── activity eye states (expression_map.yaml `activity.*.eye`) ──
    "boot": "neutral",
    "closed": "sleepy",
    "waking": "neutral",
    "open": "neutral",
    "focused": "neutral",
    "distant": "neutral",
    "animated": "happy",
    "closing": "sleepy",
}
_ESP32_FALLBACK = "neutral"


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
        # The Head ESP32 BOOTS ASLEEP and ignores set_expression until
        # explicitly woken (REG_SYSTEM_STATUS "woke_up"; known Pi/ESP32
        # boot-order gotcha). Without this the eyes stay closed.
        # Activity-driven sleep/wake mapping is Story 7.3; here we just
        # ensure the device is responsive for rendering.
        # Code review 2026-05-17: a failed/absent wake is a CONNECT
        # FAILURE, not an INFO. An unwoken head silently renders no
        # eyes — NFR7 says connect failure is fatal, so escalate
        # (raises out of node.connect_adapters → §9 step 5 fatal →
        # systemd restart) rather than starting blind.
        set_status = getattr(self._client, "set_system_status", None)
        if not callable(set_status):
            raise RuntimeError(
                "eye client has no set_system_status — cannot wake the "
                "Head ESP32 (it boots asleep and ignores set_expression)"
            )
        if not set_status("woke_up"):
            raise RuntimeError(
                "Head ESP32 wake (set_system_status 'woke_up') failed — "
                "the eyes would stay closed; failing fast (NFR7)"
            )
        log_event(logging.INFO, "eye_adapter_connected", woke_up=True)

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
