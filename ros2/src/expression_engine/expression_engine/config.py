"""Engine configuration loader — Story 6.1 Task 1 (AC #1, NFR7).

Loads `expression_engine.toml` (architecture §8 / companion brief
Appendix B.1). Story 6.1 only needs `[dds].domain_id` and the four
`[topics]` names; later sections are parsed lazily by their own
stories' consumers and intentionally NOT validated here.

Fail-fast posture (NFR7): a missing file, unparseable TOML, or a
missing required key is fatal — the engine must never start on silent
defaults that would mask a contract/config mismatch.

This is a small SRP leaf module supporting `node.py`'s startup
sequence; the architecture §3 core modules (node / schema /
subscribers / state) are unchanged.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

from expression_engine.logging_setup import log_event

try:  # Python 3.11+ (Jazzy / Ubuntu 24.04 runs 3.12)
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - pyproject pins ^3.10 floor
    import tomli as tomllib  # type: ignore[no-redef]

#: The four canonical topics the engine subscribes to (companion
#: Appendix A.2). Order is irrelevant; membership is the contract.
REQUIRED_TOPICS = ("mood", "activity", "speech_emotion", "vocalization")


@dataclass(frozen=True)
class AnimationConfig:
    """`[animation]` timing — Story 6.3 (architecture §8).

    Unlike `[dds]`/`[topics]` (strict, fail-fast), animation timing is
    tunable with sane defaults: a missing section/key falls back to the
    §8 default rather than failing startup ("default … configurable").
    All seconds/ms/Hz are floats so they stay tunable without code.
    """

    servo_tick_hz: float = 100.0
    led_tick_hz: float = 30.0
    mood_ease_seconds: float = 3.0
    emotion_anticipatory_ms: float = 50.0
    gesture_attack_ms: float = 80.0
    gesture_settle_ms: float = 200.0


@dataclass(frozen=True)
class ListeningConfig:
    """`[listening]` ambient motion — WALL-E side-to-side head cock.

    While the activity is ``listening`` the neck cocks left↔right (ROLL
    axis). Each swing samples a RANDOM amplitude (large range), a RANDOM
    ease (speed) and a random period, so it never looks mechanical.
    Neck-only — no ears. ``roll_amp_max_deg = 0`` disables it.
    """

    roll_amp_min_deg: float = 3.0    # smallest cock (deg)
    roll_amp_max_deg: float = 14.0   # largest cock (deg; roll clamp is ±15)
    roll_period_min_s: float = 4.0   # min time before swinging to the other side
    roll_period_max_s: float = 10.0  # max time before swinging
    roll_ease_min_s: float = 0.8     # fastest swing (quick cock)
    roll_ease_max_s: float = 3.0     # slowest swing (slow drift)


@dataclass(frozen=True)
class SpeakingConfig:
    """`[speaking]` ambient "talking motion" (Story 7.6).

    Subtle procedural head movement while the activity is ``speaking`` so
    OLAF doesn't look frozen mid-reply: small random nods (neck TILT) +
    slight glances (neck PAN) + ear perk-twitches, each re-sampled every
    ``period_*`` seconds with a random ease. Layered on the speaking pose.
    Procedural, not audio-synced. Set all ``*_amp_max_deg`` to 0 to
    disable.
    """

    tilt_amp_min_deg: float = 1.5    # smallest nod (neck tilt)
    tilt_amp_max_deg: float = 4.0    # largest nod
    pan_amp_min_deg: float = 1.0     # smallest glance (neck pan)
    pan_amp_max_deg: float = 3.0     # largest glance
    ear_amp_min_deg: float = 2.0     # smallest ear perk-twitch (ear tilt)
    ear_amp_max_deg: float = 6.0     # largest ear perk-twitch
    period_min_s: float = 1.0        # min time between moves (livelier than listening)
    period_max_s: float = 3.0        # max time between moves
    ease_min_s: float = 0.4          # fastest move
    ease_max_s: float = 1.0          # slowest move


@dataclass(frozen=True)
class IdleConfig:
    """`[idle]` drift-to-sleep decay — Story 6.5 (architecture §7/§8).

    When idle (not speaking/processing), the whole head decays toward the
    `sleeping` pose along a fixed expression path at a random per-step
    pace, with sub-degree mood-modulated micro-movement, then loops at
    deep sleep so it's never statue-still. Tunable, sane defaults.
    """

    idle_after_seconds: float = 60.0     # quiet time before decay starts (listening etc.)
    step_min_seconds: float = 10.0       # random per-step pace, min
    step_max_seconds: float = 30.0       # random per-step pace, max
    ease_seconds: float = 2.0            # gentle per-step transition (dreamy, not a jerk)
    drift_amplitude_deg: float = 0.6     # sub-degree micro-movement amplitude
    drift_period_seconds: float = 4.0    # breath-like micro-movement period


@dataclass(frozen=True)
class EngineConfig:
    """Resolved engine configuration.

    Attributes:
        domain_id: DDS domain — must match the companion publisher's
            ``[publisher].dds_domain_id`` (Appendix A.1).
        topics: Mapping of canonical topic key -> ROS 2 topic name,
            keyed by every entry in :data:`REQUIRED_TOPICS`.
        animation: `[animation]` timing (Story 6.3) — defaults applied
            for any absent key.
        idle: `[idle]` drift-to-sleep tuning (Story 6.5).
    """

    domain_id: int
    topics: dict[str, str]
    animation: AnimationConfig = AnimationConfig()
    idle: IdleConfig = IdleConfig()
    listening: ListeningConfig = ListeningConfig()
    speaking: SpeakingConfig = SpeakingConfig()


def load_config(path: str | Path) -> EngineConfig:
    """Load and validate the engine config, failing fast on any defect.

    Raises:
        FileNotFoundError: the config file does not exist.
        ValueError: the file is not valid TOML, or a required
            ``[dds].domain_id`` / ``[topics].<name>`` key is missing or
            mistyped. Wrapped as ``ValueError`` so callers have a single
            fail-fast exception type for config defects (NFR7).
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"expression_engine config not found: {path}")

    try:
        with path.open("rb") as fh:
            raw = tomllib.load(fh)
    except tomllib.TOMLDecodeError as exc:
        raise ValueError(f"invalid TOML in {path}: {exc}") from exc

    dds = raw.get("dds")
    if not isinstance(dds, dict) or "domain_id" not in dds:
        raise ValueError(f"{path}: missing required [dds].domain_id")
    domain_id = dds["domain_id"]
    if not isinstance(domain_id, int) or isinstance(domain_id, bool):
        raise ValueError(
            f"{path}: [dds].domain_id must be an int, got {domain_id!r}"
        )
    # DDS domain IDs are 0..232 (code review 2026-05-17). An
    # out-of-range value otherwise surfaces as an obscure rmw error or
    # — worse — the engine silently joins no domain and hears nothing
    # while looking alive. Fail fast (NFR7).
    if not 0 <= domain_id <= 232:
        raise ValueError(
            f"{path}: [dds].domain_id must be 0..232, got {domain_id}"
        )

    topics_section = raw.get("topics")
    if not isinstance(topics_section, dict):
        raise ValueError(f"{path}: missing required [topics] section")

    topics: dict[str, str] = {}
    for key in REQUIRED_TOPICS:
        name = topics_section.get(key)
        if not isinstance(name, str) or not name:
            raise ValueError(
                f"{path}: [topics].{key} missing or not a non-empty string"
            )
        topics[key] = name

    # NFR7 — surface a mistyped section/key (e.g. [animatoin]) rather
    # than silently running on defaults (code review 2026-05-17).
    unknown = sorted(set(raw) - _KNOWN_TOP_LEVEL)
    if unknown:
        log_event(
            logging.WARNING,
            "config_unknown_top_level_keys",
            keys=unknown,
            hint="typo? these tables/keys are ignored",
        )

    return EngineConfig(
        domain_id=domain_id,
        topics=topics,
        animation=_parse_animation(raw.get("animation")),
        idle=_parse_idle(raw.get("idle")),
        listening=_parse_listening(raw.get("listening")),
        speaking=_parse_speaking(raw.get("speaking")),
    )


#: Recognised top-level toml keys (architecture §8). Anything else is
#: likely a typo and is warned about (NFR7 no-silent-misconfig).
_KNOWN_TOP_LEVEL = {
    "schema_version",
    "dds",
    "topics",
    "hardware",
    "animation",
    "idle",
    "listening",
    "speaking",
}


def _parse_animation(section: object) -> AnimationConfig:
    """Parse `[animation]` leniently — defaults for any absent/blank key.

    A present-but-mistyped value is the one fatal case (a config typo
    must not silently fall back to a default that masks it, NFR7).
    """
    if section is None:
        return AnimationConfig()
    if not isinstance(section, dict):
        raise ValueError("[animation] must be a TOML table")
    defaults = AnimationConfig()
    values: dict[str, float] = {}
    for field_name in (
        "servo_tick_hz",
        "led_tick_hz",
        "mood_ease_seconds",
        "emotion_anticipatory_ms",
        "gesture_attack_ms",
        "gesture_settle_ms",
    ):
        if field_name not in section:
            values[field_name] = getattr(defaults, field_name)
            continue
        raw_val = section[field_name]
        if isinstance(raw_val, bool) or not isinstance(raw_val, (int, float)):
            raise ValueError(
                f"[animation].{field_name} must be a number, "
                f"got {raw_val!r}"
            )
        if raw_val <= 0:
            raise ValueError(
                f"[animation].{field_name} must be > 0, got {raw_val!r}"
            )
        # Sane upper bounds (code review 2026-05-17). Without these,
        # servo_tick_hz=0.001 → ~1000s/tick (engine alive but frozen)
        # and servo_tick_hz=1e5 → busy-spin at 100% CPU. The most
        # safety-relevant tunable must be range-checked (NFR7).
        lo, hi = _ANIM_BOUNDS[field_name]
        if not lo <= raw_val <= hi:
            raise ValueError(
                f"[animation].{field_name} must be in [{lo}, {hi}], "
                f"got {raw_val!r}"
            )
        values[field_name] = float(raw_val)
    return AnimationConfig(**values)


#: Sane operating ranges for the tunable timing values.
_ANIM_BOUNDS = {
    "servo_tick_hz": (1.0, 1000.0),
    "led_tick_hz": (1.0, 240.0),
    "mood_ease_seconds": (0.05, 60.0),
    "emotion_anticipatory_ms": (0.0, 1000.0),
    "gesture_attack_ms": (1.0, 5000.0),
    "gesture_settle_ms": (1.0, 10000.0),
}

#: Sane operating ranges for `[idle]` (Story 6.5). Lower bounds allow 0
#: where it's meaningful (idle_after_seconds=0 → decay immediately;
#: drift_amplitude_deg=0 → no micro-movement).
_IDLE_BOUNDS = {
    "idle_after_seconds": (0.0, 3600.0),
    "step_min_seconds": (0.1, 600.0),
    "step_max_seconds": (0.1, 600.0),
    "ease_seconds": (0.1, 30.0),
    "drift_amplitude_deg": (0.0, 5.0),
    "drift_period_seconds": (0.5, 60.0),
}


_LISTENING_BOUNDS = {
    "roll_amp_min_deg": (0.0, 15.0),   # set both to 0 to disable; ±15 ceiling
    "roll_amp_max_deg": (0.0, 15.0),
    "roll_period_min_s": (0.5, 120.0),
    "roll_period_max_s": (0.5, 120.0),
    "roll_ease_min_s": (0.1, 30.0),
    "roll_ease_max_s": (0.1, 30.0),
}


def _parse_listening(section: object) -> ListeningConfig:
    """Parse `[listening]` leniently — defaults for any absent key; a
    present-but-mistyped/out-of-range value is fatal (NFR7)."""
    if section is None:
        return ListeningConfig()
    if not isinstance(section, dict):
        raise ValueError("[listening] must be a TOML table")
    defaults = ListeningConfig()
    values: dict[str, float] = {}
    for field_name in _LISTENING_BOUNDS:
        if field_name not in section:
            values[field_name] = getattr(defaults, field_name)
            continue
        raw_val = section[field_name]
        if isinstance(raw_val, bool) or not isinstance(raw_val, (int, float)):
            raise ValueError(
                f"[listening].{field_name} must be a number, got {raw_val!r}"
            )
        lo, hi = _LISTENING_BOUNDS[field_name]
        if not lo <= raw_val <= hi:
            raise ValueError(
                f"[listening].{field_name} must be in [{lo}, {hi}], got {raw_val!r}"
            )
        values[field_name] = float(raw_val)
    for lo_key, hi_key in (
        ("roll_amp_min_deg", "roll_amp_max_deg"),
        ("roll_period_min_s", "roll_period_max_s"),
        ("roll_ease_min_s", "roll_ease_max_s"),
    ):
        if values[lo_key] > values[hi_key]:
            raise ValueError(
                f"[listening].{lo_key} ({values[lo_key]}) must be "
                f"<= {hi_key} ({values[hi_key]})"
            )
    return ListeningConfig(**values)


_SPEAKING_BOUNDS = {
    "tilt_amp_min_deg": (0.0, 20.0),
    "tilt_amp_max_deg": (0.0, 20.0),
    "pan_amp_min_deg": (0.0, 30.0),
    "pan_amp_max_deg": (0.0, 30.0),
    "ear_amp_min_deg": (0.0, 40.0),
    "ear_amp_max_deg": (0.0, 40.0),
    "period_min_s": (0.2, 60.0),
    "period_max_s": (0.2, 60.0),
    "ease_min_s": (0.1, 30.0),
    "ease_max_s": (0.1, 30.0),
}


def _parse_speaking(section: object) -> SpeakingConfig:
    """Parse `[speaking]` leniently — defaults for any absent key; a
    present-but-mistyped/out-of-range value is fatal (NFR7)."""
    if section is None:
        return SpeakingConfig()
    if not isinstance(section, dict):
        raise ValueError("[speaking] must be a TOML table")
    defaults = SpeakingConfig()
    values: dict[str, float] = {}
    for field_name in _SPEAKING_BOUNDS:
        if field_name not in section:
            values[field_name] = getattr(defaults, field_name)
            continue
        raw_val = section[field_name]
        if isinstance(raw_val, bool) or not isinstance(raw_val, (int, float)):
            raise ValueError(
                f"[speaking].{field_name} must be a number, got {raw_val!r}"
            )
        lo, hi = _SPEAKING_BOUNDS[field_name]
        if not lo <= raw_val <= hi:
            raise ValueError(
                f"[speaking].{field_name} must be in [{lo}, {hi}], got {raw_val!r}"
            )
        values[field_name] = float(raw_val)
    for lo_key, hi_key in (
        ("tilt_amp_min_deg", "tilt_amp_max_deg"),
        ("pan_amp_min_deg", "pan_amp_max_deg"),
        ("ear_amp_min_deg", "ear_amp_max_deg"),
        ("period_min_s", "period_max_s"),
        ("ease_min_s", "ease_max_s"),
    ):
        if values[lo_key] > values[hi_key]:
            raise ValueError(
                f"[speaking].{lo_key} ({values[lo_key]}) must be "
                f"<= {hi_key} ({values[hi_key]})"
            )
    return SpeakingConfig(**values)


def _parse_idle(section: object) -> IdleConfig:
    """Parse `[idle]` leniently — defaults for any absent key; a
    present-but-mistyped/out-of-range value is fatal (NFR7)."""
    if section is None:
        return IdleConfig()
    if not isinstance(section, dict):
        raise ValueError("[idle] must be a TOML table")
    defaults = IdleConfig()
    values: dict[str, float] = {}
    for field_name in _IDLE_BOUNDS:
        if field_name not in section:
            values[field_name] = getattr(defaults, field_name)
            continue
        raw_val = section[field_name]
        if isinstance(raw_val, bool) or not isinstance(raw_val, (int, float)):
            raise ValueError(
                f"[idle].{field_name} must be a number, got {raw_val!r}"
            )
        lo, hi = _IDLE_BOUNDS[field_name]
        if not lo <= raw_val <= hi:
            raise ValueError(
                f"[idle].{field_name} must be in [{lo}, {hi}], got {raw_val!r}"
            )
        values[field_name] = float(raw_val)
    if values["step_min_seconds"] > values["step_max_seconds"]:
        raise ValueError(
            f"[idle].step_min_seconds ({values['step_min_seconds']}) must be "
            f"<= step_max_seconds ({values['step_max_seconds']})"
        )
    return IdleConfig(**values)
