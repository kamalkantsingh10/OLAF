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

from dataclasses import dataclass
from pathlib import Path

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
class EngineConfig:
    """Resolved engine configuration.

    Attributes:
        domain_id: DDS domain — must match the companion publisher's
            ``[publisher].dds_domain_id`` (Appendix A.1).
        topics: Mapping of canonical topic key -> ROS 2 topic name,
            keyed by every entry in :data:`REQUIRED_TOPICS`.
        animation: `[animation]` timing (Story 6.3) — defaults applied
            for any absent key.
    """

    domain_id: int
    topics: dict[str, str]
    animation: AnimationConfig = AnimationConfig()


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

    return EngineConfig(
        domain_id=domain_id,
        topics=topics,
        animation=_parse_animation(raw.get("animation")),
    )


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
        values[field_name] = float(raw_val)
    return AnimationConfig(**values)
