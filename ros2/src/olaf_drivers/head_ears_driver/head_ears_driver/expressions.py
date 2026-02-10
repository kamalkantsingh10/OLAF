"""Ear expression presets for OLAF.

Each preset defines angles in degrees for all 4 ear servos.
Pan: 0 = center (straight up), positive = outward (max 90).
Tilt: 0 = center, negative = backward, positive = forward (-90 to +110).

Intensity (0.0-1.0) scales all angles proportionally.
"""

# Emotion type constants matching olaf_interfaces/msg/Expression.msg
EMOTION_NEUTRAL = 0
EMOTION_HAPPY = 1
EMOTION_CURIOUS = 2
EMOTION_THINKING = 3
EMOTION_CONFUSED = 4
EMOTION_SAD = 5
EMOTION_EXCITED = 6

# Doberman-style ear presets.
# Pan: outward splay (0=straight up, +90=full airplane).
# Tilt: forward/back pitch (+=forward, -=backward).
# Hardware limits: left_tilt [-90,+20], right_tilt [-7,+110], right_pan [0,70].
PRESETS: dict[int, dict[str, float]] = {
    EMOTION_NEUTRAL: {
        "left_pan": 0, "left_tilt": 0, "right_pan": 0, "right_tilt": 0,
    },
    EMOTION_HAPPY: {  # Doberman greeting: ears pulled back + wide splay
        "left_pan": 50, "left_tilt": -7, "right_pan": 50, "right_tilt": -7,
    },
    EMOTION_CURIOUS: {  # Asymmetric, tilted forward, one ear cocked
        "left_pan": 15, "left_tilt": 20, "right_pan": 35, "right_tilt": 20,
    },
    EMOTION_THINKING: {  # Subtle asymmetric forward scan
        "left_pan": 10, "left_tilt": 15, "right_pan": 25, "right_tilt": 10,
    },
    EMOTION_CONFUSED: {  # Strong asymmetry — one up, one way out
        "left_pan": 5, "left_tilt": -7, "right_pan": 65, "right_tilt": 20,
    },
    EMOTION_SAD: {  # Full airplane droop — wide splay, backward where possible
        "left_pan": 70, "left_tilt": -40, "right_pan": 65, "right_tilt": -7,
    },
    EMOTION_EXCITED: {  # Perked forward + moderate splay, ready to play
        "left_pan": 30, "left_tilt": 20, "right_pan": 30, "right_tilt": 20,
    },
}

# String-based lookup (for /ears/emote topic convenience)
PRESET_NAMES: dict[str, int] = {
    "neutral": EMOTION_NEUTRAL,
    "happy": EMOTION_HAPPY,
    "curious": EMOTION_CURIOUS,
    "thinking": EMOTION_THINKING,
    "confused": EMOTION_CONFUSED,
    "sad": EMOTION_SAD,
    "excited": EMOTION_EXCITED,
}


def get_preset(emotion_type: int, intensity: float = 1.0) -> dict[str, float] | None:
    """Get ear angles for an emotion, scaled by intensity.

    Args:
        emotion_type: Emotion constant (0-6).
        intensity: Scale factor 0.0-1.0 (1.0 = full expression).

    Returns:
        Dict with left_pan, left_tilt, right_pan, right_tilt angles, or None if unknown.
    """
    preset = PRESETS.get(emotion_type)
    if preset is None:
        return None
    intensity = max(0.0, min(1.0, intensity))
    return {k: v * intensity for k, v in preset.items()}


def get_preset_by_name(name: str, intensity: float = 1.0) -> dict[str, float] | None:
    """Get ear angles by emotion name string."""
    emotion_type = PRESET_NAMES.get(name.lower())
    if emotion_type is None:
        return None
    return get_preset(emotion_type, intensity)
