"""Ear expression presets for OLAF.

Each preset defines angles in degrees for all 4 ear servos.
Pan: 0 = center (straight up), positive = outward (max 90°).
Tilt: 0 = center, positive = backward, negative = forward.
Hardware limits: left_tilt [-90,+20], right_tilt [-7,+110], right_pan [0,70].

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
EMOTION_ANGRY = 7
EMOTION_SURPRISED = 8
EMOTION_SLEEPY = 9

# Doberman-style ear presets inspired by animated dog expression reference.
# Pan: outward splay (0=straight up, +90=full airplane).
# Tilt: forward/back pitch (+=forward, -=backward).
PRESETS: dict[int, dict[str, float]] = {
    EMOTION_NEUTRAL: {  # Ears straight up, calm and relaxed
        "left_pan": 0, "left_tilt": 0, "right_pan": 0, "right_tilt": 0,
    },
    EMOTION_HAPPY: {  # Both ears perked forward — joyful greeting
        "left_pan": 20, "left_tilt": 18, "right_pan": 20, "right_tilt": 18,
    },
    EMOTION_CURIOUS: {  # Asymmetric: left ear up + forward, right cocked wide out
        "left_pan": 5, "left_tilt": 20, "right_pan": 55, "right_tilt": 12,
    },
    EMOTION_THINKING: {  # Both ears up and scanning forward, attentive
        "left_pan": 8, "left_tilt": 18, "right_pan": 12, "right_tilt": 12,
    },
    EMOTION_CONFUSED: {  # Strong asymmetry — one fully up, one completely splayed out
        "left_pan": 0, "left_tilt": 20, "right_pan": 55, "right_tilt": 5,
    },
    EMOTION_SAD: {  # Full airplane droop — both ears hanging wide and down
        "left_pan": 80, "left_tilt": 30, "right_pan": 55, "right_tilt": 30,
    },
    EMOTION_EXCITED: {  # Both ears fully forward, high energy ready-to-play
        "left_pan": 15, "left_tilt": 20, "right_pan": 15, "right_tilt": 20,
    },
    EMOTION_ANGRY: {  # Ears pinned flat back — aggressive, tilted forward/down
        "left_pan": 15, "left_tilt": 30, "right_pan": 10, "right_tilt": 30,
    },
    EMOTION_SURPRISED: {  # Ears swept wide — startled, drooping outward
        "left_pan": 55, "left_tilt": 25, "right_pan": 50, "right_tilt": 25,
    },
    EMOTION_SLEEPY: {  # Heavy droop — ears hanging wide and down
        "left_pan": 65, "left_tilt": 30, "right_pan": 50, "right_tilt": 30,
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
    "angry": EMOTION_ANGRY,
    "surprised": EMOTION_SURPRISED,
    "sleepy": EMOTION_SLEEPY,
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
