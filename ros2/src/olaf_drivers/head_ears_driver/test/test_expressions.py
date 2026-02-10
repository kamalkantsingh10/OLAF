"""Tests for ear expression presets."""

import pytest
from head_ears_driver.expressions import (
    get_preset,
    get_preset_by_name,
    PRESETS,
    PRESET_NAMES,
    EMOTION_NEUTRAL,
    EMOTION_HAPPY,
    EMOTION_SAD,
    EMOTION_EXCITED,
)

SERVO_KEYS = {"left_pan", "left_tilt", "right_pan", "right_tilt"}


class TestPresetLookup:
    """Test preset retrieval by type and name."""

    def test_all_presets_have_four_servos(self):
        for emotion_type, preset in PRESETS.items():
            assert set(preset.keys()) == SERVO_KEYS, f"Emotion {emotion_type} missing keys"

    def test_get_preset_neutral(self):
        angles = get_preset(EMOTION_NEUTRAL)
        assert angles is not None
        for v in angles.values():
            assert v == 0.0

    def test_get_preset_happy(self):
        angles = get_preset(EMOTION_HAPPY)
        assert angles is not None
        assert angles["left_pan"] > 0
        assert angles["left_tilt"] > 0

    def test_get_preset_sad(self):
        angles = get_preset(EMOTION_SAD)
        assert angles is not None
        assert angles["left_tilt"] < 0  # Ears tilt backward

    def test_get_preset_unknown_returns_none(self):
        assert get_preset(99) is None

    def test_get_preset_by_name(self):
        angles = get_preset_by_name("happy")
        assert angles is not None
        assert angles == get_preset(EMOTION_HAPPY)

    def test_get_preset_by_name_case_insensitive(self):
        assert get_preset_by_name("Happy") == get_preset_by_name("happy")

    def test_get_preset_by_name_unknown_returns_none(self):
        assert get_preset_by_name("nonexistent") is None

    def test_all_names_map_to_valid_presets(self):
        for name, emotion_type in PRESET_NAMES.items():
            assert emotion_type in PRESETS, f"Name '{name}' maps to missing preset"


class TestIntensityScaling:
    """Test intensity scaling of presets."""

    def test_intensity_zero_all_zeros(self):
        angles = get_preset(EMOTION_EXCITED, intensity=0.0)
        assert angles is not None
        for v in angles.values():
            assert v == 0.0

    def test_intensity_one_full_values(self):
        angles = get_preset(EMOTION_EXCITED, intensity=1.0)
        assert angles == PRESETS[EMOTION_EXCITED]

    def test_intensity_half_scales(self):
        full = get_preset(EMOTION_HAPPY, intensity=1.0)
        half = get_preset(EMOTION_HAPPY, intensity=0.5)
        for key in SERVO_KEYS:
            assert half[key] == pytest.approx(full[key] * 0.5)

    def test_intensity_clamped_above_one(self):
        angles = get_preset(EMOTION_HAPPY, intensity=1.5)
        assert angles == get_preset(EMOTION_HAPPY, intensity=1.0)

    def test_intensity_clamped_below_zero(self):
        angles = get_preset(EMOTION_HAPPY, intensity=-0.5)
        assert angles == get_preset(EMOTION_HAPPY, intensity=0.0)


class TestAngleBounds:
    """Test that presets stay within hardware limits."""

    PAN_MIN = 0
    PAN_MAX = 90
    TILT_MIN = -90
    TILT_MAX = 110

    def test_all_presets_within_pan_limits(self):
        for emotion_type, preset in PRESETS.items():
            for key in ["left_pan", "right_pan"]:
                assert self.PAN_MIN <= preset[key] <= self.PAN_MAX, (
                    f"Emotion {emotion_type} {key}={preset[key]} out of pan range"
                )

    def test_all_presets_within_tilt_limits(self):
        for emotion_type, preset in PRESETS.items():
            for key in ["left_tilt", "right_tilt"]:
                assert self.TILT_MIN <= preset[key] <= self.TILT_MAX, (
                    f"Emotion {emotion_type} {key}={preset[key]} out of tilt range"
                )
