"""Story 6.4 Tasks 2/3 — real adapters (hardware-free).

Driver/client construction is injected, so the Protocol↔driver
mapping, the ears safety clamp, and the AR10 eye translation table are
verified without a serial port or I2C bus. The end-to-end on real
hardware is Task 5.
"""

import logging

import pytest

from expression_engine.adapters.base import (
    ContinuousAdapter,
    DelegatingAdapter,
)
from expression_engine.adapters.ears_adapter import EarsAdapter
from expression_engine.adapters.eye_adapter import EyeAdapter, _CANONICAL_TO_ESP32
from expression_engine.adapters.neck_adapter import NeckAdapter


class FakeNeck:
    def __init__(self):
        self.poses = []
        self.closed = False

    def move_pose(self, pan=0.0, tilt=0.0, roll=0.0, speed=None):
        self.poses.append((pan, tilt, roll))
        return True

    def close(self):
        self.closed = True


class FakeEars:
    def __init__(self):
        self.calls = {}
        self.closed = False

    def _rec(self, k, d, s):
        self.calls[k] = (d, s)
        return True

    def move_left_pan(self, d, s=0.0):
        return self._rec("left_pan", d, s)

    def move_left_tilt(self, d, s=0.0):
        return self._rec("left_tilt", d, s)

    def move_right_pan(self, d, s=0.0):
        return self._rec("right_pan", d, s)

    def move_right_tilt(self, d, s=0.0):
        return self._rec("right_tilt", d, s)

    def close(self):
        self.closed = True


class FakeEyeClient:
    def __init__(self):
        self.expressions = []
        self.opened = False
        self.closed = False
        self.blinks = 0

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True

    def set_system_status(self, status):
        self.status = status  # real HeadI2CClient has this; wake = woke_up
        return True

    def set_expression(self, expr, intensity=3):
        self.expressions.append((expr, intensity))
        return True

    def trigger_blink(self):
        self.blinks += 1
        return True

    def set_look_direction(self, x, y):
        return True


class TestNeckAdapter:
    def test_satisfies_protocol(self):
        assert isinstance(NeckAdapter(lambda: FakeNeck()), ContinuousAdapter)

    def test_apply_maps_to_move_pose(self):
        fake = FakeNeck()
        a = NeckAdapter(lambda: fake)
        a.connect()
        a.apply({"pan": 3.0, "tilt": -2.0, "roll": 1.5})
        assert fake.poses[-1] == (3.0, -2.0, 1.5)

    def test_neutral_and_close(self):
        fake = FakeNeck()
        a = NeckAdapter(lambda: fake)
        a.connect()
        assert a.neutral() == {"pan": 0.0, "tilt": 0.0, "roll": 0.0}
        a.close()
        assert fake.closed

    def test_apply_noop_before_connect(self):
        NeckAdapter(lambda: FakeNeck()).apply({"pan": 1})  # must not raise


class TestEarsAdapter:
    def test_satisfies_protocol(self):
        assert isinstance(EarsAdapter(lambda: FakeEars()), ContinuousAdapter)

    def test_apply_maps_all_four_joints(self):
        fake = FakeEars()
        a = EarsAdapter(lambda: fake)
        a.connect()
        a.apply({"left_pan": 20, "left_tilt": 18, "right_pan": 20, "right_tilt": 18}, 0.1)
        assert fake.calls["left_pan"] == (20, 0.1)
        assert fake.calls["right_tilt"] == (18, 0.1)

    def test_right_pan_clamped_at_50(self, caplog):
        fake = FakeEars()
        a = EarsAdapter(lambda: fake)
        a.connect()
        a.apply({"right_pan": 80})  # binds ≥65 — must clamp ≤50
        assert fake.calls["right_pan"][0] == 50.0


class TestEyeAdapter:
    def test_satisfies_protocol(self):
        assert isinstance(EyeAdapter(lambda: FakeEyeClient()), DelegatingAdapter)

    def test_happy_translates_to_esp32_happy(self):
        # Reference expression for Story 6.4.
        assert EyeAdapter.translate("happy") == "happy"

    def test_table_covers_all_12_speech_emotions(self):
        from expression_engine import schema

        for name in schema.SPEECH_EMOTION_CANONICAL:
            assert name in _CANONICAL_TO_ESP32, f"{name} missing from AR10 table"

    def test_all_12_map_to_distinct_targets(self):
        # Story 7.1a: AR10 no longer squashes 12→7. Each canonical
        # speech-emotion must reach a DISTINCT device expression so it
        # renders distinctly on the eyes (owner-authorised AR10
        # freeze deviation, 2026-05-19).
        from expression_engine import schema

        targets = [
            EyeAdapter.translate(n)
            for n in schema.SPEECH_EMOTION_CANONICAL
        ]
        assert len(set(targets)) == len(
            schema.SPEECH_EMOTION_CANONICAL
        ), dict(zip(schema.SPEECH_EMOTION_CANONICAL, targets))

    def test_unknown_canonical_falls_back_to_neutral(self):
        assert EyeAdapter.translate("no_such_eye") == "neutral"

    def test_set_expression_sends_translated_and_clamps_intensity(self):
        fake = FakeEyeClient()
        a = EyeAdapter(lambda: fake)
        a.connect()
        assert fake.opened
        # 7.1a: frustrated is now its own device target (no squash).
        a.set_expression("frustrated", 9)  # → frustrated, intensity clamp 5
        assert fake.expressions[-1] == ("frustrated", 5)

    def test_blink_and_close(self):
        fake = FakeEyeClient()
        a = EyeAdapter(lambda: fake)
        a.connect()
        a.blink()
        a.close()
        assert fake.blinks == 1 and fake.closed
