"""Regression tests for the 2026-05-17 code-review hardening patches.

Each test pins a specific finding's fix so it cannot silently regress.
"""

import json
import math
import uuid

import pytest
import yaml

from expression_engine import schema
from expression_engine.config import AnimationConfig, load_config
from expression_engine.map_loader import MapValidationError, load_expression_map
from expression_engine.node import _default_config_path, _default_map_path
import random

from expression_engine.adapters import neck_gestures
from expression_engine.render_loop import _VocalizationAction

PKG_MAP = _default_map_path()


def _valid_map():
    with open(PKG_MAP) as fh:
        return yaml.safe_load(fh)


def _w(tmp, data):
    p = tmp / "expression_map.yaml"
    p.write_text(yaml.safe_dump(data))
    return p


# P7/P12 — schema_version absent or wrong-typed is fatal
class TestSchemaVersionHardening:
    def _env(self, payload, **over):
        d = {
            "schema_version": 3,
            "timestamp": "2026-05-10T13:42:18.123456+00:00",
            "source": "voice_agent_pipeline",
            "correlation_id": str(uuid.uuid4()),
            "payload": payload,
        }
        d.update(over)
        if over.get("_drop_version"):
            d.pop("schema_version")
            d.pop("_drop_version")
        return json.dumps(d)

    def test_missing_schema_version_is_fatal(self):
        raw = self._env({"mood": "calm"}, _drop_version=True)
        with pytest.raises(schema.SchemaVersionError):
            schema.parse_event("mood", raw)

    def test_string_version_is_fatal_not_crash(self):
        with pytest.raises(schema.SchemaVersionError):
            schema.parse_event("mood", self._env({"mood": "calm"}, schema_version="3"))

    def test_float_three_accepted(self):
        ev = schema.parse_event("mood", self._env({"mood": "calm"}, schema_version=3.0))
        assert ev.payload.mood == "calm"

    def test_bool_version_rejected(self):
        with pytest.raises(schema.SchemaVersionError):
            schema.parse_event("mood", self._env({"mood": "calm"}, schema_version=True))


# P2 — map value/shape + duplicate-key validation at startup
class TestMapValueValidation:
    def test_non_numeric_pose_fatal(self, tmp_path):
        m = _valid_map()
        m["speech_emotion"]["happy"]["pose"]["neck"]["tilt"] = "high"
        with pytest.raises(MapValidationError, match="number"):
            load_expression_map(_w(tmp_path, m))

    def test_non_dict_entry_fatal(self, tmp_path):
        m = _valid_map()
        m["mood"]["calm"] = 5
        with pytest.raises(MapValidationError, match="mapping"):
            load_expression_map(_w(tmp_path, m))

    def test_degenerate_defaults_fatal(self, tmp_path):
        m = _valid_map()
        m["defaults"]["pose"] = "TODO"
        with pytest.raises(MapValidationError, match="defaults"):
            load_expression_map(_w(tmp_path, m))

    def test_duplicate_key_fatal(self, tmp_path):
        p = tmp_path / "expression_map.yaml"
        text = PKG_MAP.read_text() + "\nmood:\n  calm: { lean_bias: 9 }\n"
        p.write_text(text)
        with pytest.raises(MapValidationError, match="duplicate"):
            load_expression_map(p)

    def test_valid_map_still_loads(self):
        assert load_expression_map(PKG_MAP) is not None


# P4 — neck adapter clamps to a safe envelope
class TestNeckClamp:
    def test_neck_target_clamped(self):
        from expression_engine.adapters.neck_adapter import NeckAdapter, _LIMITS

        class FakeNeck:
            def __init__(s):
                s.last = None

            def move_pose(s, pan=0.0, tilt=0.0, roll=0.0, speed=None):
                s.last = (pan, tilt, roll)
                return True

            def close(s):
                ...

        fake = FakeNeck()
        a = NeckAdapter(lambda: fake)
        a.connect()
        a.apply({"pan": 200.0, "tilt": 99.0, "roll": -90.0})
        pan, tilt, roll = fake.last
        # Clamp to the adapter envelope (tilt raised to ±28 in Story 7.3).
        assert pan == _LIMITS["pan"][1]
        assert tilt == _LIMITS["tilt"][1]
        assert roll == _LIMITS["roll"][0]


# P8 — eye wake failure is a fatal connect error
class TestEyeWakeFatal:
    def test_missing_set_system_status_raises(self):
        from expression_engine.adapters.eye_adapter import EyeAdapter

        class NoWake:
            def open(s): ...
            def close(s): ...
            def set_expression(s, e, i=3): return True

        with pytest.raises(RuntimeError, match="set_system_status"):
            EyeAdapter(lambda: NoWake()).connect()

    def test_false_wake_raises(self):
        from expression_engine.adapters.eye_adapter import EyeAdapter

        class BadWake:
            def open(s): ...
            def close(s): ...
            def set_system_status(s, st): return False
            def set_expression(s, e, i=3): return True

        with pytest.raises(RuntimeError, match="wake"):
            EyeAdapter(lambda: BadWake()).connect()


# P6 — Story 7.2 update: the old _Gesture's hard-coded shake (linear
# attack+settle multiplied by sin) is gone. The equivalent property
# now lives on `neck_gestures.shake`: at u=1.0 (gesture end) the
# offset must be ~0 — no neck SNAP when the vocalization action
# expires. _VocalizationAction additionally returns an empty offset
# AFTER the sampled window (the expired-then-popped path).
class TestShakeSettles:
    def test_neck_gesture_shake_zero_at_u_one(self):
        # `shake(u, amp) = (amp*sin(4πu), 0, 0)` — sin(4π) = 0 exactly.
        p_off, t_off, r_off = neck_gestures.GESTURES["shake"](1.0 - 1e-6, 16.0)
        assert abs(p_off) < 0.01
        assert t_off == 0.0 and r_off == 0.0

    def test_vocalization_action_returns_empty_after_window(self):
        spec = {
            "attack_ms": 80,
            "settle_ms": 320,
            "eye": {"intensity": 2},
            "neck_gesture": {"token": "shake", "amp_deg": 16.0, "dur_s": 0.4},
        }
        action = _VocalizationAction(
            tag="shake",
            spec=spec,
            start=0.0,
            rng=random.Random(0),
            mood_eye_expression=None,
        )
        # Inside the dur window: a real offset.
        mid = action.neck_offset(0.1)
        assert "pan" in mid
        # Past the dur window: no contribution.
        past = action.neck_offset(action.window_s + 0.01)
        assert past == {}


# P11 — config range checks (NFR7)
class TestConfigBounds:
    def test_domain_id_out_of_range_fatal(self, tmp_path):
        p = tmp_path / "expression_engine.toml"
        p.write_text(
            '[dds]\ndomain_id = 999\n[topics]\nmood="/m"\nactivity="/a"\n'
            'speech_emotion="/s"\nvocalization="/v"\n'
        )
        with pytest.raises(ValueError, match="0..232"):
            load_config(p)

    def test_insane_tick_hz_fatal(self, tmp_path):
        p = tmp_path / "expression_engine.toml"
        p.write_text(
            '[dds]\ndomain_id = 0\n[topics]\nmood="/m"\nactivity="/a"\n'
            'speech_emotion="/s"\nvocalization="/v"\n'
            '[animation]\nservo_tick_hz = 0.001\n'
        )
        with pytest.raises(ValueError, match="servo_tick_hz"):
            load_config(p)
