"""Story 6.2 — expression_map.yaml loader + vocabulary completeness.

AC#1 loader reads the packaged map. AC#2 incomplete vocabulary vs the
pinned canonical set is startup-fatal. AC#3 nod/shake must be
visible_only:true. AC#4/NFR5 a synthetic added entry loads with NO
code change. AC#5/FR13 an unknown runtime name → WARN + default_*,
never raises.

Strict-at-startup vs graceful-at-runtime asymmetry IS the design
(Dev Notes) — these tests pin both halves.
"""

import logging
import os
import subprocess
import sys
import textwrap
from pathlib import Path
from typing import get_args

import pytest
import yaml

from expression_engine import schema
from expression_engine.map_loader import (
    ExpressionMap,
    MapValidationError,
    load_expression_map,
)

PKG = Path(__file__).resolve().parents[1]
PACKAGED_MAP = PKG / "config" / "expression_map.yaml"


@pytest.fixture()
def valid_map_dict():
    with PACKAGED_MAP.open() as fh:
        return yaml.safe_load(fh)


def _write(tmp_path, data) -> Path:
    p = tmp_path / "expression_map.yaml"
    p.write_text(yaml.safe_dump(data))
    return p


# ── AC#1 — loads the packaged map ───────────────────────────────────


class TestLoadsPackagedMap:
    def test_returns_expression_map(self):
        m = load_expression_map(PACKAGED_MAP)
        assert isinstance(m, ExpressionMap)

    def test_interface_version_matches_schema(self):
        m = load_expression_map(PACKAGED_MAP)
        assert m.interface_version == schema.INTERFACE_VERSION

    def test_defaults_resolved(self):
        m = load_expression_map(PACKAGED_MAP)
        for k in ("pose", "eye", "led", "heart"):
            assert k in m.defaults

    def test_missing_file_raises(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            load_expression_map(tmp_path / "absent.yaml")

    def test_malformed_yaml_raises(self, tmp_path):
        bad = tmp_path / "expression_map.yaml"
        bad.write_text("key: : : [unbalanced")
        with pytest.raises(MapValidationError):
            load_expression_map(bad)

    def test_missing_top_level_key_fatal(self, tmp_path, valid_map_dict):
        del valid_map_dict["defaults"]
        with pytest.raises(MapValidationError):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_interface_version_mismatch_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["interface_version"] = "9.9.9"
        with pytest.raises(MapValidationError):
            load_expression_map(_write(tmp_path, valid_map_dict))


# ── AC#2 — completeness vs pinned canonical set ─────────────────────


class TestCompletenessFatal:
    def test_all_canonical_names_present_in_packaged_map(self):
        m = load_expression_map(PACKAGED_MAP)
        assert set(m.mood) >= set(get_args(schema.Mood))
        assert set(m.activity) >= set(get_args(schema.ActivityState))
        assert set(m.activity["working"]) >= set(get_args(schema.WorkingSubmode))
        assert set(m.speech_emotion) >= set(schema.SPEECH_EMOTION_CANONICAL)
        assert set(m.vocalization) >= set(schema.VOCALIZATION_TAGS)

    def test_missing_mood_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["mood"].pop("grumpy")
        with pytest.raises(MapValidationError, match="grumpy"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_missing_activity_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["activity"].pop("waking")
        with pytest.raises(MapValidationError, match="waking"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_missing_working_submode_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["activity"]["working"].pop("thinking")
        with pytest.raises(MapValidationError, match="thinking"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_missing_speech_emotion_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["speech_emotion"].pop("melancholic")
        with pytest.raises(MapValidationError, match="melancholic"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_missing_vocalization_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["vocalization"].pop("gasp")
        with pytest.raises(MapValidationError, match="gasp"):
            load_expression_map(_write(tmp_path, valid_map_dict))


# ── AC#3 — visible_only invariant on nod & shake (FR7) ──────────────


class TestVisibleOnlyInvariant:
    def test_nod_missing_visible_only_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["vocalization"]["nod"] = {}
        with pytest.raises(MapValidationError, match="nod"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_shake_visible_only_false_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["vocalization"]["shake"] = {"visible_only": False}
        with pytest.raises(MapValidationError, match="shake"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_packaged_nod_shake_visible_only_true(self):
        m = load_expression_map(PACKAGED_MAP)
        assert m.vocalization["nod"]["visible_only"] is True
        assert m.vocalization["shake"]["visible_only"] is True


# ── AC#4 / NFR5 — synthetic added entry, NO code change ─────────────


class TestExtensibilityNFR5:
    def test_synthetic_extra_speech_emotion_loads(self, tmp_path, valid_map_dict):
        # A name beyond the pinned canonical set is ACCEPTED (forward-
        # compat); proves vocabulary growth needs no Python change.
        valid_map_dict["speech_emotion"]["ecstatic"] = {
            "pose": {"neck": {"tilt": 12}, "ears": {"left_tilt": 20, "right_tilt": 20}},
            "eye": {"expression": "ecstatic", "intensity": 5},
            "led_overlay": "bright",
        }
        m = load_expression_map(_write(tmp_path, valid_map_dict))
        assert "ecstatic" in m.speech_emotion
        assert m.speech_emotion["ecstatic"]["eye"]["expression"] == "ecstatic"

    def test_synthetic_extra_mood_loads(self, tmp_path, valid_map_dict):
        valid_map_dict["mood"]["zen"] = {
            "lean_bias": 1,
            "led_bias": {"color": "#80ffd0", "intensity": 0.2},
        }
        m = load_expression_map(_write(tmp_path, valid_map_dict))
        assert m.mood["zen"]["lean_bias"] == 1


# ── AC#5 / FR13 — runtime unknown name → WARN + default, alive ──────


class TestRuntimeFallback:
    def test_known_name_returns_entry(self):
        m = load_expression_map(PACKAGED_MAP)
        assert m.resolve("mood", "calm") == m.mood["calm"]

    def test_unknown_name_returns_defaults_no_raise(self):
        # logging_setup sets propagate=False, so pytest's root-attached
        # caplog can't see it — attach our own capture handler to the
        # engine logger instead.
        records: list[logging.LogRecord] = []
        sink = logging.Handler()
        sink.emit = records.append  # type: ignore[method-assign]
        engine_log = logging.getLogger("expression_engine")
        engine_log.addHandler(sink)
        try:
            m = load_expression_map(PACKAGED_MAP)
            out = m.resolve("speech_emotion", "no_such_emotion")
        finally:
            engine_log.removeHandler(sink)
        assert out == m.defaults  # FR13: default_* render
        events = [r.getMessage() for r in records]
        assert "expression.unmapped_speech_emotion" in events

    def test_unknown_name_does_not_crash_process(self):
        m = load_expression_map(PACKAGED_MAP)
        # Many misses in a row must never raise (never freezes/crashes).
        for i in range(100):
            assert m.resolve("vocalization", f"bogus_{i}") == m.defaults

    def test_topic_namespacing_preserved(self):
        # mood.happy and speech_emotion.happy are DISTINCT entries.
        m = load_expression_map(PACKAGED_MAP)
        assert m.resolve("mood", "happy") is not m.resolve(
            "speech_emotion", "happy"
        )
        assert "led_bias" in m.resolve("mood", "happy")
        assert "pose" in m.resolve("speech_emotion", "happy")

    def test_unknown_topic_raises_keyerror(self):
        # A bad TOPIC (not a bad name) is a programming error, not a
        # wire event — fail loudly, not the FR13 graceful path.
        m = load_expression_map(PACKAGED_MAP)
        with pytest.raises(KeyError):
            m.resolve("not_a_topic", "x")


# ── Story 7.2 — layered_action + mood.eye validation ────────────────


class TestStory72LayeredActionValidation:
    """Story 7.2 — `vocalization.<tag>.layered_action` + `mood.<m>.eye`
    are fail-fast validated at startup (NFR5/NFR7)."""

    def test_packaged_map_has_layered_action_on_every_vocalization(self):
        m = load_expression_map(PACKAGED_MAP)
        for tag in schema.VOCALIZATION_TAGS:
            entry = m.vocalization[tag]
            assert "layered_action" in entry, f"{tag} missing layered_action"
            assert "attack_ms" in entry["layered_action"]
            assert "settle_ms" in entry["layered_action"]
        # Review iter #2: only laughter/sigh/gasp carry an eye override;
        # clears_throat/nod/shake are body-only.
        for tag in ("laughter", "sigh", "gasp"):
            assert "eye" in m.vocalization[tag]["layered_action"], (
                f"{tag} expected to author eye override"
            )
        for tag in ("clears_throat", "nod", "shake"):
            assert "eye" not in m.vocalization[tag]["layered_action"], (
                f"{tag} must NOT author eye override (speech_emotion shows through)"
            )

    def test_packaged_map_has_mood_eye_on_every_mood(self):
        m = load_expression_map(PACKAGED_MAP)
        for mood_name in get_args(schema.Mood):
            entry = m.mood[mood_name]
            assert "eye" in entry, f"{mood_name} missing eye accent"
            assert "expression" in entry["eye"]
            assert "intensity" in entry["eye"]

    def test_range_field_accepts_2_list(self, tmp_path, valid_map_dict):
        # A [min,max] range is the canonical form.
        valid_map_dict["vocalization"]["laughter"]["layered_action"]["attack_ms"] = [50, 100]
        load_expression_map(_write(tmp_path, valid_map_dict))

    def test_range_field_accepts_scalar(self, tmp_path, valid_map_dict):
        # A scalar is also accepted (treated as fixed).
        valid_map_dict["vocalization"]["laughter"]["layered_action"]["attack_ms"] = 80
        load_expression_map(_write(tmp_path, valid_map_dict))

    def test_range_with_wrong_length_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["vocalization"]["laughter"]["layered_action"]["attack_ms"] = [60]
        with pytest.raises(MapValidationError, match="attack_ms"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_range_with_min_gt_max_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["vocalization"]["laughter"]["layered_action"]["settle_ms"] = [800, 400]
        with pytest.raises(MapValidationError, match="settle_ms"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_unknown_neck_gesture_token_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["vocalization"]["laughter"]["layered_action"]["neck_gesture"]["token"] = "not_a_gesture"
        with pytest.raises(MapValidationError, match="neck_gestures"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_unknown_ears_gesture_token_fatal(self, tmp_path, valid_map_dict):
        valid_map_dict["vocalization"]["gasp"]["layered_action"]["ears_gesture"]["token"] = "not_a_gesture"
        with pytest.raises(MapValidationError, match="ears_gestures"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_audible_vocalization_eye_block_present_must_have_expression(self, tmp_path, valid_map_dict):
        # When the optional `eye` block IS authored on an audible
        # vocalization, expression is required (otherwise the override
        # would have no name to send to the eye).
        del valid_map_dict["vocalization"]["laughter"]["layered_action"]["eye"]["expression"]
        with pytest.raises(MapValidationError, match="eye.expression"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_gesture_cue_eye_block_present_cannot_author_expression(self, tmp_path, valid_map_dict):
        # nod has no eye block in the packaged map. If an author adds
        # one AND puts an expression in it, the validator points them
        # at the right design (drop the block entirely).
        valid_map_dict["vocalization"]["nod"]["layered_action"]["eye"] = {
            "expression": "happy", "intensity": [2, 3],
        }
        with pytest.raises(MapValidationError, match="nod|gesture cues"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_eye_block_is_optional(self, tmp_path, valid_map_dict):
        # Sanity: a vocalization without `eye` loads fine (body-only).
        # Already true for clears_throat/nod/shake in the packaged map.
        load_expression_map(PACKAGED_MAP)

    def test_layered_action_extra_field_fatal(self, tmp_path, valid_map_dict):
        # Typos like `led_pulse` (dropped in 7.2 scope reshape) must
        # surface at startup, not be silently ignored.
        valid_map_dict["vocalization"]["laughter"]["layered_action"]["led_pulse"] = {
            "color": "warm", "dur_s": 0.5,
        }
        with pytest.raises(MapValidationError, match="led_pulse"):
            load_expression_map(_write(tmp_path, valid_map_dict))

    def test_mood_eye_intensity_accepts_range(self, tmp_path, valid_map_dict):
        valid_map_dict["mood"]["calm"]["eye"]["intensity"] = [2, 4]
        load_expression_map(_write(tmp_path, valid_map_dict))

    def test_mood_eye_missing_expression_fatal(self, tmp_path, valid_map_dict):
        del valid_map_dict["mood"]["calm"]["eye"]["expression"]
        with pytest.raises(MapValidationError, match="expression"):
            load_expression_map(_write(tmp_path, valid_map_dict))


class TestNodeStartupFatal:  # AC#2/#3 — §9: incomplete map → exit 1
    def test_node_main_exits_nonzero_on_incomplete_map(self, tmp_path):
        # node.main loads + validates the map BEFORE rclpy.init, so an
        # incomplete map terminates the process non-zero with no ROS
        # involvement (NFR7/AR8 — identical posture to 6.1's config).
        with PACKAGED_MAP.open() as fh:
            data = yaml.safe_load(fh)
        data["speech_emotion"].pop("scared")  # break completeness
        bad = tmp_path / "expression_map.yaml"
        bad.write_text(yaml.safe_dump(data))
        script = textwrap.dedent(
            f"""
            import expression_engine.node as n
            n._default_map_path = lambda: __import__("pathlib").Path({str(bad)!r})
            n.main()
            """
        )
        env = dict(os.environ)
        env["PYTHONPATH"] = str(PKG) + os.pathsep + env.get("PYTHONPATH", "")
        proc = subprocess.run(
            [sys.executable, "-c", script],
            cwd=str(PKG.parents[2]),
            env=env,
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert proc.returncode == 1, (
            f"incomplete map must exit 1, got {proc.returncode}\n"
            f"{proc.stderr}"
        )
        assert "expression_map_load_failed" in proc.stderr
        assert "scared" in proc.stderr
