"""Story 6.1 Task 1 — config plumbing tests (AC #1).

Verifies the engine loads [dds].domain_id and the four [topics] names
from expression_engine.toml, and fails fast (NFR7) on a missing or
malformed file rather than starting with silent defaults.
"""

from pathlib import Path

import pytest

from expression_engine.config import EngineConfig, load_config

PACKAGE_TOML = (
    Path(__file__).resolve().parents[1] / "config" / "expression_engine.toml"
)


class TestLoadShippedConfig:
    def test_loads_packaged_toml(self):
        cfg = load_config(PACKAGE_TOML)
        assert isinstance(cfg, EngineConfig)

    def test_domain_id(self):
        cfg = load_config(PACKAGE_TOML)
        assert cfg.domain_id == 0

    def test_all_four_topic_names(self):
        cfg = load_config(PACKAGE_TOML)
        assert cfg.topics == {
            "mood": "/olaf/mood",
            "activity": "/olaf/activity",
            "speech_emotion": "/olaf/speech_emotion",
            "vocalization": "/olaf/vocalization",
        }


class TestFailFast:
    def test_missing_file_raises(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            load_config(tmp_path / "nope.toml")

    def test_malformed_toml_raises(self, tmp_path):
        bad = tmp_path / "bad.toml"
        bad.write_text("this is = = not valid toml [[[")
        with pytest.raises(ValueError):
            load_config(bad)

    def test_missing_topics_section_raises(self, tmp_path):
        partial = tmp_path / "partial.toml"
        partial.write_text("[dds]\ndomain_id = 0\n")
        with pytest.raises(ValueError):
            load_config(partial)

    def test_incomplete_topics_raises(self, tmp_path):
        partial = tmp_path / "partial.toml"
        partial.write_text(
            '[dds]\ndomain_id = 0\n[topics]\nmood = "/olaf/mood"\n'
        )
        with pytest.raises(ValueError):
            load_config(partial)

    def test_missing_domain_id_raises(self, tmp_path):
        partial = tmp_path / "partial.toml"
        partial.write_text(
            '[topics]\nmood = "/olaf/mood"\nactivity = "/olaf/activity"\n'
            'speech_emotion = "/olaf/speech_emotion"\n'
            'vocalization = "/olaf/vocalization"\n'
        )
        with pytest.raises(ValueError):
            load_config(partial)
