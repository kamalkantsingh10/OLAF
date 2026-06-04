"""Headless tests for the chest heart logic (Story 8.4) — no pygame, no ROS."""

import json
from pathlib import Path

from chest_display.emotion_source import HeartConfig, HeartDirector, load_heart_config
from chest_display.ros_link import EmotionLink


CFG = HeartConfig(
    speech={
        "excited": {"image": "calm", "bpm": 115, "amplitude": 0.9},
        "sad": {"image": "sad", "bpm": 58, "amplitude": 0.22},
    },
    activity={
        "sleeping": {"image": "sleepy", "bpm": 45, "amplitude": 0.15},
        "listening": {"image": "calm", "bpm": 66, "amplitude": 0.35},
        "working.thinking": {"image": "calm", "bpm": 72, "amplitude": 0.40},
        "speaking": {"image": "calm", "bpm": 78, "amplitude": 0.50},
    },
    default={"image": "calm", "bpm": 62, "amplitude": 0.35},
)


def test_activity_drives_when_not_speaking():
    assert CFG.target(activity_state="sleeping")["image"] == "sleepy"
    assert CFG.target(activity_state="sleeping")["bpm"] == 45
    assert CFG.target(activity_state="listening")["bpm"] == 66


def test_working_submode():
    assert CFG.target(activity_state="working", working_submode="thinking")["bpm"] == 72


def test_speech_overrides_while_speaking():
    t = CFG.target(activity_state="speaking", speech_emotion="excited")
    assert t["bpm"] == 115 and t["image"] == "calm"
    t2 = CFG.target(activity_state="speaking", speech_emotion="sad")
    assert t2["image"] == "sad" and t2["bpm"] == 58


def test_speech_ignored_when_not_speaking():
    # a stale emotion while listening must NOT change the heart
    assert CFG.target(activity_state="listening", speech_emotion="excited")["bpm"] == 66


def test_default_fallback():
    assert CFG.target()["bpm"] == 62
    assert CFG.target(activity_state="unknown_state")["bpm"] == 62


def test_director_eases_toward_target():
    d = HeartDirector(CFG, ease=0.3)
    first = None
    bpm = d.bpm
    for _ in range(60):                  # 1s at 60fps, excited while speaking
        _img, bpm, _amp = d.update(1 / 60, activity_state="speaking", speech_emotion="excited")
        if first is None:
            first = bpm
    assert first < bpm                   # eased upward
    assert bpm > 100                     # approached 115
    for _ in range(180):                 # now sleeping -> ramps down
        _img, bpm, _amp = d.update(1 / 60, activity_state="sleeping")
    assert bpm < 55 and d.image == "sleepy"


def test_director_image_switches_discretely():
    d = HeartDirector(CFG)
    d.update(0.1, activity_state="speaking", speech_emotion="sad")
    assert d.image == "sad"


class _Msg:
    def __init__(self, data):
        self.data = data


def _envelope(payload):
    return _Msg(json.dumps({"payload": payload}))


def test_speech_emotion_is_per_utterance():
    # rclpy is absent in CI -> EmotionLink is a stub, but the handlers still run.
    link = EmotionLink()
    link._on_activity(_envelope({"state": "speaking"}))
    link._on_speech(_envelope({"emotion": "excited"}))
    assert link.speech_emotion == "excited"
    # speech ends -> emotion cleared; only a fresh publish can set it again
    link._on_activity(_envelope({"state": "listening"}))
    assert link.speech_emotion is None
    assert link.activity_state == "listening"


def test_load_heart_config_reads_the_real_map():
    p = Path(__file__).resolve().parents[2] / "expression_engine/config/expression_map.yaml"
    cfg = load_heart_config(p)
    assert cfg.speech["excited"]["bpm"] == 81      # bpm rescaled to 25-85 range
    assert cfg.activity["sleeping"]["image"] == "calm"   # sleep uses calm img, slowest bpm
    assert "working.thinking" in cfg.activity
    assert cfg.default["image"] == "calm"          # boot default = asleep (calm img, slowest bpm)
