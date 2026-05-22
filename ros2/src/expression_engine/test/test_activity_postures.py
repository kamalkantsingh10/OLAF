"""Story 7.3 — activity → posture authoring.

Every leaf ``ActivityState`` (the 6 top-level + the 2 ``working``
submodes = 8 leaves) must drive a COMPLETE, distinct body posture:
a full ``pose.neck`` (pan/tilt/roll) and ``pose.ears`` (LP/LT/RP/RT)
block, all within the mechanical envelopes the real adapters clamp to
(AC#1/#2/#3). Each ``eye.expression`` must resolve via the AR10
``eye_adapter`` table (AC#4). Compatibility with the 15 frozen
speech-emotion poses is covered by ``test_speech_emotions.py``
(unchanged) + the whole-map load here (AC#5).

This complements ``test_map_loader.py``'s *completeness-of-names* check
(activity keys present) by pinning the per-leaf *pose shape + bounds*.
"""

from pathlib import Path
from typing import get_args

import pytest

from expression_engine import schema
from expression_engine.adapters import eye_adapter
from expression_engine.adapters.ears_adapter import _LIMITS as EARS_LIMITS
from expression_engine.adapters.neck_adapter import _LIMITS as NECK_LIMITS
from expression_engine.map_loader import load_expression_map

PKG = Path(__file__).resolve().parents[1]
PACKAGED_MAP = PKG / "config" / "expression_map.yaml"

_NECK = ("pan", "tilt", "roll")
_EARS = ("left_pan", "left_tilt", "right_pan", "right_tilt")
# SCS0009 right_pan binds ≥65°; story Dev Notes mandate ≤50 (the ears
# adapter clamps there too). Author conservatively.
_RIGHT_PAN_SAFE_MAX = 50.0


def _leaf_activities(amap: dict) -> dict[str, dict]:
    """Flatten ``activity.*`` to leaf states (``working`` expanded)."""
    out: dict[str, dict] = {}
    for name, entry in amap.items():
        if name == "working":
            for sub, sub_entry in entry.items():
                out[f"working.{sub}"] = sub_entry
        else:
            out[name] = entry
    return out


@pytest.fixture(scope="module")
def amap():
    return load_expression_map(PACKAGED_MAP).activity


@pytest.fixture(scope="module")
def leaves(amap):
    return _leaf_activities(amap)


def _expected_leaf_labels() -> set[str]:
    states = set(get_args(schema.ActivityState)) - {"working"}
    states |= {f"working.{s}" for s in get_args(schema.WorkingSubmode)}
    return states


# ── AC#1 — completeness: every leaf has full neck + ears ─────────────


def test_all_eight_leaf_states_present(leaves):
    assert set(leaves) == _expected_leaf_labels()
    assert len(leaves) == 8


@pytest.mark.parametrize("label", sorted(_expected_leaf_labels()))
def test_leaf_has_full_neck_and_ears_block(leaves, label):
    pose = leaves[label].get("pose")
    assert isinstance(pose, dict), f"{label}: pose must be a mapping"

    neck = pose.get("neck")
    assert isinstance(neck, dict), f"{label}: pose.neck missing"
    assert set(neck) == set(_NECK), f"{label}: pose.neck must be pan/tilt/roll"

    ears = pose.get("ears")
    assert isinstance(ears, dict), f"{label}: pose.ears missing"
    assert set(ears) == set(_EARS), f"{label}: pose.ears must be 4 joints"

    for axis, keys in (("neck", neck), ("ears", ears)):
        for k, v in keys.items():
            assert isinstance(v, (int, float)) and not isinstance(v, bool), (
                f"{label}.pose.{axis}.{k} not numeric: {v!r}"
            )


# ── AC#3 — bounds: within the adapter mechanical envelopes ───────────


@pytest.mark.parametrize("label", sorted(_expected_leaf_labels()))
def test_neck_within_envelope(leaves, label):
    neck = leaves[label]["pose"]["neck"]
    for joint in _NECK:
        lo, hi = NECK_LIMITS[joint]
        v = neck[joint]
        assert lo <= v <= hi, f"{label}.neck.{joint}={v} outside [{lo},{hi}]"


@pytest.mark.parametrize("label", sorted(_expected_leaf_labels()))
def test_ears_within_envelope(leaves, label):
    ears = leaves[label]["pose"]["ears"]
    for joint in _EARS:
        lo, hi = EARS_LIMITS[joint]
        v = ears[joint]
        assert lo <= v <= hi, f"{label}.ears.{joint}={v} outside [{lo},{hi}]"


@pytest.mark.parametrize("label", sorted(_expected_leaf_labels()))
def test_right_pan_safe_for_scs0009(leaves, label):
    # right_pan ≥65 BINDS on the SCS0009; keep ≤50 (project memory).
    rp = leaves[label]["pose"]["ears"]["right_pan"]
    assert abs(rp) <= _RIGHT_PAN_SAFE_MAX, (
        f"{label}.ears.right_pan={rp} exceeds safe ±{_RIGHT_PAN_SAFE_MAX}"
    )


# ── AC#2 — distinctness: no two leaf postures are identical ──────────


def test_leaf_postures_are_pairwise_distinct(leaves):
    # boot = sleep mode by design: `starting` intentionally shares
    # `sleeping`'s body pose (the bot wakes up FROM sleep), so that pair
    # is an allowed duplicate. Every OTHER state stays distinct (AC#2).
    allowed_dupes = {frozenset({"starting", "sleeping"})}
    seen: dict[tuple, str] = {}
    for label, entry in leaves.items():
        pose = entry["pose"]
        sig = (
            tuple(float(pose["neck"][j]) for j in _NECK),
            tuple(float(pose["ears"][j]) for j in _EARS),
        )
        if sig in seen:
            assert frozenset({label, seen[sig]}) in allowed_dupes, (
                f"{label} has an identical posture to {seen[sig]} — "
                f"AC#2 requires distinct postures"
            )
            continue
        seen[sig] = label


# ── AC#4 — every activity eye target resolves via the AR10 table ─────


@pytest.mark.parametrize("label", sorted(_expected_leaf_labels()))
def test_eye_expression_resolves_via_ar10(leaves, label):
    eye = leaves[label].get("eye")
    if label == "speaking":
        # speaking deliberately authors NO eye — the eye is driven by the
        # active speech_emotion (Story 7.3). Activity = body pose + LED only.
        assert eye is None, "speaking must NOT author an activity eye (speech drives it)"
        return
    assert isinstance(eye, dict), f"{label}: eye block missing"
    expr = eye.get("expression")
    assert isinstance(expr, str) and expr, f"{label}: eye.expression missing"
    # MUST be a known canonical → no silent neutral fallback (the
    # adapter logs + falls back, but an *authored* activity eye should
    # be intentional).
    assert expr in eye_adapter._CANONICAL_TO_ESP32, (
        f"{label}: eye.expression {expr!r} not in eye_adapter "
        f"_CANONICAL_TO_ESP32 — add a device mapping (AC#4)"
    )
    if "intensity" in eye:
        i = eye["intensity"]
        assert isinstance(i, int) and 1 <= i <= 5, (
            f"{label}: eye.intensity {i!r} must be int 1..5"
        )


# ── AC#6 — YAML integrity: no `key:value` no-space traps ─────────────


def test_activity_block_has_no_nospace_colon_traps():
    """A `key:value` with no space after the colon parses as one scalar
    string, not a mapping — a classic hand-edit trap. The map_loader
    would reject it (pose becomes a str), but pin it here too with a
    targeted scan of the activity block lines."""
    lines = PACKAGED_MAP.read_text().splitlines()
    in_activity = False
    offenders: list[str] = []
    for i, raw in enumerate(lines, 1):
        stripped = raw.strip()
        # Track entry into the top-level `activity:` block; leave it at
        # the next top-level key (column-0, non-comment).
        if raw.startswith("activity:"):
            in_activity = True
            continue
        if in_activity and raw and raw[0] not in " #":
            break  # next top-level block
        if not in_activity or not stripped or stripped.startswith("#"):
            continue
        # Look for `word:nonspace` inside the flow content (a real YAML
        # mapping always has `key: value`). Hex colours (#xxxx) and
        # bare URLs aren't present in this block.
        import re

        for m in re.finditer(r"[A-Za-z_][\w]*:[^\s]", stripped):
            offenders.append(f"line {i}: {stripped!r} near {m.group()!r}")
    assert not offenders, "no-space colon trap(s) in activity block:\n" + "\n".join(
        offenders
    )


# ── AC#5 — whole-map still loads (no regression to frozen poses) ─────


def test_packaged_map_loads_clean():
    m = load_expression_map(PACKAGED_MAP)
    assert m.activity
    # speech_emotion poses untouched — exact-shape pinned by
    # test_speech_emotions.py; here just assert the 12 still load.
    assert set(m.speech_emotion) >= set(schema.SPEECH_EMOTION_CANONICAL)
