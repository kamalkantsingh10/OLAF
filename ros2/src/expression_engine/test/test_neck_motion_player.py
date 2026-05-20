"""Unit tests for NeckMotionPlayer + neck_gestures (Story 7.1b)."""

from __future__ import annotations

import math
from typing import List

import pytest

from expression_engine.adapters import neck_gestures
from expression_engine.adapters.neck_motion_player import NeckMotionPlayer


# ── shared fixture: a player wrapped around a stub driver ────────
class _StubDriver:
    def __init__(self) -> None:
        self.calls: List[dict] = []

    def move_pose(self, **kw: float) -> None:
        self.calls.append(kw)


@pytest.fixture
def player() -> NeckMotionPlayer:
    p = NeckMotionPlayer(_StubDriver())
    # Pin start_t so drift phase is deterministic.
    p._start_t = 0.0
    return p


# ── intensity → level mapping ────────────────────────────────────
@pytest.mark.parametrize(
    "intensity,expected",
    [(1, 1), (2, 2), (3, 3), (4, 3), (5, 3), (0, 1), (-1, 1)],
)
def test_intensity_to_level(intensity: int, expected: int) -> None:
    assert NeckMotionPlayer.intensity_to_level(intensity) == expected


# ── L1: exact pose, no drift, no gesture ─────────────────────────
def test_l1_static_returns_exact_pose(player: NeckMotionPlayer) -> None:
    player.set_expression("happy", 1)
    pan, tilt, roll = player.compute_target(t=0.0)
    spec = player._emotions["happy"]["l1"]            # noqa: SLF001
    assert (pan, tilt, roll) == pytest.approx(
        (spec["pan"], spec["tilt"], spec["roll"]))


def test_l1_neutral_zero(player: NeckMotionPlayer) -> None:
    player.set_expression("neutral", 1)
    assert player.compute_target(t=0.0) == pytest.approx((0.0, 0.0, 0.0))


# ── L2: drift adds bounded perturbation ──────────────────────────
def test_l2_drift_adds_bounded_offset(player: NeckMotionPlayer) -> None:
    player.set_expression("happy", 2)
    spec = player._emotions["happy"]                  # noqa: SLF001
    l1 = spec["l1"]
    amp = float(spec["l2_drift_deg"])
    for t in (0.0, 0.5, 1.2, 3.3, 7.7):
        pan, tilt, roll = player.compute_target(t=t)
        assert abs(pan  - l1["pan"])  <= amp        + 1e-6
        assert abs(tilt - l1["tilt"]) <= amp * 0.7  + 1e-6
        assert abs(roll - l1["roll"]) <= amp * 0.5  + 1e-6


# ── L3: gesture fires, peaks mid-way, decays to zero, then clears ─
def test_l3_gesture_fires_and_decays(player: NeckMotionPlayer) -> None:
    player.set_expression("happy", 3)
    spec = player._emotions["happy"]                  # noqa: SLF001
    l1_tilt = float(spec["l1"]["tilt"])
    amp     = float(spec["l3_gesture"]["amp_deg"])
    dur     = float(spec["l3_gesture"]["dur_s"])
    drift   = float(spec["l2_drift_deg"])
    t0 = player._gesture.start                        # noqa: SLF001
    # At u = 0.25 → nod = −amp·sin(π/2) = −amp on tilt (clamped to env).
    _, tilt, _ = player.compute_target(t=t0 + dur * 0.25)
    expected_tilt = max(-20.0, min(20.0, l1_tilt - amp))
    assert tilt <= expected_tilt + drift * 0.7 + 1e-6
    # After duration, gesture clears itself.
    player.compute_target(t=t0 + dur + 0.1)
    assert player._gesture is None                    # noqa: SLF001


def test_l3_curious_gesture_rolls_right(player: NeckMotionPlayer) -> None:
    # curious L1 roll = 14; tilt_right adds +amp at u=0.5; clamp 15.
    player.set_expression("curious", 3)
    t0 = player._gesture.start                  # noqa: SLF001
    pan, tilt, roll = player.compute_target(t=t0 + 0.7 * 0.5)
    assert roll == pytest.approx(15.0)          # clamped to ±15° envelope


# ── Safety clamp ─────────────────────────────────────────────────
def test_safety_clamp_envelope(player: NeckMotionPlayer) -> None:
    # frustrated L1 has pan 18 / tilt -6 / roll -8 — well within
    # mechanical envelope, no clamp at L1.
    player.set_expression("frustrated", 1)
    pan, tilt, roll = player.compute_target(t=0.0)
    assert abs(pan)  <= 80.0
    assert abs(tilt) <= 20.0
    assert abs(roll) <= 15.0


def test_safety_clamp_kicks_in_for_extreme_gesture(player: NeckMotionPlayer) -> None:
    # Inject a gesture amp that would push roll past ±15.
    player.set_expression("curious", 3)
    player._gesture.amp_deg = 50.0              # noqa: SLF001
    t0 = player._gesture.start                  # noqa: SLF001
    _, _, roll = player.compute_target(t=t0 + 0.7 * 0.5)
    assert roll == pytest.approx(15.0)          # clamped


# ── Fallbacks ────────────────────────────────────────────────────
def test_unknown_emotion_falls_back_to_neutral(
    player: NeckMotionPlayer,
) -> None:
    player.set_expression("nope-not-real", 3)
    assert player._emotion == "neutral"         # noqa: SLF001


def test_unknown_gesture_token_skips_silently(
    player: NeckMotionPlayer, monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Corrupt one emotion's gesture token; L3 should not crash and
    # gesture stays None.
    player._emotions["happy"]["l3_gesture"] = {  # noqa: SLF001
        "token": "does_not_exist", "amp_deg": 5, "dur_s": 0.4}
    player.set_expression("happy", 3)
    assert player._gesture is None              # noqa: SLF001
    # compute_target still works.
    player.compute_target(t=0.0)


def test_set_expression_with_uppercase_normalises(
    player: NeckMotionPlayer,
) -> None:
    player.set_expression("HAPPY", 1)
    assert player._emotion == "happy"           # noqa: SLF001


# ── gesture registry shape invariants ────────────────────────────
@pytest.mark.parametrize("token", list(neck_gestures.GESTURES))
def test_gesture_returns_zero_at_endpoints(token: str) -> None:
    fn = neck_gestures.GESTURES[token]
    for u in (0.0, 1.0):
        out = fn(u, 10.0)
        # All gestures must return ≈0 at u=0 and u=1 so the neck
        # snaps cleanly back to L1+drift after the gesture ends.
        assert all(abs(v) < 1e-6 for v in out), (
            f"gesture {token!r} non-zero at u={u}: {out}")


@pytest.mark.parametrize("token", list(neck_gestures.GESTURES))
def test_gesture_peak_is_bounded(token: str) -> None:
    fn = neck_gestures.GESTURES[token]
    amp = 10.0
    peak = max(max(abs(v) for v in fn(u / 100.0, amp)) for u in range(101))
    # No gesture should overshoot beyond the requested amp.
    assert peak <= amp + 1e-6, f"{token} overshoots: peak={peak} > amp={amp}"


# ── Bundled YAML integrity: every emotion has full L1 + gestures ──
def test_bundled_yaml_every_emotion_has_full_l1(
    player: NeckMotionPlayer,
) -> None:
    # Guards against the "tilt:-16 vs tilt: -16" YAML-spacing trap
    # (a `key:value` token with no space after the colon parses as a
    # single scalar, silently dropping the key — caught on hardware
    # at sad's first run, 2026-05-20).
    required = {"pan", "tilt", "roll"}
    for name, spec in player._emotions.items():       # noqa: SLF001
        l1 = spec.get("l1") or {}
        missing = required - set(l1.keys())
        assert not missing, (
            f"emotion {name!r} L1 missing keys {sorted(missing)}: {l1!r}")


def test_bundled_yaml_gestures_reference_registry(
    player: NeckMotionPlayer,
) -> None:
    for name, spec in player._emotions.items():       # noqa: SLF001
        g = spec.get("l3_gesture")
        if not g:
            continue
        token = g.get("token")
        assert token in neck_gestures.GESTURES, (
            f"{name}.l3_gesture references unknown token {token!r}")


# ── Re-fire L3 resets the gesture timer ──────────────────────────
def test_re_fire_l3_restarts_gesture(player: NeckMotionPlayer) -> None:
    player.set_expression("happy", 3)
    g1 = player._gesture                        # noqa: SLF001
    assert g1 is not None
    player.set_expression("happy", 3)
    g2 = player._gesture                        # noqa: SLF001
    assert g2 is not None
    assert g2 is not g1
