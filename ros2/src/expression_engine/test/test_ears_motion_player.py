"""Unit tests for EarsMotionPlayer + ears_gestures (Story 7.1c)."""

from __future__ import annotations

from typing import List

import pytest

from expression_engine.adapters import ears_gestures
from expression_engine.adapters.ears_motion_player import (
    EarsMotionPlayer, JOINTS,
)


class _StubDriver:
    def __init__(self) -> None:
        self.calls: List[tuple[str, float]] = []

    def move_left_pan(self, deg: float) -> None:
        self.calls.append(("left_pan", deg))

    def move_left_tilt(self, deg: float) -> None:
        self.calls.append(("left_tilt", deg))

    def move_right_pan(self, deg: float) -> None:
        self.calls.append(("right_pan", deg))

    def move_right_tilt(self, deg: float) -> None:
        self.calls.append(("right_tilt", deg))


@pytest.fixture
def player() -> EarsMotionPlayer:
    p = EarsMotionPlayer(_StubDriver())
    p._start_t = 0.0                                  # noqa: SLF001
    return p


# ── intensity → level mapping ────────────────────────────────────
@pytest.mark.parametrize(
    "intensity,expected",
    [(1, 1), (2, 2), (3, 3), (4, 3), (5, 3), (0, 1), (-1, 1)],
)
def test_intensity_to_level(intensity: int, expected: int) -> None:
    assert EarsMotionPlayer.intensity_to_level(intensity) == expected


# ── L1: exact pose, no drift, no gesture ─────────────────────────
def test_l1_static_returns_exact_pose(player: EarsMotionPlayer) -> None:
    player.set_expression("happy", 1)
    l1 = player._emotions["happy"]["l1"]              # noqa: SLF001
    out = player.compute_target(t=0.0)
    assert out == pytest.approx((l1["left_pan"], l1["left_tilt"],
                                 l1["right_pan"], l1["right_tilt"]))


def test_l1_neutral_returns_neutral(player: EarsMotionPlayer) -> None:
    player.set_expression("neutral", 1)
    l1 = player._emotions["neutral"]["l1"]            # noqa: SLF001
    assert player.compute_target(t=0.0) == pytest.approx(
        (l1["left_pan"], l1["left_tilt"], l1["right_pan"], l1["right_tilt"]))


# ── L2: drift adds bounded perturbation ──────────────────────────
def test_l2_drift_adds_bounded_offset(player: EarsMotionPlayer) -> None:
    player.set_expression("happy", 2)
    spec = player._emotions["happy"]                  # noqa: SLF001
    l1 = spec["l1"]
    amp = float(spec["l2_drift_deg"])
    for t in (0.0, 0.5, 1.2, 3.3, 7.7):
        lp, lt, rp, rt = player.compute_target(t=t)
        assert abs(lp - l1["left_pan"])   <= amp       + 1e-6
        assert abs(lt - l1["left_tilt"])  <= amp * 0.7 + 1e-6
        assert abs(rp - l1["right_pan"])  <= amp       + 1e-6
        assert abs(rt - l1["right_tilt"]) <= amp * 0.7 + 1e-6


# ── L3: fires then decays then clears ────────────────────────────
def test_l3_fires_and_clears(player: EarsMotionPlayer) -> None:
    player.set_expression("excited", 3)
    assert player._gesture is not None                # noqa: SLF001
    t0 = player._gesture.start                        # noqa: SLF001
    dur = player._gesture.dur_s                       # noqa: SLF001
    # Mid-gesture, output differs from L1.
    spec_l1 = player._emotions["excited"]["l1"]       # noqa: SLF001
    mid = player.compute_target(t=t0 + dur * 0.5)
    assert mid != pytest.approx(
        (spec_l1["left_pan"], spec_l1["left_tilt"],
         spec_l1["right_pan"], spec_l1["right_tilt"]),
        abs=0.5)
    # After duration, gesture clears.
    player.compute_target(t=t0 + dur + 0.05)
    assert player._gesture is None                    # noqa: SLF001


# ── Safety clamp ─────────────────────────────────────────────────
def test_safety_clamp_pan_envelope(player: EarsMotionPlayer) -> None:
    # Inject an absurd gesture amp on a known gesture and verify
    # pan stays within ±50.
    player.set_expression("happy", 3)
    player._gesture.amp_deg = 500.0                   # noqa: SLF001
    t0 = player._gesture.start                        # noqa: SLF001
    dur = player._gesture.dur_s                       # noqa: SLF001
    lp, lt, rp, rt = player.compute_target(t=t0 + dur * 0.5)
    assert abs(lp) <= 50.0 + 1e-6
    assert abs(rp) <= 50.0 + 1e-6
    assert -60.0 - 1e-6 <= lt <= 90.0 + 1e-6
    assert -60.0 - 1e-6 <= rt <= 90.0 + 1e-6


# ── Fallbacks ────────────────────────────────────────────────────
def test_unknown_emotion_falls_back_to_neutral(
    player: EarsMotionPlayer,
) -> None:
    player.set_expression("doesnt-exist", 3)
    assert player._emotion == "neutral"               # noqa: SLF001


def test_unknown_gesture_token_skips_silently(
    player: EarsMotionPlayer,
) -> None:
    player._emotions["happy"]["l3_gesture"] = {       # noqa: SLF001
        "token": "no_such_token", "amp_deg": 5, "dur_s": 0.4}
    player.set_expression("happy", 3)
    assert player._gesture is None                    # noqa: SLF001
    # compute_target must not crash.
    player.compute_target(t=0.0)


def test_uppercase_emotion_normalises(player: EarsMotionPlayer) -> None:
    player.set_expression("CURIOUS", 1)
    assert player._emotion == "curious"               # noqa: SLF001


# ── Bundled YAML integrity ───────────────────────────────────────
def test_bundled_yaml_every_emotion_has_full_l1(
    player: EarsMotionPlayer,
) -> None:
    # Guards the "tilt:-16" YAML-spacing trap caught at neck-sad,
    # 2026-05-20.
    required = set(JOINTS)
    for name, spec in player._emotions.items():       # noqa: SLF001
        l1 = spec.get("l1") or {}
        missing = required - set(l1.keys())
        assert not missing, (
            f"ears emotion {name!r} L1 missing {sorted(missing)}: {l1!r}")


def test_bundled_yaml_gestures_reference_registry(
    player: EarsMotionPlayer,
) -> None:
    for name, spec in player._emotions.items():       # noqa: SLF001
        g = spec.get("l3_gesture")
        if not g:
            continue
        assert g.get("token") in ears_gestures.GESTURES, (
            f"{name}.l3_gesture token {g.get('token')!r} unknown")


# ── Gesture-registry shape invariants ───────────────────────────
@pytest.mark.parametrize("token", list(ears_gestures.GESTURES))
def test_gesture_returns_zero_at_endpoints(token: str) -> None:
    fn = ears_gestures.GESTURES[token]
    for u in (0.0, 1.0):
        out = fn(u, 10.0)
        assert all(abs(v) < 1e-6 for v in out), (
            f"{token} non-zero at u={u}: {out}")


@pytest.mark.parametrize("token", list(ears_gestures.GESTURES))
def test_gesture_peak_bounded_by_amp(token: str) -> None:
    fn = ears_gestures.GESTURES[token]
    amp = 10.0
    peak = max(
        max(abs(v) for v in fn(u / 100.0, amp))
        for u in range(101)
    )
    assert peak <= amp + 1e-6, (
        f"{token} overshoots: peak={peak} > amp={amp}")


# ── Re-fire restarts gesture ─────────────────────────────────────
def test_re_fire_l3_restarts(player: EarsMotionPlayer) -> None:
    player.set_expression("happy", 3)
    g1 = player._gesture                              # noqa: SLF001
    assert g1 is not None
    player.set_expression("happy", 3)
    g2 = player._gesture                              # noqa: SLF001
    assert g2 is not None and g2 is not g1
