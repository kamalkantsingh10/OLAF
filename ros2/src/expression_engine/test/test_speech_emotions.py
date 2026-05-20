"""Story 7.1 — speech_emotion authoring + expressions.py retirement.

AC#1/#4: the packaged map's ``speech_emotion`` block covers EXACTLY
the 12 ``schema.SPEECH_EMOTION_CANONICAL`` names, every entry matches
the frozen §5.2-Amendment shape (``pose{neck,ears}`` numeric, ``eye``
{expression, intensity 1-5}, token ``led_overlay``), and still loads
under the hardened ``map_loader`` value validation.

AC#3/AR11/NFR5: ``head_ears_driver/expressions.py`` no longer exists
and NO source module imports it — the map is the single source of
expression data.

This complements (does not duplicate) ``test_map_loader.py``'s
*superset* coverage check: 7.1 pins the *exact* set + entry shape and
adds the retirement guard.
"""

import ast
from pathlib import Path

import pytest

from expression_engine import schema
from expression_engine.map_loader import load_expression_map

PKG = Path(__file__).resolve().parents[1]
PACKAGED_MAP = PKG / "config" / "expression_map.yaml"
REPO = Path(__file__).resolve().parents[4]
RETIRED_MODULE = (
    REPO / "ros2" / "src" / "olaf_drivers" / "head_ears_driver"
    / "head_ears_driver" / "expressions.py"
)

_LED_TOKENS = {"none", "warm", "cool", "hot", "bright"}


@pytest.fixture(scope="module")
def smap():
    return load_expression_map(PACKAGED_MAP).speech_emotion


# ── AC#1/#4 — exact coverage + frozen entry shape ───────────────────


def test_loads_under_hardened_map_loader():
    # Whole-map load proves no non-numeric pose/intensity, no dup keys.
    m = load_expression_map(PACKAGED_MAP)
    assert m.speech_emotion


def test_covers_exactly_the_12_canonical(smap):
    assert set(smap) == set(schema.SPEECH_EMOTION_CANONICAL)
    assert len(schema.SPEECH_EMOTION_CANONICAL) == 12


@pytest.mark.parametrize("name", schema.SPEECH_EMOTION_CANONICAL)
def test_entry_matches_frozen_shape(smap, name):
    e = smap[name]
    assert isinstance(e, dict)

    pose = e["pose"]
    for axis, keys in (
        ("neck", ("pan", "tilt", "roll")),
        ("ears", ("left_pan", "left_tilt", "right_pan", "right_tilt")),
    ):
        part = pose[axis]
        assert isinstance(part, dict) and part, f"{name}.pose.{axis}"
        for k, v in part.items():
            assert k in keys, f"{name}.pose.{axis} bad key {k!r}"
            assert isinstance(v, (int, float)) and not isinstance(
                v, bool
            ), f"{name}.pose.{axis}.{k} not numeric"

    eye = e["eye"]
    assert eye["expression"] == name, f"{name} eye.expression"
    assert isinstance(eye["intensity"], int)
    assert 1 <= eye["intensity"] <= 5, f"{name} eye.intensity range"

    assert e["led_overlay"] in _LED_TOKENS, f"{name} led_overlay"


# ── AC#3/AR11/NFR5 — expressions.py retirement guard ────────────────


def test_expressions_module_deleted():
    assert not RETIRED_MODULE.exists(), (
        f"{RETIRED_MODULE} must be deleted — the map is the single "
        "source of expression data (AR11/NFR5)"
    )


def test_no_source_imports_expressions():
    """No .py under ros2/src imports the retired ``expressions``.

    AST-based so it ignores unrelated ``.expressions`` *attribute*
    access (e.g. ``self.expressions``) — only real ``import`` of an
    ``expressions`` module fails the guard. build/install artifacts
    and this guard file itself are skipped.
    """
    src = REPO / "ros2" / "src"
    offenders: list[str] = []
    for py in src.rglob("*.py"):
        parts = set(py.parts)
        if {"build", "install", "__pycache__"} & parts:
            continue
        if py.resolve() == Path(__file__).resolve():
            continue
        try:
            tree = ast.parse(py.read_text(), filename=str(py))
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for a in node.names:
                    if a.name.split(".")[-1] == "expressions":
                        offenders.append(f"{py}: import {a.name}")
            elif isinstance(node, ast.ImportFrom):
                mod = node.module or ""
                if mod.split(".")[-1] == "expressions":
                    offenders.append(f"{py}: from {mod} import ...")
    assert not offenders, "modules still import expressions:\n" + "\n".join(
        offenders
    )
