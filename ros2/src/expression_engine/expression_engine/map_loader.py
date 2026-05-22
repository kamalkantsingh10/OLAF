"""expression_map.yaml loader + vocabulary completeness — Story 6.2.

Two distinct failure modes (Dev Notes — do NOT conflate):

- **Startup (FR6/NFR7, AR8):** the map missing a canonical name the
  pinned companion release defines → **fatal**. `MapValidationError`
  (a `ValueError` subclass, so node.main's existing fail-fast handler
  catches it → structured journald + exit 1, same posture as 6.1).
- **Runtime (FR13):** an event arrives with a canonical name the map
  lacks (engine map lags companion) → **graceful**: WARN
  `expression.unmapped_<topic>` + the `defaults` render, never raise.

Strict-at-startup, graceful-at-runtime — that asymmetry is the design
(NFR5). The required canonical set is derived from `schema.py`
(single source of truth, re-derived @ INTERFACE_VERSION); never
re-listed by hand. Extra map entries beyond the pinned set are
ACCEPTED — vocabulary growth is a pure YAML edit (NFR5).

Disambiguation is BY TOPIC: `mood.happy` and `speech_emotion.happy`
are distinct keys (architecture §5.1/§5.2, brief #4).

Scope: pure/testable map data + the FR13 resolver. No rendering, no
hardware, no render loop (Story 6.3).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, get_args

import yaml

from expression_engine import schema
from expression_engine.adapters import ears_gestures, neck_gestures
from expression_engine.logging_setup import log_event

_TOPICS = ("mood", "activity", "speech_emotion", "vocalization")
_REQUIRED_TOP_LEVEL = (
    "schema_version",
    "interface_version",
    "defaults",
    *_TOPICS,
)


class MapValidationError(ValueError):
    """Startup-fatal expression_map.yaml defect (FR6/FR7/NFR7, AR8).

    Subclasses ``ValueError`` so node.main's existing
    ``except (FileNotFoundError, ValueError)`` fail-fast path handles
    it identically to a bad config (consistency with Story 6.1).
    """


class _UniqueKeySafeLoader(yaml.SafeLoader):
    """SafeLoader that REJECTS duplicate mapping keys.

    Code review 2026-05-17: `yaml.safe_load` silently keeps the last
    of duplicate keys. Epic 7 hand-authors this file; a fat-fingered
    duplicate `happy:` would silently drop the carefully-tuned entry
    and still pass the completeness check. Strict-at-startup (NFR5)
    must cover the file humans edit most.
    """


def _no_duplicate_keys(loader, node, deep=False):  # type: ignore[no-untyped-def]
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise MapValidationError(
                f"expression_map.yaml: duplicate key {key!r} "
                f"(line {key_node.start_mark.line + 1}) — a duplicate "
                f"silently drops the earlier entry; fix the map."
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeySafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _no_duplicate_keys
)


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


@dataclass(frozen=True)
class ExpressionMap:
    """Validated, queryable map data + the FR13 fallback resolver."""

    schema_version: int
    interface_version: str
    defaults: dict[str, Any]
    mood: dict[str, Any]
    activity: dict[str, Any]
    speech_emotion: dict[str, Any]
    vocalization: dict[str, Any]

    def _topic(self, topic: str) -> dict[str, Any]:
        if topic not in _TOPICS:
            raise KeyError(f"unknown expression topic: {topic!r}")
        return getattr(self, topic)

    def resolve(self, topic: str, name: str) -> dict[str, Any]:
        """Return the map entry for ``name`` on ``topic``.

        FR13: an unmapped *name* is graceful — log
        ``expression.unmapped_<topic>`` at WARN and return the
        ``defaults`` render. Never raises, never freezes. An unknown
        *topic* is a programming error and raises ``KeyError``.
        """
        entry = self._topic(topic).get(name)
        if entry is None:
            log_event(
                logging.WARNING,
                f"expression.unmapped_{topic}",
                topic=topic,
                name=name,
            )
            return self.defaults
        return entry


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise MapValidationError(msg)


def _assert_complete(
    block: dict[str, Any], required: tuple[str, ...], topic: str
) -> None:
    missing = sorted(set(required) - set(block))
    _require(
        not missing,
        f"expression_map.yaml: '{topic}' is missing canonical "
        f"entries for the pinned interface set "
        f"({schema.INTERFACE_VERSION}): {missing}. "
        f"Add them to the map — startup is fatal until complete "
        f"(FR6/NFR7).",
    )


def _check_pose(entry: dict, where: str) -> None:
    """A `pose` block (if present) must be neck/ears dicts of numbers.

    The render loop indexes `pose.neck.*` / `pose.ears.*` and feeds the
    values straight into easing math — a string/bool/null there crashes
    every 100Hz tick. Catch it at startup (NFR5/NFR7).
    """
    pose = entry.get("pose")
    if pose is None:
        return
    _require(isinstance(pose, dict), f"{where}: 'pose' must be a mapping")
    for part in ("neck", "ears"):
        sub = pose.get(part)
        if sub is None:
            continue
        _require(
            isinstance(sub, dict),
            f"{where}: 'pose.{part}' must be a mapping",
        )
        for joint, val in sub.items():
            _require(
                _is_number(val),
                f"{where}: 'pose.{part}.{joint}' must be a number, "
                f"got {val!r}",
            )


def _check_eye(entry: dict, where: str) -> None:
    eye = entry.get("eye")
    if eye is None:
        return
    _require(isinstance(eye, dict), f"{where}: 'eye' must be a mapping")
    if "intensity" in eye:
        _require(
            _is_number(eye["intensity"]),
            f"{where}: 'eye.intensity' must be a number, "
            f"got {eye['intensity']!r}",
        )
    if "expression" in eye:
        _require(
            isinstance(eye["expression"], str),
            f"{where}: 'eye.expression' must be a string",
        )


# ── Story 7.2 helpers — random ranges for vocalization layered_action ─

def _check_range_numeric(value: Any, where: str, *, allow_scalar: bool = True) -> None:
    """A range field is `[min, max]` of numbers (min ≤ max) OR a scalar.

    Story 7.2 — `vocalization.<tag>.layered_action.*` and
    `mood.<mood>.eye.intensity` all author ranges sampled per-fire.
    Scalars are accepted as fixed values (normalised by the render
    loop's sampler).
    """
    if _is_number(value):
        if allow_scalar:
            return
        raise MapValidationError(f"{where}: expected a range [min, max], got scalar {value!r}")
    _require(
        isinstance(value, list) and len(value) == 2,
        f"{where}: range must be a 2-list [min, max], got {value!r}",
    )
    lo, hi = value
    _require(
        _is_number(lo) and _is_number(hi),
        f"{where}: range bounds must be numbers, got {value!r}",
    )
    _require(
        lo <= hi,
        f"{where}: range min must be <= max, got [{lo}, {hi}]",
    )


def _check_vocalization_eye(eye: Any, where: str, *, expression_required: bool) -> None:
    """A vocalization `eye` block — { [expression], intensity: range }.

    Story 7.2 review iter: ``expression`` may be a single string OR a
    non-empty list of strings (sampled per fire — e.g. sigh =
    [content, frustrated]).
    """
    _require(isinstance(eye, dict), f"{where}: 'eye' must be a mapping")
    if expression_required:
        _require(
            "expression" in eye,
            f"{where}: 'eye.expression' is required for this vocalization",
        )
    if "expression" in eye:
        expr = eye["expression"]
        if isinstance(expr, list):
            _require(
                len(expr) >= 1 and all(isinstance(s, str) and s for s in expr),
                f"{where}: 'eye.expression' as a list must be non-empty "
                f"strings, got {expr!r}",
            )
        else:
            _require(
                isinstance(expr, str) and expr,
                f"{where}: 'eye.expression' must be a non-empty string "
                f"or a non-empty list of strings",
            )
    _require(
        "intensity" in eye,
        f"{where}: 'eye.intensity' (range or scalar) is required",
    )
    _check_range_numeric(eye["intensity"], f"{where}.intensity")


def _check_gesture_block(
    block: Any,
    where: str,
    valid_tokens: dict,
    token_lib_name: str,
) -> None:
    """A gesture component — { token: <known>, amp_deg: range, dur_s: range,
    [**extras] }.

    Story 7.2 review iter: extra keys beyond the three required ones are
    ACCEPTED and passed as kwargs to the gesture function at fire time
    (e.g. ``cycles`` for ``nod``). Each extra must still be a range
    list or scalar (validated by ``_check_range_numeric``).
    """
    _require(isinstance(block, dict), f"{where}: must be a mapping")
    for key in ("token", "amp_deg", "dur_s"):
        _require(key in block, f"{where}: missing required field {key!r}")
    token = block["token"]
    _require(
        isinstance(token, str) and token in valid_tokens,
        f"{where}: token {token!r} not in {token_lib_name}.GESTURES "
        f"({sorted(valid_tokens)})",
    )
    _check_range_numeric(block["amp_deg"], f"{where}.amp_deg")
    _check_range_numeric(block["dur_s"], f"{where}.dur_s")
    for k, v in block.items():
        if k in ("token", "amp_deg", "dur_s"):
            continue
        _check_range_numeric(v, f"{where}.{k}")


_LAYERED_ACTION_ALLOWED = {
    "attack_ms", "settle_ms", "eye", "neck_gesture", "ears_gesture",
}


def _check_layered_action(
    action: Any, where: str, tag: str, gesture_cue_tags: tuple[str, ...]
) -> None:
    """Story 7.2 — `vocalization.<tag>.layered_action` shape + ranges."""
    _require(isinstance(action, dict), f"{where}: must be a mapping")
    extras = set(action) - _LAYERED_ACTION_ALLOWED
    _require(
        not extras,
        f"{where}: unknown field(s) {sorted(extras)}; allowed: "
        f"{sorted(_LAYERED_ACTION_ALLOWED)}",
    )
    for key in ("attack_ms", "settle_ms"):
        _require(key in action, f"{where}: missing required field {key!r}")
    _check_range_numeric(action["attack_ms"], f"{where}.attack_ms")
    _check_range_numeric(action["settle_ms"], f"{where}.settle_ms")
    # Story 7.2 review iter #2: `eye` is OPTIONAL. When omitted, the
    # vocalization is body-only (composed speech_emotion eye shows
    # through). When present, audible tags author expression + intensity;
    # gesture cues (nod/shake) may NOT author expression (forward-compat
    # mood-derivation path — currently no tag uses it, but the guard
    # prevents silent misuse).
    if "eye" in action:
        expression_required = tag not in gesture_cue_tags
        _check_vocalization_eye(
            action["eye"], f"{where}.eye", expression_required=expression_required
        )
        if not expression_required:
            _require(
                "expression" not in action["eye"],
                f"{where}.eye: gesture cues (nod/shake) cannot author "
                f"eye.expression — drop the `eye` block entirely if no "
                f"override is intended, or take the expression from the "
                f"composed speech_emotion implicitly.",
            )
    if "neck_gesture" in action:
        _check_gesture_block(
            action["neck_gesture"],
            f"{where}.neck_gesture",
            neck_gestures.GESTURES,
            "neck_gestures",
        )
    if "ears_gesture" in action:
        _check_gesture_block(
            action["ears_gesture"],
            f"{where}.ears_gesture",
            ears_gestures.GESTURES,
            "ears_gestures",
        )


def _check_entries(block: dict, topic: str) -> None:
    for name, entry in block.items():
        where = f"expression_map.yaml: {topic}.{name}"
        _require(isinstance(entry, dict), f"{where} must be a mapping")
        if topic == "mood":
            if "lean_bias" in entry:
                _require(
                    _is_number(entry["lean_bias"]),
                    f"{where}: 'lean_bias' must be a number, "
                    f"got {entry['lean_bias']!r}",
                )
            if "led_bias" in entry:
                _require(
                    isinstance(entry["led_bias"], dict),
                    f"{where}: 'led_bias' must be a mapping",
                )
            # Story 7.2: mood eye accent — source for nod/shake's
            # vocalization eye when no per-vocalization expression is
            # authored. Intensity is a RANGE (sampled per fire).
            if "eye" in entry:
                _check_vocalization_eye(
                    entry["eye"], f"{where}.eye", expression_required=True
                )
        else:
            _check_pose(entry, where)
            _check_eye(entry, where)


def _check_defaults(defaults: dict, path: Path) -> None:
    """`defaults` is the FR13 runtime safety-net — validate it hardest.

    Every unmapped name renders this; a degenerate `defaults` would
    silently zero the whole body with no signal.
    """
    w = f"{path}: defaults"
    pose = defaults.get("pose")
    _require(isinstance(pose, dict), f"{w}.pose must be a mapping")
    for part in ("neck", "ears"):
        _require(
            isinstance(pose.get(part), dict),
            f"{w}.pose.{part} must be a mapping",
        )
        for joint, val in pose[part].items():
            _require(
                _is_number(val),
                f"{w}.pose.{part}.{joint} must be a number, got {val!r}",
            )
    eye = defaults.get("eye")
    _require(
        isinstance(eye, dict) and isinstance(eye.get("expression"), str)
        and _is_number(eye.get("intensity")),
        f"{w}.eye must have a string 'expression' and numeric 'intensity'",
    )
    for k in ("led", "heart"):
        _require(
            isinstance(defaults.get(k), dict),
            f"{w}.{k} must be a mapping",
        )


def load_expression_map(path: str | Path) -> ExpressionMap:
    """Load + fully validate the map; fail fast on any startup defect.

    Raises:
        FileNotFoundError: the map file is absent.
        MapValidationError: malformed YAML, a missing top-level key, an
            interface-version mismatch, an incomplete canonical vocabulary
            (FR6), or nod/shake without ``visible_only: true`` (FR7).
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"expression_map.yaml not found: {path}")

    try:
        with path.open() as fh:
            raw = yaml.load(fh, Loader=_UniqueKeySafeLoader)
    except MapValidationError:
        raise  # duplicate-key error — already specific
    except yaml.YAMLError as exc:
        raise MapValidationError(f"invalid YAML in {path}: {exc}") from exc

    _require(
        isinstance(raw, dict),
        f"{path}: expression_map.yaml must be a YAML mapping",
    )
    for key in _REQUIRED_TOP_LEVEL:
        _require(key in raw, f"{path}: missing required top-level key '{key}'")
    for key in (*_TOPICS, "defaults"):
        _require(
            isinstance(raw[key], dict),
            f"{path}: top-level '{key}' must be a mapping",
        )

    # Lockstep: the map's interface_version must equal the version
    # schema.py declares (NFR5; Story 7.6 — body-owned interface, the
    # map no longer pins the companion's release tag).
    _require(
        raw["interface_version"] == schema.INTERFACE_VERSION,
        f"{path}: interface_version "
        f"{raw['interface_version']!r} != schema.py "
        f"{schema.INTERFACE_VERSION!r}. The map and the re-derived "
        f"schema must move in lockstep (NFR5, §12.4).",
    )

    activity = raw["activity"]
    # Completeness vs the pinned canonical set (single source: schema).
    _assert_complete(raw["mood"], get_args(schema.Mood), "mood")
    _assert_complete(activity, get_args(schema.ActivityState), "activity")
    _require(
        isinstance(activity.get("working"), dict),
        f"{path}: 'activity.working' must be a mapping nested by "
        f"working_submode (thinking | delegating)",
    )
    _assert_complete(
        activity["working"], get_args(schema.WorkingSubmode), "activity.working"
    )
    _assert_complete(
        raw["speech_emotion"], schema.SPEECH_EMOTION_CANONICAL, "speech_emotion"
    )
    _assert_complete(
        raw["vocalization"], schema.VOCALIZATION_TAGS, "vocalization"
    )

    # Value/shape validation (code review 2026-05-17) — completeness
    # alone let a hand-edited string/bool/null pass startup then crash
    # every tick. Validate the FR13 safety-net + every entry's shape.
    _check_defaults(raw["defaults"], path)
    _check_entries(raw["mood"], "mood")
    for state_name, entry in activity.items():
        if state_name == "working":
            _require(
                isinstance(entry, dict),
                "expression_map.yaml: activity.working must be a mapping",
            )
            _check_entries(entry, "activity.working")
        else:
            _check_entries({state_name: entry}, "activity")
    _check_entries(raw["speech_emotion"], "speech_emotion")
    for name, entry in raw["vocalization"].items():
        _require(
            isinstance(entry, dict),
            f"expression_map.yaml: vocalization.{name} must be a mapping",
        )
        # Story 7.2 — `layered_action` shape + ranges + token validity.
        if "layered_action" in entry:
            _check_layered_action(
                entry["layered_action"],
                f"expression_map.yaml: vocalization.{name}.layered_action",
                tag=name,
                gesture_cue_tags=schema.VOCALIZATION_GESTURE_CUES,
            )

    # FR7 / AR8 step 4 — gesture cues are silent: visible_only MUST be
    # present AND exactly True.
    for cue in schema.VOCALIZATION_GESTURE_CUES:
        entry = raw["vocalization"][cue]
        _require(
            isinstance(entry, dict) and entry.get("visible_only") is True,
            f"{path}: vocalization '{cue}' must set "
            f"visible_only: true (FR7 — silent gesture, no audio). "
            f"Missing or false is fatal.",
        )

    return ExpressionMap(
        schema_version=raw["schema_version"],
        interface_version=raw["interface_version"],
        defaults=raw["defaults"],
        mood=raw["mood"],
        activity=activity,
        speech_emotion=raw["speech_emotion"],
        vocalization=raw["vocalization"],
    )
