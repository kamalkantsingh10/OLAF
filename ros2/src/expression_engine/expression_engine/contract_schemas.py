"""Generate the published JSON Schemas for the expression interface
(Story 7.6).

The pydantic models in :mod:`expression_engine.schema` are the SINGLE
SOURCE of truth; the committed ``contract/schemas/*.json`` are
GENERATED from them. ``test/test_contract.py`` re-generates in-memory
and asserts equality, so the published contract and the live models can
never drift (AC#2).

After a *sanctioned* model change, regenerate + commit the schemas:

    python -m expression_engine.contract_schemas

This is an OWNER action — it is NOT on the green test path (the test
FAILS on drift, it does not auto-rewrite).
"""

from __future__ import annotations

import json
from pathlib import Path

from expression_engine import schema

#: model key -> pydantic model. The key becomes
#: ``contract/schemas/<key>.schema.json``. ``envelope`` documents the
#: shared 5-field envelope; the other four are the per-topic payloads.
_MODELS = {
    "envelope": schema.EventEnvelope,
    "mood": schema.MoodPayload,
    "activity": schema.ActivityPayload,
    "speech_emotion": schema.SpeechEmotionPayload,
    "vocalization": schema.VocalizationPayload,
}

#: <pkg>/contract/schemas (self-contained — see contract/INTERFACE.md).
SCHEMAS_DIR = Path(__file__).resolve().parents[1] / "contract" / "schemas"


def build_schemas() -> dict[str, dict]:
    """Return ``{key: json-schema-dict}`` for every interface model."""
    return {key: model.model_json_schema() for key, model in _MODELS.items()}


def _dumps(obj: dict) -> str:
    # Deterministic: sorted keys + stable indent + trailing newline so a
    # regeneration is a byte-for-byte no-op when nothing changed.
    return json.dumps(obj, indent=2, sort_keys=True) + "\n"


def write_schemas(dest: Path = SCHEMAS_DIR) -> list[Path]:
    """(Re)write every JSON Schema into ``dest``; return the paths."""
    dest.mkdir(parents=True, exist_ok=True)
    written = []
    for key, sch in build_schemas().items():
        path = dest / f"{key}.schema.json"
        path.write_text(_dumps(sch))
        written.append(path)
    return written


if __name__ == "__main__":  # pragma: no cover
    for p in write_schemas():
        print(f"wrote {p}")
