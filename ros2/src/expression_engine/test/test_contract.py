"""Story 7.6 — body-owned interface contract artifact.

The contract is a versioned, self-contained artifact under
``<pkg>/contract/``. These tests enforce that the published artifact
and the live code can never silently diverge:

- ``contract/VERSION`` == ``schema.INTERFACE_VERSION`` (and the map's
  ``interface_version``, checked by test_map_loader).
- The committed JSON Schemas == what the live pydantic models generate
  (AC#2 single-source / no-drift). Regenerate after a sanctioned model
  change with ``python -m expression_engine.contract_schemas``.
"""

import json
from pathlib import Path

from expression_engine import schema
from expression_engine.contract_schemas import SCHEMAS_DIR, build_schemas

CONTRACT = Path(__file__).resolve().parents[1] / "contract"


def test_version_file_matches_schema():
    version = (CONTRACT / "VERSION").read_text().strip()
    assert version == schema.INTERFACE_VERSION


def test_interface_md_present():
    assert (CONTRACT / "INTERFACE.md").is_file()


def test_committed_schemas_in_sync_with_models():
    # The single-source guard: the published JSON Schemas MUST equal
    # what the live models emit. If this fails, a model changed without
    # regenerating — run `python -m expression_engine.contract_schemas`.
    generated = build_schemas()
    assert set(generated) == {
        "envelope", "mood", "activity", "speech_emotion", "vocalization",
    }
    for key, sch in generated.items():
        path = SCHEMAS_DIR / f"{key}.schema.json"
        assert path.is_file(), f"missing committed schema: {path}"
        committed = json.loads(path.read_text())
        assert committed == sch, (
            f"{path.name} is stale vs the models — regenerate with "
            f"`python -m expression_engine.contract_schemas`"
        )


def test_envelope_schema_is_strict_payloads_are_lenient():
    # Encodes the 7.6 design: envelope rejects extras (forbid),
    # payloads tolerate additive fields (forward-compat).
    s = build_schemas()
    assert s["envelope"].get("additionalProperties") is False
    for payload in ("mood", "activity", "speech_emotion", "vocalization"):
        assert s[payload].get("additionalProperties") is not False
