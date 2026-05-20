# Phase 2 Architecture — Expression Engine

**Status:** Draft for review · **Author:** Winston (Architect) · **Date:** 2026-05-15
**Implements:** `docs/prd/phase2-prd.md` · **Anchored on:** `docs/sprint-change-proposal-2026-05-15.md`, companion `olaf-embodiment-brief.md` (Appendix A/B)

> This is the *how* for Phase 2. It is the artifact Dev needs before Epic 6. It is deliberately concrete at the Protocol and schema level (those are the contracts that freeze in Story 6.4). Decisions already locked in the SCP/PRD are not re-opened here.

---

## 1. Scope & Posture

The `expression_engine` ROS 2 package is a **single long-running, subscribe-only consumer** of the `olaf_companion` pipeline's four canonical topics. It owns every "what does that look like?" decision via `expression_map.yaml`; it owns no "what does that mean?" decision. It never publishes to the four topics. It imports Phase 1 driver Python classes **in-process** behind Protocol-shaped adapters — no intermediate ROS topics for low-level control.

```
┌─ olaf_companion (separate repo) ─────────────────────────────┐
│  STT · reasoning · personality · mood/activity/emotion FSM   │
│  Publishes std_msgs/String (EventEnvelope JSON, schema 3):   │
│   /olaf/mood  /olaf/activity  /olaf/speech_emotion           │
│                                /olaf/vocalization            │
└───────────────────────────────┬──────────────────────────────┘
                                 │ ROS 2 / DDS (RELIABLE, subscribe-only)
┌────────────────────────────────▼─────────────────────────────┐
│ expression_engine (this repo, one ROS 2 node)                │
│                                                              │
│  subscribers ─► schema(validate, fail-fast) ─► state model   │
│       map_loader (expression_map.yaml, pinned-vocab strict)  │
│                          │                                   │
│                  render_loop (fixed tick)  ◄── idle FSM      │
│                          │                                   │
│        ┌─────────────────┼───────────────┬──────────┐        │
│     neck_adapter     ears_adapter     eye_adapter  led/heart  │
│   (continuous)       (continuous)    (delegating)  (engine-   │
│        │                 │                │         owned)    │
└────────┼─────────────────┼────────────────┼──────────────────┘
   NeckServoDriver   EarsServoDriver   HeadI2CClient   WS2812 / 4" display
   (Phase 1, in-process)                (smart peripheral: ESP32 owns eye anim)
```

---

## 2. The Defining Distinction — Continuous vs Delegating Adapters

Inspection of the Phase 1 drivers (grounding, not assumption) yields two adapter *kinds*. This is the most important architectural decision in Phase 2; everything downstream follows from it.

| Kind | Drivers | Engine responsibility | Tick? |
|---|---|---|---|
| **Continuous** | `neck_adapter` → `NeckServoDriver`, `ears_adapter` → `EarsServoDriver` | Engine computes target joint angles from the map, **interpolates per render tick**, and issues `move_*` calls. Easing, anticipatory window, gesture attack all live in the engine. | Yes — driven by `render_loop` |
| **Delegating** | `eye_adapter` → `HeadI2CClient` | Engine forwards a **semantic** `set_expression(name, intensity)` / `trigger_blink()` / `set_look_direction(x,y)` to the Head ESP32, which owns 60 FPS eye animation (Phase 1 Smart Peripheral Pattern). Engine does **not** interpolate eyes per tick. | No — fire-on-change |
| **Engine-owned** | `led_adapter` (WS2812), `heart_adapter` (4" display) | No Phase 1 driver exists; the engine renders these directly. Continuous for LED breathe; fire-on-change otherwise. | LED: yes; heart: low-rate |

**Consequence:** the render loop interpolates *neck + ears* every tick; it pushes *eye* state only on transitions (and lets the ESP32 ease); it ticks *LED* for breathe and pushes *heart* at a low rate. The 30–80ms anticipatory window (NFR2) is therefore enforced for neck/ears in the loop, and for eyes by *sending the semantic event early* (the ESP32's own ramp covers the rest).

---

## 3. Component Breakdown

`ros2/src/expression_engine/expression_engine/`

| Module | Responsibility |
|---|---|
| `node.py` | ROS 2 node lifecycle; owns the executor; wires subscribers → state → render thread; startup-validation sequence (§9). |
| `schema.py` | `EventEnvelope` + 4 payload pydantic models (schema-3). `assert_schema_version` → raise/journald/exit on mismatch (FR4). |
| `subscribers.py` | Four `std_msgs/String` subscriptions; deserialize JSON → `schema.py` models; hand validated events to the state model. |
| `state.py` | Holds current **mood** (base), **activity** (base posture), last **speech_emotion** (overlay, with TTL), pending **vocalization** queue. The single source of "what should the body be expressing now". |
| `map_loader.py` | Load + validate `expression_map.yaml` against the pinned canonical set (FR6, NFR5); enforce `visible_only` on `nod`/`shake` (FR7). |
| `render_loop.py` | Fixed-tick loop: compose target pose from state + map (§5), interpolate, drive continuous adapters; push delegating/engine-owned adapters on change. |
| `idle.py` | Idle FSM (§7). Decides when state→render falls back to ambient. |
| `adapters/base.py` | The Protocols (§4). |
| `adapters/{neck,ears,eye,led,heart}_adapter.py` | Concrete adapters wrapping Phase 1 classes / hardware. |
| `config/expression_map.yaml`, `config/expression_engine.toml` | Vocabulary→render + service config. |

**Concurrency model:** one `rclpy` executor thread handles subscriptions and writes to `state` (lock-guarded, last-write-wins per topic — the companion already dedups `speech_emotion`). A separate **render thread** runs the fixed-tick loop reading `state`. State is a small mutex-protected snapshot; the render loop copies it per tick and never blocks on DDS. This keeps the tick jitter-free regardless of event cadence.

---

## 4. Adapter Protocols (the contract that freezes in Story 6.4)

> 🔒 **FROZEN — 2026-05-17, Story 6.4, proven on real hardware @ commit `0f43dfa`.** The three Protocols (`ContinuousAdapter`, `DelegatingAdapter`, `SurfaceAdapter`) + `SurfaceFrame` in `ros2/src/expression_engine/expression_engine/adapters/base.py` are LOCKED. Concrete `neck`/`ears`/`eye` adapters were instantiated and the reference `happy` expression rendered end-to-end on neck + ears + eyes (mock publisher → engine → real drivers). **Epic 7 authors expression *content* only — no Protocol signature changes.** Any post-freeze change is a documented amendment with re-proof on hardware.

`adapters/base.py` — structural `typing.Protocol`s. Concrete adapters wrap the real Phase 1 driver classes; signatures below are grounded in the actual driver APIs.

```python
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

@runtime_checkable
class ContinuousAdapter(Protocol):
    """Joint-space adapter the render loop ticks. Engine owns interpolation."""
    def connect(self) -> None: ...           # fatal on failure (NFR7)
    def close(self) -> None: ...
    def apply(self, targets: dict[str, float], speed_pct: float = 0.0) -> None:
        """Issue absolute joint angles (deg). Called every tick with the
        loop's interpolated values — the adapter does NOT ease."""
    def neutral(self) -> dict[str, float]:
        """The joint-space 'centered' pose, for idle/return-to-base."""

@runtime_checkable
class DelegatingAdapter(Protocol):
    """Semantic adapter for a smart peripheral that owns its own animation."""
    def connect(self) -> None: ...
    def close(self) -> None: ...
    def set_expression(self, canonical_name: str, intensity: int) -> None: ...
    def blink(self) -> None: ...
    def look(self, x: int, y: int) -> None: ...

@dataclass(frozen=True)
class SurfaceFrame:                # the render() payload — frozen with the Protocols
    led_color: str = "#404040"
    led_intensity: float = 0.2
    led_pattern: str = "solid"
    heart_bpm: int = 60
    heart_intensity: int = 2
    heart_color: str = "#802020"

@runtime_checkable
class SurfaceAdapter(Protocol):
    """Engine-owned surface (LED strip, heart display)."""
    def connect(self) -> None: ...
    def close(self) -> None: ...
    def render(self, frame: SurfaceFrame) -> None: ...   # called per tick (LED) / low-rate (heart)
```

`SurfaceFrame` (defined for the freeze — Story 6.3 shipped it in `adapters/base.py`; no real surface adapter consumes it until §6.6/8.1). The three Protocols (`ContinuousAdapter`, `DelegatingAdapter`, `SurfaceAdapter`) are **unchanged** from the original §4 and freeze AS-IS in Story 6.4.

**Concrete mappings (grounded in Phase 1 driver signatures):**

| Adapter | Wraps | Maps Protocol call → driver call |
|---|---|---|
| `neck_adapter` | `NeckServoDriver` | `apply({"pan":p,"tilt":t,"roll":r})` → `move_pose(pan=p, tilt=t, roll=r, speed=…)`; `neutral()` → centered triple; `connect/close` → driver ctor / `close()` |
| `ears_adapter` | `EarsServoDriver` | `apply({"left_pan":…,"left_tilt":…,"right_pan":…,"right_tilt":…})` → `move_left_pan/left_tilt/right_pan/right_tilt(deg, speed_pct)`; `neutral()` → `center_all()` targets |
| `eye_adapter` | `HeadI2CClient` | `set_expression(name,intensity)` → `set_expression(name,intensity)`; `blink()` → `trigger_blink()`; `look(x,y)` → `set_look_direction(x,y)`; `connect()` → `open()` |
| `led_adapter` | *(engine-owned WS2812)* | `render(frame)` → drive 24 LEDs from `frame.led_color/intensity/pattern` |
| `heart_adapter` | *(engine-owned 4" display)* | `render(frame)` → draw heart from `frame.heart_bpm/intensity/color` |

`NFR6` (one-adapter hardware swap) is satisfied by construction: anything new implements one Protocol; nothing else changes.

---

## 5. `expression_map.yaml` Schema & the Layering Model

### 5.1 Layering / composition

The body output each tick is composed in a fixed order (companion brief §"What Makes This Different" #4/#5):

```
ACTIVITY  → base posture (held)            e.g. listening = upright
   └─ MOOD → subtle modifier on the base   small lean/LED bias only (low impact, PRD 7.4)
        └─ SPEECH_EMOTION → overlay        per-utterance; decays to base after 3s silence
             └─ VOCALIZATION → punctuation transient gesture added on top, then released
```

Disambiguation is **by topic, not by name** — `mood.happy` and `speech_emotion.happy` are different keys with different renders (brief #4). The render loop resolves: `target = compose(activity_base, mood_bias, speech_overlay, active_vocalization)`.

### 5.2 Schema (frozen by Story 6.4 — Epic 7 authors against this, no schema changes)

> 🔒 **FROZEN — 2026-05-17, Story 6.4, proven on real hardware @ commit `0f43dfa`.** The `expression_map.yaml` key structure (pose `neck`/`ears`; `eye`; `led`/`heart`; `mood` lean/led bias; `activity` incl. nested `working`; `speech_emotion`; `vocalization`= `{visible_only}`; `defaults`) and the §5.1 layering/composition order are LOCKED, AS AMENDED in §5.2-Amendment below. The reference `speech_emotion: happy` (seeded from `EMOTION_HAPPY`) composed correctly across activity+mood+speech layers and rendered on hardware. **Epic 7 authors *content* (entry values) only — no key-structure changes.** `schema_version` stays `1`. Timing/values in `expression_engine.toml` are NOT part of this freeze (tunable, NFR3). Post-freeze structural change = documented amendment + hardware re-proof.

```yaml
schema_version: 1                  # engine map's own version, independent of pipeline
pinned_companion_tag: "vX.Y.Z"     # canonical vocabulary is validated against THIS release

defaults:                          # FR13 runtime fallback for unmapped names
  pose:  { neck: {pan: 0, tilt: 0, roll: 0}, ears: {left_pan: 0, left_tilt: 0, right_pan: 0, right_tilt: 0} }
  eye:   { expression: neutral, intensity: 3 }
  led:   { color: "#404040", intensity: 0.2, pattern: solid }
  heart: { bpm: 60, intensity: 2, color: "#802020" }

mood:                              # ALL Mood values; subtle modifier only
  calm:    { lean_bias: 3,  led_bias: {color: "#a0c0ff", intensity: 0.3} }
  happy:   { lean_bias: 5,  led_bias: {color: "#ffd060", intensity: 0.5} }
  # … remaining Mood values …

activity:                          # ALL ActivityState values; one base posture each
  sleeping:       { pose: {neck:{tilt:-25}}, eye: {expression: closed},  led: {pattern: breathe_slow} }
  waking:         { pose: {neck:{tilt:0}},   eye: {expression: waking},  led: {pattern: fade_in} }
  listening:      { pose: {neck:{tilt:5}},   eye: {expression: open},    led: {pattern: ear_breathe} }
  working:
    thinking:     { pose: {neck:{pan:-12}},  eye: {expression: focused}, led: {pattern: pulse_blue} }
    delegating:   { pose: {neck:{pan: 12}},  eye: {expression: distant}, led: {pattern: spinner} }
  speaking:       { pose: {neck:{tilt:3}},   eye: {expression: animated},led: {pattern: warm_active} }
  going_to_sleep: { pose: {neck:{tilt:-15}}, eye: {expression: closing}, led: {pattern: fade_out} }
  starting:       { pose: {},                eye: {expression: boot},    led: {pattern: boot_seq} }

speech_emotion:                    # ALL 12 first-class names; overlay on mood base
                                   # pose MAY set neck and/or ears partials
  neutral:  { pose: {neck:{tilt:0}, ears:{left_tilt:0, right_tilt:0}},   eye: {expression: neutral, intensity: 3}, led_overlay: none }
  happy:    { pose: {neck:{tilt:6}, ears:{left_tilt:15, right_tilt:15}}, eye: {expression: happy,   intensity: 4}, led_overlay: warm }
  # … remaining first-class emotions, seeded from head_ears_driver/expressions.py …

vocalization:                      # ALL tags; gesture cues MUST have visible_only: true.
                                   # v1 FROZEN SHAPE: { visible_only: bool } ONLY.
                                   # The tag itself selects the code-side parametric
                                   # trajectory (render_loop._Gesture). No `gesture:`
                                   # name, no `audio_asset:` — the engine produces NO
                                   # audio (TTS is the companion's; brief §A.7).
  laughter:      { visible_only: false }
  sigh:          { visible_only: false }
  gasp:          { visible_only: false }
  clears_throat: { visible_only: false }
  nod:           { visible_only: true }   # enforced (FR7)
  shake:         { visible_only: true }   # enforced (FR7)
```

Composition merges partial `pose` dicts; missing joints inherit from the layer below. Gesture *shapes* are parametric joint trajectories defined in engine code (attack/settle from config); the vocalization **tag is the selector** — the map only asserts `visible_only`. A new gesture shape is a code change (rare, safety-sensitive — §12.4 #3).

#### 5.2-Amendment — 2026-05-17 (freeze reconciliation, pre-Story-6.4)

Reconciles §5.2 to the shipped `ros2/src/expression_engine/config/expression_map.yaml` (Stories 6.2/6.3) so Story 6.4 freezes a consistent spec. Two owner-directed changes, both *tightening* toward the architecture's existing intent:

1. **`vocalization` entry shape → `{ visible_only: bool }` only.** Dropped the `gesture:` name and `audio_asset:` from the §5.2 example. Rationale: §12.4 #3 already mandates gesture shapes be parametric engine code and the map stay declarative/unable to express unsafe motion; the v1 tag set is fixed at 6 and the *tag* is a sufficient selector, so the `gesture:` indirection bought nothing v1 needs while leaking motion intent into YAML. `audio_asset:` removed because the engine produces no audio (TTS is the companion's responsibility; gesture cues `nod`/`shake` must be silent — brief §A.7). **`visible_only: true` on `nod`/`shake` remains a startup-fatal invariant (FR7).** Vocalization is **structurally frozen but content-deferred**: richer per-tag animation parameters (when v1 vocalizations become full laugh / yes-no animations) are a *future documented amendment*, **not** Epic 7 authoring. Epic 7 authors `mood` / `activity` / `speech_emotion` content only.
2. **`speech_emotion.pose` carries `neck` and `ears`.** Not a schema change — `pose` always admitted both (see `defaults.pose`); the prior example was illustratively ears-only. Example updated to be representative.

`schema_version` stays `1` (no structural break for the consumed contract; the example correction + the vocabulary-deferral note do not change the loader's parse). `mood` / `activity` (incl. nested `working`, `starting`) / `defaults` are unchanged and already match the shipped map. `led` / `heart` keys freeze now even though their adapters arrive later (LED §6.6, heart 8.1) — NFR6's one-adapter swap depends on those keys being fixed.

**Story 6.4 freezes §5.2 AS AMENDED HERE** (this subsection + the updated code block above are the authoritative schema).

---

## 6. Animation & Timing Model

- **Tick:** `render_loop` runs at `servo_tick_hz` (default 100Hz, config). Each tick: snapshot `state` → `compose()` target → ease current→target → `continuous.apply(...)`.
- **Easing:** critically-damped interpolation toward target. Mood uses a long time-constant (`mood_ease_seconds` 2–4s, NFR3); speech_emotion a short one; activity medium.
- **Anticipatory window (NFR2):** `speech_emotion` payloads carry `audio_frame_id`. The engine targets the pose to *arrive* at `(audio_anchor − emotion_anticipatory_ms)` where `emotion_anticipatory_ms` ∈ [30,80] (config default 50). For **continuous** adapters this means biasing the easing start; for the **delegating** eye adapter this means *sending `set_expression` early* and letting the ESP32's own ramp land in-window.
- **Gestures (NFR4):** vocalization gestures are short joint trajectories with `gesture_attack_ms` (~80) and `gesture_settle_ms` (~200); they superimpose on the composed base and release — overall land ≤150ms.
- **Wake (NFR1):** `activity sleeping→waking` short-circuits easing to begin head-lift/eye-open within 100ms (event-receipt to first motion), then eases the remainder.

---

## 7. Idle FSM

Owned by `idle.py`; consulted by `render_loop` to decide whether the composed target is "live" or "ambient".

```
            speech_emotion within 3s
        ┌──────────────────────────────┐
        ▼                              │
   [ACTIVE] ── no speech_emotion >3s ──► [AMBIENT]
        ▲   (activity=listening)         │  hold mood-tinted neutral pose
        │                                │  + micro-movement + breath-LED
        └──── any new event ─────────────┘
   activity=sleeping ─► [DORMANT] eyes-closed posture + slow LED breathe
   activity=starting  ─► [BOOT]   boot sequence, then ACTIVE
```

- `AMBIENT` and `DORMANT` are never statue-still (FR10): a low-amplitude sinusoidal micro-movement (sub-degree neck) + `breath_period_seconds` LED breathe.
- Transitions out of `AMBIENT`/`DORMANT` are immediate on any qualifying event.

---

## 8. Configuration (`expression_engine.toml`)

Mirrors companion brief Appendix B.1. Lives in the package `config/`.

```toml
schema_version = 1
[dds]        domain_id = 0
[topics]     mood="/olaf/mood"  activity="/olaf/activity"
             speech_emotion="/olaf/speech_emotion"  vocalization="/olaf/vocalization"
[hardware]   neck_port="…"  ears_port="…"  head_i2c_addr=0x08
             led_strip_count=24  led_strip_pin=18  heart_display="spi-0.0"
[animation]  servo_tick_hz=100  led_tick_hz=30
             mood_ease_seconds=3.0  emotion_anticipatory_ms=50
             gesture_attack_ms=80  gesture_settle_ms=200
[idle]       return_to_neutral_after_seconds=3.0  breath_period_seconds=4.0
```

Hardware ports/addresses are read from / cross-checked against the existing `config/servo-ids.yaml` (Phase 1) — the engine does not duplicate calibration; calibration stays the single source of truth.

---

## 9. Startup Validation Sequence (NFR7 — every step fatal)

```
1. Load expression_engine.toml + (no secrets in v1)
2. Load expression_map.yaml
3. Assert vocabulary completeness vs pinned_companion_tag canonical set   (FR6)
4. Assert visible_only:true on nod & shake                                (FR7)
5. adapter.connect() for every configured adapter, in sequence            (NFR7)
6. DDS init + subscribe to all 4 topics on the domain
7. All green → start render thread → state RUNNING
```

Any failure → clear operator-facing message, exit non-zero, journald captures, systemd restarts. Identical posture to the companion.

---

## 10. Testing Strategy

- **Mock publisher** (`test/mock_publisher.py`, FR14): drives all four topics with schema-3 events; scriptable sequences; the dev/CI substitute for the live pipeline.
- **Schema tests:** reject `schema_version != 3`; reject malformed envelopes.
- **Map-loader tests:** reject incomplete vocabulary; reject `nod/shake` missing `visible_only`; accept a synthetic added entry with no code change (proves NFR5).
- **Regression harness** (`test/test_locked_expressions.py`, NFR10): replays recorded event sequences, asserts composed pose/LED/eye/heart targets against locked fixtures. Re-lock workflow documented for vocabulary growth.
- **Phase-1 non-regression (NFR11):** existing component test scripts run unchanged in CI.

---

## 11. Deployment

Single Pi 5, co-located with the pipeline over loopback DDS (NFR9). systemd unit, `Restart=always`, structured JSON logs to journald (NFR8). Multi-host over LAN is DDS-supported but not the v1 timing-validated scenario (cross-host clock sync would re-open NFR2).

---

## 12. Resolved Architectural Decisions (locked 2026-05-15)

1. **Eye vocabulary translation → `eye_adapter`.** A name→ESP32-expression-string table lives as data inside `eye_adapter.py`. `expression_map.yaml` stays purely canonical; the hardware-specific eye vocabulary is hidden in the adapter. Swapping the eye display = rewrite that one table (consistent with NFR6).
2. **`expressions.py` → migrate then retire.** Epic 7.1 lifts the existing emotion→ear-pose preset *data* into `expression_map.yaml` and **deletes `head_ears_driver/expressions.py`**. The map is the single source of expression data (NFR5). *(Implication for SM: add an AC to Story 7.1 — "expressions.py removed; no module imports it.")*
3. **Gestures → parametric in engine code.** The engine ships a tested library of parametric gesture trajectories (attack/settle from config). The vocalization **tag is the selector** (`render_loop._Gesture`); `expression_map.yaml` only asserts `visible_only` per tag (see §5.2-Amendment — the earlier `gesture:`/`audio_asset:` example fields were dropped). The map stays declarative and cannot express unsafe motion. A brand-new gesture *shape* is a code change (acceptable — gesture shapes are rare and safety-sensitive).
4. **Schema → re-derive from Appendix A.** `schema.py` defines our own pydantic models matching the companion's documented JSON shape. The pinned companion tag is recorded in `expression_map.yaml` (`pinned_companion_tag`) and cross-checked at startup. **Zero cross-repo build coupling.** Envelope drift is caught by FR4 fail-fast + schema tests; on a companion bump we update `schema.py` + re-pin in lockstep (NFR5).

---

*Next: SM expands Epic 6 stories against §3–§9; Dev implements after the Epic 5.3 gate. The Protocols (§4) and map schema (§5) are the freeze points — change them only via a documented amendment. **Pre-freeze reconciliation applied 2026-05-17** (see §5.2-Amendment + the §4 `SurfaceFrame` note). **🔒 FROZEN 2026-05-17 — Story 6.4 proved the reference `happy` expression end-to-end on real hardware @ commit `0f43dfa`; §4 Protocols and §5.2 schema (as amended) are LOCKED. Epic 7 = content only.***
