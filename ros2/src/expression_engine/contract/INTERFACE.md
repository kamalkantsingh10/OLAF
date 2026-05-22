# OLAF Expression Interface — Producer Publishing Contract

**Status:** the body-owned, versioned interface contract (Story 7.6).
Interface version **`1.0.0`** (`contract/VERSION`); wire envelope
`schema_version: 3`, re-derived from companion `v3.0.0`.

**Audience:** anyone building a **producer** (today: `olaf_companion`'s
voice-agent pipeline; tomorrow: a test rig, a joystick, `ros2 topic
pub`, or a different AI brain) that wants to drive an OLAF-class body.

---

## The model

This interface is **owned by the body** (the expression engine) and
implemented by every OLAF-class robot. A **producer** publishes
**abstract intent**; the body maps that intent onto whatever DOF it
actually has (eyes / ears / neck / LEDs / heart). The producer **never**
sends actuator commands — it does not know, and must not assume, what
hardware the body has.

```
PRODUCER  ──(publishes intent)──▶  4 ROS 2 topics  ──▶  BODY (subscribes, renders)
(companion)                                              (expression_engine)
```

The data flow is producer → body (pub/sub). The body is **subscribe-only**
on these topics; it never publishes them.

---

## Transport

| Property | Value |
|---|---|
| Middleware | ROS 2 (DDS) |
| DDS domain | `0` (default; configurable both sides — **must match**) |
| Message type | `std_msgs/String` — `.data` is a **UTF-8 JSON object** (the envelope) |
| Encoding | JSON; one envelope per message |

> The body never depends on a ROS `.msg`/IDL package for these topics —
> the payload is plain JSON validated against a schema. This keeps
> producers loosely coupled (even a non-ROS or polyglot producer can
> participate).

### Topics & QoS

QoS **must match** or DDS will not connect the endpoints even with the
right topic name.

| Topic | Kind | Reliability | Durability | Depth |
|---|---|---|---|---|
| `/olaf/mood` | **state** | RELIABLE | **TRANSIENT_LOCAL** (latched) | 1 |
| `/olaf/activity` | **state** | RELIABLE | **TRANSIENT_LOCAL** (latched) | 1 |
| `/olaf/speech_emotion` | **event** | RELIABLE | VOLATILE | 8 |
| `/olaf/vocalization` | **event** | RELIABLE | VOLATILE | 8 |

- **State topics** (`mood`, `activity`) are **latched**: publish them
  `TRANSIENT_LOCAL` so a body that joins *after* you still receives the
  current value. Publish **on change**, and re-publish the current value
  on startup.
- **Event topics** (`speech_emotion`, `vocalization`) are momentary:
  publish **one message per occurrence**. No replay.

---

## The envelope

**Every** message on **every** topic is this envelope. All five fields
are **required on every publish**. Unknown **envelope** fields are
**rejected** (`extra = "forbid"`) — but payloads are forward-compatible
(additive payload fields are tolerated; see *Producer rules*).

```json
{
  "schema_version": 3,
  "timestamp": "2026-05-23T10:15:30.123456Z",
  "source": "voice_agent_pipeline",
  "correlation_id": "550e8400-e29b-41d4-a716-446655440000",
  "payload": { "...": "topic-specific, see below" }
}
```

| Field | Type | Required | Notes |
|---|---|---|---|
| `schema_version` | integer | ✅ | **Must be exactly `3`.** Absent / non-int / ≠3 is **fatal** to the body (it `exit(1)`s). |
| `timestamp` | string (ISO-8601 datetime, UTC) | ✅ | When the event occurred. |
| `source` | string (non-empty) | ✅ | Producer identity — any id (`voice_agent_pipeline`, `test_rig`, …). |
| `correlation_id` | string (UUID) | ✅ | Ties related events together (e.g. one speech turn). |
| `payload` | object | ✅ | Topic-specific; one of the four below. |

---

## Payloads

### `/olaf/mood` — `MoodPayload`

```json
{ "mood": "curious", "reason": "user asked a question" }
```

| Field | Type | Required | Notes |
|---|---|---|---|
| `mood` | enum (8, below) | ✅ | |
| `reason` | string \| null | ➖ | Optional, human-readable. |

`mood` ∈ `calm` · `happy` · `playful` · `curious` · `thoughtful` ·
`sleepy` · `grumpy` · `excited`

### `/olaf/activity` — `ActivityPayload`

```json
{ "state": "working", "working_submode": "thinking",
  "from_state": "listening", "transition_reason": "received a task" }
```

| Field | Type | Required | Notes |
|---|---|---|---|
| `state` | enum (7, below) | ✅ | |
| `working_submode` | `thinking` \| `delegating` \| null | conditional | **Non-null iff `state == "working"`.** |
| `from_state` | activity enum \| null | conditional | **`null` iff `state == "starting"`** (the boot event); otherwise required. |
| `transition_reason` | string \| null | ➖ | Optional. |

`state` ∈ `starting` · `sleeping` · `waking` · `listening` · `working` ·
`speaking` · `going_to_sleep`

The boot event:

```json
{ "state": "starting", "working_submode": null,
  "from_state": null, "transition_reason": null }
```

### `/olaf/speech_emotion` — `SpeechEmotionPayload`

```json
{ "emotion": "excited", "source_tag": "llm:sentiment",
  "audio_frame_id": "frame_42", "raw_tag": "very_excited",
  "resolved_fallback": null }
```

| Field | Type | Required | Notes |
|---|---|---|---|
| `emotion` | string | ✅ | Canonical name (12, below). `str` on the wire for forward-compat. |
| `source_tag` | string | ✅ | Where the emotion came from. |
| `audio_frame_id` | string \| null | ➖ | Anchors the emotion to an audio frame. |
| `raw_tag` | string | ✅ | The pre-resolution tag. |
| `resolved_fallback` | string \| null | ✅ (nullable) | The fallback used if `raw_tag` was unknown, else `null`. **Must be present.** |

`emotion` ∈ **primary:** `neutral` · `content` · `excited` · `sad` ·
`angry` · `scared` — **secondary:** `happy` · `curious` · `sympathetic`
· `surprised` · `frustrated` · `melancholic`

### `/olaf/vocalization` — `VocalizationPayload`

```json
{ "tag": "laughter", "audio_frame_id": "frame_99", "tts_supported": true }
```

| Field | Type | Required | Notes |
|---|---|---|---|
| `tag` | string | ✅ | Canonical tag (6, below). `str` on the wire for forward-compat. |
| `audio_frame_id` | string \| null | ➖ | |
| `tts_supported` | bool | ✅ | |

`tag` ∈ **audio:** `laughter` · `sigh` · `gasp` · `clears_throat` —
**silent gesture cues:** `nod` · `shake`

> `nod` / `shake` are **visible-only** (no audio) — they're physical
> yes/no responses. The body renders them as silent gestures.

---

## Producer rules (do these, or the body rejects you)

1. **All five envelope fields on every publish.** No omission.
2. **`schema_version` must be the integer `3`.** Not `"3"`, not absent.
   Getting this wrong **crashes the body** (fatal by design — no silent
   drift).
3. **No extra envelope fields** (`extra = forbid`). **Payloads**,
   however, tolerate *additive* fields — a newer interface MINOR may add
   payload fields and older bodies ignore them (forward-compat).
4. **Honour the conditional invariants** on `activity`
   (`working_submode` / `from_state`).
5. **Publish only canonical vocabulary.** Unknown `emotion`/`tag` values
   are *tolerated at runtime* (the body falls back to a default — FR13)
   but are not guaranteed to render meaningfully. Adding a new term is a
   **contract change** (see *Governance*), not a unilateral producer edit.
6. **Match the QoS** in the table above.
7. **State vs event:** latch + publish-on-change for `mood`/`activity`;
   one volatile message per occurrence for the event topics.

### How the body reacts to a bad message

| Problem | Body behaviour |
|---|---|
| `schema_version` absent / non-int / ≠ 3 | **Fatal** — structured log + `exit(1)` (systemd restarts). |
| Malformed JSON, extra **envelope** field, bad enum, broken invariant | **Rejected loudly** (structured `event_rejected` log) and **dropped** — the body keeps running on its last good state. |
| Additive **payload** field (unknown but well-formed) | **Ignored** (forward-compat) — render proceeds normally. |
| Unknown `emotion`/`tag` (otherwise valid) | Accepted; body falls back to a default render (FR13). |

---

## Governance

Adding/removing a mood / activity state / emotion / vocalization, or
changing the envelope or any payload, is a **contract change** — it bumps
the interface version and updates this document. Either side may
*propose* it, but it lands **here** (body-owned). This is what keeps the
producer and body from silently drifting once they live in separate
repos.

---

## Source of truth

This contract is **re-derived, not imported** (intentional zero
cross-repo build coupling). The authoritative mirror lives in
`ros2/src/expression_engine/expression_engine/schema.py` (models
re-derived from companion `v3.0.0`, commit `321d9f8`). The
machine-readable `contract/schemas/*.json` are **generated** from those
models (`python -m expression_engine.contract_schemas`); a test fails on
drift. The interface version is `contract/VERSION` — mirrored as
`schema.INTERFACE_VERSION` and cross-checked against the map's
`interface_version` at startup. Any edit here must stay in lockstep.

---

## Implemented in Story 7.6

This interface is now a **body-owned, versioned standard**:

- **Versioned, body-owned contract** — `contract/VERSION` (`1.0.0`) is
  the governance version; `schema.INTERFACE_VERSION` mirrors it and the
  map's `interface_version` is cross-checked at startup. The old
  `pinned_companion_tag` (which pinned the companion *release*) is gone.
- **Forward-compatible payloads** — additive payload fields tolerated
  (`extra="ignore"`); the envelope stays strict.
- **`source` relaxed** — any non-empty producer id.
- **Namespaceable topics** — relative names (`expression/mood`) scoped
  by the node namespace; `/olaf/*` remains the default.
- **Machine-readable JSON Schemas** — `contract/schemas/*.json`,
  generated from the models, drift-guarded by a test.

### Versioning model

The wire **`schema_version`** (int `3`) is the **major** gate — a
breaking wire change bumps it (fatal mismatch). Minor/additive changes
ride payload forward-compat. There is intentionally **no separate wire
`interface_version` field** — that keeps the current companion working
with no change. `contract/VERSION` (semver) is the governance version of
the whole spec.

### Still planned (deferred)

- **`expression/capabilities`** (latched) — the *one* topic the body
  would publish, announcing its interface version + which channels it
  drives, so producers adapt to heterogeneous hardware. **Deferred** — it
  introduces the body's first publisher; pursue as a follow-up.
- **Neutral shared-interface package** — extract `contract/` into a
  standalone package once a second hardware body exists (`git mv`).
