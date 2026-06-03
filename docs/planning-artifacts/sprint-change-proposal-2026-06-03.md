# Sprint Change Proposal — Chest Display & Animated Heart

**Date:** 2026-06-03
**Author:** Kamal (with Claude)
**Trigger:** Epic 8 design session — the heart surface is a 4.3" DSI chest panel with a full dashboard, not a low-rate engine-drawn display.
**Scope classification:** **Moderate** (backlog reorganization + doc edits; **no code rollback** — the heart adapter and Epic 8 were never built).

---

## Section 1 — Issue Summary

Epic 8 originally specced the heart as an **Engine-owned `SurfaceAdapter`** that the expression-engine render loop draws directly to a "4" SPI display" at a low rate (FR12, AR1, PRD Epic 8).

The design session + on-robot inspection (2026-06-03) established a different reality:

- The chest panel is a **LUCKFOX 4.3" DSI capacitive touchscreen (800×480)** on DRM `card0`/`DSI-2`, **already kernel-rotated to portrait** (`video=DSI-2:800x480,rotate=90` → app draws **480×800**). It is **not** an SPI display.
- The panel hosts far more than a heart: a **portrait 2×2 dashboard** (animated anatomical heart top-left + three log panels) and a **view-manager** for full-screen takeover by a single app, returning to the dashboard when done.
- Driving a 30 fps GPU framebuffer + dashboard + logs from inside the engine's servo-tick thread is the wrong place for it.

**Owner decision (this session):** the heart's *emotional input* must come from `expression_map.yaml`, composed by the engine **exactly like neck/ears/eyes** — not from the chest app independently subscribing to the companion. The chest app is a **renderer that owns its animation**, mirroring how the **ESP32 owns eye animation** (the existing Delegating / Smart-Peripheral pattern).

## Section 2 — Impact Analysis

| Artifact | Impact |
|---|---|
| **PRD** `phase2-prd.md` | FR12 reworded (engine-owned-draw → delegating/forward to chest renderer); NFR6 heart-swap wording; Epic 8 section + summary table rewritten. |
| **Architecture** `phase2-expression-engine.md` | §2 adapter classification: heart **Engine-owned → Delegating surface** (+ amendment note); §4 `heart_adapter` row note (render() forwards over IPC); config example (`heart_display="spi-0.0"` → IPC to chest app). |
| **§4 FROZEN Protocols** | **No change.** `SurfaceFrame`, `SurfaceAdapter`, `DelegatingAdapter`, `ContinuousAdapter` signatures are untouched — `heart_adapter` still implements `render(SurfaceFrame)`. Freeze is **not** broken; no hardware re-proof triggered. |
| **`expression_map.yaml` §5.2 freeze** | **No structural change** (freeze holds). New content/coverage rule (§4.6): `heart:` on every `speech_emotion` + `activity` entry, **none on `vocalization`**; eased in lockstep with the body (§4.7). |
| **Epics** `epics.md` | Epic 8 rewritten to "Chest Display & Animated Heart" (done 2026-06-03; Architecture Note corrected from "independent subscriber" → "delegating, map-driven"). |
| **sprint-status.yaml** | Epic 8 stories replaced: 8.1/8.2/8.3 MVP; 8.4–8.8 deferred. |
| **Code** | None to roll back. heart_adapter unimplemented; Epic 8 backlog. |

**Sequencing:** unchanged — Epic 8 still follows 7; still the secondary surface added after core body language. Stories 8.1–8.3 (chest-app foundation) have **no dependency on the engine** and can proceed immediately as a standalone app; 8.4 wires the engine's delegating `heart_adapter` to the chest renderer.

## Section 3 — Recommended Approach

**Direct Adjustment** (modify the plan before building). The heart becomes a **Delegating surface**:

```
expression_map.yaml (heart: bpm/intensity/color)   ← unchanged source of truth
        │  composed by engine per mood/activity/speech_emotion (like neck/ears/eyes)
        ▼
engine heart_adapter.render(SurfaceFrame)           ← same frozen Protocol…
        │  …but render() FORWARDS heart_* over local IPC (not draw-local)
        ▼
standalone chest-display app (pygame/KMSDRM on DSI) ← owns the lub-dub animation,
   portrait 2×2 dashboard: heart + 3 log panels        jitter, dashboard, view-manager
```

This mirrors the **eye / ESP32** split exactly: engine sends *semantic* state derived from the map; the smart peripheral owns the animation. Effort: doc edits only. Risk: low. Timeline: none (Epic 8 not started).

## Section 4 — Detailed Change Proposals

### 4.1 PRD — FR12
**OLD:** The engine MUST drive the 4" Pi heart display via an **engine-owned adapter**, animating it from `mood` + `activity` (this replaces the retired `HeartRate.msg`).
**NEW:** The engine MUST drive the chest heart surface via a **delegating adapter**: it composes the heart state (`bpm`/`intensity`/`color`) from `expression_map.yaml` — exactly as for neck/ears/eyes — across the **mood** (slow base), **activity** (held base, incl. `starting`/`sleeping`/`working`), and **speech_emotion** (per-utterance overlay) layers, **eased in lockstep with the body's emotion easing** (the heart eases out as the emotion eases out, never jumps independently), and **forwards** the eased state over local IPC to the standalone chest-display renderer, which owns the heartbeat animation (mirrors the eye-adapter / ESP32 split). **Vocalizations do NOT affect the heart** — punctual gestures (`nod`/`shake`) touch pose only. This replaces the retired `HeartRate.msg`.

### 4.2 PRD — NFR6
**OLD:** …(servo bus, eye display, LED strip, heart display) MUST be a single new Protocol-adapter implementation — no change to the mapping, DDS layer, or animation loop.
**NEW:** …(servo bus, eye display, LED strip, **chest heart renderer**) MUST be a single new Protocol-adapter implementation (for the heart, swapping the renderer behind the same forwarding `heart_adapter`) — no change to the mapping, DDS layer, or animation loop.

### 4.3 PRD — Epic 8 section + summary row
Rewrite "Heart Display Animation / Story 8.1 (engine-owned heart)" → "**Chest Display & Animated Heart**" with stories 8.1 (foundation), 8.2 (heart widget), 8.3 (log panels), and deferred 8.4–8.8 — matching `epics.md`.

### 4.4 Architecture — §2 adapter classification
Move heart from **Engine-owned** → **Delegating** (add a dated amendment note). LED remains the sole Engine-owned adapter. Update the §4 `heart_adapter` mapping row: `render(frame)` **forwards** `frame.heart_bpm/intensity/color` to the chest renderer over local IPC (chest app owns the animation). **Frozen Protocol signatures unchanged.**

### 4.5 Architecture — config example
`heart_display="spi-0.0"` → reference the chest-app IPC endpoint (transport TBD in 8.4: ROS topic or local socket).

### 4.6 `expression_map.yaml`
**No structural change** (freeze holds). New **content/coverage contract** (authored in Epic 7/8, validated at map-load):
- `heart: {bpm, intensity, color}` present on **every `speech_emotion`** entry and **every `activity`** entry (incl. `starting`/`sleeping`/`working` and both `working_submode`s); `mood` carries a slow heart base bias.
- `heart:` **absent on all `vocalization`** entries (and ignored if present) — vocalizations are punctual and must not perturb the heart.
- Map-load validation enforces the above (heart required on speech_emotion+activity, forbidden/ignored on vocalization).

### 4.7 Architecture — composition & easing (§5.1 / AR6)
Heart `bpm`/`intensity`/`color` participates in `compose(activity_base, mood_bias, speech_overlay)` — **not** `active_vocalization` — and is **eased by the engine with the same critically-damped time-constants as the pose** (mood 2–4s, speech short ease-out, activity medium). The engine forwards the *eased current* heart state to the chest renderer each meaningful change, so heart easing tracks the body's emotion easing exactly; the chest app renders the beat from the current value and owns only the lub-dub animation.

## Section 5 — Implementation Handoff

**Scope: Moderate → PO/DEV.**
- **Doc edits** (this proposal): apply 4.1–4.5 to PRD + architecture; reconcile `epics.md` Architecture Note + `sprint-status.yaml`.
- **Then:** create story files for 8.1 / 8.2 / 8.3 via `bmad-create-story`.
- **Success criteria:** PRD/architecture/epics consistently describe the heart as a *delegating surface*; the frozen §4 Protocols and §5.2 map freeze remain intact; chest-app MVP (8.1–8.3) is buildable standalone; 8.4 reserved for engine↔chest wiring.
