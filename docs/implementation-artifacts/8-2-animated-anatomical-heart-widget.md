# Story 8.2: Animated anatomical heart widget

Status: review

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the avatar,
I want a living, beating anatomical heart in the top-left cell,
so that my chest reads as alive and warm — not as a clinical organ diagram.

> Depends on Story 8.1 (`DashboardView` + the 240×400 top-left cell + render loop). This story builds the heart **widget** only — self-contained, always-alive, with a small input interface for later reactive wiring (Story 8.4). It is **not** driven by the engine yet.

## Acceptance Criteria

1. **Given** the top-left 240×400 cell, **When** the app runs, **Then** an anatomical heart renders centered in the cell in a **warm, painterly style** (deep crimson → coral → amber, rounded forms, simplified vessels — no surgical/medical look) with a **glow-from-within** (fake subsurface scattering) as its focal quality.
2. **Given** the heart, **When** it beats, **Then** it performs an asymmetric **lub-dub**: a fast systolic contraction (scale down ~8% + inner-glow flash), a short held gap, a smaller second beat, then a slow diastolic relaxation back to rest (fast-in / slow-out easing) so it reads as a pump, not a throb.
3. **Given** no external state is driving it, **When** the app idles, **Then** the heart still beats continuously at a resting rate (~60–80 bpm feel) with per-interval **jitter** so it is never metronomic and never static — alive by default, robust to having no data source yet.
4. **Given** the app first shows the heart, **When** it starts, **Then** a one-shot **wake** animation plays (glow ignites → first beat) rather than the heart snapping on.
5. **Given** the Pi 5 render budget, **When** the heart animates, **Then** it sustains **≥30 fps** with no per-pixel Python in the hot loop (pre-rendered glow gradient, hardware/additive blits).
6. **Given** the widget will be driven later, **When** it is built, **Then** it exposes a small interface (`set_beat_rate`, `set_intensity`, `set_tint`) for future reactive wiring — present but **not yet driven** in this story.

## Tasks / Subtasks

> **Design evolved during the live build (Kamal feedback) — see Completion Notes.** Net: **no glow** (removed — it spilled and Kamal didn't want it); the heart is a **per-emotion PNG image** (calm/neutral/content = glossy 3D red heart) rather than a procedural anatomical+glow shape; the beat is an **amplitude-scaled size throb** (extreme = 110%↔40%); the input API became **bpm + amplitude + image** (`set_profile`).

- [x] Task 1: Heart art (AC: #1) — *glow removed; art = per-emotion image*
  - [x] Heart rendered centered in the heart band — final look is a **per-emotion PNG** (`assets/heart_calm.png`, glossy 3D red), swappable via `set_image`. (Procedural anatomical + glow-from-within was built first, then dropped — Kamal: "we do not need the glow", and the glow circle spilled into other blocks.)
  - [x] No glow / no spill — each widget is also clipped to its cell as a safety net
- [x] Task 2: Lub-dub beat engine (AC: #2)
  - [x] Beat phase clock → contraction via asymmetric lub-dub envelope (fast systole, smaller dub, slow diastole) — `heart_beat.py`, tested
  - [x] Beat drives **scale** (amplitude-scaled throb), not glow
- [x] Task 3: Always-alive resting beat + jitter (AC: #3)
  - [x] Resting beat (~62 bpm default) with ±5% per-beat jitter; never metronomic — **Saved Q#2 defaults set, Kamal-tunable**
  - [x] Beats with zero external input — the calm resting beat is the fallback, not an add-on
- [x] Task 4: Wake animation (AC: #4)
  - [x] One-shot fade-in on first show (alpha ramp over 0.8s) — glow-ignite replaced by image fade-in (no glow)
- [x] Task 5: Performance (AC: #5)
  - [x] `smoothscale` of the transparent PNG per frame + alpha; **≥30 fps confirmed** (service render loop runs at 30 fps on the Pi)
- [x] Task 6: Input interface for 8.4 (AC: #6) — *API evolved to the emotion model*
  - [x] `set_beat_rate(bpm)`, `set_amplitude(a)`, `set_image(path)`, `set_profile(image, bpm, amplitude)` — emotion drives **bpm + amplitude + image** (replaces the original bpm/intensity/color). **Saved Q#1 resolved: per-emotion images** (map references image by name; chest app owns the PNGs).
  - [x] Defaults = calm resting beat; not yet engine-driven (that is Story 8.4)
- [x] Task 7: Tests / verification (AC: #2, #3, #6)
  - [x] Unit: lub-dub envelope asymmetric + bounded; jitter deterministic-with-seed; scale within amplitude bounds; extreme = 110%↔40%; wake ramp; setters — **11 beat tests + asset test (27 total in the package)**
  - [x] On the Pi: heart renders (glossy 3D — Kamal: "looks perfect"), beats, fades in, never static, 30 fps; emotion image/bpm/amplitude path smoke-tested

## Dev Notes

### The core move: a *living* heart, not an organ

The single decision that avoids the "creepy medical specimen" risk Kamal flagged: **the heart glows from within and the glow is the beat.** Light bleeds through the tissue, brightening on contraction. That turns "diagram of a human heart" into "a warm bioluminescent core that happens to be heart-shaped." Supporting rules: painterly not photoreal; warm palette only (never the purplish-grey of real muscle); soft rounded forms; gentle ambient vessel sway between beats. [Source: design session 2026-06-03; epics.md#Story-8.2]

### Lub-dub is the signature

A real heartbeat is two thumps, not one throb. Asymmetric easing (fast contraction / slow relaxation) is what reads as a *pump*. Indicative resting shape: S1 fast contraction ~120ms (ease-in) → held ~140ms → S2 ~90ms → slow diastole ~450ms (ease-out). These are starting points to feel out on hardware, not locked numbers. [Source: epics.md#Story-8.2]

### Always-alive (robustness, not decoration)

Critical principle: the heart beats at a resting rate **no matter what** — even with the engine down and zero data. Reactivity (8.4) is a *modulation layer* on top, never the on/off switch. This makes the chest surface robust to dropouts and means 8.2 ships a complete, self-contained widget. [Source: epics.md#Story-8.2, #Deferred 8.4]

### Inputs mirror the frozen map fields

The widget API (`set_beat_rate`/`set_intensity`/`set_tint`) deliberately mirrors the **frozen** `expression_map.yaml` `heart: {bpm, intensity, color}` keys (§5.2 freeze). In Story 8.4 the engine composes those fields from mood+activity+speech_emotion (NOT vocalization), eases them in lockstep with the pose, and forwards them — the chest app just calls these setters. Keep the field names/semantics aligned so 8.4 is a thin mapping. [Source: phase2-expression-engine.md#§4 SurfaceFrame heart_bpm/intensity/color, #5.2 freeze]

### Tuning belongs to Kamal, one expression at a time

Kamal tunes expressive numerics himself and prefers tunable values over hardcoded ones, tuning one thing at a time rather than batched. Expose resting bpm, jitter %, easing constants, and palette as easily-tweakable constants/config (not buried in the draw call). [Source: memory feedback_expression_tuning, feedback_eye_rendering]

### Anti-patterns

- Do **not** render a clinically realistic organ (textbook colors, labeled vessels, surgical sheen) — warm/painterly only.
- Do **not** make the beat metronomic — jitter is mandatory.
- Do **not** do per-pixel work in Python each frame — pre-render the glow, blit it.
- Do **not** wire this to the engine/ROS or read `expression_map.yaml` here — that is Story 8.4; this widget is self-contained.
- Do **not** let the heart freeze when idle — the resting beat is the default state.

### Project Structure Notes

```
ros2/src/chest_display/chest_display/
  widgets/heart.py        # NEW — HeartWidget: draw(surface, rect), update(dt), set_beat_rate/intensity/tint
  assets/                 # NEW (if pre-rendered glow/keyframes are used)
  views/dashboard.py      # UPDATE — instantiate HeartWidget in the TL cell
  test/test_heart.py      # NEW — beat-phase + setters
```
Builds on the 8.1 package skeleton. [Source: Story 8.1 Project Structure Notes]

### References

- [Source: docs/planning-artifacts/epics.md#Epic-8 — Story 8.2]
- [Source: docs/planning-artifacts/prd/phase2-prd.md#Epic-8 — Story 8.2]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#§4 (SurfaceFrame heart fields), #5.2 freeze]
- [Source: memory feedback_expression_tuning, feedback_eye_rendering — tuning preferences]

### Saved Questions

1. **Art source** — procedural vector heart (fast, ships today) vs a generated painterly anatomical sprite/keyframes (imagen, like Epic 7 images) interpolated with procedural scale+glow? Recommend procedural for MVP, swap art later.
2. **Resting bpm + jitter values** — Kamal to tune; ship sensible defaults (~70 bpm, ±6% jitter, fast-in/slow-out) as tweakable constants.

## Dev Agent Record

### Agent Model Used

claude-opus-4-8 (1M context)

### Debug Log References

- `python3 -m pytest test/ -o addopts=""` → **27 passed** (dev PC and Pi). Headless smoke-render (SDL `dummy`) of the dashboard + heart + profile change: OK.

### Completion Notes List

**Design evolution (all Kamal live feedback on hardware, 2026-06-03) — recorded because it diverges from the written ACs:**
1. **Glow removed (AC#1).** Built the procedural anatomical heart with glow-from-within first; the glow rendered as a big circle that **spilled into the other blocks**, and Kamal said "we do not need the glow." → removed the glow entirely; added per-cell clipping in the dashboard as a safety net. The beat now reads purely through the **size throb**.
2. **Per-emotion image, not procedural (AC#1 / Saved Q#1).** The look is a **full-colour PNG that swaps per emotion** (calm/neutral/content = a glossy 3D red heart, sourced). The widget owns the asset files; the expression map will reference an image by **name** (8.4). Tried a tintable white-silhouette first, but Kamal chose distinct per-emotion images.
3. **Amplitude model (AC#6).** Emotion drives **bpm + amplitude + image** (not the original `bpm/intensity/color`). `amplitude` scales the throb: at **1.0 (extreme)** the heart swings **relaxed 110% ↔ full-contraction 40%**; calm uses a small amplitude. `set_profile(image, bpm, amplitude)` is the one-call hook for 8.4.
4. **Wake** is now an alpha **fade-in** of the image (the glow-ignite is gone with the glow).

**Consequence for 8.4 (the next emotion-config story):** the frozen `expression_map.yaml` `heart:` schema `{bpm, intensity, color}` must be **amended to `{image, bpm, amplitude}`** (documented amendment to the §5.2 freeze), then authored per emotion; the engine forwards the eased values and the chest app calls `set_profile`.

**ACs:** #2 (lub-dub), #3 (always-alive + jitter), #4 (wake fade-in), #5 (≥30 fps), #6 (reactive API) — met. #1's *intent* (a warm, alive, non-clinical heart) is met via the 3D image + throb; the literal "glow-from-within" was dropped by owner choice.

### File List

_New (under `ros2/src/chest_display/`):_
- `chest_display/heart_beat.py` — pure lub-dub/jitter/wake/amplitude model
- `chest_display/widgets/__init__.py`, `chest_display/widgets/heart.py` — HeartWidget (image + beat)
- `chest_display/assets/heart_calm.png` — calm/neutral/content heart image
- `test/test_heart_beat.py`, `test/test_heart_asset.py`

_Modified:_
- `chest_display/views/dashboard.py` — instantiate HeartWidget in the `heart` slot; per-cell clipping
- `setup.py` — add `widgets` package + `assets/*.png` package data
- `docs/implementation-artifacts/sprint-status.yaml` (8-2 → review)

### Change Log

- 2026-06-03: Implemented 8.2 — lub-dub beat engine + HeartWidget; 27 tests green; verified on the panel.
- 2026-06-03 (Kamal live feedback): removed glow (spilled); switched to per-emotion PNG (glossy 3D heart); reworked to bpm+amplitude+image model (extreme throb 110%↔40%) with `set_profile`. **Story 8.2 → review.** Emotion config + engine wiring deferred to 8.4 (requires amending the `heart:` map schema to `{image,bpm,amplitude}`).
