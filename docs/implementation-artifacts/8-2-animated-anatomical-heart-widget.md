# Story 8.2: Animated anatomical heart widget

Status: ready-for-dev

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

- [ ] Task 1: Heart art + glow-from-within (AC: #1)
  - [ ] Render a stylized anatomical heart centered in the TL cell; warm palette (crimson→coral→amber), rounded/simplified, no clinical detail
  - [ ] Inner glow via a **pre-rendered radial gradient** blitted additively under/over the heart body (fake SSS); glow brightness is the focal pulse [Saved Question #1 — art source]
- [ ] Task 2: Lub-dub beat engine (AC: #2)
  - [ ] Drive a beat phase clock; map phase → (scale, glow) with **asymmetric easing** (fast systolic contraction, short gap, smaller S2, slow diastolic relaxation)
  - [ ] Contraction = scale down ~8% + glow flash; relaxation = slow ease back to baseline
- [ ] Task 3: Always-alive resting beat + jitter (AC: #3)
  - [ ] Default resting rate ~60–80 bpm feel; apply ±5–8% per-interval jitter so it is never metronomic [Saved Question #2 — exact bpm/jitter, Kamal-tuned]
  - [ ] Beats with zero external input (no engine, no data) — the resting beat is the fallback, not an add-on
- [ ] Task 4: Wake animation (AC: #4)
  - [ ] One-shot on first show: glow ignites from dark → first lub-dub → settle into resting cadence
- [ ] Task 5: Performance (AC: #5)
  - [ ] Pre-render the glow gradient once; use hardware/additive blits; **no per-pixel Python** in the loop
  - [ ] Confirm ≥30 fps on the Pi 5 with the heart animating in the dashboard
- [ ] Task 6: Input interface for 8.4 (AC: #6)
  - [ ] `set_beat_rate(bpm)`, `set_intensity(level)`, `set_tint(color)` — mirror the frozen `expression_map.yaml` `heart: {bpm, intensity, color}` fields so 8.4 maps the forwarded state 1:1
  - [ ] Defaults produce the resting beat; methods present but unused this story (no engine link)
- [ ] Task 7: Tests / verification (AC: #2, #3, #6)
  - [ ] Unit: beat-phase function is periodic and asymmetric (systole faster than diastole); jitter stays within bounds and is non-zero
  - [ ] Unit: `set_beat_rate/intensity/tint` change the widget's target state (no display needed)
  - [ ] On the Pi: visually warm (not clinical), glow pulses with the beat, never static, ≥30 fps

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

### Debug Log References

### Completion Notes List

### File List
