---
stepsCompleted: [1, 2, 3, 4, 5, 6, 7, 8, 9]
inputDocuments:
  - docs/planning-artifacts/brief.md
  - docs/planning-artifacts/prd/phase2-prd.md
  - docs/planning-artifacts/epics.md
  - docs/planning-artifacts/architecture/phase2-expression-engine.md
  - ros2/src/expression_engine/config/expression_map.yaml
  - ros2/src/expression_engine/config/expression_engine.toml
  - scripts/olaf_demo.py
  - ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/expressions.py
---

# UX Design Specification OLAF

**Author:** Kamal
**Date:** 2026-05-19

---

<!-- UX design content will be appended sequentially through collaborative workflow steps -->

## Executive Summary

### Project Vision
OLAF is personality-first robotics — emotional connection through non-verbal,
R2D2-style body language. This UX effort authors the *expressive content* the
(frozen-schema) expression engine renders: a finalized library where each
canonical emotion maps to a designed neck/ears/eye choreography. Deliverable
per expression: a motif (intent + feeling + inspiration) and an Imagen
reference sheet the Developer parameterizes into expression_map.yaml content
and animation timing.

### Target Users
- **Primary — the human OLAF lives with:** must read OLAF's emotional state
  instantly and non-verbally; the design succeeds only if expression is
  legible at a glance and emotionally resonant.
- **Direct consumer — the Developer:** needs each expression as an
  unambiguous, parameterized reference sheet (pose values + tempo/easing +
  target eye-art) that drops into the frozen map schema.
- **Director/steward — Kamal:** authors intent, validates on hardware via
  webcam, and reuses artifacts as build-in-public + research-paper figures.

### Key Design Challenges
- Carry 12 distinct, readable emotions from a sparse rig (3-DOF neck, 4-DOF
  ears, 2 eye screens) with no words.
- Tempo/easing is the primary emotional carrier — must be a first-class,
  documented design parameter, not an afterthought (current default ease
  reads as too slow).
- Eye-display art is an open redesign track, not a fixed vocabulary.
- The reference sheet must simultaneously be Imagen-renderable and
  Developer-parameterizable.
- Author within a frozen map schema: freedom is in values + timing, not
  structure; mood stays deliberately low-impact.

### Design Opportunities
- A repeatable motif → design → reference-sheet pipeline that fixes intent
  before numbers.
- Tempo/easing as an explicitly designed dimension feeding the engine's
  timing config.
- A unified eye-art shape grammar so all emotions read as one character.
- Idle micro-motion as both an aliveness feature and the research-paper spine.
- Webcam ground-truth loop: author → render on hardware → capture → compare.

## Core User Experience

### Defining Experience
Two faces of one experience:

- **For the human (the felt experience):** OLAF's emotional state is readable
  in a single glance, non-verbally, and the body is never frozen — three idle
  pose variants per emotion plus procedural micro-drift keep it alive.
- **For the Developer/Kamal (the production experience):** a repeatable
  per-expression pipeline whose output drops into the frozen map with no
  interpretation:
  1. **Motif** — intent + feeling + collated inspiration for the emotion.
  2. **Design** — 3 idle-pose variants as concrete neck/ears/eye values +
     tempo/easing.
  3. **Reference sheet** — one Imagen image, animator expression-sheet style
     (loose, characterful, Disney-grayscale lineage), OLAF's head shown in the
     3 poses, annotated with actuator values + tempo.
  4. **Ground-truth** — render on hardware, webcam-capture, compare to sheet.
  5. **Approve** — lock; the sheet *is* the idle-motion spec for that emotion.

### Platform Strategy
- **Render target:** OLAF's physical body — neck (3-DOF: pan/tilt/roll +
  speed), ears (2× pan/tilt, asymmetry-capable), 2 eye displays (art is a
  per-expression redesign track).
- **Authoring surface:** the frozen `expression_map.yaml` schema — design
  freedom is in values + tempo, not structure; mood stays low-impact.
- **Visualization:** Imagen generates the 3-pose expression sheets.
- **Verification:** dev-PC webcam pointed at OLAF closes an author → render →
  capture → compare loop.
- **Pilot:** `happy` proves the full pipeline before scale-out.

### Effortless Interactions
- **Human:** reading OLAF's emotion requires zero thought — pose + tempo +
  eye-art land instantly.
- **Developer:** the sheet's 3 poses map 1:1 to the 3 idle variants;
  annotations are literally the map values + timing — implement without
  asking a question.
- **Kamal:** webcam compare makes "does reality match intent?" a glance, not
  an investigation.

### Critical Success Moments
- The **"it's alive"** moment — OLAF holds an emotion, drifts between its 3
  poses, and never reads as a statue or a twitch.
- The **zero-question handoff** — the Developer builds an expression purely
  from its sheet.
- The **fidelity match** — the hardware webcam capture visibly matches the
  Imagen sheet's intent.

### Experience Principles
1. **Tempo is emotion.** Timing/easing is a first-class designed parameter on
   every sheet, never an afterthought.
2. **Legible at a glance, no words.** If it needs explaining, it failed.
3. **Never statue-still.** 3 idle poses + drift is the aliveness contract.
4. **Asymmetry is vocabulary.** Independent ear poses are a primary signal,
   not noise.
5. **One artifact, no handoff loss.** The reference sheet *is* the idle spec.
6. **Coherence emerges, then is enforced.** Eyes evolve per expression; an
   eye-art ledger feeds an end-state grammar consolidation.
7. **Ground-truth on hardware.** Screen intent is provisional until the
   webcam confirms it.

## Desired Emotional Response

### Primary Emotional Goals
- **Core feeling in the human:** "There is a sincere someone in there." OLAF
  reads as an earnest presence with a genuine inner life — not a gadget
  performing tricks. The human feels companionship, tenderness, and a
  protective warmth toward OLAF (WALL-E / EVE register).
- **The signature moment:** the involuntary "...it's *alive*" — the human
  catches OLAF drifting between idle poses and forgets, for a second, that
  it's a machine.
- **Shareable emotion:** delight born of sincerity, not spectacle — "you have
  to *see* how it looks at you."

### Emotional Journey Mapping
- **First encounter:** curiosity → disarming warmth. OLAF's first look is
  gentle, attentive, a little vulnerable — it invites, never performs.
- **During an expression:** the human reads the emotion pre-cognitively
  (pose + tempo) and feels *with* OLAF — empathy, not observation.
- **Negative emotions (honest & expressive):** sadness lands with real
  heaviness (slow tempo, low droop) — the human feels for OLAF and wants to
  comfort it; anger/frustration lands genuinely sharp (fast attack, pinned
  ears) — taken seriously, not laughed off.
- **Recovery:** the return to warm-earnest baseline is itself expressive —
  the human feels relief alongside OLAF; emotions resolve, they don't just
  switch off.
- **Returning over time:** familiarity deepens into attachment — the idle
  liveliness makes OLAF feel continuously present, never "powered off."

### Micro-Emotions
- **Trust over novelty:** sincerity must beat gimmick — OLAF earns belief.
- **Tenderness over cuteness:** warmth that respects the human, not
  saccharine performance.
- **Empathy over amusement** in negative states: the human feels *with* OLAF.
- **Presence over idleness:** even at rest, OLAF feels inhabited.
- **Avoid:** uncanny/creepy, twitchy/nervous, robotic/dead, relentlessly
  cute, emotionally dishonest (a sadness that's secretly cute).

### Design Implications
- **Earnest baseline → eyes & tempo:** neutral/positive states use soft eye
  art, gentle easing, modest amplitude — open and unguarded, never slick.
- **Honest negatives → amplitude & tempo are the truth-tellers:** sad =
  genuinely slow + low (heavy), angry = genuinely fast attack + tight,
  contained pose. No softening a negative into a wink.
- **Sincerity → asymmetry used for *attention/vulnerability*, not gags:** a
  curious ear-cock reads as "I'm listening to you," not a punchline.
- **"It's alive" → idle drift is emotional, not mechanical:** the 3 idle
  poses per emotion sit *within* that emotion's feeling; drift tempo carries
  the mood (calm = near-still, excited = livelier).
- **Recovery is authored, not instant:** every emotion's sheet implies how it
  *resolves* toward baseline, not just how it peaks.

### Emotional Design Principles
1. **Sincerity beats spectacle.** If a choice reads as a trick, cut it.
2. **Tempo + amplitude tell the emotional truth.** They are not allowed to
   lie to make an emotion cuter.
3. **Warm baseline, honest peaks.** Earnest at rest; full-weight in emotion.
4. **Empathy is the target for negatives.** The human should feel *with*
   OLAF, never *at* it.
5. **Aliveness is emotional, not mechanical.** Idle motion expresses the
   current feeling; it is never filler.
6. **Resolution is part of the expression.** How an emotion fades matters as
   much as how it strikes.

## UX Pattern Analysis & Inspiration

### Inspiring Products Analysis
- **Disney/Nimona-style animated head motion (anchor — MOTION):** the neck
  must read as a *character's* head, not a servo rig. Transferable
  principles: motion on arcs (never straight servo lines); lead-and-follow
  (eyes/gaze initiate, head follows, ears trail); ease + slight overshoot
  into a settle; and the **off-axis side-glance** — head oriented a touch
  away from gaze direction, which is the single biggest "it's alive" tell.
  The full ±55° neck envelope is design vocabulary, not a limit to respect
  timidly.
- **Disney animation expression sheets (anchor — ARTIFACT STYLE):** loose
  gestural grayscale head/ear poses; "the pose tells the story" discipline;
  honest exaggeration; clear line-of-action. Defines how every OLAF
  reference sheet should look and read.
- **KAWAII eye sheets (anchor — EYE SYSTEM):** minimal eye, maximal feeling.
  No eyebrows, no lids — the eyeball alone is the whole eye. Emotion is
  carried by pupil **scale + shape + COLOR**: large/round (wonder, love,
  surprise), small/tight (anger, focus), closed-curve (bliss, sleep), with
  an emotion-specific eye color (warm = positive, cool = subdued/sad,
  hot = anger, bright = alert). Exaggeration is on-brand.
- **Labeled-emotion cat sheet (anchor — EAR SYSTEM):** ears as a primary
  emotional carrier with a discrete pose vocabulary — perk-forward, pin-back,
  droop, and left/right asymmetry — and rhythmic, lagging motion rather than
  static set-and-hold.

### Transferable UX Patterns
- **Motion:** arc-based head paths; lead-and-follow + overlapping action
  (gaze → head → ears, each trailing the last); ease-out with micro-overshoot
  and settle; deliberate off-axis side-glances; tempo as the emotional
  truth-teller.
- **Eye art:** a pupil-driven KAWAII grammar — scale, roundness,
  closed-curve states, AND eye color map to emotion families; eyeball-only
  (no brows/lids); eyes lead the gesture in time.
- **Ears:** a small, legible ear-pose lexicon (perk / pin / droop / cock)
  reused across emotions; ears always lag the head for life; asymmetry
  reserved for attention & uncertainty.
- **Artifact:** Disney expression-sheet visual language for every reference
  sheet — loose, exaggerated, pose-first, 3 poses per emotion.

### Anti-Patterns to Avoid
- **Servo-tell motion:** single-axis moves, constant velocity, simultaneous
  start/stop of all DOF, robotic linearity — the #1 thing that kills the
  illusion.
- **Timid range:** using a small slice of the ±55° envelope so motion looks
  cautious/mechanical.
- **Eyes-and-head perfectly aligned:** dead-on tracking with no side-glance
  reads as a security camera, not a character.
- **Static ears:** ears that snap to a pose and freeze (no lag, no rhythm).
- **Brows / lids / facial detail:** forbidden — the eyeball alone
  (scale/shape/color) is the entire eye system.
- **Symmetry everywhere:** perfectly mirrored ears/pose draining personality.

### Design Inspiration Strategy
- **Adopt:** Disney arc/lead-follow/overshoot motion model; pupil-scale +
  color KAWAII eye grammar; discrete ear-pose lexicon with lag; expression-
  sheet artifact style; deliberate side-glance as a signature.
- **Adapt:** translate 2D animation principles to 3-DOF neck + 4-DOF ears
  within the ±55° envelope and the frozen map schema (values + tempo only);
  scale exaggeration to what hardware reads cleanly on the webcam.
- **Avoid:** every servo-tell anti-pattern above; treat eliminating them as
  an explicit acceptance check on each expression's hardware capture.
- **Author per expression:** a specific eye color (and pupil scale/shape) is
  authored on every reference sheet — eye color is an emotional dimension,
  not a fixed value.

## Design System Foundation

### 1.1 Design System Choice
A **custom Expression Design System** layered on two fixed substrates:
the **frozen `expression_map.yaml` schema** (structure — not ours to change)
and the engine **timing config** (`expression_engine.toml`: tick rate,
mood_ease, gesture attack/settle, breath period). The design system is the
set of reusable expressive *primitives + tokens* every expression is composed
from, plus the standard reference-sheet template.

### Rationale for Selection
- No established robot-expression system exists; expression is OLAF's core
  differentiator → custom is the only fit.
- But structure is already frozen (Story 6.4) and timing is already
  configurable → we don't reinvent a foundation, we author tokens into it.
- A token system is what makes 12+ expressions read as ONE character
  (coherence principle) and makes the Developer handoff zero-question.

### Implementation Approach
Define the system as four token families, all expressible within the frozen
schema (values + tempo only):

- **Neck/motion tokens:** named tempo presets (e.g. `heavy ≈ speed 150`,
  `natural ≈ 500–800`, `snap ≈ 2500+`), the ±55° envelope, and motion-shape
  rules (arc, lead-and-follow order gaze→head→ears, ease+overshoot+settle,
  side-glance offset).
- **Eye tokens:** pupil scale steps (tiny/small/normal/large/huge),
  shape states (round / closed-curve), and a named **eye-color palette**
  keyed to emotion families (warm / cool / hot / bright / dim).
- **Ear tokens:** the pose lexicon (perk / pin / droop / cock) + asymmetry
  rule + mandatory head-lag offset.
- **Sheet template:** the standard 3-pose Disney-style reference-sheet
  layout + annotation block (each pose's neck/ears/eye values + tempo token
  + eye-color token + resolution note).

### Customization Strategy
- Each expression = a *composition of tokens*, not bespoke values, so the
  character stays coherent (eyes "evolve per expression" but draw from the
  shared palette; the eye-art ledger tracks deviations for end-state
  grammar consolidation).
- New tokens may be added during authoring, but adding one is a deliberate
  system-level decision (logged), not an ad-hoc per-expression escape hatch.
- Tokens map directly onto `expression_map.yaml` fields + `.toml` timing —
  no schema change, ever (NFR5).

## 2. Core User Experience

### 2.1 Defining Experience
**"Author one expression, end to end, until OLAF feels alive doing it."**
The defining interaction is a single repeatable loop that turns an emotion
name into a locked, hardware-true reference sheet the Developer implements
with zero questions:

  MOTIF → DESIGN → REFERENCE SHEET → HARDWARE GROUND-TRUTH → APPROVE/LOCK

If this loop is effortless and trustworthy for one expression, the full
library is just repetition. Everything else follows from nailing this.

### 2.2 User Mental Model
- **Kamal (director):** thinks like an animation director — "what is OLAF
  *feeling*, and what's the pose that tells it" — not like a servo
  programmer. The loop must let him work in intent + inspiration, with
  numbers as an output, not an input.
- **Developer (Amelia):** expects a spec that drops into `expression_map.yaml`
  + `.toml` with no interpretation — token-named, value-explicit, the 3
  poses == the 3 idle variants.
- **Failure expectation to design against:** the dread that "it looked right
  on screen but the servos make it robotic" — the loop must resolve that
  before lock, not after.

### 2.3 Success Criteria
- **Intent-first:** an expression is defined in feeling/inspiration words
  before a single number is chosen.
- **Zero-question handoff:** the reference sheet alone is sufficient to
  implement; no follow-up needed.
- **Hardware-true:** the webcam capture visibly matches the sheet's intent
  (no servo-tell, side-glance present, ears lagging, never statue-still).
- **Coherent:** the result composes from the token system; deviations are
  logged to the eye-art ledger, not silently bespoke.
- **Fast to first artifact:** the pilot (`happy`) proves the full loop in one
  sitting.

### 2.4 Novel UX Patterns
- **Familiar substrate:** animation expression-sheet convention (Disney) and
  design-token thinking — both proven; low education cost.
- **Novel combination:** the reference sheet's 3 poses ARE the engine's 3
  idle variants — the artifact and the runtime behavior are the same object.
  This is OLAF's unique twist and must be explicit on every sheet.
- **Novel verification:** an author→render→webcam→compare ground-truth gate
  built into the creative loop, not a separate QA phase.

### 2.5 Experience Mechanics
**1. Initiation** — pick the next emotion (pilot = `happy`); state its
canonical map key and where it sits (speech_emotion overlay on a mood base).

**2. Interaction**
  - *Motif:* write intent + the felt quality + collated inspiration refs for
    THIS emotion (a focused board, drawing on the system inspiration DNA).
  - *Design:* compose the 3 idle-pose variants from tokens — per pose:
    neck {pan,tilt,roll} within ±55° + tempo token; ears {4 angles} from the
    pose lexicon + lag note; eye {pupil scale, shape, color token}; plus the
    motion-shape rules (arc, gaze→head→ears lead-follow, side-glance offset,
    overshoot+settle) and a resolution-to-baseline note.
  - *Reference sheet:* generate ONE Imagen image in the Disney expression-
    sheet style — OLAF's head in the 3 poses — with the annotation block.

**3. Feedback**
  - Visual self-check against the anti-pattern list (servo-tell, timid range,
    aligned eyes/head, static ears, brows/lids, total symmetry).
  - Render on hardware via the mock publisher; capture via dev-PC webcam;
    compare capture ↔ sheet for fidelity and aliveness.

**4. Completion**
  - On match: lock the expression — its sheet becomes the idle-motion spec;
    values land in `expression_map.yaml`, timing tokens in `.toml`.
  - On mismatch: adjust tokens/values, re-render, re-capture (loop), never
    lock on screen-only confidence.
  - Output: a locked sheet + ledger entry; advance to the next emotion.

## Visual Design Foundation

### Color System
OLAF's eyes are **near-monochrome by default** — one signature color (the
established cyan/blue). Emotion is carried by pupil scale + shape, not by a
color palette. A *subtle* warm/cool undertone may shift slightly with
valence (warmer = positive, cooler = subdued), but it stays deliberately
non-obvious — imperceptible-at-a-glance for ≥85% of scenarios. Color only
becomes overt inside a special eye motif at an emotional peak (e.g. the red
flush of true anger, the shine of tears, sparkle) — the remaining <15%.
Mood stays a low-impact LED tint and does not recolor the eyes.

### Eye Special-Motif Library (anime tropes)
The eyeball is the whole eye (no brows/lids). Most expressions use the
signature near-monochrome eyeball at varying pupil scale/shape. A small,
iconic set of strong emotions instead trigger a recognizable **anime eye
motif**:
- **sad / crying:** tears / water streaming from the eyes.
- **angry:** sharp narrowed eye shape + the anime cross-vein anger mark +
  brief red flush.
- **frustrated:** blank/twitching "done with this" eyes (steam/gritted feel).
- **surprised:** oversized sparkle / star-burst eyes.
- **adoring / love (if used):** heart-shaped pupils.
- **confused / dizzy:** spiral eyes.
- **sleepy:** heavy half-lidded eyeball + ZZZ motif.
These motifs are a finite library; each is authored once as a reusable token
and invoked by the emotions that need a peak punctuation. Default for every
other state is the plain signature eyeball.

### Typography System
"Typography" = the reference-sheet annotation language (OLAF has no on-screen
text UI in scope). Sheets use the Disney expression-sheet convention:
hand-lettered-feel emotion title, then a compact annotation block per pose
(neck/ears/eye values + tempo token + eye-motif token + resolution note).
Hierarchy: emotion name > pose label > parameter block. Annotation
legibility is a hard requirement — the sheet is a spec.

### Spacing & Layout Foundation
- **Sheet layout:** one image, three poses left→right (idle variants A/B/C),
  generous margin, annotation block beneath each pose, emotion title top.
  Loose/gestural art — pose reads before detail.
- **Motion pose-space:** the ±55° neck envelope is the spatial canvas;
  principle is "use the space boldly" (anti-timid-range). Gaze, head, and
  ear vectors are intentionally non-coincident (side-glance offset) —
  spatial asymmetry is the layout grammar of aliveness.

### Accessibility Considerations
- **Never signal-by-eye-alone:** emotion is redundantly carried by pose +
  tempo + ear lexicon, so the read survives even ignoring the eyes
  (low-light / distance / webcam-safe).
- **Motifs are shape-first:** anime motifs read by silhouette (tears, spiral,
  sparkle), not by color, so they survive a colorblind / dim-light read; the
  optional color flush is reinforcement only.
- **Glance-legibility:** core metric is "readable in <1s from across a room"
  — verified at every hardware ground-truth.

## Design Direction Decision

### Design Directions Explored
Iterated Imagen reference sheets v1–v10: Disney color sketch → polished color
cartoon → photo-anchored technical engineering drawing. Imagen was then
**dropped** — it was not converging on OLAF's likeness and a picture is not
required for a usable spec. The canonical artifact is a **pure tabular
expression specification** that maps directly into `expression_map.yaml`.

### Chosen Direction
Per-expression tabular spec. Eyes are a **cyan shape-only system** (OLAF has
no lids, brows, mouth, or pupils) grounded in Disney facial-acting principles
translated to OLAF's constraint. Each expression = a precise **Eye spec
(dev)** + **3 named idle poses** (neck pan/tilt/roll + ear pan/tilt) + a
**blink** parameter + an **eye color**. The 3 poses are the 3 idle variants
the engine cycles ([[project_epic7_idle_motion]]).

### Design Rationale
- Tables are an unambiguous, zero-question Developer handoff and edit
  straight into the frozen map schema (values + tempo only).
- Disney grounding gives each emotion a defensible acting basis, not taste.
- Shape-only eyes respect the hardware (no lids/brows) while still carrying
  the emotion the way lids/brows/cheeks normally would.

### Implementation Approach
Per-expression process: propose design in words → Kamal agrees → record in
this spec. A consolidated eye-shape catalogue (below) is the single "eyes
reference sheet". Imagen reference images are optional/non-canonical.

## Expression Library Specification

### Shared eye rendering conventions
- Two eyes on the black visor: **glowing cyan FILLED shapes** with a soft
  2–3 px outer bloom. No outline, no pupil, no mouth, no brows.
- Placement: horizontally centred in the **upper-mid visor**; inter-eye gap
  ≈ **1.4 × eye-width**; eyes vertically aligned unless a pose specifies
  asymmetry.
- All sizes are **% of visor height (H)**. The eye is **static** per
  expression — only **blink** animates. **No gaze/eye drift**: head, neck,
  and ears carry 100% of motion.
- **Blink** = vertical collapse of the shape to a thin cyan line and back,
  ease-in-out. `interval` = random seconds between blinks; `duration` =
  full close→open ms; `close-style` is per emotion.

### Disney facial-cue → OLAF shape translation key
| Disney cue | Signals | OLAF cyan-shape equivalent |
|---|---|---|
| Eyelids wide open | surprise, fear, excitement | taller/fuller shape, raised in visor |
| Eyelids narrowed | anger, suspicion, stress | reduced height, compressed/slanted |
| Brow down-inward | anger | inner corners angled DOWN (sharp wedge) |
| Inner brow raised | sadness, sympathy | inner corners raised, outer drooping |
| Cheek-raise + squint | happiness | upward crescent, vertically squashed |
| Pupil dilation/constriction | arousal vs focus | glow intensity + core brightness |
| Droopy lids + distant gaze | sadness, low energy | shape lowered in visor + dimmed |
| Blink "for a reason" | thought, realization, hiding | per-emotion blink character |

Sources: NYFA 12 Principles; Animation Mentor (Facial Expressions; Blinks &
Eye Movement; Blink for a Reason); Oreate AI – Disney Facial Expressions.

### Eye-shape lexicon
`oval-steady` (calm) · `crescent-up` (happy/content) · `arc-down`
(sad/melancholic/sympathetic) · `narrow-slant` (anger/frustration) ·
`wide-round` (surprise/scared/excited) · `tall-alert` (curious). Plus per-eye
size and inner/outer-corner tilt so asymmetry is expressible.

### Pose & axis conventions
- Neck = **pan** (turn L/R) · **tilt** (chin up + / down −) · **roll** (head
  cock). Working envelope ≈ ±55° from set position; values below are
  Nimona-natural, not extremes.
- Ears (each) = **pan** (0 = upright → + outward splay) · **tilt** (+ forward
  / − back). Ears lag the head by ~1 beat (overlapping action).
- Fixed datum: scarf + cropped body identical across all 3 poses; only
  head+neck move relative to it.
- Each expression has 3 idle-pose variants the engine cycles
  ([[project_epic7_idle_motion]]). Pose ① is the primary.

### The 12 speech-emotion specs

#### neutral — calm, awake, present (the datum)
Eye: `oval-steady`, H 34%, W:H 1.2, corners level 0°, centred, glow 3,
color `#34C6DC` (signature reference) · Blink 3.0–6.0 s / 100–150 ms, even
quick close · Tempo: slowest, long ease, minimal overshoot, tiny drift

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① LEVEL REST | 0/0/0 | +5/0 | +5/0 | True datum |
| ② ATTENTIVE DRIFT | +8/+2/−3 | +6/+2 | +8/+4 | Alive "listening" lean |
| ③ SETTLE-BACK | −6/0/+2 | +5/0 | +5/0 | Quiet return |

#### happy — warm, earnest, glad
Eye: `crescent-up`, H 30% (squashed), W:H 1.6, ends +6° up, centred, glow 4
warm, color `#3CD0D0` (whisper-warm) · Blink 2.0–5.0 s / 90–140 ms, quick
happy flick · Tempo: natural ease, small overshoot→settle

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① LOOK-UP | 0/+14/0 | +18/+15 | +18/+15 | Earnest, sincere warmth |
| ② SIDE-GLANCE | +24/+7/−8 | +20/+14 | +10/+16 | Off-axis Nimona glance |
| ③ SETTLE | −16/+6/+6 | −5/+10 | −5/+10 | Eases back to baseline |

#### content — settled, quiet warmth
Eye: `crescent-up` shallow, H 26%, W:H 1.7, ends +3°, centred, glow 3,
color `#37CBD6` · Blink 3.0–6.0 s / 110–160 ms, relaxed slow close ·
Tempo: slow-natural, low amplitude

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① SOFT REST | 0/+6/0 | +10/+8 | +10/+8 | Mild glad, at ease |
| ② EASY LEAN | +6/+5/−2 | +12/+8 | +10/+6 | Gentle alive drift |
| ③ SETTLE | −5/+4/+2 | +9/+6 | +9/+6 | Sinks comfortably |

#### excited — high-energy joy
Eye: `wide-round`, H 46%, W:H 1.0, level, slightly raised, glow 5 + bright
core, color `#45E0E8` (bright) · Blink 1.5–3.5 s / 70–110 ms, snappy frequent
· Tempo: snappy, big amplitude, overshoot

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① UP-BURST | 0/+18/0 | +22/+18 | +22/+18 | Big chin-up, ears high-forward |
| ② BOUNCE-LEFT | +30/+12/−6 | +24/+16 | +18/+18 | Energetic swing |
| ③ BOUNCE-RIGHT | −28/+12/+6 | +18/+18 | +24/+16 | Can't sit still |

#### sad — honest heaviness
Eye: `arc-down`, inner +10° / outer −8°, H 22%, W:H 1.3, lowered −6% H,
glow 2 dim, color `#4F8FC8` (cool, dimmed) · Blink 4.0–8.0 s / 200–320 ms,
slow heavy droop-close, lingers shut · Tempo: very slow, no overshoot, sinks

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① DOWNCAST | 0/−16/0 | +30/−10 | +30/−10 | Honest heaviness |
| ② SLOW SAG | −6/−18/−3 | +34/−12 | +32/−11 | Sinks further, listless |
| ③ HEAVY SETTLE | +5/−15/+2 | +28/−8 | +28/−8 | Lingers low |

#### melancholic — quiet, wistful (slowest)
Eye: `arc-down` softer, inner +6° / outer −5°, H 20%, lowered −8% H, glow 1
(dimmest), color `#5878B0` · Blink 5.0–10.0 s / 260–380 ms, slowest close,
long hold · Tempo: slowest of all, drifting, low

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① QUIET DOWN | −8/−12/−3 | +24/−6 | +24/−6 | Low, gazing nowhere |
| ② DRIFT-AWAY | −16/−13/−5 | +26/−8 | +24/−6 | Slow wistful drift |
| ③ STILL-LOW | +6/−11/+2 | +22/−5 | +22/−5 | Settles low, lingers |

#### sympathetic — tender, with-you
Eye: `arc-down` gentle, inner +6° / outer −2°, H 28% soft, W:H 1.4, centred,
glow 3 warm, color `#3AC9D2` · Blink 3.0–6.0 s / 150–200 ms, soft slow
"caring" blink · Tempo: slow, gentle head-tilt toward

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① GENTLE TILT | +6/+2/+10 | +12/+8 | +12/+8 | Head-tilt = "I'm with you" |
| ② LEAN-CLOSER | +12/+4/−8 | +14/+10 | +10/+6 | Moving in to comfort |
| ③ SOFT SETTLE | −6/+2/+6 | +10/+6 | +10/+6 | Stays near, warm |

#### angry — sharp, genuine (overt color)
Eye: `narrow-slant`, inner −14° down (sharp), H 16% (narrowed), W:H 1.7,
centred, glow 4 + hard bright core, color `#FF5A33` (OVERT) · Blink
4.0–7.0 s / 80–120 ms, hard snap close · Tempo: fast attack, tense, contained

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① HARD-STARE | 0/−10/0 | +8/−14 | +8/−14 | Chin-down, ears pinned fwd-down |
| ② LEAN-IN GLARE | +12/−8/−4 | +6/−16 | +10/−12 | Pressing toward target |
| ③ HOLD | −10/−9/+3 | +8/−14 | +8/−14 | Doesn't soften |

#### frustrated — tense exasperation (overt, softer-hot)
Eye: `narrow-slant` softer, inner −9°, H 20%, W:H 1.6, centred, glow 3,
color `#FF8A4D` (OVERT, softer) · Blink 3.0–6.0 s / 90–130 ms terse,
occasional hard double-blink · Tempo: tense, clipped, turned-away

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① LOOK-AWAY | +18/−4/−4 | +14/−6 | +20/−10 | Can't even look at it |
| ② TENSE-HOLD | +24/−6/−6 | +12/−8 | +22/−12 | Jaw-clench equivalent |
| ③ HUFF-SETTLE | +8/−3/+2 | +14/−6 | +14/−6 | A beat of release |

#### scared — alarmed (honest negative)
Eye: `wide-round` tense, H 48%, W:H 1.0, thin tense edge, slight inner-up,
raised +4% H, glow 3 PALE unstable core, color `#9FD8E8` (drained) · Blink
5.0–9.0 s / 60–90 ms, suppressed, rare tiny flutter · Tempo: fast recoil →
rigid hold

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① RECOIL | 0/−6/0 | +10/−20 | +10/−20 | Pull back, ears flat back |
| ② SHRINK-AWAY | −14/−8/−5 | +14/−22 | +12/−20 | Cowering aside |
| ③ FROZEN WATCH | +8/−5/+3 | +10/−20 | +10/−20 | Rigid, can't relax |

#### curious — attentive (signature asymmetry)
Eye: `tall-alert`, H 38%, W:H 1.0, centred, glow 4 bright, color `#34C6DC` ·
asymmetry allowed: cocked-side eye +10% H larger · Blink 2.0–4.5 s /
80–120 ms, a single "realization" blink on pose change · Tempo: brisk,
head-cock + lean-in

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① HEAD-COCK | +10/+6/+12 | +6/+16 | +30/+6 | Strong ear asymmetry — "huh?" |
| ② LEAN-IN | +16/+8/−8 | +4/+18 | +26/+8 | Pressing closer |
| ③ OTHER-COCK | −12/+5/+10 | +28/+8 | +6/+16 | Mirror cock — re-examining |

#### surprised — startled (sharp, brief)
Eye: `wide-round` MAX, H 50%, W:H 1.0, perfectly round, raised +5% H, glow 5
bright core, color `#45E0E8` (bright) · Blink: held wide then one delayed
snap-blink (the realization); interval 6.0–10.0 s / 90–130 ms · Tempo: very
fast snap up/back → hold → settle

| Pose | Neck p/t/r | Ear L p/t | Ear R p/t | Rationale |
|---|---|---|---|---|
| ① SNAP-UP | 0/+16/0 | +28/+6 | +28/+6 | Head up & back, ears bolt wide |
| ② FROZEN WIDE | +10/+12/−4 | +30/+4 | +26/+6 | Caught mid-startle |
| ③ EASE-DOWN | −8/+8/+3 | +18/+8 | +18/+8 | Releasing the jolt |

### Consolidated eye-shape catalogue (the eyes reference sheet)
| Emotion | Shape | H (%visor) | W:H | Corner tilt | Vert pos | Glow | Color |
|---|---|---|---|---|---|---|---|
| neutral | oval-steady | 34 | 1.2 | level | centred | 3 | `#34C6DC` |
| happy | crescent-up | 30 | 1.6 | +6° ends | centred | 4 | `#3CD0D0` |
| content | crescent-up | 26 | 1.7 | +3° ends | centred | 3 | `#37CBD6` |
| excited | wide-round | 46 | 1.0 | level | +slight | 5 | `#45E0E8` |
| sad | arc-down | 22 | 1.3 | in +10/out −8 | −6% | 2 | `#4F8FC8` |
| melancholic | arc-down | 20 | 1.3 | in +6/out −5 | −8% | 1 | `#5878B0` |
| sympathetic | arc-down | 28 | 1.4 | in +6/out −2 | centred | 3 | `#3AC9D2` |
| angry | narrow-slant | 16 | 1.7 | inner −14 | centred | 4 | `#FF5A33` |
| frustrated | narrow-slant | 20 | 1.6 | inner −9 | centred | 3 | `#FF8A4D` |
| scared | wide-round | 48 | 1.0 | inner +slight | +4% | 3 (pale) | `#9FD8E8` |
| curious | tall-alert | 38 | 1.0 | level (asym) | centred | 4 | `#34C6DC` |
| surprised | wide-round | 50 | 1.0 | level | +5% | 5 | `#45E0E8` |

### Handoff Notes
- This library populates Epic 7 Story 7.1 content for the 12 first-class
  `speech_emotion` names, authored against the frozen schema
  ([[project_expression_map_schema]]).
- Mood / activity / vocalization render are out of this UX pass (mood stays
  low-impact per the spec; vocalization deferred to animations later).
- Webcam ground-truth ([[reference_webcam_capture]]) each expression on
  hardware before marking it locked.
