---
stepsCompleted: [1]
inputDocuments: []
session_topic: "OLAF ear-movement vocabulary — what should the ears DO"
session_goals: "Define the expressive ear-motion repertoire (speaking + vocalization reactive layer first), the character each move conveys, and which DOF (pan-swivel / tilt-perk) carries it"
selected_approach: ''
techniques_used: []
ideas_generated: []
context_file: ''
---

# Brainstorming Session Results

**Facilitator:** Kamal
**Date:** 2026-06-04

## Session Overview

**Topic:** What should OLAF's ear movement be — an expressive ear-motion vocabulary.

**Goals:** Define the reactive ear repertoire (speaking + vocalizations first; ambient idle-life later), what each move *conveys*, and how the two DOF per ear carry it.

### Hardware / constraint frame
- Each ear: **PAN** (0 = up, + = outward swivel) + **TILT** (+ = forward perk, − = back droop). SCS0009 servos.
- Safe envelope: pan ±50° (right binds ≥65°), tilt −60..+90° (right_tilt physical min −7°).
- **Hard rule:** ears must NOT move while *listening* (servo noise into the mic).
- Direction locked: **mostly symmetric, occasional asymmetric accents**; reactive layer (speaking + vocalization) before any always-on ambient idle motion.
- Today's motion = one coupled tilt twitch while speaking + transient vocalization gestures (perk_up / droop / flatten / flick / twitch / swivel_in / swivel_out / one_cock).

## Technique Selection

**Approach:** Progressive Technique Flow
- Phase 1 Explore — Analogical + Cross-Pollination
- Phase 2 Pattern — Morphological Analysis (PAN×TILT×symmetry×speed grid)
- Phase 3 Develop — Emotion→Motion mapping
- Phase 4 Action — repertoire → code (tokens + speaking moves + toml) + hardware test order

### Ideas Generated

#### Phase 1 — divergent flood (long erect Doberman-style ears; 2 DOF: pan swivel + tilt perk/droop)

**Animal ethology**
1. Prick/perk — both snap forward+up, tips to listener → alert, sudden interest "!"
2. Airplane / pin-back — rotate back + flatten → displeasure, submission, "uh-oh"
3. Swivel-to-source — independent pan toward a sound → considering (speaking/thinking only; frozen while listening)
4. Radar sweep — slow side-to-side pan scan → searching/thinking
5. Single-ear cock — one up, one relaxed → quizzical "hmm?"
6. Flick — rapid single twitch (flick off a fly) → punctuation, mild irritation
7. Double flutter — both rapid multi-twitch → excitement, laughter, glee
8. Droop/melt — slow fold down+back → sadness, tiredness
9. Drop-and-spring — droop then pop back → a sigh that recovers
10. Tremble — tiny high-freq quiver → nervous, cold, fear, anticipation

**Cartoon exaggeration**
11. Antenna-shoot-up — shoot straight up + overshoot/settle → "Aha!" idea/surprise
12. Wilt — slow asymmetric droop → bad news landing
13. Blow-back — ears curl back as if in wind → shock / being yelled at
14. Springy bounce — bobble per emphatic word (long ears wobble naturally)
15. Lean-in — both tilt forward toward listener → engagement
16. Heartbeat-sync — subtle pulse matched to chest-display BPM (cross-system)

**Human-gesture analogs**
17. Brow-raise — quick symmetric perk as "oh really?" acknowledgment beat
18. Shrug — both droop-out briefly → "I dunno"
19. Nod-accent — ears dip with each neck nod (couple ears to neck gestures)
20. Cock-and-hold — asymmetric cock held through a question

**Speech-rhythm / prosody**
21. Talk-bob — bob with speech cadence (tie to syllable/emphasis, not pure random)
22. Emphasis-prick — crisp symmetric forward prick on an emphasis-tagged word
23. Energy-amplitude — overall liveliness scales with speech-emotion arousal
24. Settle-between-clauses — relax/neutralize at sentence ends, then re-engage

**Physics / secondary motion (long-ear texture)**
25. ★ Inertia overshoot — every base move gives a tip overshoot+wobble (free "alive", biggest lever for LONG ears)
26. Asymmetric settle — two ears settle at slightly different rates (not robotic)
27. ★ Constant micro-asymmetry — even "symmetric" poses keep a tiny L/R offset so they never look mechanically identical

**Emergent ★ insights:** the rigidity is mostly the absence of (25) secondary overshoot and (27) micro-asymmetry, plus motion being random rather than tied to speech events (21/22).

#### MUSE CHOSEN: Hare / jackrabbit 🐇 (tall upright-default ears, max legibility)

**The hare 3-axis emotion code:**
- TILT (fwd↔back) = engagement / valence — fwd-up = keen/positive, back-down = withdrawn/shy/sad/scared
- PAN (independent swivel / cock) = attention / cognition — swivel/cock = locating, considering, thinking
- TWITCH (rate + amplitude) = arousal — lots of tremor = excited/nervous, still = calm
- + long-light-ear secondary motion: every move overshoots, tips quiver (the aliveness)

**Hare move repertoire (pan/tilt signatures):**
- H1 Tower — bolt upright, tall, still (pan≈0, tilt≈slight fwd) → neutral attentive resting / default
- H2 Prick — sharp forward reach (tilt +fwd fast) → interest, "tell me more", alert
- H3 Pin/flatten — rotate back+down (tilt −−, pan splay) → fear, shy, scolded — withdrawn extreme
- H4 Lower-soften — tip back a little + relax (tilt −mild) → content, calm, sympathetic
- H5 Single-cock — one up / one back-swivel (asymmetric) → quizzical, curious, "hmm?", processing
- H6 Swivel-scan — slow independent pan sweep → searching, thinking, considering
- H7 Twitch ★signature — rapid tiny tremor/flick → nervous, anticipation, baseline aliveness
- H8 Flutter — both quick multi-twitch → glee, laughter, delight
- H9 Startle-bolt — snap bolt upright + freeze + tremble, slow release → surprise, gasp
- H10 Eager perk-tremble — forward AND trembling → keen anticipation

#### Phase 2 — mapping hare repertoire onto OLAF surfaces

**H3 decision:** cap the flatten — only `sleeping` gets full ears-down; scared/frustrated use a MODERATE pin + tremble (reads as emotion, not dead robot).

**12 speech-emotions → hare pose** (tilt + = fwd/perk, − = back/droop; pan 0 = up, + = outward splay):
| emotion | tilt | pan/asym | twitch | move |
|---|---|---|---|---|
| neutral | slight fwd | upright | low | Tower |
| content | mild back | upright | low | Lower-soften |
| happy | fwd | slight out | med | Prick-light |
| excited | fwd++ | slight out | high | Prick+tremble |
| surprised | bolt upright | upright | med freeze | Startle-bolt pose |
| curious | one up/one back | asymmetric | low-med | Single-cock |
| sympathetic | mild back, lean-in | upright | low | Lower-soften warm |
| sad | back−− | slight splay | none | deep droop |
| melancholic | back−−− deepest | splay + wilt-asym | none | deepest droop |
| scared | back−− capped | pull in | high tremble | Pin+tremble |
| angry | back− | slight out, rigid | none | Pin-rigid |
| frustrated | back− | asymmetric, rigid | low | Pin-uneven |

Differentiator: back-ear emotions separated by TWITCH (scared vs angry), ASYMMETRY (frustrated vs sad), DEPTH (melancholic deepest).

**Vocalizations → hare transient:** laughter→Flutter(H8) · gasp→Startle-bolt(H9) · sigh→drop-and-spring(H4) · surprised/emphasis→Prick beat(H2) · nod→ears dip with nod · clears_throat→tiny twitch · shake→still.

**Activity poses → hare:** listening→Tower (FROZEN, ears bolt up to catch sound) · speaking→Tower base + dynamic layer · thinking→Single-cock + slow swivel-scan · delegating→forward prick (outward) · sleeping→full droop · waking→rise from droop to tower.

**Speaking-rhythm layer:** H7 twitch always-on (signature) + talk-bob on cadence + emphasis-prick on stressed words + amplitude ∝ emotion arousal + mostly-symmetric w/ occasional single-ear accent + secondary-motion overshoot on every settle.

#### LOCKED behavioral model — ear-motion gating by state

| State | Ears |
|---|---|
| sleeping / going-to-sleep | full droop, still + **dream-twitch** (single tiny flick every ~10–20s random) |
| waking | one-time rise droop→tower |
| listening | Tower, **FROZEN** (mic rule) |
| speaking | full dynamic: twitch + talk-bob + emphasis-prick + emotion pose + accents + overshoot |
| thinking / delegating | ambient twitch + occasional cock/swivel |
| idle-alert (awake, pre-decay) | ambient twitch |

**Ambient hare-tremor (H7) = ALL awake-active** = every state EXCEPT sleeping / going-to-sleep / listening.
Cross-cutting: **secondary-motion overshoot** (tip wobble on settle) + **constant micro-asymmetry** (sym ≠ identical) on all ear motion — the core anti-rigidity levers.

## Phase 3/4 — Implementation plan (3 code surfaces)

**Surface A — per-emotion + activity ear POSES** (`expression_map.yaml`): re-author the 12 speech-emotion ear blocks to the hare table; set activity poses (listening=Tower, thinking=Single-cock, speaking=Tower base, sleeping=full droop). Static values, hardware-tunable.

**Surface B — vocalization gestures** (`expression_map.yaml` + `ears_gestures.py`): laughter→Flutter · gasp→Startle-bolt · sigh→drop-and-spring · emphasis→Prick · nod→dip-with-nod · clears_throat→tiny twitch · shake→still. Mostly reuse existing tokens; maybe add `startle_bolt` / `flutter`.

**Surface C — dynamic layers** (`render_loop.py` + `expression_engine.toml`):
- C1 speaking-rhythm: twitch + talk-bob + emphasis-prick + arousal-scaled amplitude + single-ear accents (started 2026-06-04: decorrelated tilt+pan+accent already added).
- C2 ambient twitch: low-amp tremor + occasional cock/swivel, gated to awake-active (NOT listening/sleeping).
- C3 dream-twitch: rare single flick while sleeping.
- C4 ★secondary-motion overshoot: allow a slight overshoot on ear settles (spring-ish), tip wobble — cheap, biggest aliveness gain.

**Build order (Kamal's sequencing):** A (poses) → B (vocalizations) → C1 (speaking) → C2/C3 (ambient + dream) ; C4 overshoot can land early (cross-cuts everything).
