# Research Paper — Meeting Brief

**Collaboration:** Kamal Singh × Dr. Alaa Khamis (Assoc. Prof., KFUPM; IEEE SMIEEE; Dir. AI for Smart Mobility Lab; IEEE ITS Saudi Chapter Chair)
**Topic:** Physical / embodied AI, OLAF as case study
**Status:** pre-meeting alignment

---

## 1. Project pitch (2 min)
OLAF is a low-cost expressive avatar: an LLM "companion" publishes affective intent over a small schema-based topic contract; a decoupled **expression engine** translates that into head/neck/ear motion on hobby servos. The architecture deliberately separates *affective reasoning* (LLM) from *embodied expression* (low-cost hardware) — reproducible on a hobbyist budget.

## 2. Proposed contribution — anchor this early
Not a descriptive case study. The defensible, testable contribution:

> **A layered idle-motion policy for embodied expression** — N authored poses per expression + re-pick-on-change-or-timeout (±jitter) + continuous procedural micro-drift — and its measured effect on perceived aliveness/agency, with per-expression amplitude reconciling liveliness against an energy/relax-when-idle constraint.

Delta vs. prior idle/breathing-motion work = the *specific re-pick-or-timeout policy* + the calm-state relax reconciliation, proven by ablation. State this precisely or reviewers call it incremental.

## 3. Study design (the empirical spine)
- Within-subjects, video-based, online (Prolific). ~30–40 participants.
- **Conditions:** (1) Static, (2) Procedural-drift only, (3) Full policy. Identical expression sequence/timing across conditions; only motion policy varies. Order counterbalanced.
- **Measures:** Godspeed II (Animacy) + III (Likeability) + forced-choice "most alive" + attention checks.
- **Hypothesis:** Full > Procedural-only > Static on Animacy.
- **Analysis:** Friedman + post-hoc, effect sizes.
- Doubles as engineering validation: tells us if the 3-pose layer earns its 3× authoring cost.

## 4. Proposed division of labour
- **Kamal:** artifact, expression engine, idle-motion policy, stimuli/video production, embodied/affective domain, draft.
- **Alaa:** methodological rigour, ethics/IRB & affiliation, IEEE venue access, study-design review, senior-author guidance.

## 5. Venue & timeline (align early — don't let this drift)
- Candidates: HRI (full or LBR), RO-MAN, HAI, an embodied-AI workshop, or IEEE Access. Pick one that serves *embodied/affective* positioning, not generic/ITS-adjacent.
- Rough timeline: policy implemented in Epic 7 (in progress) → stimuli (1–2 d) → survey + pilot (1–2 d) → data (1–3 d) → analysis + write-up (~1 wk). ≈ a few weeks of marginal effort on top of Epic 7.

## 6. Kamal's interest points — raise these in the meeting, as normal
- **Authorship order:** Kamal first author (own project + artifact); Alaa senior/last. Settle now.
- **Open-sourcing the platform:** confirm in scope and acceptable (portfolio value).
- **Scope / time commitment** on Kamal's side; **IP** of OLAF work.
- **Venue decision** made explicitly, not by default.

## Meeting outcomes to leave with
1. Agreed contribution framing (empirical, not pure case study).
2. Target venue + deadline.
3. Authorship order confirmed.
4. Division of labour + next checkpoint.
