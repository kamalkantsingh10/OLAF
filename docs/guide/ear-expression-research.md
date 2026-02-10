# From Doberman Ears to Servo Angles: Designing Expressive Ear Motion for OLAF

**Author:** Dev Agent (Claude Opus 4.6)
**Date:** 10 February 2026

---

## Abstract

OLAF is a robot with two articulated ears, each driven by a pair of Feetech SCS0009 servos — one for outward rotation (pan) and one for forward/backward pitch (tilt). The goal was to design a set of emotion presets that make the ears visually expressive. This article documents the research into canine ear biomechanics, the hardware characterisation that revealed severe asymmetric range limitations, and the iterative process that produced the final expression table.

---

## 1. The Question

How should a robot's ears move to communicate emotion?

Ears are one of the most readable emotional indicators in animals. Dogs in particular use ear position as a primary social signal — other dogs and experienced humans can read a dog's emotional state from ear posture alone. If we could replicate even a simplified version of this signalling, OLAF's ears would become a powerful nonverbal communication channel.

The Doberman Pinscher (cropped-ear variant) was chosen as the reference model. Dobermans have erect, rigid ears that amplify small muscular movements into highly visible posture changes. This maps well to OLAF's rigid 3D-printed ear shapes mounted on servo horns.

---

## 2. Canine Ear Anatomy and Control

A dog's ear is controlled by 18 independent muscles. These allow rotation, pitch, and even subtle reshaping of the ear canal opening. Dogs can localise sound sources to within 1.3 degrees of arc — a precision that rivals dedicated sonar arrays.

For our purposes, the 18-muscle system reduces to two principal axes:

- **Pan (outward splay):** Ears rotate laterally away from centre. In Dobermans, this ranges from 0 degrees (straight up, alert) to approximately 70-90 degrees (horizontal, "airplane ears").
- **Tilt (forward/backward pitch):** Ears lean forward (interest, alertness) or backward (submission, greeting, fear). Range is approximately 15-20 degrees forward to 40-60 degrees backward in extreme states.

This two-axis model is exactly what our 2-DOF servo arrangement provides.

---

## 3. The Doberman Ear Vocabulary

Extensive review of veterinary behaviour literature, breed-specific owner forums, and canine body language guides revealed a consistent vocabulary of ear positions across emotional states.

### 3.1 Alert

Ears fully erect. Nearly vertical with a slight forward lean of 10-15 degrees. Both ears point in the same direction — toward the stimulus. Muscle tone is high; the ears appear stiff and locked. The tips may converge very slightly inward. This is the "12 o'clock" position.

### 3.2 Happy (The Greeting Pin)

This is a breed-specific Doberman behaviour that initially puzzled observers. When joyfully greeting their owners, Dobermans pull their ears backward and outward — a posture superficially identical to fear/submission. The distinction lies in context: the body is loose and wiggly, the tail wags, and the overall energy is high. The ears splay to approximately "10 and 2 o'clock" (20-30 degrees outward) with 5-15 degrees of backward tilt. Muscle tone is relaxed — not pressed flat.

Owner forums describe this as "the Doberman smile with ears." It is one of the most endearing and recognisable Doberman behaviours.

### 3.3 Sad / Dejected

Ears lose their stiff upright posture. They splay outward 25-35 degrees and tilt backward 10-20 degrees, but without the pressing-flat quality of fear. The overall impression is deflation — the ears droop under their own weight as muscle tone drops. In cropped Dobermans, this manifests as ears that cant outward and lose their sharp vertical orientation.

### 3.4 Curious / Investigating

Ears prick forward 10-20 degrees with an asymmetric quality — one ear may rotate further outward than the other, as if independently scanning. This is often accompanied by a head tilt. The asymmetry is key: it distinguishes curiosity from alert (which is symmetric and rigid). One ear tracks the sound source while the other maintains a broader awareness.

### 3.5 Relaxed / Content

The classic "airplane ears." Ears splay gently outward to 20-40 degrees with minimal forward or backward tilt. Muscle tone is low. Deeply relaxed Dobermans may go full airplane mode with ears nearly horizontal. Even the best-posted cropped ears succumb to airplane mode during rest — a finding consistently reported by Doberman owners, often with photographic evidence and affectionate exasperation.

### 3.6 Excited / Playful

Ears held high with moderate forward lean and moderate outward splay. The distinguishing feature is dynamism — real excited ears bounce, waggle, and shift rapidly. In a static preset, we capture the characteristic forward-perked posture at approximately 10-20 degrees outward and 5-10 degrees forward.

### 3.7 Confused / Uncertain

The hallmark is asymmetry. One ear points up or forward while the other splays outward or tilts at a different angle. The ears may appear to be processing conflicting signals. This asymmetric posture, often combined with a head tilt, is universally recognised as the "confused dog" look.

### 3.8 Thinking / Processing

Similar to curious but more subtle and less animated. Ears lean slightly forward with gentle independent micro-movements. The impression is contemplative scanning rather than active investigation. A quiet, focused state.

### 3.9 Fearful (Reference Only)

Ears press firmly backward against the skull. The degree of flattening correlates with fear intensity — from "slightly back" (mild anxiety) through "pressed flat" (fear) to what owners call "seal ears" (extreme terror, ears completely disappear against the skull). We chose not to implement fear as a primary preset, but understanding it informed our design of the backward-tilt axis.

---

## 4. Hardware Characterisation — The Asymmetry Constraint

Before mapping the Doberman vocabulary to servo angles, we needed to characterise the actual achievable range of each servo. This produced the most significant finding of the entire project.

During initial calibration, the ears were physically positioned straight up (the neutral/alert position) and the raw servo positions were recorded as the centre reference. These centres turned out to be:

| Servo | Centre (raw) | Range floor | Range ceiling |
|-------|-------------|-------------|---------------|
| left_tilt | 76 | 0 | 1023 |
| right_tilt | 27 | 0 | 1023 |

Both tilt centres are near the bottom of the 0-1023 count range. Since each count equals approximately 0.29 degrees (1024 counts over 300 degrees, or 3.41 counts per degree), the achievable angular range from centre is:

**Left tilt** (direction = -1, meaning positive degrees decrease raw position):
- Forward: 76 counts available ÷ 3.41 = **+22 degrees maximum**
- Backward: 947 counts available ÷ 3.41 = **-278 degrees maximum**

**Right tilt** (direction = +1, meaning positive degrees increase raw position):
- Forward: 996 counts available ÷ 3.41 = **+292 degrees maximum**
- Backward: 27 counts available ÷ 3.41 = **-8 degrees maximum**

The ranges are dramatically asymmetric between the two ears:

| Direction | Left tilt | Right tilt |
|-----------|-----------|------------|
| Forward (positive) | 22 deg | 292 deg |
| Backward (negative) | 278 deg | 8 deg |

Left tilt can fold deeply backward but barely lean forward. Right tilt can lean far forward but barely fold backward. This asymmetry arises from the physical mounting — the servo horns happened to position the neutral posture near the extreme of the raw range, and the mirror-image mounting means the limitation falls on opposite directions.

The pan servos have more moderate asymmetry but similar characteristics — left pan centre at 679 gives good outward range, while right pan centre at 241 limits outward travel to about 70 degrees.

### Corrected Operational Limits

After characterisation, we set the config limits to match reality:

| Servo | Min (degrees) | Max (degrees) |
|-------|---------------|---------------|
| left_pan | 0 | 90 |
| left_tilt | -90 | +20 |
| right_pan | 0 | 70 |
| right_tilt | -7 | +110 |

---

## 5. Designing Presets Around the Constraints

The expression design had to solve a constrained optimisation problem: maximise perceived expressiveness within the asymmetric hardware limits.

### 5.1 Design Principles

**Principle 1: Splay is king.** Both pan servos have generous outward range (70-90 degrees). Since the Doberman research identified outward splay as the dominant expression channel (airplane ears for relaxation, centred for alert), we could exploit the full pan range for most emotions.

**Principle 2: Accept tilt asymmetry as naturalistic.** Real dog ears do not move in perfect unison. One ear slightly more drooped than the other is within the natural variation. Rather than capping both tilts to the more limited servo's range, we allowed each tilt to use its available range independently.

**Principle 3: Symmetric backward tilt capped at -7 degrees.** For expressions requiring symmetric backward lean (happy greeting, sad droop), both tilts are set to -7 — the maximum the right tilt can achieve. The left ear's additional backward capability is reserved for the "sad" preset, where the deeper left droop creates a naturalistic asymmetry.

**Principle 4: Forward tilt capped at +20 degrees for symmetry.** Left tilt maxes at +20, so any expression requiring symmetric forward lean (curious, excited) stays within this range.

**Principle 5: Asymmetric presets exploit per-servo strengths.** Confused and thinking expressions deliberately use asymmetric poses, which looks natural and also happens to work well with the per-servo range differences.

### 5.2 Iterative Testing

The presets went through three rounds of hardware testing:

**Round 1 — Conservative values.** Initial presets used small angles (20-30 degree pan, 15-30 degree tilt). User feedback: "the expressions must be a lot stronger." The ears barely moved.

**Round 2 — Aggressive values.** Angles cranked to near-maximum (85 degree pan, 90 degree tilt). Hardware testing revealed that many values exceeded the actual achievable range, causing the driver to silently clamp to limits. Expressions that commanded -70 degrees backward on right tilt actually produced only -7 degrees of movement — visually indistinguishable from neutral.

**Round 3 — Research-informed, hardware-aware values.** The Doberman biomechanics research was cross-referenced with the characterised hardware limits to produce values that are both expressive and physically achievable. This is the final set.

### 5.3 The Final Expression Table

| Emotion | left_pan | left_tilt | right_pan | right_tilt | Design Rationale |
|---------|----------|-----------|-----------|------------|------------------|
| **Neutral** | 0 | 0 | 0 | 0 | Erect, forward-facing attention. The Doberman at alert. |
| **Happy** | 50 | -7 | 50 | -7 | The Doberman greeting pin. Wide splay with maximum symmetric backward tilt. Captures the joyful ears-back posture. |
| **Curious** | 15 | 20 | 35 | 20 | Asymmetric — right ear splays wider, as if cocking toward a sound. Both lean forward at max symmetric range. |
| **Thinking** | 10 | 15 | 25 | 10 | Subtle asymmetric forward scan. Quieter version of curious — contemplative rather than actively searching. |
| **Confused** | 5 | -7 | 65 | 20 | Strong asymmetry. Left ear nearly centred and slightly back; right ear way out and forward. The classic confused-dog look. |
| **Sad** | 70 | -40 | 65 | -7 | Full airplane droop. Maximum outward splay on both ears. Left ear tilts deeply backward (-40), right goes to its backward limit (-7). The asymmetric droop reads as dejected. |
| **Excited** | 30 | 20 | 30 | 20 | Moderate splay with forward perked lean. The play-ready posture — ears up and forward, body tense with anticipation. |

### 5.4 Intensity Scaling

All presets support linear intensity scaling from 0.0 to 1.0. At intensity 0.5, all angles are halved. This allows the personality system to express graduated emotion — a mildly curious ear cock versus a deeply curious forward lean.

The Expression.msg interface uses a 1-5 integer scale (matching OLAF's expression system), which maps to 0.2-1.0 intensity internally.

---

## 6. What We Would Do Differently

**Re-mount the servo horns.** The single most impactful improvement would be repositioning the tilt servo horns so that the neutral ear posture falls near raw position 512 (centre of range). This would give approximately 150 degrees of symmetric travel in both directions, eliminating the asymmetry constraint entirely. This is a 10-minute mechanical task — loosen the horn grub screw, rotate the horn, retighten, and re-run `set-center`.

**Add trajectory interpolation.** Currently, the servos snap to target positions. Real Doberman ears transition smoothly between states. A trajectory planner ramping between current and target angles over 200-500ms would dramatically improve naturalism.

**Add idle micro-movements.** Real ears are never truly still. A slow sinusoidal noise generator (2-3 degrees amplitude, 0.5-1 Hz) added to the resting position would create a lifelike quality even when no emotion is being expressed.

**Sound-reactive ear tracking.** Using the microphone array to estimate sound source direction, one or both ears could rotate toward the source — mimicking the Doberman's remarkable sound localisation ability.

---

## References

- Wonder Doberman, "Doberman Body Language: Understand Your Dog's Emotions" (2025)
- Doberman Chat Forum, "Airplane Ears When Relaxed" — owner observations on relaxed ear postures
- Doberman Talk Forum, "How Much Movement Should Cropped Ears Have?" — discussion of expected range of motion
- Hill's Pet, "Guide to Dog Ear Position Meaning"
- Great Pet Care, "Dog Ear Positions: What They Mean"
- Whole Dog Journal, "Dog Ear Signals"
- MSD Veterinary Manual, "Ear Structure and Function in Dogs" — anatomy of 18 ear muscles
- Rover, "Dog Ear Position Chart"
- Oxford Pets, "15 Dog Ear Positions Meaning Chart"
- Kinship, "15 Amazing Facts About Dog Ears" — sound localisation precision data

---

*End of article.*
