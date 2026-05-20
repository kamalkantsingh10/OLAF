/**
 * led_strip.cpp - WS2812 LED Strip Animations
 *
 * 8 LEDs, all animations symmetric about the center seam:
 *
 *   [0][1][2][3] | [4][5][6][7]
 *    edge ←───── center ─────→ edge
 *
 * distance 0 = center pair (indices 3,4)
 * distance 3 = edge pair   (indices 0,7)
 *
 * Color theme: White (matching eye color) for all states except SPEAKING,
 * which uses full-spectrum color for expressive personality.
 *
 * Brightness: base kLedDefaultBrightness (~10%), peaks during transient
 * effects, always returns to base.
 */

#include "led_strip.h"
#include "config.h"
#include <FastLED.h>

// ============================================================================
// Internal State
// ============================================================================

static CRGB leds[kLedStripCount];
static LedState current_state = LedState::IDLE;
static bool animation_complete = true;
static uint32_t state_start_ms = 0;
static uint8_t active_brightness = kLedDefaultBrightness;

// Emotional overlay — a SEPARATE channel from current_state. Never
// written by the state machine; only by ledStripSetOverlay() (its own
// I2C event). NONE is a no-op so system-status LEDs are unchanged.
static LedOverlay current_overlay = LedOverlay::NONE;

// ============================================================================
// Unified white palette — matches eye color on the displays
// ============================================================================

// Helper: scale white by float factor (0.0–1.0)
static CRGB whiteAt(float intensity) {
    uint8_t v = (uint8_t)(255 * intensity);
    return CRGB(v, v, v);
}

// ============================================================================
// Math Helpers
// ============================================================================

static float easeInOut(float t) {
    return 0.5f * (1.0f - cosf(t * PI));
}

static float easeOut(float t) {
    return sinf(t * PI * 0.5f);
}

// ============================================================================
// Symmetric LED Helpers
// ============================================================================

static void clearAll() {
    fill_solid(leds, kLedStripCount, CRGB::Black);
}

static void setSymPair(uint8_t dist, CRGB color) {
    leds[kLedCenterLeft  - dist] = color;
    leds[kLedCenterRight + dist] = color;
}

static void addSymPair(uint8_t dist, CRGB color) {
    leds[kLedCenterLeft  - dist] |= color;
    leds[kLedCenterRight + dist] |= color;
}

// ============================================================================
// IDLE — Subtle center breathing
// Dim white glow at center pair, 4-second sine cycle.
// ============================================================================

static void animateIdle() {
    clearAll();

    float t = (millis() % 4000) / 4000.0f;
    float breath = 0.15f + 0.85f * (0.5f * (1.0f + sinf(t * 2.0f * PI)));

    setSymPair(0, whiteAt(breath * 0.15f));

    active_brightness = kLedDefaultBrightness;
    animation_complete = true;
}

// ============================================================================
// WOKE_UP — Gentle white sweep inward → soft glow → settle  (one-shot ~900 ms)
//
// Phase 1 (0-350 ms):   Cyan dot sweeps from edges toward center
// Phase 2 (350-550 ms): Soft cyan glow fills outward from center
// Phase 3 (550-900 ms): Everything fades down to base
// ============================================================================

static void animateWokeUp() {
    uint32_t elapsed = millis() - state_start_ms;
    clearAll();

    if (elapsed < 350) {
        // White comet sweeps from edges (3) to center (0)
        float pos = 3.0f * (1.0f - easeOut(elapsed / 350.0f));

        for (uint8_t d = 0; d <= 3; d++) {
            float gap = fabsf((float)d - pos);
            if (gap < 1.8f) {
                float intensity = 1.0f - (gap / 1.8f);
                intensity *= intensity;
                setSymPair(d, whiteAt(intensity * 0.9f));
            }
        }
        // Gentle brightness lift — not a flash, just a bit brighter than base
        active_brightness = kLedDefaultBrightness +
            (uint8_t)((kLedMaxBrightness / 2 - kLedDefaultBrightness) *
                      easeOut(elapsed / 350.0f));

    } else if (elapsed < 550) {
        // Soft glow expands from center
        float progress = easeOut((elapsed - 350) / 200.0f);
        float spread = progress * 3.5f;

        for (uint8_t d = 0; d <= 3; d++) {
            if ((float)d <= spread) {
                float intensity = 0.7f * (1.0f - (float)d * 0.2f);
                setSymPair(d, whiteAt(intensity));
            }
        }
        active_brightness = kLedMaxBrightness / 2;

    } else if (elapsed < 900) {
        // Settle: fade everything down smoothly
        float progress = easeInOut((elapsed - 550) / 350.0f);
        float fade = 1.0f - progress;

        for (uint8_t d = 0; d <= 3; d++) {
            float intensity = fade * 0.5f * (1.0f - (float)d * 0.25f);
            setSymPair(d, whiteAt(intensity));
        }
        active_brightness = kLedDefaultBrightness +
            (uint8_t)((kLedMaxBrightness / 2 - kLedDefaultBrightness) * fade);

    } else {
        active_brightness = kLedDefaultBrightness;
        animation_complete = true;
    }
}

// ============================================================================
// LISTENING — White sonar pulse from center  (looping, 1.5 s cycle)
//
// Bell-curve wave radiates outward. Dim cyan glow persists at center.
// ============================================================================

static void animateListening() {
    clearAll();

    float phase = (millis() % 1500) / 1500.0f;
    float wave_pos = phase * 5.5f;

    for (uint8_t d = 0; d <= 3; d++) {
        float behind = wave_pos - (float)d;

        if (behind >= 0.0f && behind < 2.0f) {
            float t = behind / 2.0f;
            float intensity = sinf(t * PI);
            intensity *= intensity;

            setSymPair(d, whiteAt(intensity * 0.85f));
        }
    }

    // Persistent dim origin glow at center
    float glow = 0.08f + 0.05f * sinf(millis() / 400.0f);
    addSymPair(0, whiteAt(glow));

    active_brightness = kLedDefaultBrightness;
    animation_complete = false;
}

// ============================================================================
// PROCESSING — White comet bouncing edge ↔ center  (looping, 700 ms)
//
// Eased ping-pong with a fading tail.
// ============================================================================

static void animateProcessing() {
    clearAll();

    float phase = (millis() % 700) / 700.0f;

    float pos;
    if (phase < 0.5f) {
        pos = 3.0f * (1.0f - easeInOut(phase * 2.0f));
    } else {
        pos = 3.0f * easeInOut((phase - 0.5f) * 2.0f);
    }

    for (uint8_t d = 0; d <= 3; d++) {
        float gap = fabsf((float)d - pos);
        if (gap < 2.0f) {
            float intensity = (gap < 0.6f)
                ? 1.0f
                : (1.0f - (gap - 0.6f) / 1.4f);
            intensity *= intensity;

            setSymPair(d, whiteAt(intensity * 0.9f));
        }
    }

    active_brightness = kLedDefaultBrightness;
    animation_complete = false;
}

// ============================================================================
// SPEAKING — Colorful overlapping waves + sparkle  (looping)
//
// The ONE state that breaks from cyan — full-spectrum personality.
// Dual waves at different speeds, slowly rotating hue, random sparkle.
// ============================================================================

static void animateSpeaking() {
    // Fade existing pixels for trail effect
    for (uint8_t i = 0; i < kLedStripCount; i++) {
        leds[i].fadeToBlackBy(45);
    }

    uint32_t now = millis();
    uint8_t hue_base = (now / 40) & 0xFF;

    float w1 = ((now % 500) / 500.0f) * 5.5f;
    float w2 = ((now % 800) / 800.0f) * 5.5f;

    for (uint8_t d = 0; d <= 3; d++) {
        float b1 = w1 - (float)d;
        if (b1 >= 0.0f && b1 < 1.5f) {
            float intensity = sinf((b1 / 1.5f) * PI);
            addSymPair(d, CHSV(hue_base, 200, (uint8_t)(intensity * 255)));
        }

        float b2 = w2 - (float)d;
        if (b2 >= 0.0f && b2 < 1.5f) {
            float intensity = sinf((b2 / 1.5f) * PI);
            addSymPair(d, CHSV(hue_base + 90, 220, (uint8_t)(intensity * 200)));
        }
    }

    if (random(100) < 30) {
        addSymPair(random(4), CHSV(random(256), 180, 220));
    }

    active_brightness = kLedDefaultBrightness;
    animation_complete = false;
}

// ============================================================================
// GOING_IDLE — White contracts inward and fades  (one-shot ~600 ms)
//
// Lit region shrinks from edges toward center. Center dims last.
// ============================================================================

static void animateGoingIdle() {
    uint32_t elapsed = millis() - state_start_ms;
    clearAll();

    if (elapsed < 600) {
        float progress = easeInOut(elapsed / 600.0f);
        float lit_radius = 3.0f * (1.0f - progress);
        float overall_fade = 1.0f - progress;

        for (uint8_t d = 0; d <= 3; d++) {
            float fd = (float)d;
            if (fd <= lit_radius + 0.5f) {
                float dist_factor = 1.0f - (fd * 0.25f);
                float edge_fade = (fd <= lit_radius)
                    ? 1.0f
                    : 1.0f - (fd - lit_radius) / 0.5f;
                float intensity = overall_fade * dist_factor * edge_fade;

                setSymPair(d, whiteAt(intensity * 0.7f));
            }
        }
        active_brightness = kLedDefaultBrightness;

    } else {
        active_brightness = kLedDefaultBrightness;
        animation_complete = true;
    }
}

// ============================================================================
// Emotional Overlay — separate wash, layered after the state animation
// ============================================================================

static void applyOverlay() {
    if (current_overlay == LedOverlay::NONE) {
        return;  // no tint — system-status animation unchanged
    }

    // Slow breath so the wash feels alive, not a flat fill.
    float breath = 0.65f + 0.35f * (0.5f * (1.0f + sinf(millis() / 900.0f)));

    if (current_overlay == LedOverlay::BRIGHT) {
        // Hue unchanged — just lift brightness. Guarantee a glow even
        // if the active state left the strip dark.
        bool dark = true;
        for (uint8_t i = 0; i < kLedStripCount; i++) {
            if (leds[i].r || leds[i].g || leds[i].b) { dark = false; break; }
        }
        if (dark) {
            setSymPair(0, whiteAt(0.5f * breath));
        }
        active_brightness = kLedMaxBrightness;
        return;
    }

    CRGB wash;
    switch (current_overlay) {
        case LedOverlay::WARM: wash = CRGB(255, 120, 30); break;
        case LedOverlay::COOL: wash = CRGB(20,  90, 255); break;
        case LedOverlay::HOT:  wash = CRGB(255, 15,  0);  break;
        default:               return;
    }

    // Ambient floor in the wash colour (so the emotion reads even when
    // the system-status animation is dark), then bias every lit pixel
    // toward the wash.
    uint8_t floor_amt = (uint8_t)(40 * breath);
    uint8_t bias_amt  = (current_overlay == LedOverlay::HOT) ? 150 : 110;
    for (uint8_t i = 0; i < kLedStripCount; i++) {
        CRGB amb = wash;
        amb.nscale8(floor_amt);
        leds[i] |= amb;
        nblend(leds[i], wash, bias_amt);
    }
    if (current_overlay == LedOverlay::HOT &&
        active_brightness < kLedMaxBrightness / 2) {
        active_brightness = kLedMaxBrightness / 2;  // anger runs hotter
    }
}

// ============================================================================
// Public API
// ============================================================================

void ledStripInit() {
    FastLED.addLeds<WS2812, kLedStripPin, GRB>(leds, kLedStripCount);
    FastLED.setBrightness(kLedDefaultBrightness);
    clearAll();
    FastLED.show();

    current_state = LedState::IDLE;
    active_brightness = kLedDefaultBrightness;
    animation_complete = true;

    Serial.printf("[LED] Initialized: %d LEDs on GPIO%d, brightness %d\n",
                  kLedStripCount, kLedStripPin, kLedDefaultBrightness);
}

void ledStripSetState(LedState state) {
    if (state == current_state && !animation_complete) {
        return;
    }

    current_state = state;
    animation_complete = false;
    state_start_ms = millis();
    active_brightness = kLedDefaultBrightness;

    const char* names[] = {
        "IDLE", "WOKE_UP", "LISTENING", "PROCESSING", "SPEAKING", "GOING_IDLE"
    };
    Serial.printf("[LED] -> %s\n", names[(uint8_t)state]);
}

LedState ledStripGetState() {
    return current_state;
}

void ledStripSetOverlay(LedOverlay overlay) {
    if (overlay == current_overlay) {
        return;
    }
    current_overlay = overlay;
    const char* names[] = { "NONE", "WARM", "COOL", "HOT", "BRIGHT" };
    Serial.printf("[LED] overlay -> %s\n", names[(uint8_t)overlay]);
}

LedOverlay ledStripGetOverlay() {
    return current_overlay;
}

void ledStripUpdate() {
    switch (current_state) {
        case LedState::SPEAKING:   animateSpeaking();    break;
        default:
            clearAll();
            active_brightness = kLedDefaultBrightness;
            break;
    }

    // Emotional overlay is layered LAST and is independent of the
    // state above (separate I2C event — REG_LED_OVERLAY).
    applyOverlay();

    FastLED.setBrightness(active_brightness);
    FastLED.show();
}

void ledStripSetBrightness(uint8_t brightness) {
    active_brightness = brightness;
    FastLED.setBrightness(brightness);
}

bool ledStripAnimationComplete() {
    return animation_complete;
}
