/**
 * led_strip.h - WS2812 LED Strip Controller for Face Status Indicator
 * 8 LEDs with symmetrical animations radiating from center
 *
 * LED Layout: [0][1][2][3] | [4][5][6][7]
 *              edges ←──── center ────→ edges
 */

#pragma once

#include <Arduino.h>

/**
 * LED strip states for system status indication
 * Each state has a distinct symmetrical animation pattern
 */
enum class LedState : uint8_t {
    IDLE = 0,        // Off or single dim center pixel
    WOKE_UP = 1,     // Edges sweep inward → meet center → flash
    LISTENING = 2,   // Blue pulse radiating from center outward
    PROCESSING = 3,  // Chase pattern, both sides toward center, loop
    SPEAKING = 4,    // Rainbow colors randomly appearing
    GOING_IDLE = 5   // Fade from edges inward → off
};

/**
 * LED emotional overlay — a SEPARATE event from LedState and from the
 * eye expression. Driven by its own I2C register (REG_LED_OVERLAY,
 * 0x40), never derived from expression_type or system_status. Layered
 * as a colour wash on top of whatever LedState animation is running;
 * NONE = no change (system-status animation only — no regression).
 * Mirrors expression_map.yaml `led_overlay` tokens (Story 7.1 authors
 * the token; this enum is its render target).
 */
enum class LedOverlay : uint8_t {
    NONE   = 0,  // no emotional tint
    WARM   = 1,  // positive valence — warm amber wash
    COOL   = 2,  // subdued / sad — cool blue wash
    HOT    = 3,  // anger / frustration — strong red wash
    BRIGHT = 4   // high arousal — brightness lift, hue unchanged
};

/**
 * Initialize the LED strip
 * Must be called in setup() before any LED operations
 */
void ledStripInit();

/**
 * Set the current LED state
 * Triggers the corresponding animation pattern
 *
 * @param state The new state to display
 */
void ledStripSetState(LedState state);

/**
 * Get the current LED state
 *
 * @return Current LedState
 */
LedState ledStripGetState();

/**
 * Set the emotional colour overlay (separate event — independent of
 * LedState and eye expression). Applied as a wash in ledStripUpdate().
 *
 * @param overlay The new overlay (NONE clears any tint)
 */
void ledStripSetOverlay(LedOverlay overlay);

/**
 * Get the current emotional overlay
 *
 * @return Current LedOverlay
 */
LedOverlay ledStripGetOverlay();

/**
 * Update LED animations
 * Must be called regularly in loop() for smooth animations
 * Uses non-blocking timing internally
 */
void ledStripUpdate();

/**
 * Set brightness level (0-255)
 * Default is 10, max for effects is 80
 *
 * @param brightness Brightness level
 */
void ledStripSetBrightness(uint8_t brightness);

/**
 * Check if current animation has completed
 * Useful for one-shot animations (WOKE_UP, GOING_IDLE)
 *
 * @return true if animation cycle is complete
 */
bool ledStripAnimationComplete();
