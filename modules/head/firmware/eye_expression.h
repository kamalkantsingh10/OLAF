/**
 * eye_expression.h - Eye Expression Animation Engine
 * Story 1.4: Head Module ESP32 Firmware - Eye Expressions
 *
 * Purpose:
 *   Renders emotion-driven eye expressions on dual GC9A01 displays.
 *   Translates high-level emotion commands (e.g., "happy at intensity 3")
 *   into animated pupil positions, sizes, and movements.
 *
 * Expression Types (7 emotions):
 *   0. NEUTRAL: Circular pupils, steady gaze (calm, attentive)
 *   1. HAPPY: Wide pupils, slight upward curve (joyful, excited)
 *   2. CURIOUS: Large pupils, slight offset (interested, investigating)
 *   3. THINKING: Pupils looking up-right, occasional blink (pondering, processing)
 *   4. CONFUSED: Pupils wandering, asymmetric (uncertain, puzzled)
 *   5. SAD: Small pupils, downward gaze (disappointed, melancholy)
 *   6. EXCITED: Very wide pupils, rapid micro-movements (enthusiastic, energetic)
 *
 * Intensity Levels (1-5):
 *   1 = Subtle (small changes, minimal movement)
 *   2 = Light (noticeable but restrained)
 *   3 = Moderate (clear expression, balanced)
 *   4 = Strong (exaggerated, theatrical)
 *   5 = Extreme (maximum effect, cartoon-like)
 *
 * Animation System:
 *   - 60 FPS target (16.67ms per frame)
 *   - Smooth interpolation between states
 *   - Synchronized blink animations
 *   - Micro-movements for liveliness
 *
 * Design Philosophy:
 *   - Expressions are ADDITIVE: Base pupil + expression offsets
 *   - Intensity scales animation parameters (pupil size, movement speed, etc.)
 *   - Both eyes coordinated but can be asymmetric for certain expressions
 */

#pragma once

#include <Arduino.h>
#include "gc9a01_driver_spi.h"
#include "i2c_slave.h"

// Eye shape types (inspired by Vector robot and chopsticks1 face controller)
enum EyeShape {
  SHAPE_ROUNDED_RECT = 0,   // Default: Rounded rectangles (neutral, thinking)
  SHAPE_CIRCLE = 1,         // Circles (curious, excited, scared)
  SHAPE_HAPPY_ARC = 2,      // Upward arcs (happy - smiling eyes)
  SHAPE_SAD_ARC = 3,        // Downward arcs (sad - frowning eyes)
  SHAPE_LINE = 4            // Horizontal line (confused, or for blink animation)
};

// Expression parameters structure
struct ExpressionParams {
  // Pupil properties
  uint16_t pupil_radius;        // Pupil size in pixels
  int16_t pupil_offset_x;       // Horizontal offset from center
  int16_t pupil_offset_y;       // Vertical offset from center

  // Shape properties
  EyeShape eye_shape;           // Which shape to draw
  uint16_t color;               // Eye color (RGB565)

  // Animation properties
  float movement_speed;         // Movement animation speed (0.0-1.0)
  float movement_amplitude;     // How much pupils move (0.0-1.0)

  // Eye shape modifiers
  bool asymmetric;              // Different left/right eye positions
  int16_t right_eye_offset_x;   // Additional offset for right eye (if asymmetric)
  int16_t right_eye_offset_y;   // Additional offset for right eye (if asymmetric)
  uint16_t right_eye_radius;    // Different size for right eye (if asymmetric)
};

// Blink animation state
struct BlinkState {
  bool active;                  // Is blink animation playing?
  uint8_t phase;                // Blink phase (0=open, 1=closing, 2=closed, 3=opening)
  uint32_t start_millis;        // When blink started
  uint16_t duration_ms;         // Total blink duration
};

/**
 * Eye Expression Engine
 *
 * Manages expression rendering, animation state, and blink sequences.
 */
class EyeExpressionEngine {
public:
  /**
   * Initialize expression engine
   *
   * @param driver Pointer to initialized GC9A01DualDriver
   */
  void begin(GC9A01DualDriver* driver);

  /**
   * Set current expression
   *
   * Transitions to new expression with smooth interpolation.
   *
   * @param expression_type Expression ID (0-6)
   * @param intensity Intensity level (1-5)
   *
   * Note: Expression change is NOT instant - animates over ~200ms
   */
  void setExpression(uint8_t expression_type, uint8_t intensity);

  /**
   * Trigger blink animation
   *
   * Plays synchronized blink on both eyes.
   * Blink duration: ~200-400ms depending on intensity.
   */
  void triggerBlink();

  /**
   * Update animation (call every frame)
   *
   * Must be called regularly (60 FPS) to update animations.
   * Renders current expression state to displays.
   *
   * @return Frame render time in milliseconds
   */
  uint32_t update();

  /**
   * Get current expression type
   */
  uint8_t getCurrentExpression();

  /**
   * Get current intensity
   */
  uint8_t getCurrentIntensity();

  /**
   * Check if blink is currently playing
   */
  bool isBlinking();

private:
  GC9A01DualDriver* driver_;    // Display driver

  // Current expression state
  uint8_t current_expression_;
  uint8_t current_intensity_;
  ExpressionParams current_params_;

  // Target expression state (for smooth transitions)
  uint8_t target_expression_;
  uint8_t target_intensity_;
  ExpressionParams target_params_;

  // Animation state
  float transition_progress_;   // 0.0-1.0, for smooth expression changes
  uint32_t last_update_millis_;
  uint32_t animation_phase_;    // For cyclic animations (wandering eyes, etc.)

  // Blink state
  BlinkState blink_state_;

  // Automatic blinking (natural human-like behavior)
  uint32_t last_blink_millis_;
  uint32_t next_blink_interval_ms_;  // Time until next automatic blink
  bool double_blink_pending_;        // Should we do a double blink?
  uint8_t blink_count_in_cluster_;   // How many blinks in current cluster

  // Color palette
  uint16_t color_background_;   // Background color
  uint16_t color_pupil_;        // Pupil color

  /**
   * Calculate expression parameters for given type and intensity
   *
   * Maps expression type + intensity to concrete animation parameters.
   *
   * @param expression_type Expression ID (0-6)
   * @param intensity Intensity level (1-5)
   * @return ExpressionParams structure with calculated values
   */
  ExpressionParams calculateExpressionParams(uint8_t expression_type, uint8_t intensity);

  /**
   * Render pupils on both eyes
   *
   * Draws pupils based on current_params_ with blink applied if active.
   */
  void renderPupils();

  /**
   * Render blink overlay
   *
   * Draws eyelid closing effect during blink animation.
   *
   * @param blink_amount 0.0 (fully open) to 1.0 (fully closed)
   */
  void renderBlinkOverlay(float blink_amount);

  /**
   * Interpolate between current and target expression params
   *
   * Smooth transition for expression changes.
   *
   * @param progress Transition progress (0.0-1.0)
   */
  void interpolateParams(float progress);

  /**
   * Apply animation offsets (micro-movements, wandering, etc.)
   *
   * Adds small time-based variations to pupil positions.
   */
  void applyAnimationOffsets();

  /**
   * Update blink animation state
   *
   * Advances blink phase and checks for completion.
   */
  void updateBlinkAnimation();

  /**
   * Draw individual eye shape at given position
   *
   * Renders the appropriate shape based on expression parameters.
   *
   * @param x Center X coordinate
   * @param y Center Y coordinate
   * @param radius Eye radius
   * @param shape Shape type to draw
   * @param color Eye color
   * @param blink_amount 0.0 (open) to 1.0 (closed) for blink compression
   */
  void drawEyeShape(int16_t x, int16_t y, uint16_t radius, EyeShape shape,
                    uint16_t color, float blink_amount);

  /**
   * Draw happy arc (upward curve like smiling eyes)
   */
  void drawHappyArc(int16_t x, int16_t y, uint16_t radius, uint16_t color, float blink_amount);

  /**
   * Draw sad arc (downward curve like frowning eyes)
   */
  void drawSadArc(int16_t x, int16_t y, uint16_t radius, uint16_t color, float blink_amount);

  /**
   * Calculate next blink interval based on expression and intensity
   *
   * Different expressions blink at different rates:
   * - Excited: Fast blinking (1-2 seconds)
   * - Neutral: Normal (3-5 seconds)
   * - Sad: Slow (6-10 seconds)
   * - Thinking: Irregular (2-8 seconds)
   *
   * @return Milliseconds until next blink
   */
  uint32_t calculateNextBlinkInterval();
};
