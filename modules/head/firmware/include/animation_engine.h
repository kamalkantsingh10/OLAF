/**
 * animation_engine.h - Eye Expression Animation Engine
 * Story 1.3: Develop Head ESP32 Firmware
 *
 * Expression Types (per story spec):
 *   0. NEUTRAL: Calm, attentive baseline
 *   1. HAPPY: Joyful, smiling eyes (upward arcs)
 *   2. SAD: Disappointed, downward gaze
 *   3. SURPRISED: Wide-eyed, startled
 *   4. ANGRY: Intense, narrowed eyes
 *   5. SLEEPY: Tired, half-closed
 *   6. WINK: Playful, one eye closed
 *
 * Intensity Levels (1-5):
 *   1 = Subtle
 *   2 = Light
 *   3 = Moderate
 *   4 = Strong
 *   5 = Extreme
 */

#pragma once

#include <Arduino.h>
#include "display_driver.h"
#include "i2c_slave.h"

// Eye shape types
enum EyeShape {
  SHAPE_ROUNDED_RECT = 0,  // Default neutral
  SHAPE_CIRCLE = 1,        // Surprised, alert
  SHAPE_HAPPY_ARC = 2,     // Happy (smiling)
  SHAPE_SAD_ARC = 3,       // Sad (frowning)
  SHAPE_ANGRY_V = 4,       // Angry (V-shaped)
  SHAPE_SLEEPY_LINE = 5,   // Sleepy (half-closed)
  SHAPE_WINK = 6           // Wink (one eye closed)
};

// Expression parameters
struct ExpressionParams {
  uint16_t pupil_radius;
  int16_t pupil_offset_x;
  int16_t pupil_offset_y;
  EyeShape eye_shape;
  uint16_t color;
  float movement_speed;
  float movement_amplitude;
  bool asymmetric;
  int16_t right_eye_offset_x;
  int16_t right_eye_offset_y;
  uint16_t right_eye_radius;
  EyeShape right_eye_shape;  // For wink (different shape per eye)
};

// Blink animation state
struct BlinkState {
  bool active;
  uint8_t phase;
  uint32_t start_millis;
  uint16_t duration_ms;
};

/**
 * Eye Expression Animation Engine
 */
class EyeExpressionEngine {
public:
  /**
   * Initialize with display driver
   */
  void begin(GC9A01DualDriver* driver);

  /**
   * Set expression type and intensity
   *
   * @param expression_type 0-6 (NEUTRAL to WINK)
   * @param intensity 1-5
   */
  void setExpression(uint8_t expression_type, uint8_t intensity);

  /**
   * Set look direction
   *
   * @param x -100 (left) to +100 (right)
   * @param y -100 (down) to +100 (up)
   */
  void setLookDirection(int8_t x, int8_t y);

  /**
   * Trigger blink animation
   */
  void triggerBlink();

  /**
   * Update animation (call every frame)
   *
   * @return Frame render time in ms
   */
  uint32_t update();

  /**
   * Get current state
   */
  uint8_t getCurrentExpression();
  uint8_t getCurrentIntensity();
  bool isBlinking();

private:
  GC9A01DualDriver* driver_;

  // Expression state
  uint8_t current_expression_;
  uint8_t current_intensity_;
  ExpressionParams current_params_;

  // Target state (for transitions)
  uint8_t target_expression_;
  uint8_t target_intensity_;
  ExpressionParams target_params_;

  // Look direction
  int8_t look_x_;
  int8_t look_y_;

  // Animation state
  float transition_progress_;
  uint32_t last_update_millis_;
  uint32_t animation_phase_;

  // Blink state
  BlinkState blink_state_;
  uint32_t last_blink_millis_;
  uint32_t next_blink_interval_ms_;
  bool double_blink_pending_;
  uint8_t blink_count_in_cluster_;

  // Colors
  uint16_t color_background_;
  uint16_t color_pupil_;

  // Private methods
  ExpressionParams calculateExpressionParams(uint8_t expression_type, uint8_t intensity);
  void renderPupils();
  void renderBlinkOverlay(float blink_amount);
  void interpolateParams(float progress);
  void applyAnimationOffsets();
  void updateBlinkAnimation();
  uint32_t calculateNextBlinkInterval();

  // Shape drawing
  void drawEyeShape(int16_t x, int16_t y, uint16_t radius, EyeShape shape,
                    uint16_t color, float blink_amount, bool is_right_eye = false);
  void drawHappyArc(int16_t x, int16_t y, uint16_t radius, uint16_t color, float blink_amount);
  void drawSadArc(int16_t x, int16_t y, uint16_t radius, uint16_t color, float blink_amount);
  void drawAngryV(int16_t x, int16_t y, uint16_t radius, uint16_t color, float blink_amount, bool is_right);
  void drawSleepyLine(int16_t x, int16_t y, uint16_t radius, uint16_t color, float blink_amount);
};
