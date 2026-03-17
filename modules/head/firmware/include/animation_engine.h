/**
 * animation_engine.h - Eye Expression & Status Animation Engine
 * Story 1.3: Develop Head ESP32 Firmware
 *
 * Sprite-based rendering: all drawing goes to an off-screen buffer,
 * then pushes the complete frame to the display. Zero flicker.
 *
 * Layered eye rendering:
 *   1. Colored iris circle (teal, height-clipped for blink)
 *   2. Black eyelid mask (expression-specific shape)
 *   3. Dark pupil circle (offset by look direction)
 *   4. White specular highlight dot
 */

#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "display_driver.h"
#include "i2c_slave.h"

// Sprite dimensions — centered on the 240x240 round display
constexpr uint16_t SPRITE_SIZE   = 200;
constexpr uint16_t SPRITE_CENTER = SPRITE_SIZE / 2;                        // 100
constexpr uint16_t SPRITE_OFFSET = (kDisplayWidth - SPRITE_SIZE) / 2;     // 20

// Eye color scheme
constexpr uint16_t IRIS_COLOR      = 0x07FF;  // Bright teal/cyan
constexpr uint16_t HIGHLIGHT_COLOR = TFT_WHITE;

// Eye shape types (control eyelid mask applied over iris)
enum EyeShape {
  SHAPE_NEUTRAL     = 0,  // Slight top eyelid — relaxed
  SHAPE_CIRCLE      = 1,  // No eyelid — fully open (surprised)
  SHAPE_HAPPY       = 2,  // Top 50% masked — upward arc (squinting happy)
  SHAPE_SAD         = 3,  // Angled drooping inner corner
  SHAPE_ANGRY       = 4,  // V-brow: inner corner pulled far down
  SHAPE_SLEEPY      = 5,  // Top 65% masked — heavy drooping lid
  SHAPE_WINK        = 6,  // Fully closed — horizontal line
};

struct ExpressionParams {
  uint16_t pupil_radius;
  int16_t  pupil_offset_x;
  int16_t  pupil_offset_y;
  EyeShape eye_shape;
  bool     asymmetric;
  int16_t  right_eye_offset_x;
  int16_t  right_eye_offset_y;
  uint16_t right_eye_radius;
  EyeShape right_eye_shape;
};

struct BlinkState {
  bool     active;
  uint32_t start_millis;
  uint16_t duration_ms;
};

class EyeExpressionEngine {
public:
  void begin(GC9A01DualDriver* driver);

  // Expression control
  void    setExpression(uint8_t expression_type, uint8_t intensity);
  void    setLookDirection(int8_t x, int8_t y);
  void    triggerBlink();

  // System status
  void    setSystemStatus(uint8_t status);
  uint8_t getSystemStatus();
  float   getWakeLevel();

  // Frame update — call every loop, returns frame time ms
  uint32_t update();

  // Queries
  uint8_t getCurrentExpression();
  uint8_t getCurrentIntensity();
  bool    isBlinking();

private:
  GC9A01DualDriver* driver_;
  TFT_eSprite*      sprite_;

  // Expression state
  uint8_t          current_expression_;
  uint8_t          current_intensity_;
  ExpressionParams current_params_;
  uint8_t          target_expression_;
  uint8_t          target_intensity_;
  ExpressionParams target_params_;

  // Look direction (-100 to +100)
  int8_t look_x_;
  int8_t look_y_;

  // Expression transition
  float    transition_progress_;
  uint32_t last_update_millis_;

  // Blink
  BlinkState blink_state_;
  uint32_t   last_blink_millis_;
  uint32_t   next_blink_interval_ms_;
  bool       double_blink_pending_;

  // System status / wake level
  uint8_t system_status_;
  float   wake_level_;
  float   wake_target_;
  float   wake_start_level_;
  uint32_t wake_start_ms_;
  uint32_t wake_duration_ms_;
  int8_t  auto_look_x_;
  int8_t  auto_look_y_;

  // High-level
  ExpressionParams calculateExpressionParams(uint8_t type, uint8_t intensity);
  void renderFrame();
  void renderEye(int16_t x, int16_t y, uint16_t radius,
                 EyeShape shape, float closedness, bool is_right);
  void interpolateParams(float progress);
  void updateWakeTransition();
  void updateAutoLook();
  void updateBlinkAnimation();
  float    computeClosedness();
  uint32_t calculateNextBlinkInterval();

  // Layered eye primitives
  void drawIrisCircle(int16_t x, int16_t y, uint16_t r, float height_scale);
  void applyEyelidMask(int16_t x, int16_t y, uint16_t r,
                       EyeShape shape, float height_scale, bool is_right);
  void drawClosedLine(int16_t x, int16_t y, uint16_t radius);
};
