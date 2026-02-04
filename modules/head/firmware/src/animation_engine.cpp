/**
 * animation_engine.cpp - Eye Expression Animation Engine Implementation
 * Story 1.3: Develop Head ESP32 Firmware
 *
 * Expression Types:
 *   NEUTRAL (0): Calm rounded rectangles
 *   HAPPY (1): Upward arcs (smiling eyes)
 *   SAD (2): Downward arcs (frowning)
 *   SURPRISED (3): Large circles (wide-eyed)
 *   ANGRY (4): V-shaped (narrowed)
 *   SLEEPY (5): Half-closed lines
 *   WINK (6): One eye closed
 */

#include "animation_engine.h"
#include <math.h>

// Constants
constexpr uint16_t BASE_PUPIL_RADIUS = 40;
constexpr uint16_t MAX_PUPIL_RADIUS = 80;
constexpr uint16_t MIN_PUPIL_RADIUS = 20;
constexpr int16_t MAX_PUPIL_OFFSET = 30;
constexpr uint32_t TRANSITION_DURATION_MS = 200;
constexpr uint32_t BLINK_DURATION_MS = 250;

void EyeExpressionEngine::begin(GC9A01DualDriver* driver) {
  driver_ = driver;

  current_expression_ = EXPR_NEUTRAL;
  current_intensity_ = 2;
  target_expression_ = EXPR_NEUTRAL;
  target_intensity_ = 2;

  current_params_ = calculateExpressionParams(EXPR_NEUTRAL, 2);
  target_params_ = current_params_;

  look_x_ = 0;
  look_y_ = 0;

  transition_progress_ = 1.0;
  last_update_millis_ = millis();
  animation_phase_ = 0;

  blink_state_.active = false;
  blink_state_.phase = 0;
  blink_state_.start_millis = 0;
  blink_state_.duration_ms = BLINK_DURATION_MS;

  last_blink_millis_ = millis();
  next_blink_interval_ms_ = calculateNextBlinkInterval();
  double_blink_pending_ = false;
  blink_count_in_cluster_ = 0;

  color_background_ = TFT_BLACK;
  color_pupil_ = TFT_CYAN;

  renderPupils();

  Serial.println("[Expression] Engine initialized - NEUTRAL expression");
}

void EyeExpressionEngine::setExpression(uint8_t expression_type, uint8_t intensity) {
  if (expression_type >= EXPR_COUNT) {
    Serial.printf("[Expression] ERROR: Invalid expression type %d\n", expression_type);
    return;
  }
  if (intensity < 1 || intensity > 5) {
    Serial.printf("[Expression] ERROR: Invalid intensity %d\n", intensity);
    return;
  }

  bool expression_changed = (expression_type != current_expression_);

  target_expression_ = expression_type;
  target_intensity_ = intensity;
  target_params_ = calculateExpressionParams(expression_type, intensity);

  transition_progress_ = 0.0;

  const char* expr_names[] = {"NEUTRAL", "HAPPY", "SAD", "SURPRISED", "ANGRY", "SLEEPY", "WINK"};
  Serial.printf("[Expression] Transitioning to %s (intensity %d)\n",
                expr_names[expression_type], intensity);

  if (expression_changed && !blink_state_.active) {
    triggerBlink();
  }
}

void EyeExpressionEngine::setLookDirection(int8_t x, int8_t y) {
  look_x_ = constrain(x, -100, 100);
  look_y_ = constrain(y, -100, 100);
}

void EyeExpressionEngine::triggerBlink() {
  if (blink_state_.active) return;

  uint16_t blink_duration = BLINK_DURATION_MS;

  switch (current_expression_) {
    case EXPR_SURPRISED:
      blink_duration = 180 + random(0, 20);
      break;
    case EXPR_SLEEPY:
      blink_duration = 300 + random(0, 50);
      break;
    default:
      blink_duration = 200 + random(0, 30);
      break;
  }

  blink_state_.active = true;
  blink_state_.phase = 0;
  blink_state_.start_millis = millis();
  blink_state_.duration_ms = blink_duration;

  Serial.printf("[Expression] Blink triggered (duration: %dms)\n", blink_duration);
}

uint32_t EyeExpressionEngine::update() {
  driver_->startFrameTiming();

  uint32_t now = millis();
  uint32_t delta_ms = now - last_update_millis_;
  last_update_millis_ = now;

  bool needs_redraw = false;

  // Update expression transition
  if (transition_progress_ < 1.0) {
    transition_progress_ += (float)delta_ms / TRANSITION_DURATION_MS;
    if (transition_progress_ >= 1.0) {
      transition_progress_ = 1.0;
      current_expression_ = target_expression_;
      current_intensity_ = target_intensity_;
      current_params_ = target_params_;
      next_blink_interval_ms_ = calculateNextBlinkInterval();
      last_blink_millis_ = now;
      needs_redraw = true;
    } else {
      if (blink_state_.active) {
        uint32_t elapsed_blink = millis() - blink_state_.start_millis;
        float blink_progress = (float)elapsed_blink / blink_state_.duration_ms;

        if (blink_progress >= 0.4 && blink_progress < 0.6) {
          current_expression_ = target_expression_;
          current_intensity_ = target_intensity_;
          current_params_ = target_params_;
          transition_progress_ = 1.0;
        }
      } else {
        interpolateParams(transition_progress_);
        needs_redraw = true;
      }
    }
  }

  // Auto-blink
  if (!blink_state_.active && (now - last_blink_millis_ >= next_blink_interval_ms_)) {
    if (double_blink_pending_) {
      triggerBlink();
      double_blink_pending_ = false;
      last_blink_millis_ = now;
      next_blink_interval_ms_ = calculateNextBlinkInterval();
    } else {
      triggerBlink();
      last_blink_millis_ = now;

      if (random(100) < 20) {
        double_blink_pending_ = true;
        next_blink_interval_ms_ = 300 + random(200);
      } else {
        next_blink_interval_ms_ = calculateNextBlinkInterval();
      }
    }
  }

  // Update blink animation
  static float last_blink_amount = 0.0;
  float current_blink_amount = 0.0;

  if (blink_state_.active) {
    uint32_t elapsed_check = millis() - blink_state_.start_millis;
    float progress_check = (float)elapsed_check / blink_state_.duration_ms;

    if (progress_check < 0.25) {
      current_blink_amount = 0.4;
    } else if (progress_check < 0.5) {
      current_blink_amount = 1.0;
    } else if (progress_check < 0.75) {
      current_blink_amount = 0.4;
    } else {
      current_blink_amount = 0.0;
    }

    if (current_blink_amount != last_blink_amount) {
      needs_redraw = true;
    }
    last_blink_amount = current_blink_amount;

    updateBlinkAnimation();
  } else {
    last_blink_amount = 0.0;
  }

  if (needs_redraw) {
    renderPupils();
  }

  driver_->endFrameTiming();
  return driver_->getLastFrameTimeMs();
}

uint8_t EyeExpressionEngine::getCurrentExpression() {
  return current_expression_;
}

uint8_t EyeExpressionEngine::getCurrentIntensity() {
  return current_intensity_;
}

bool EyeExpressionEngine::isBlinking() {
  return blink_state_.active;
}

ExpressionParams EyeExpressionEngine::calculateExpressionParams(uint8_t expression_type, uint8_t intensity) {
  ExpressionParams params;

  params.pupil_radius = BASE_PUPIL_RADIUS;
  params.pupil_offset_x = 0;
  params.pupil_offset_y = 0;
  params.eye_shape = SHAPE_ROUNDED_RECT;
  params.color = TFT_CYAN;
  params.movement_speed = 0.5;
  params.movement_amplitude = 0.1;
  params.asymmetric = false;
  params.right_eye_offset_x = 0;
  params.right_eye_offset_y = 0;
  params.right_eye_radius = BASE_PUPIL_RADIUS;
  params.right_eye_shape = SHAPE_ROUNDED_RECT;

  float intensity_factor = 0.5 + (intensity * 0.3);

  switch (expression_type) {
    case EXPR_NEUTRAL:
      params.pupil_radius = BASE_PUPIL_RADIUS;
      params.eye_shape = SHAPE_ROUNDED_RECT;
      params.color = TFT_CYAN;
      break;

    case EXPR_HAPPY:
      params.pupil_radius = BASE_PUPIL_RADIUS + (8 * intensity_factor);
      params.eye_shape = SHAPE_HAPPY_ARC;
      params.color = TFT_GREEN;
      params.pupil_offset_y = -3 * intensity_factor;
      break;

    case EXPR_SAD:
      params.pupil_radius = BASE_PUPIL_RADIUS - (5 * intensity_factor);
      if (params.pupil_radius < MIN_PUPIL_RADIUS) params.pupil_radius = MIN_PUPIL_RADIUS;
      params.eye_shape = SHAPE_SAD_ARC;
      params.color = TFT_BLUE;
      params.pupil_offset_y = 8 * intensity_factor;
      break;

    case EXPR_SURPRISED:
      params.pupil_radius = BASE_PUPIL_RADIUS + (20 * intensity_factor);
      if (params.pupil_radius > MAX_PUPIL_RADIUS) params.pupil_radius = MAX_PUPIL_RADIUS;
      params.eye_shape = SHAPE_CIRCLE;
      params.color = TFT_WHITE;
      break;

    case EXPR_ANGRY:
      params.pupil_radius = BASE_PUPIL_RADIUS - (3 * intensity_factor);
      params.eye_shape = SHAPE_ANGRY_V;
      params.color = TFT_RED;
      params.pupil_offset_y = 5 * intensity_factor;
      break;

    case EXPR_SLEEPY:
      params.pupil_radius = BASE_PUPIL_RADIUS - (10 * intensity_factor);
      if (params.pupil_radius < MIN_PUPIL_RADIUS) params.pupil_radius = MIN_PUPIL_RADIUS;
      params.eye_shape = SHAPE_SLEEPY_LINE;
      params.color = TFT_DARKGREY;
      params.pupil_offset_y = 10 * intensity_factor;
      break;

    case EXPR_WINK:
      params.pupil_radius = BASE_PUPIL_RADIUS;
      params.eye_shape = SHAPE_HAPPY_ARC;
      params.color = TFT_CYAN;
      params.asymmetric = true;
      params.right_eye_shape = SHAPE_WINK;
      params.right_eye_radius = BASE_PUPIL_RADIUS;
      break;

    default:
      break;
  }

  return params;
}

void EyeExpressionEngine::renderPupils() {
  static uint8_t last_rendered_expression = 0xFF;
  static bool first_render = true;

  bool expression_changed = (current_expression_ != last_rendered_expression);
  if (first_render || expression_changed) {
    driver_->clearBothEyes();
    first_render = false;
    last_rendered_expression = current_expression_;
  }

  float blink_amount = 0.0;
  if (blink_state_.active) {
    uint32_t elapsed = millis() - blink_state_.start_millis;
    float progress = (float)elapsed / blink_state_.duration_ms;

    if (progress < 0.25) {
      blink_amount = 0.4;
    } else if (progress < 0.5) {
      blink_amount = 1.0;
    } else if (progress < 0.75) {
      blink_amount = 0.4;
    } else {
      blink_amount = 0.0;
    }
  }

  // Apply look direction offset
  int16_t look_offset_x = (look_x_ * MAX_PUPIL_OFFSET) / 100;
  int16_t look_offset_y = -(look_y_ * MAX_PUPIL_OFFSET) / 100;  // Invert Y

  uint16_t left_radius = current_params_.pupil_radius;
  uint16_t right_radius = current_params_.asymmetric ? current_params_.right_eye_radius : left_radius;

  int16_t left_x = DISPLAY_CENTER_X + current_params_.pupil_offset_x + look_offset_x;
  int16_t left_y = DISPLAY_CENTER_Y + current_params_.pupil_offset_y + look_offset_y;
  int16_t right_x = DISPLAY_CENTER_X + current_params_.pupil_offset_x + look_offset_x;
  int16_t right_y = DISPLAY_CENTER_Y + current_params_.pupil_offset_y + look_offset_y;

  if (current_params_.asymmetric) {
    right_x += current_params_.right_eye_offset_x;
    right_y += current_params_.right_eye_offset_y;
  }

  // Render LEFT EYE
  driver_->selectEye(LEFT);
  drawEyeShape(left_x, left_y, left_radius, current_params_.eye_shape,
               current_params_.color, blink_amount, false);

  // Render RIGHT EYE
  driver_->selectEye(RIGHT);
  EyeShape right_shape = current_params_.asymmetric ? current_params_.right_eye_shape : current_params_.eye_shape;
  drawEyeShape(right_x, right_y, right_radius, right_shape,
               current_params_.color, blink_amount, true);
}

void EyeExpressionEngine::renderBlinkOverlay(float blink_amount) {
  // Handled in drawEyeShape
}

void EyeExpressionEngine::interpolateParams(float progress) {
  ExpressionParams& c = current_params_;
  ExpressionParams& t = target_params_;

  c.pupil_radius = c.pupil_radius + (t.pupil_radius - c.pupil_radius) * progress;
  c.pupil_offset_x = c.pupil_offset_x + (t.pupil_offset_x - c.pupil_offset_x) * progress;
  c.pupil_offset_y = c.pupil_offset_y + (t.pupil_offset_y - c.pupil_offset_y) * progress;

  if (progress > 0.5) {
    c.eye_shape = t.eye_shape;
    c.color = t.color;
  }

  if (t.asymmetric) {
    c.asymmetric = true;
    c.right_eye_offset_x = c.right_eye_offset_x + (t.right_eye_offset_x - c.right_eye_offset_x) * progress;
    c.right_eye_offset_y = c.right_eye_offset_y + (t.right_eye_offset_y - c.right_eye_offset_y) * progress;
    c.right_eye_radius = c.right_eye_radius + (t.right_eye_radius - c.right_eye_radius) * progress;
  } else {
    c.asymmetric = false;
    c.right_eye_radius = c.pupil_radius;
  }
}

void EyeExpressionEngine::applyAnimationOffsets() {
  // Static eyes - no micro-movements
}

void EyeExpressionEngine::updateBlinkAnimation() {
  uint32_t elapsed = millis() - blink_state_.start_millis;

  if (elapsed >= blink_state_.duration_ms) {
    blink_state_.active = false;
    blink_state_.phase = 0;
  }
}

uint32_t EyeExpressionEngine::calculateNextBlinkInterval() {
  uint32_t base_interval = 5000;
  uint32_t variation = 2500;

  switch (current_expression_) {
    case EXPR_NEUTRAL:
      base_interval = 5000;
      variation = 2500;
      break;
    case EXPR_HAPPY:
      base_interval = 4000;
      variation = 2000;
      break;
    case EXPR_SAD:
      base_interval = 6000;
      variation = 2000;
      break;
    case EXPR_SURPRISED:
      base_interval = 3000;
      variation = 1500;
      break;
    case EXPR_ANGRY:
      base_interval = 4000;
      variation = 1500;
      break;
    case EXPR_SLEEPY:
      base_interval = 3000;
      variation = 1000;
      break;
    case EXPR_WINK:
      base_interval = 4000;
      variation = 2000;
      break;
  }

  float intensity_factor = 1.4 - (current_intensity_ * 0.08);
  base_interval = base_interval * intensity_factor;

  int32_t random_offset = random(-variation, variation);
  uint32_t interval = base_interval + random_offset;

  if (interval < 1200) interval = 1200;
  if (interval > 8000) interval = 8000;

  return interval;
}

void EyeExpressionEngine::drawEyeShape(int16_t x, int16_t y, uint16_t radius,
                                        EyeShape shape, uint16_t color, float blink_amount,
                                        bool is_right_eye) {
  TFT_eSPI& tft = driver_->getTFT();

  int16_t clear_size = (radius * 2) + 20;
  tft.fillRect(x - (clear_size / 2), y - (clear_size / 2), clear_size, clear_size, color_background_);

  // Fully closed (blink)
  if (blink_amount > 0.8) {
    int16_t line_width = radius * 1.8;
    for (int16_t i = 0; i < 3; i++) {
      int16_t curve_offset = (i == 1) ? 1 : 0;
      tft.drawFastHLine(x - (line_width / 2), y + i - 1 + curve_offset, line_width, color);
    }
    return;
  }

  float height_scale = 1.0 - (blink_amount * 0.875);

  switch (shape) {
    case SHAPE_ROUNDED_RECT: {
      int16_t rect_size_x = radius * 2;
      int16_t rect_size_y = (radius * 2) * height_scale;
      uint8_t corner_radius = radius / 3;
      tft.fillRoundRect(x - radius, y - (rect_size_y / 2),
                       rect_size_x, rect_size_y, corner_radius, color);
      break;
    }

    case SHAPE_CIRCLE: {
      if (blink_amount < 0.05) {
        tft.fillCircle(x, y, radius, color);
      } else {
        int16_t ellipse_height = radius * height_scale;
        for (int16_t dy = -ellipse_height; dy <= ellipse_height; dy++) {
          int16_t dx = sqrt(radius * radius - (dy * dy / height_scale / height_scale));
          tft.drawFastHLine(x - dx, y + dy, dx * 2, color);
        }
      }
      break;
    }

    case SHAPE_HAPPY_ARC:
      drawHappyArc(x, y, radius, color, blink_amount);
      break;

    case SHAPE_SAD_ARC:
      drawSadArc(x, y, radius, color, blink_amount);
      break;

    case SHAPE_ANGRY_V:
      drawAngryV(x, y, radius, color, blink_amount, is_right_eye);
      break;

    case SHAPE_SLEEPY_LINE:
      drawSleepyLine(x, y, radius, color, blink_amount);
      break;

    case SHAPE_WINK: {
      // Closed winking eye (horizontal line)
      int16_t line_width = radius * 1.5;
      int16_t line_height = 4;
      tft.fillRoundRect(x - (line_width / 2), y - (line_height / 2),
                       line_width, line_height, 2, color);
      break;
    }

    default:
      tft.fillRoundRect(x - radius, y - radius,
                       radius * 2, radius * 2, radius / 3, color);
      break;
  }
}

void EyeExpressionEngine::drawHappyArc(int16_t x, int16_t y, uint16_t radius,
                                        uint16_t color, float blink_amount) {
  TFT_eSPI& tft = driver_->getTFT();

  float height_scale = 1.0 - (blink_amount * 0.875);

  int16_t arc_width = radius * 2;
  int16_t arc_height = radius * 0.8 * height_scale;
  int16_t arc_thickness = radius / 4;

  for (int16_t dy = 0; dy < arc_height; dy++) {
    float t = (float)dy / arc_height;
    float curve = sqrt(1.0 - t * t);
    int16_t half_width = arc_width * 0.5 * curve;

    for (int16_t thick = 0; thick < arc_thickness; thick++) {
      tft.drawFastHLine(x - half_width, y + dy + thick, half_width * 2, color);
    }
  }
}

void EyeExpressionEngine::drawSadArc(int16_t x, int16_t y, uint16_t radius,
                                      uint16_t color, float blink_amount) {
  TFT_eSPI& tft = driver_->getTFT();

  float height_scale = 1.0 - (blink_amount * 0.875);

  int16_t arc_width = radius * 2;
  int16_t arc_height = radius * 0.8 * height_scale;
  int16_t arc_thickness = radius / 4;

  for (int16_t dy = 0; dy < arc_height; dy++) {
    float t = (float)dy / arc_height;
    float curve = sqrt(1.0 - (1.0 - t) * (1.0 - t));
    int16_t half_width = arc_width * 0.5 * curve;

    for (int16_t thick = 0; thick < arc_thickness; thick++) {
      tft.drawFastHLine(x - half_width, y - dy - thick, half_width * 2, color);
    }
  }
}

void EyeExpressionEngine::drawAngryV(int16_t x, int16_t y, uint16_t radius,
                                      uint16_t color, float blink_amount, bool is_right) {
  TFT_eSPI& tft = driver_->getTFT();

  float height_scale = 1.0 - (blink_amount * 0.875);

  int16_t v_width = radius * 1.5;
  int16_t v_height = radius * 0.6 * height_scale;
  int16_t thickness = radius / 5;

  // Draw V-shape (angled eyebrow over eye)
  // Mirror for left vs right eye
  int16_t angle_dir = is_right ? 1 : -1;

  for (int16_t thick = 0; thick < thickness; thick++) {
    // Draw angled line from outer to inner
    for (int16_t dx = 0; dx < v_width; dx++) {
      int16_t dy = (v_height * dx / v_width) * angle_dir;
      tft.drawPixel(x - (v_width / 2) + dx, y - (v_height / 2) + dy + thick, color);
    }
  }

  // Draw small pupil below
  int16_t pupil_y = y + (v_height / 2) + (radius / 3);
  int16_t pupil_radius = radius / 2;
  tft.fillCircle(x, pupil_y, pupil_radius, color);
}

void EyeExpressionEngine::drawSleepyLine(int16_t x, int16_t y, uint16_t radius,
                                          uint16_t color, float blink_amount) {
  TFT_eSPI& tft = driver_->getTFT();

  // Half-closed eyes (thick horizontal line)
  int16_t line_width = radius * 1.8;
  int16_t line_height = max(8, (int)(radius / 3));

  float height_scale = 1.0 - (blink_amount * 0.5);
  line_height = line_height * height_scale;
  if (line_height < 4) line_height = 4;

  tft.fillRoundRect(x - (line_width / 2), y - (line_height / 2),
                   line_width, line_height, 3, color);
}
