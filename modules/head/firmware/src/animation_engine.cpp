/**
 * animation_engine.cpp - Sprite-Based Eye Animation Engine
 * Story 1.3: Develop Head ESP32 Firmware
 *
 * Renders to a 200x200 TFT_eSprite in PSRAM, then pushes the
 * complete frame to each display.  Zero flicker.
 *
 * All expressions use white (TFT_WHITE).  Shape conveys emotion.
 * Eyes are sized to fill the round display with minimal dead space.
 */

#include "animation_engine.h"
#include <math.h>

// ============================================================================
// Constants
// ============================================================================

constexpr uint16_t EYE_COLOR = TFT_WHITE;

constexpr uint16_t BASE_PUPIL_RADIUS = 60;   // Fills ~50% of display
constexpr uint16_t MAX_PUPIL_RADIUS = 85;    // Surprised: ~71% of display
constexpr uint16_t MIN_PUPIL_RADIUS = 25;
constexpr int16_t MAX_PUPIL_OFFSET = 25;     // Look direction range
constexpr uint32_t TRANSITION_DURATION_MS = 200;
constexpr uint32_t BLINK_DURATION_MS = 250;
constexpr uint32_t WAKE_DURATION_MS = 250;
constexpr uint32_t SLEEP_DURATION_MS = 600;

static float easeOut(float t) {
    return sinf(t * PI * 0.5f);
}

static float easeInOut(float t) {
    return 0.5f * (1.0f - cosf(t * PI));
}

// ============================================================================
// Initialization
// ============================================================================

void EyeExpressionEngine::begin(GC9A01DualDriver* driver) {
  driver_ = driver;

  // Create sprite in PSRAM (200x200 × 16-bit = 80 KB)
  sprite_ = new TFT_eSprite(&driver_->getTFT());
  sprite_->setColorDepth(16);
  void* buf = sprite_->createSprite(SPRITE_SIZE, SPRITE_SIZE);
  if (buf) {
    Serial.printf("[Eyes] Sprite: %dx%d (PSRAM: %s)\n",
                  SPRITE_SIZE, SPRITE_SIZE,
                  psramFound() ? "yes" : "no");
  } else {
    Serial.println("[Eyes] ERROR: Sprite allocation failed!");
  }

  current_expression_ = EXPR_NEUTRAL;
  current_intensity_ = 2;
  target_expression_ = EXPR_NEUTRAL;
  target_intensity_ = 2;
  current_params_ = calculateExpressionParams(EXPR_NEUTRAL, 2);
  target_params_ = current_params_;

  look_x_ = 0;
  look_y_ = 0;
  transition_progress_ = 1.0f;
  last_update_millis_ = millis();

  blink_state_ = {false, 0, BLINK_DURATION_MS};
  last_blink_millis_ = millis();
  next_blink_interval_ms_ = calculateNextBlinkInterval();
  double_blink_pending_ = false;

  // Start asleep
  system_status_ = 0;
  wake_level_ = 0.0f;
  wake_target_ = 0.0f;
  wake_start_level_ = 0.0f;
  wake_start_ms_ = millis();
  wake_duration_ms_ = WAKE_DURATION_MS;
  auto_look_x_ = 0;
  auto_look_y_ = 0;

  // Clear displays and draw initial closed eyes
  driver_->clearBothEyes();
  renderFrame();

  Serial.println("[Eyes] Engine ready — asleep");
}

// ============================================================================
// System Status
// ============================================================================

void EyeExpressionEngine::setSystemStatus(uint8_t status) {
  if (status > 5 || status == system_status_) return;

  uint8_t prev = system_status_;
  system_status_ = status;

  const char* names[] = {"IDLE","WOKE_UP","LISTENING","PROCESSING","SPEAKING","GOING_IDLE"};
  Serial.printf("[Eyes] %s -> %s\n", names[prev], names[status]);

  switch (status) {
    case 0:  // IDLE
      if (wake_level_ > 0.01f) {
        wake_target_ = 0.0f;
        wake_start_level_ = wake_level_;
        wake_start_ms_ = millis();
        wake_duration_ms_ = SLEEP_DURATION_MS;
      }
      break;

    case 1:  // WOKE_UP
      wake_target_ = 1.0f;
      wake_start_level_ = wake_level_;
      wake_start_ms_ = millis();
      wake_duration_ms_ = WAKE_DURATION_MS;
      setExpression(EXPR_NEUTRAL, 2);
      break;

    case 2:  // LISTENING
      wake_target_ = 1.0f;
      wake_start_level_ = wake_level_;
      wake_start_ms_ = millis();
      wake_duration_ms_ = 300;
      setExpression(EXPR_NEUTRAL, 3);
      auto_look_x_ = 0;
      auto_look_y_ = 0;
      break;

    case 3:  // PROCESSING
      wake_target_ = 1.0f;
      wake_start_level_ = wake_level_;
      wake_start_ms_ = millis();
      wake_duration_ms_ = 300;
      setExpression(EXPR_NEUTRAL, 2);
      break;

    case 4:  // SPEAKING
      wake_target_ = 1.0f;
      wake_start_level_ = wake_level_;
      wake_start_ms_ = millis();
      wake_duration_ms_ = 300;
      break;

    case 5:  // GOING_IDLE
      wake_target_ = 0.0f;
      wake_start_level_ = wake_level_;
      wake_start_ms_ = millis();
      wake_duration_ms_ = SLEEP_DURATION_MS;
      setExpression(EXPR_SLEEPY, 3);
      break;
  }
}

uint8_t EyeExpressionEngine::getSystemStatus()  { return system_status_; }
float   EyeExpressionEngine::getWakeLevel()      { return wake_level_; }
uint8_t EyeExpressionEngine::getCurrentExpression() { return current_expression_; }
uint8_t EyeExpressionEngine::getCurrentIntensity()  { return current_intensity_; }
bool    EyeExpressionEngine::isBlinking()            { return blink_state_.active; }

// ============================================================================
// Expression Control
// ============================================================================

void EyeExpressionEngine::setExpression(uint8_t type, uint8_t intensity) {
  if (type >= EXPR_COUNT || intensity < 1 || intensity > 5) return;

  bool changed = (type != current_expression_);
  target_expression_ = type;
  target_intensity_ = intensity;
  target_params_ = calculateExpressionParams(type, intensity);
  transition_progress_ = 0.0f;

  if (changed && !blink_state_.active && wake_level_ > 0.8f) {
    triggerBlink();
  }
}

void EyeExpressionEngine::setLookDirection(int8_t x, int8_t y) {
  look_x_ = constrain(x, -100, 100);
  look_y_ = constrain(y, -100, 100);
}

void EyeExpressionEngine::triggerBlink() {
  if (blink_state_.active || wake_level_ < 0.5f) return;

  uint16_t dur;
  switch (current_expression_) {
    case EXPR_SURPRISED: dur = 180 + random(20); break;
    case EXPR_SLEEPY:    dur = 300 + random(50); break;
    default:             dur = 200 + random(30); break;
  }

  blink_state_.active = true;
  blink_state_.start_millis = millis();
  blink_state_.duration_ms = dur;
}

// ============================================================================
// Frame Update
// ============================================================================

uint32_t EyeExpressionEngine::update() {
  driver_->startFrameTiming();

  uint32_t now = millis();
  uint32_t delta_ms = now - last_update_millis_;
  last_update_millis_ = now;

  updateWakeTransition();
  updateAutoLook();

  // Expression transition
  if (transition_progress_ < 1.0f) {
    transition_progress_ += (float)delta_ms / TRANSITION_DURATION_MS;
    if (transition_progress_ >= 1.0f) {
      transition_progress_ = 1.0f;
      current_expression_ = target_expression_;
      current_intensity_ = target_intensity_;
      current_params_ = target_params_;
      next_blink_interval_ms_ = calculateNextBlinkInterval();
      last_blink_millis_ = now;
    } else {
      if (blink_state_.active) {
        float bp = (float)(millis() - blink_state_.start_millis) / blink_state_.duration_ms;
        if (bp >= 0.4f && bp < 0.6f) {
          current_expression_ = target_expression_;
          current_intensity_ = target_intensity_;
          current_params_ = target_params_;
          transition_progress_ = 1.0f;
        }
      } else {
        interpolateParams(transition_progress_);
      }
    }
  }

  // Auto-blink
  if (wake_level_ > 0.8f && !blink_state_.active &&
      (now - last_blink_millis_ >= next_blink_interval_ms_)) {
    if (double_blink_pending_) {
      triggerBlink();
      double_blink_pending_ = false;
      next_blink_interval_ms_ = calculateNextBlinkInterval();
    } else {
      triggerBlink();
      if (random(100) < 20) {
        double_blink_pending_ = true;
        next_blink_interval_ms_ = 300 + random(200);
      } else {
        next_blink_interval_ms_ = calculateNextBlinkInterval();
      }
    }
    last_blink_millis_ = now;
  }

  if (blink_state_.active) updateBlinkAnimation();

  // Render every frame (sprite makes this flicker-free)
  renderFrame();

  driver_->endFrameTiming();
  return driver_->getLastFrameTimeMs();
}

// ============================================================================
// Wake / Sleep
// ============================================================================

void EyeExpressionEngine::updateWakeTransition() {
  if (fabsf(wake_level_ - wake_target_) < 0.005f) {
    wake_level_ = wake_target_;
    return;
  }

  float t = (float)(millis() - wake_start_ms_) / (float)wake_duration_ms_;
  if (t > 1.0f) t = 1.0f;

  float eased = (wake_target_ > wake_start_level_) ? easeOut(t) : easeInOut(t);
  wake_level_ = wake_start_level_ + (wake_target_ - wake_start_level_) * eased;

  if (t >= 1.0f) {
    wake_level_ = wake_target_;
    if (system_status_ == 5 && wake_level_ <= 0.01f) {
      system_status_ = 0;  // Settle into IDLE
    }
  }
}

// ============================================================================
// Auto-Look (PROCESSING: thinking — pupils drift in figure-8)
// ============================================================================

void EyeExpressionEngine::updateAutoLook() {
  if (system_status_ != 3) {
    auto_look_x_ = 0;
    auto_look_y_ = 0;
    return;
  }
  float t = millis() / 3000.0f;
  auto_look_x_ = (int8_t)(sinf(t * 2.0f * PI) * 40);
  auto_look_y_ = (int8_t)(sinf(t * 4.0f * PI) * 15);
}

// ============================================================================
// Closedness (combines blink + wake)
// ============================================================================

float EyeExpressionEngine::computeClosedness() {
  float blink_amount = 0.0f;
  if (blink_state_.active) {
    float p = (float)(millis() - blink_state_.start_millis) / blink_state_.duration_ms;
    if (p < 0.25f)       blink_amount = p / 0.25f;
    else if (p < 0.5f)   blink_amount = 1.0f;
    else if (p < 0.75f)  blink_amount = 1.0f - (p - 0.5f) / 0.25f;
    else                  blink_amount = 0.0f;
  }
  return fmaxf(blink_amount, 1.0f - wake_level_);
}

// ============================================================================
// Sprite-Based Rendering
// ============================================================================

void EyeExpressionEngine::renderFrame() {
  float closedness = computeClosedness();

  // Calculate eye position in sprite coordinates
  int16_t total_lx = constrain(look_x_ + auto_look_x_, -100, 100);
  int16_t total_ly = constrain(look_y_ + auto_look_y_, -100, 100);
  int16_t look_dx = (total_lx * MAX_PUPIL_OFFSET) / 100;
  int16_t look_dy = -(total_ly * MAX_PUPIL_OFFSET) / 100;

  int16_t eye_x = SPRITE_CENTER + current_params_.pupil_offset_x + look_dx;
  int16_t eye_y = SPRITE_CENTER + current_params_.pupil_offset_y + look_dy;

  // --- Left eye ---
  sprite_->fillSprite(TFT_BLACK);
  renderEye(eye_x, eye_y, current_params_.pupil_radius,
            current_params_.eye_shape, closedness, false);
  driver_->selectEye(LEFT);
  sprite_->pushSprite(SPRITE_OFFSET, SPRITE_OFFSET);

  // --- Right eye ---
  if (current_params_.asymmetric) {
    sprite_->fillSprite(TFT_BLACK);
    int16_t rx = eye_x + current_params_.right_eye_offset_x;
    int16_t ry = eye_y + current_params_.right_eye_offset_y;
    renderEye(rx, ry, current_params_.right_eye_radius,
              current_params_.right_eye_shape, closedness, true);
  }
  driver_->selectEye(RIGHT);
  sprite_->pushSprite(SPRITE_OFFSET, SPRITE_OFFSET);
}

void EyeExpressionEngine::renderEye(int16_t x, int16_t y, uint16_t radius,
                                      EyeShape shape, float closedness,
                                      bool is_right) {
  // Fully closed → thin line
  if (closedness > 0.85f) {
    drawClosedLine(x, y, radius);
    return;
  }

  float height_scale = 1.0f - (closedness * 0.875f);

  switch (shape) {
    case SHAPE_ROUNDED_RECT: drawRoundedRect(x, y, radius, height_scale); break;
    case SHAPE_CIRCLE:       drawCircle(x, y, radius, height_scale);      break;
    case SHAPE_HAPPY_ARC:    drawHappyArc(x, y, radius, height_scale);    break;
    case SHAPE_SAD_ARC:      drawSadArc(x, y, radius, height_scale);      break;
    case SHAPE_ANGRY_V:      drawAngryV(x, y, radius, height_scale, is_right); break;
    case SHAPE_SLEEPY_LINE:  drawSleepyLine(x, y, radius, height_scale);  break;
    case SHAPE_WINK:         drawClosedLine(x, y, radius);                break;
    default:                 drawRoundedRect(x, y, radius, height_scale); break;
  }
}

// ============================================================================
// Shape Primitives — all draw to sprite_ in white
// ============================================================================

void EyeExpressionEngine::drawClosedLine(int16_t x, int16_t y, uint16_t radius) {
  int16_t w = (int16_t)(radius * 1.8f);
  sprite_->fillRoundRect(x - w / 2, y - 2, w, 5, 2, EYE_COLOR);
}

void EyeExpressionEngine::drawRoundedRect(int16_t x, int16_t y,
                                            uint16_t radius, float hs) {
  int16_t w = radius * 2;
  int16_t h = (int16_t)(radius * 2 * hs);
  uint8_t cr = radius / 3;
  sprite_->fillRoundRect(x - radius, y - h / 2, w, h, cr, EYE_COLOR);
}

void EyeExpressionEngine::drawCircle(int16_t x, int16_t y,
                                      uint16_t radius, float hs) {
  if (hs > 0.95f) {
    sprite_->fillCircle(x, y, radius, EYE_COLOR);
  } else {
    int16_t eh = (int16_t)(radius * hs);
    for (int16_t dy = -eh; dy <= eh; dy++) {
      float ratio = (float)dy / (float)eh;
      int16_t dx = (int16_t)(sqrtf(1.0f - ratio * ratio) * radius);
      sprite_->drawFastHLine(x - dx, y + dy, dx * 2, EYE_COLOR);
    }
  }
}

void EyeExpressionEngine::drawHappyArc(int16_t x, int16_t y,
                                         uint16_t radius, float hs) {
  int16_t arc_w = (int16_t)(radius * 2.2f);
  int16_t arc_h = (int16_t)(radius * 1.0f * hs);
  int16_t thick = max(6, (int)(radius / 3));

  for (int16_t dy = 0; dy < arc_h; dy++) {
    float t = (float)dy / arc_h;
    float curve = sqrtf(1.0f - t * t);
    int16_t hw = (int16_t)(arc_w * 0.5f * curve);
    for (int16_t th = 0; th < thick; th++) {
      sprite_->drawFastHLine(x - hw, y + dy + th, hw * 2, EYE_COLOR);
    }
  }
}

void EyeExpressionEngine::drawSadArc(int16_t x, int16_t y,
                                       uint16_t radius, float hs) {
  int16_t arc_w = (int16_t)(radius * 2.2f);
  int16_t arc_h = (int16_t)(radius * 1.0f * hs);
  int16_t thick = max(6, (int)(radius / 3));

  for (int16_t dy = 0; dy < arc_h; dy++) {
    float t = (float)dy / arc_h;
    float curve = sqrtf(1.0f - (1.0f - t) * (1.0f - t));
    int16_t hw = (int16_t)(arc_w * 0.5f * curve);
    for (int16_t th = 0; th < thick; th++) {
      sprite_->drawFastHLine(x - hw, y - dy - th, hw * 2, EYE_COLOR);
    }
  }
}

void EyeExpressionEngine::drawAngryV(int16_t x, int16_t y,
                                       uint16_t radius, float hs,
                                       bool is_right) {
  int16_t v_w = (int16_t)(radius * 1.6f);
  int16_t v_h = (int16_t)(radius * 0.7f * hs);
  int16_t thick = max(4, (int)(radius / 4));
  int16_t dir = is_right ? 1 : -1;

  // V-shaped brow
  for (int16_t th = 0; th < thick; th++) {
    for (int16_t dx = 0; dx < v_w; dx++) {
      int16_t dy = (v_h * dx / v_w) * dir;
      sprite_->drawPixel(x - v_w / 2 + dx, y - v_h / 2 + dy + th, EYE_COLOR);
    }
  }

  // Pupil below
  int16_t py = y + v_h / 2 + radius / 3;
  int16_t pr = (int16_t)(radius * 0.45f * hs);
  if (pr > 3) sprite_->fillCircle(x, py, pr, EYE_COLOR);
}

void EyeExpressionEngine::drawSleepyLine(int16_t x, int16_t y,
                                           uint16_t radius, float hs) {
  int16_t w = (int16_t)(radius * 1.8f);
  int16_t h = max(6, (int)(radius / 2.5f));
  h = (int16_t)(h * hs);
  if (h < 4) h = 4;
  sprite_->fillRoundRect(x - w / 2, y - h / 2, w, h, 3, EYE_COLOR);
}

// ============================================================================
// Expression Parameters
// ============================================================================

ExpressionParams EyeExpressionEngine::calculateExpressionParams(
    uint8_t type, uint8_t intensity) {
  ExpressionParams p = {};
  p.pupil_radius = BASE_PUPIL_RADIUS;
  p.eye_shape = SHAPE_ROUNDED_RECT;
  p.asymmetric = false;
  p.right_eye_radius = BASE_PUPIL_RADIUS;
  p.right_eye_shape = SHAPE_ROUNDED_RECT;

  float f = 0.5f + (intensity * 0.3f);  // intensity factor

  switch (type) {
    case EXPR_NEUTRAL:
      break;

    case EXPR_HAPPY:
      p.pupil_radius = BASE_PUPIL_RADIUS + (int16_t)(10 * f);
      p.eye_shape = SHAPE_HAPPY_ARC;
      p.pupil_offset_y = (int16_t)(-4 * f);
      break;

    case EXPR_SAD:
      p.pupil_radius = BASE_PUPIL_RADIUS - (int16_t)(5 * f);
      if (p.pupil_radius < MIN_PUPIL_RADIUS) p.pupil_radius = MIN_PUPIL_RADIUS;
      p.eye_shape = SHAPE_SAD_ARC;
      p.pupil_offset_y = (int16_t)(10 * f);
      break;

    case EXPR_SURPRISED:
      p.pupil_radius = BASE_PUPIL_RADIUS + (int16_t)(20 * f);
      if (p.pupil_radius > MAX_PUPIL_RADIUS) p.pupil_radius = MAX_PUPIL_RADIUS;
      p.eye_shape = SHAPE_CIRCLE;
      break;

    case EXPR_ANGRY:
      p.pupil_radius = BASE_PUPIL_RADIUS - (int16_t)(5 * f);
      p.eye_shape = SHAPE_ANGRY_V;
      p.pupil_offset_y = (int16_t)(6 * f);
      p.asymmetric = true;
      p.right_eye_radius = p.pupil_radius;
      p.right_eye_shape = SHAPE_ANGRY_V;
      break;

    case EXPR_SLEEPY:
      p.pupil_radius = BASE_PUPIL_RADIUS - (int16_t)(12 * f);
      if (p.pupil_radius < MIN_PUPIL_RADIUS) p.pupil_radius = MIN_PUPIL_RADIUS;
      p.eye_shape = SHAPE_SLEEPY_LINE;
      p.pupil_offset_y = (int16_t)(10 * f);
      break;

    case EXPR_WINK:
      p.eye_shape = SHAPE_HAPPY_ARC;
      p.asymmetric = true;
      p.right_eye_shape = SHAPE_WINK;
      break;
  }

  return p;
}

// ============================================================================
// Interpolation
// ============================================================================

void EyeExpressionEngine::interpolateParams(float progress) {
  ExpressionParams& c = current_params_;
  ExpressionParams& t = target_params_;

  c.pupil_radius += (int16_t)((t.pupil_radius - c.pupil_radius) * progress);
  c.pupil_offset_x += (int16_t)((t.pupil_offset_x - c.pupil_offset_x) * progress);
  c.pupil_offset_y += (int16_t)((t.pupil_offset_y - c.pupil_offset_y) * progress);

  if (progress > 0.5f) {
    c.eye_shape = t.eye_shape;
  }

  if (t.asymmetric) {
    c.asymmetric = true;
    c.right_eye_offset_x += (int16_t)((t.right_eye_offset_x - c.right_eye_offset_x) * progress);
    c.right_eye_offset_y += (int16_t)((t.right_eye_offset_y - c.right_eye_offset_y) * progress);
    c.right_eye_radius += (int16_t)((t.right_eye_radius - c.right_eye_radius) * progress);
  } else {
    c.asymmetric = false;
    c.right_eye_radius = c.pupil_radius;
  }
}

// ============================================================================
// Blink
// ============================================================================

void EyeExpressionEngine::updateBlinkAnimation() {
  if (millis() - blink_state_.start_millis >= blink_state_.duration_ms) {
    blink_state_.active = false;
  }
}

uint32_t EyeExpressionEngine::calculateNextBlinkInterval() {
  uint32_t base = 5000, vary = 2500;
  switch (current_expression_) {
    case EXPR_HAPPY:     base = 4000; vary = 2000; break;
    case EXPR_SAD:       base = 6000; vary = 2000; break;
    case EXPR_SURPRISED: base = 3000; vary = 1500; break;
    case EXPR_ANGRY:     base = 4000; vary = 1500; break;
    case EXPR_SLEEPY:    base = 3000; vary = 1000; break;
    case EXPR_WINK:      base = 4000; vary = 2000; break;
    default: break;
  }
  float f = 1.4f - (current_intensity_ * 0.08f);
  base = (uint32_t)(base * f);
  return constrain(base + random(-vary, vary), 1200UL, 8000UL);
}
