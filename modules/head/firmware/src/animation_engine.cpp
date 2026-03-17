/**
 * animation_engine.cpp - Layered Eye Animation Engine
 * Story 1.3: Develop Head ESP32 Firmware
 *
 * Renders to a 200x200 TFT_eSprite in PSRAM, then pushes the
 * complete frame to each display. Zero flicker.
 *
 * Each eye is drawn in four layers:
 *   1. Colored iris circle  (teal, height-clipped for blink/wake)
 *   2. Black eyelid mask    (expression-specific shape)
 *   3. Dark pupil circle    (offset by look direction, ~40% iris radius)
 *   4. White highlight dot  (specular, top-right of pupil)
 */

#include "animation_engine.h"
#include <math.h>

// ============================================================================
// Constants
// ============================================================================

constexpr uint16_t BASE_IRIS_RADIUS  = 65;   // Fills ~65% of sprite half-width
constexpr uint16_t MAX_IRIS_RADIUS   = 88;   // Surprised: near display edge
constexpr uint16_t MIN_IRIS_RADIUS   = 30;
constexpr int16_t  MAX_PUPIL_OFFSET  = 22;   // Look direction range (px)

constexpr uint32_t TRANSITION_MS  = 200;
constexpr uint32_t BLINK_MS       = 250;
constexpr uint32_t WAKE_MS        = 250;
constexpr uint32_t SLEEP_MS       = 600;

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
  current_intensity_  = 2;
  target_expression_  = EXPR_NEUTRAL;
  target_intensity_   = 2;
  current_params_     = calculateExpressionParams(EXPR_NEUTRAL, 2);
  target_params_      = current_params_;

  look_x_ = 0;
  look_y_ = 0;
  transition_progress_ = 1.0f;
  last_update_millis_  = millis();

  blink_state_           = {false, 0, BLINK_MS};
  last_blink_millis_     = millis();
  next_blink_interval_ms_ = calculateNextBlinkInterval();
  double_blink_pending_  = false;

  system_status_   = 0;
  wake_level_      = 0.0f;
  wake_target_     = 0.0f;
  wake_start_level_ = 0.0f;
  wake_start_ms_   = millis();
  wake_duration_ms_ = WAKE_MS;
  auto_look_x_     = 0;
  auto_look_y_     = 0;

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
        wake_target_      = 0.0f;
        wake_start_level_ = wake_level_;
        wake_start_ms_    = millis();
        wake_duration_ms_ = SLEEP_MS;
      }
      break;

    case 1:  // WOKE_UP
      wake_target_      = 1.0f;
      wake_start_level_ = wake_level_;
      wake_start_ms_    = millis();
      wake_duration_ms_ = WAKE_MS;
      setExpression(EXPR_NEUTRAL, 2);
      break;

    case 2:  // LISTENING
      wake_target_      = 1.0f;
      wake_start_level_ = wake_level_;
      wake_start_ms_    = millis();
      wake_duration_ms_ = 300;
      setExpression(EXPR_NEUTRAL, 3);
      auto_look_x_ = 0;
      auto_look_y_ = 0;
      break;

    case 3:  // PROCESSING
      wake_target_      = 1.0f;
      wake_start_level_ = wake_level_;
      wake_start_ms_    = millis();
      wake_duration_ms_ = 300;
      setExpression(EXPR_NEUTRAL, 2);
      break;

    case 4:  // SPEAKING
      wake_target_      = 1.0f;
      wake_start_level_ = wake_level_;
      wake_start_ms_    = millis();
      wake_duration_ms_ = 300;
      break;

    case 5:  // GOING_IDLE
      wake_target_      = 0.0f;
      wake_start_level_ = wake_level_;
      wake_start_ms_    = millis();
      wake_duration_ms_ = SLEEP_MS;
      setExpression(EXPR_SLEEPY, 3);
      break;
  }
}

uint8_t EyeExpressionEngine::getSystemStatus()        { return system_status_; }
float   EyeExpressionEngine::getWakeLevel()           { return wake_level_; }
uint8_t EyeExpressionEngine::getCurrentExpression()   { return current_expression_; }
uint8_t EyeExpressionEngine::getCurrentIntensity()    { return current_intensity_; }
bool    EyeExpressionEngine::isBlinking()             { return blink_state_.active; }

// ============================================================================
// Expression Control
// ============================================================================

void EyeExpressionEngine::setExpression(uint8_t type, uint8_t intensity) {
  if (type >= EXPR_COUNT || intensity < 1 || intensity > 5) return;

  bool changed      = (type != current_expression_);
  target_expression_ = type;
  target_intensity_  = intensity;
  target_params_     = calculateExpressionParams(type, intensity);
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
    case EXPR_SURPRISED: dur = 160 + random(20); break;
    case EXPR_SLEEPY:    dur = 320 + random(50); break;
    default:             dur = 200 + random(30); break;
  }

  blink_state_.active       = true;
  blink_state_.start_millis = millis();
  blink_state_.duration_ms  = dur;
}

// ============================================================================
// Frame Update
// ============================================================================

uint32_t EyeExpressionEngine::update() {
  driver_->startFrameTiming();

  uint32_t now      = millis();
  uint32_t delta_ms = now - last_update_millis_;
  last_update_millis_ = now;

  updateWakeTransition();
  updateAutoLook();

  // Expression transition
  if (transition_progress_ < 1.0f) {
    transition_progress_ += (float)delta_ms / TRANSITION_MS;
    if (transition_progress_ >= 1.0f) {
      transition_progress_ = 1.0f;
      current_expression_  = target_expression_;
      current_intensity_   = target_intensity_;
      current_params_      = target_params_;
      next_blink_interval_ms_ = calculateNextBlinkInterval();
      last_blink_millis_   = now;
    } else {
      if (blink_state_.active) {
        float bp = (float)(millis() - blink_state_.start_millis) / blink_state_.duration_ms;
        if (bp >= 0.4f && bp < 0.6f) {
          current_expression_  = target_expression_;
          current_intensity_   = target_intensity_;
          current_params_      = target_params_;
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
      double_blink_pending_    = false;
      next_blink_interval_ms_ = calculateNextBlinkInterval();
    } else {
      triggerBlink();
      if (random(100) < 20) {
        double_blink_pending_    = true;
        next_blink_interval_ms_ = 300 + random(200);
      } else {
        next_blink_interval_ms_ = calculateNextBlinkInterval();
      }
    }
    last_blink_millis_ = now;
  }

  if (blink_state_.active) updateBlinkAnimation();

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

  float eased  = (wake_target_ > wake_start_level_) ? easeOut(t) : easeInOut(t);
  wake_level_  = wake_start_level_ + (wake_target_ - wake_start_level_) * eased;

  if (t >= 1.0f) {
    wake_level_ = wake_target_;
    if (system_status_ == 5 && wake_level_ <= 0.01f) {
      system_status_ = 0;  // Settle into IDLE
    }
  }
}

// ============================================================================
// Auto-Look (PROCESSING: figure-8 drift)
// ============================================================================

void EyeExpressionEngine::updateAutoLook() {
  if (system_status_ != 3) {
    auto_look_x_ = 0;
    auto_look_y_ = 0;
    return;
  }
  float t      = millis() / 3000.0f;
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
    if (p < 0.25f)      blink_amount = p / 0.25f;
    else if (p < 0.5f)  blink_amount = 1.0f;
    else if (p < 0.75f) blink_amount = 1.0f - (p - 0.5f) / 0.25f;
    else                blink_amount = 0.0f;
  }
  return fmaxf(blink_amount, 1.0f - wake_level_);
}

// ============================================================================
// Frame Rendering
// ============================================================================

void EyeExpressionEngine::renderFrame() {
  float closedness = computeClosedness();

  int16_t total_lx = constrain(look_x_ + auto_look_x_, -100, 100);
  int16_t total_ly = constrain(look_y_ + auto_look_y_, -100, 100);
  int16_t look_dx  = (total_lx * MAX_PUPIL_OFFSET) / 100;
  int16_t look_dy  = -(total_ly * MAX_PUPIL_OFFSET) / 100;

  int16_t eye_x = SPRITE_CENTER + current_params_.pupil_offset_x + look_dx;
  int16_t eye_y = SPRITE_CENTER + current_params_.pupil_offset_y + look_dy;

  // Left eye
  sprite_->fillSprite(TFT_BLACK);
  renderEye(eye_x, eye_y, current_params_.pupil_radius,
            current_params_.eye_shape, closedness, false);
  driver_->selectEye(LEFT);
  sprite_->pushSprite(SPRITE_OFFSET, SPRITE_OFFSET);

  // Right eye
  sprite_->fillSprite(TFT_BLACK);
  if (current_params_.asymmetric) {
    int16_t rx = eye_x + current_params_.right_eye_offset_x;
    int16_t ry = eye_y + current_params_.right_eye_offset_y;
    renderEye(rx, ry, current_params_.right_eye_radius,
              current_params_.right_eye_shape, closedness, true);
  } else {
    renderEye(eye_x, eye_y, current_params_.pupil_radius,
              current_params_.eye_shape, closedness, true);
  }
  driver_->selectEye(RIGHT);
  sprite_->pushSprite(SPRITE_OFFSET, SPRITE_OFFSET);
}

// ============================================================================
// Eye Rendering — Layered: Iris → Eyelid mask → Pupil → Highlight
// ============================================================================

void EyeExpressionEngine::renderEye(int16_t x, int16_t y, uint16_t radius,
                                      EyeShape shape, float closedness,
                                      bool is_right) {
  // Fully closed → single thin line
  if (closedness > 0.85f) {
    drawClosedLine(x, y, radius);
    return;
  }

  // Wink → always closed
  if (shape == SHAPE_WINK) {
    drawClosedLine(x, y, radius);
    return;
  }

  float hs = 1.0f - (closedness * 0.875f);  // height scale (1.0 = fully open)

  // Layer 1: Colored iris
  drawIrisCircle(x, y, radius, hs);

  // Layer 2: Eyelid mask in black
  applyEyelidMask(x, y, radius, shape, hs, is_right);

  // Layer 3: Pupil (dark circle — skip only for SLEEPY which is too narrow)
  bool has_pupil = (shape != SHAPE_SLEEPY);
  if (has_pupil) {
    uint16_t pr = (uint16_t)(radius * 0.40f);
    if (pr >= 4) {
      int16_t py = y + (int16_t)(radius * 0.05f);  // Slightly below center
      sprite_->fillCircle(x, py, pr, TFT_BLACK);
    }
  }

  // Layer 4: Double specular highlights (large + small, anime/cartoon style)
  if (shape != SHAPE_WINK && hs > 0.35f) {
    uint16_t hr1 = max(4, (int)(radius * 0.20f));   // Primary highlight
    uint16_t hr2 = max(2, (int)(radius * 0.09f));   // Secondary highlight

    if (shape == SHAPE_HAPPY) {
      // Happy: bottom is masked — place highlights in visible top arc
      int16_t hx1 = x + (int16_t)(radius * 0.22f);
      int16_t hy1 = y - (int16_t)(radius * 0.30f);
      sprite_->fillCircle(hx1, hy1, hr1, HIGHLIGHT_COLOR);
      int16_t hx2 = x - (int16_t)(radius * 0.16f);
      int16_t hy2 = y - (int16_t)(radius * 0.12f);
      sprite_->fillCircle(hx2, hy2, hr2, HIGHLIGHT_COLOR);
    } else {
      // All other expressions: top-right primary, bottom-left secondary
      int16_t hx1 = x + (int16_t)(radius * 0.24f);
      int16_t hy1 = y - (int16_t)(radius * 0.26f);
      sprite_->fillCircle(hx1, hy1, hr1, HIGHLIGHT_COLOR);
      int16_t hx2 = x - (int16_t)(radius * 0.18f);
      int16_t hy2 = y + (int16_t)(radius * 0.20f);
      sprite_->fillCircle(hx2, hy2, hr2, HIGHLIGHT_COLOR);
    }
  }
}

// ============================================================================
// Layer 1: Iris Circle (height-clipped for blink/wake)
// ============================================================================

void EyeExpressionEngine::drawIrisCircle(int16_t x, int16_t y,
                                           uint16_t r, float hs) {
  int16_t clipped_h = (int16_t)(r * hs);
  for (int16_t dy = -clipped_h; dy <= clipped_h; dy++) {
    float norm = (float)dy / (float)r;          // Use full r for round shape
    if (fabsf(norm) >= 1.0f) continue;
    int16_t dx = (int16_t)(sqrtf(1.0f - norm * norm) * r);
    sprite_->drawFastHLine(x - dx, y + dy, dx * 2 + 1, IRIS_COLOR);
  }
}

// ============================================================================
// Layer 2: Eyelid Mask (black overlay, expression-specific)
// ============================================================================

void EyeExpressionEngine::applyEyelidMask(int16_t x, int16_t y, uint16_t r,
                                            EyeShape shape, float hs,
                                            bool is_right) {
  int16_t top    = y - (int16_t)(r * hs);      // Top edge of iris
  int16_t width  = (int16_t)(r * 2) + 4;       // Full iris width + margin

  switch (shape) {

    case SHAPE_HAPPY: {
      // Cover bottom 50% → top arc visible (^ ^ smiling eyes)
      int16_t bottom_start = y;                  // Mask from center downward
      int16_t bottom_end   = y + (int16_t)(r * hs) + 2;
      sprite_->fillRect(x - r - 2, bottom_start, width, bottom_end - bottom_start, TFT_BLACK);
      break;
    }

    case SHAPE_SAD: {
      // Drooping inner corner eyelid — inner side pulled much lower
      // Left eye (is_right=false): inner = right edge (t=1.0)
      // Right eye (is_right=true): inner = left edge  (t=0.0)
      float outer_frac = 0.12f;   // 12% covered at outer edge
      float inner_frac = 0.58f;   // 58% covered at inner edge
      for (int16_t col = 0; col < (int16_t)(r * 2); col++) {
        float t    = (float)col / (float)(r * 2);
        float frac = is_right
                       ? (inner_frac + (outer_frac - inner_frac) * t)
                       : (outer_frac + (inner_frac - outer_frac) * t);
        int16_t drop = (int16_t)(frac * 2.0f * r * hs);
        sprite_->drawFastVLine(x - r + col, top, drop, TFT_BLACK);
      }
      break;
    }

    case SHAPE_ANGRY: {
      // V-brow: inner corner pulled far down, outer side barely cut
      float outer_frac = 0.05f;   // 5% at outer edge
      float inner_frac = 0.68f;   // 68% at inner edge
      for (int16_t col = 0; col < (int16_t)(r * 2); col++) {
        float t    = (float)col / (float)(r * 2);
        float frac = is_right
                       ? (inner_frac + (outer_frac - inner_frac) * t)
                       : (outer_frac + (inner_frac - outer_frac) * t);
        int16_t cut = (int16_t)(frac * 2.0f * r * hs);
        sprite_->drawFastVLine(x - r + col, top, cut, TFT_BLACK);
      }
      break;
    }

    case SHAPE_SLEEPY: {
      // Heavy top eyelid — covers 65% from top
      int16_t cut = (int16_t)(r * hs * 1.3f);  // 65% of visible height
      sprite_->fillRect(x - r - 2, top, width, cut, TFT_BLACK);
      break;
    }

    case SHAPE_NEUTRAL: {
      // Slight top lid (8%) — natural relaxed look
      int16_t cut = (int16_t)(r * hs * 0.16f);
      sprite_->fillRect(x - r - 2, top, width, cut, TFT_BLACK);
      break;
    }

    case SHAPE_CIRCLE:
    default:
      // No mask — fully open (surprised / wide awake)
      break;
  }
}

// ============================================================================
// Closed Line (blink / wink / fully asleep)
// ============================================================================

void EyeExpressionEngine::drawClosedLine(int16_t x, int16_t y, uint16_t radius) {
  int16_t w = (int16_t)(radius * 1.8f);
  sprite_->fillRoundRect(x - w / 2, y - 3, w, 6, 3, IRIS_COLOR);
}

// ============================================================================
// Expression Parameters
// ============================================================================

ExpressionParams EyeExpressionEngine::calculateExpressionParams(
    uint8_t type, uint8_t intensity) {
  ExpressionParams p = {};
  p.pupil_radius    = BASE_IRIS_RADIUS;
  p.eye_shape       = SHAPE_NEUTRAL;
  p.asymmetric      = false;
  p.right_eye_radius = BASE_IRIS_RADIUS;
  p.right_eye_shape  = SHAPE_NEUTRAL;

  float f = 0.5f + (intensity * 0.3f);  // Intensity scale factor

  switch (type) {
    case EXPR_NEUTRAL:
      break;

    case EXPR_HAPPY:
      p.pupil_radius    = BASE_IRIS_RADIUS + (int16_t)(8 * f);
      p.eye_shape       = SHAPE_HAPPY;
      p.pupil_offset_y  = (int16_t)(-5 * f);
      break;

    case EXPR_SAD:
      p.pupil_radius    = BASE_IRIS_RADIUS - (int16_t)(8 * f);
      if (p.pupil_radius < MIN_IRIS_RADIUS) p.pupil_radius = MIN_IRIS_RADIUS;
      p.eye_shape       = SHAPE_SAD;
      p.pupil_offset_y  = (int16_t)(8 * f);
      break;

    case EXPR_SURPRISED:
      p.pupil_radius    = BASE_IRIS_RADIUS + (int16_t)(22 * f);
      if (p.pupil_radius > MAX_IRIS_RADIUS) p.pupil_radius = MAX_IRIS_RADIUS;
      p.eye_shape       = SHAPE_CIRCLE;
      break;

    case EXPR_ANGRY:
      p.pupil_radius    = BASE_IRIS_RADIUS - (int16_t)(6 * f);
      p.eye_shape       = SHAPE_ANGRY;
      p.pupil_offset_y  = (int16_t)(8 * f);
      p.asymmetric      = true;
      p.right_eye_radius = p.pupil_radius;
      p.right_eye_shape  = SHAPE_ANGRY;
      break;

    case EXPR_SLEEPY:
      p.pupil_radius    = BASE_IRIS_RADIUS - (int16_t)(10 * f);
      if (p.pupil_radius < MIN_IRIS_RADIUS) p.pupil_radius = MIN_IRIS_RADIUS;
      p.eye_shape       = SHAPE_SLEEPY;
      p.pupil_offset_y  = (int16_t)(10 * f);
      break;

    case EXPR_WINK:
      p.eye_shape        = SHAPE_HAPPY;
      p.asymmetric       = true;
      p.right_eye_shape  = SHAPE_WINK;
      p.right_eye_radius = p.pupil_radius;
      break;
  }

  // Mirror right eye params when not already asymmetric
  if (!p.asymmetric) {
    p.right_eye_radius = p.pupil_radius;
    p.right_eye_shape  = p.eye_shape;
  }

  return p;
}

// ============================================================================
// Interpolation
// ============================================================================

void EyeExpressionEngine::interpolateParams(float progress) {
  ExpressionParams& c = current_params_;
  ExpressionParams& t = target_params_;

  c.pupil_radius    += (int16_t)((t.pupil_radius    - c.pupil_radius)    * progress);
  c.pupil_offset_x  += (int16_t)((t.pupil_offset_x  - c.pupil_offset_x)  * progress);
  c.pupil_offset_y  += (int16_t)((t.pupil_offset_y  - c.pupil_offset_y)  * progress);

  if (progress > 0.5f) {
    c.eye_shape = t.eye_shape;
  }

  if (t.asymmetric) {
    c.asymmetric          = true;
    c.right_eye_offset_x += (int16_t)((t.right_eye_offset_x - c.right_eye_offset_x) * progress);
    c.right_eye_offset_y += (int16_t)((t.right_eye_offset_y - c.right_eye_offset_y) * progress);
    c.right_eye_radius   += (int16_t)((t.right_eye_radius   - c.right_eye_radius)   * progress);
  } else {
    c.asymmetric       = false;
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
    case EXPR_SAD:       base = 6500; vary = 2000; break;
    case EXPR_SURPRISED: base = 2800; vary = 1200; break;
    case EXPR_ANGRY:     base = 4000; vary = 1500; break;
    case EXPR_SLEEPY:    base = 2500; vary = 800;  break;
    case EXPR_WINK:      base = 4000; vary = 2000; break;
    default: break;
  }
  float f = 1.4f - (current_intensity_ * 0.08f);
  base = (uint32_t)(base * f);
  return constrain(base + random(-(int32_t)vary, (int32_t)vary), 1200UL, 8000UL);
}
