/**
 * eye_expression.cpp - Eye Expression Animation Engine Implementation
 * Story 1.4: Head Module ESP32 Firmware - Eye Expressions
 *
 * Expression Design Notes:
 *
 * NEUTRAL (0): Calm, attentive baseline
 *   - Medium pupils, centered
 *   - Minimal movement
 *   - Intensity scales pupil size slightly
 *
 * HAPPY (1): Joyful, positive emotion
 *   - Larger pupils (dilated from positive emotion)
 *   - Slight upward offset
 *   - Intensity increases pupil size and upward shift
 *
 * CURIOUS (2): Interested, investigating
 *   - Large pupils (attention/interest response)
 *   - Slight horizontal offset (looking at something)
 *   - Intensity increases size and offset
 *
 * THINKING (3): Pondering, processing
 *   - Medium pupils looking up-right (accessing memory)
 *   - Slower movement
 *   - Intensity increases offset angle
 *
 * CONFUSED (4): Uncertain, puzzled
 *   - Pupils wandering asymmetrically
 *   - Left/right eyes look different directions
 *   - Intensity increases wandering speed and asymmetry
 *
 * SAD (5): Disappointed, melancholy
 *   - Small pupils (low energy)
 *   - Downward gaze
 *   - Intensity increases downward offset
 *
 * EXCITED (6): Enthusiastic, energetic
 *   - Very large pupils (high arousal)
 *   - Rapid micro-movements
 *   - Intensity increases pupil size and movement speed
 */

#include "../firmware/eye_expression.h"
#include <math.h>

// Base constants
constexpr uint16_t BASE_PUPIL_RADIUS = 40;        // Base pupil size
constexpr uint16_t MAX_PUPIL_RADIUS = 80;         // Maximum pupil size
constexpr uint16_t MIN_PUPIL_RADIUS = 20;         // Minimum pupil size
constexpr int16_t MAX_PUPIL_OFFSET = 30;          // Maximum offset from center
constexpr uint32_t TRANSITION_DURATION_MS = 200;  // Expression transition time
constexpr uint32_t BLINK_DURATION_MS = 250;       // Standard blink duration

// Animation timing
constexpr uint32_t TARGET_FRAME_TIME_MS = 16;     // 60 FPS = 16.67ms per frame

// ============================================================================
// Public Methods
// ============================================================================

void EyeExpressionEngine::begin(GC9A01DualDriver* driver) {
  driver_ = driver;

  // Initialize with neutral expression at medium intensity
  current_expression_ = EXPR_NEUTRAL;
  current_intensity_ = 2;
  target_expression_ = EXPR_NEUTRAL;
  target_intensity_ = 2;

  // Calculate initial params
  current_params_ = calculateExpressionParams(EXPR_NEUTRAL, 2);
  target_params_ = current_params_;

  // Initialize animation state
  transition_progress_ = 1.0;  // Start fully transitioned
  last_update_millis_ = millis();
  animation_phase_ = 0;

  // Initialize blink state
  blink_state_.active = false;
  blink_state_.phase = 0;
  blink_state_.start_millis = 0;
  blink_state_.duration_ms = BLINK_DURATION_MS;

  // Initialize automatic blinking (natural human-like)
  last_blink_millis_ = millis();
  next_blink_interval_ms_ = calculateNextBlinkInterval();
  double_blink_pending_ = false;
  blink_count_in_cluster_ = 0;

  // Set color palette (default black background)
  color_background_ = TFT_BLACK;
  color_pupil_ = TFT_CYAN;  // Default cyan (will be overridden per expression)

  // Render initial eyes (static, won't redraw unless needed)
  renderPupils();

  Serial.println("[Expression] Engine initialized - NEUTRAL expression");
}

void EyeExpressionEngine::setExpression(uint8_t expression_type, uint8_t intensity) {
  // Validate inputs
  if (expression_type > EXPR_EXCITED) {
    Serial.printf("[Expression] ERROR: Invalid expression type %d\n", expression_type);
    return;
  }
  if (intensity < 1 || intensity > 5) {
    Serial.printf("[Expression] ERROR: Invalid intensity %d\n", intensity);
    return;
  }

  // Set new target expression
  target_expression_ = expression_type;
  target_intensity_ = intensity;
  target_params_ = calculateExpressionParams(expression_type, intensity);

  // Start transition animation
  transition_progress_ = 0.0;

  const char* expr_names[] = {"NEUTRAL", "HAPPY", "CURIOUS", "THINKING", "CONFUSED", "SAD", "EXCITED"};
  Serial.printf("[Expression] Transitioning to %s (intensity %d)\n",
                expr_names[expression_type], intensity);
}

void EyeExpressionEngine::triggerBlink() {
  if (blink_state_.active) {
    // Blink already in progress, ignore
    return;
  }

  // Reduced blink duration range for more consistent timing (180-250ms)
  uint16_t blink_duration = BLINK_DURATION_MS;

  // Expression affects blink speed (but within narrower range)
  switch (current_expression_) {
    case EXPR_EXCITED:
    case EXPR_CURIOUS:
      blink_duration = 180 + random(0, 20);  // Quick: 180-200ms
      break;
    case EXPR_THINKING:
    case EXPR_SAD:
      blink_duration = 230 + random(0, 20);  // Slower: 230-250ms
      break;
    default:
      blink_duration = 200 + random(0, 30);  // Normal: 200-230ms
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
      Serial.println("[Expression] Transition complete");

      // Calculate new blink interval for new expression
      next_blink_interval_ms_ = calculateNextBlinkInterval();
      last_blink_millis_ = now;
    } else {
      interpolateParams(transition_progress_);
    }
    needs_redraw = true;  // Redraw during transition
  }

  // Check for automatic blink (only when not already blinking)
  if (!blink_state_.active && (now - last_blink_millis_ >= next_blink_interval_ms_)) {

    // Check if this should be a double blink (humans do this ~20% of the time)
    if (double_blink_pending_) {
      // Second blink of a double blink - happens quickly after first
      Serial.println("[Expression] Auto-blink (double-blink #2)");
      triggerBlink();
      double_blink_pending_ = false;
      last_blink_millis_ = now;
      next_blink_interval_ms_ = calculateNextBlinkInterval();
    } else {
      // Normal blink
      Serial.println("[Expression] Auto-blink triggered");
      triggerBlink();
      last_blink_millis_ = now;

      // 20% chance of double blink (more likely when confused/excited)
      bool is_high_energy = (current_expression_ == EXPR_EXCITED ||
                              current_expression_ == EXPR_CONFUSED ||
                              current_expression_ == EXPR_CURIOUS);
      int double_blink_chance = is_high_energy ? 30 : 15;  // 30% or 15%

      if (random(100) < double_blink_chance) {
        // Schedule second blink very soon (300-500ms)
        double_blink_pending_ = true;
        next_blink_interval_ms_ = 300 + random(200);
        Serial.println("[Expression] -> Double blink scheduled");
      } else {
        next_blink_interval_ms_ = calculateNextBlinkInterval();
      }
    }
  }

  // Update blink animation
  if (blink_state_.active) {
    updateBlinkAnimation();
    needs_redraw = true;  // Redraw during blink
  }

  // Only render if something changed (Vector-style: mostly static)
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

// ============================================================================
// Private Methods
// ============================================================================

ExpressionParams EyeExpressionEngine::calculateExpressionParams(uint8_t expression_type, uint8_t intensity) {
  ExpressionParams params;

  // Default values (will be modified per expression)
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

  // Intensity multiplier (1.0 to 2.0)
  float intensity_factor = 0.5 + (intensity * 0.3);  // Maps 1-5 to 0.8-2.0

  switch (expression_type) {
    case EXPR_NEUTRAL:
      // Calm rounded rectangles (Vector style)
      params.pupil_radius = BASE_PUPIL_RADIUS;
      params.eye_shape = SHAPE_ROUNDED_RECT;
      params.color = TFT_CYAN;  // Cyan
      params.pupil_offset_x = 0;
      params.pupil_offset_y = 0;
      params.movement_speed = 0.3;
      params.movement_amplitude = 0.05 * intensity_factor;
      break;

    case EXPR_HAPPY:
      // Upward arcs (smiling eyes) - inspired by chopsticks1
      params.pupil_radius = BASE_PUPIL_RADIUS + (8 * intensity_factor);
      params.eye_shape = SHAPE_HAPPY_ARC;
      params.color = TFT_GREEN;  // Green for happiness
      params.pupil_offset_x = 0;
      params.pupil_offset_y = -3 * intensity_factor;  // Slight upward shift
      params.movement_speed = 0.7;
      params.movement_amplitude = 0.15 * intensity_factor;
      break;

    case EXPR_CURIOUS:
      // Large circles (alert, wide-eyed) - inspired by chopsticks1 ALERT
      params.pupil_radius = BASE_PUPIL_RADIUS * (1.0 + 0.15 * intensity_factor);
      params.eye_shape = SHAPE_CIRCLE;
      params.color = TFT_CYAN;  // Cyan circles for curiosity
      params.pupil_offset_x = 5 * intensity_factor;
      params.pupil_offset_y = 0;
      params.movement_speed = 0.8;
      params.movement_amplitude = 0.2 * intensity_factor;

      // Asymmetric sizing (left larger, right smaller) for curiosity
      params.asymmetric = true;
      params.right_eye_radius = BASE_PUPIL_RADIUS * (0.85 + 0.05 * intensity_factor);  // Smaller right eye
      params.right_eye_offset_x = -3 * intensity_factor;
      params.right_eye_offset_y = 0;
      break;

    case EXPR_THINKING:
      // Rounded rectangles looking up-right
      params.pupil_radius = BASE_PUPIL_RADIUS;
      params.eye_shape = SHAPE_ROUNDED_RECT;
      params.color = TFT_CYAN;  // Cyan for neutral thinking
      params.pupil_offset_x = 12 * intensity_factor;
      params.pupil_offset_y = -10 * intensity_factor;
      params.movement_speed = 0.2;
      params.movement_amplitude = 0.08 * intensity_factor;
      break;

    case EXPR_CONFUSED:
      // Horizontal lines (squinty, uncertain) - inspired by chopsticks1 MISCHIEVOUS
      params.pupil_radius = BASE_PUPIL_RADIUS - (5 * intensity_factor);
      params.eye_shape = SHAPE_LINE;
      params.color = TFT_ORANGE;  // Orange for confusion/uncertainty
      params.pupil_offset_x = 0;
      params.pupil_offset_y = 0;
      params.movement_speed = 1.2 * intensity_factor;
      params.movement_amplitude = 0.4 * intensity_factor;

      // Asymmetric (one eye different)
      params.asymmetric = true;
      params.right_eye_offset_x = 8 * intensity_factor;
      params.right_eye_offset_y = 3 * intensity_factor;
      break;

    case EXPR_SAD:
      // Downward arcs (frowning eyes) - inspired by chopsticks1
      params.pupil_radius = BASE_PUPIL_RADIUS - (5 * intensity_factor);
      if (params.pupil_radius < MIN_PUPIL_RADIUS) params.pupil_radius = MIN_PUPIL_RADIUS;
      params.eye_shape = SHAPE_SAD_ARC;
      params.color = TFT_BLUE;  // Blue for sadness
      params.pupil_offset_x = 0;
      params.pupil_offset_y = 8 * intensity_factor;  // Downward gaze
      params.movement_speed = 0.2;
      params.movement_amplitude = 0.05 * intensity_factor;
      break;

    case EXPR_EXCITED:
      // Large circles (energetic, wide-eyed) - white for high energy
      params.pupil_radius = BASE_PUPIL_RADIUS + (15 * intensity_factor);
      if (params.pupil_radius > MAX_PUPIL_RADIUS) params.pupil_radius = MAX_PUPIL_RADIUS;
      params.eye_shape = SHAPE_CIRCLE;
      params.color = TFT_WHITE;  // White for excitement (different from cyan!)
      params.pupil_offset_x = 0;
      params.pupil_offset_y = 0;
      params.movement_speed = 2.0 * intensity_factor;
      params.movement_amplitude = 0.3 * intensity_factor;
      break;

    default:
      // Fallback to neutral
      break;
  }

  return params;
}

void EyeExpressionEngine::renderPupils() {
  // Chopsticks1-inspired rendering with different shapes per expression

  // Only clear screen if expression changed (not during blink to avoid flicker)
  static bool first_render = true;
  if (first_render || transition_progress_ < 1.0) {
    driver_->clearBothEyes();
    first_render = false;
  }

  // Calculate blink overlay amount (0.0 = open, 1.0 = closed)
  // Chopsticks1 style: Compress to thin horizontal lines during blink
  float blink_amount = 0.0;
  if (blink_state_.active) {
    uint32_t elapsed = millis() - blink_state_.start_millis;
    float progress = (float)elapsed / blink_state_.duration_ms;

    // Blink curve: 0 -> 1 -> 0 (smooth ease in/out)
    if (progress < 0.5) {
      // Closing phase (0 -> 1)
      blink_amount = progress * 2.0;
    } else {
      // Opening phase (1 -> 0)
      blink_amount = (1.0 - progress) * 2.0;
    }

    // Clamp
    if (blink_amount > 1.0) blink_amount = 1.0;
    if (blink_amount < 0.0) blink_amount = 0.0;
  }

  // Calculate eye positions (static, no micro-movements for Vector style)
  uint16_t left_radius = current_params_.pupil_radius;
  uint16_t right_radius = current_params_.asymmetric ? current_params_.right_eye_radius : left_radius;

  int16_t left_x = DISPLAY_CENTER_X + current_params_.pupil_offset_x;
  int16_t left_y = DISPLAY_CENTER_Y + current_params_.pupil_offset_y;
  int16_t right_x = DISPLAY_CENTER_X + current_params_.pupil_offset_x;
  int16_t right_y = DISPLAY_CENTER_Y + current_params_.pupil_offset_y;

  // Apply asymmetric offset for right eye (if expression is asymmetric)
  if (current_params_.asymmetric) {
    right_x += current_params_.right_eye_offset_x;
    right_y += current_params_.right_eye_offset_y;
  }

  // Render LEFT EYE
  driver_->selectEye(LEFT);
  drawEyeShape(left_x, left_y, left_radius, current_params_.eye_shape,
               current_params_.color, blink_amount);

  // Render RIGHT EYE
  driver_->selectEye(RIGHT);
  drawEyeShape(right_x, right_y, right_radius, current_params_.eye_shape,
               current_params_.color, blink_amount);
}

void EyeExpressionEngine::renderBlinkOverlay(float blink_amount) {
  // Blink overlay draws eyelids closing from top and bottom
  // For simplicity, we'll just squash the pupils (handled in renderPupils)
  // More advanced: Draw actual eyelid shapes

  // Future enhancement: Draw curved eyelid shapes
}

void EyeExpressionEngine::interpolateParams(float progress) {
  // Smooth interpolation between current and target params
  // Uses simple linear interpolation (could use easing functions for smoother feel)

  ExpressionParams& c = current_params_;
  ExpressionParams& t = target_params_;

  c.pupil_radius = c.pupil_radius + (t.pupil_radius - c.pupil_radius) * progress;
  c.pupil_offset_x = c.pupil_offset_x + (t.pupil_offset_x - c.pupil_offset_x) * progress;
  c.pupil_offset_y = c.pupil_offset_y + (t.pupil_offset_y - c.pupil_offset_y) * progress;
  c.movement_speed = c.movement_speed + (t.movement_speed - c.movement_speed) * progress;
  c.movement_amplitude = c.movement_amplitude + (t.movement_amplitude - c.movement_amplitude) * progress;

  // Shape and color transition (snap at 50% progress - no blending)
  if (progress > 0.5) {
    c.eye_shape = t.eye_shape;
    c.color = t.color;
  }

  // Asymmetric interpolation
  if (t.asymmetric) {
    c.asymmetric = true;
    c.right_eye_offset_x = c.right_eye_offset_x + (t.right_eye_offset_x - c.right_eye_offset_x) * progress;
    c.right_eye_offset_y = c.right_eye_offset_y + (t.right_eye_offset_y - c.right_eye_offset_y) * progress;
    c.right_eye_radius = c.right_eye_radius + (t.right_eye_radius - c.right_eye_radius) * progress;
  } else {
    // Reset asymmetric if target is not asymmetric
    c.asymmetric = false;
    c.right_eye_radius = c.pupil_radius;  // Match left eye
  }
}

void EyeExpressionEngine::applyAnimationOffsets() {
  // Vector-style: NO micro-movements (eyes are static until expression changes)
  // This eliminates flicker and reduces frame updates to only when needed
}

void EyeExpressionEngine::updateBlinkAnimation() {
  uint32_t elapsed = millis() - blink_state_.start_millis;

  if (elapsed >= blink_state_.duration_ms) {
    // Blink complete
    blink_state_.active = false;
    blink_state_.phase = 0;
  }
}

uint32_t EyeExpressionEngine::calculateNextBlinkInterval() {
  // Natural human blinking: Average 4-6 seconds, but can range 2-20+ seconds
  // Implements "clustering" - sometimes blinks cluster together, then long gaps

  uint32_t base_interval = 5000;  // Default 5 seconds (human average)
  uint32_t variation = 2500;      // ±2.5 seconds wide variation

  switch (current_expression_) {
    case EXPR_NEUTRAL:
      base_interval = 5000;  // Normal: 2.5-7.5 seconds (human natural rate)
      variation = 2500;
      break;

    case EXPR_HAPPY:
      base_interval = 4000;  // Happy: 2-6 seconds (slightly more frequent)
      variation = 2000;
      break;

    case EXPR_CURIOUS:
      base_interval = 3500;  // Curious: 1.5-5.5 seconds (alert, attentive)
      variation = 2000;
      break;

    case EXPR_THINKING:
      base_interval = 7000;  // Thinking: 3-11 seconds (can go LONG without blinking)
      variation = 4000;      // HUGE variation (sometimes stares for 15+ seconds)
      break;

    case EXPR_CONFUSED:
      base_interval = 3000;  // Confused: 1.5-4.5 seconds (rapid, uncertain)
      variation = 1500;
      break;

    case EXPR_SAD:
      base_interval = 8000;  // Sad: 4-12 seconds (tired, slow)
      variation = 4000;      // Very irregular
      break;

    case EXPR_EXCITED:
      base_interval = 2500;  // Excited: 1-4 seconds (very frequent)
      variation = 1500;
      break;
  }

  // Intensity affects frequency AND randomness
  float intensity_factor = 1.4 - (current_intensity_ * 0.08);
  base_interval = base_interval * intensity_factor;

  // Intensity increases chaos
  variation = variation * (0.8 + current_intensity_ * 0.12);

  // Blink clustering: 30% chance to enter "cluster mode"
  // In cluster mode: 2-4 quick blinks, then extra long gap
  blink_count_in_cluster_++;

  if (blink_count_in_cluster_ == 1 && random(100) < 30) {
    // Start a cluster (2-4 quick blinks)
    Serial.println("[Expression] -> Starting blink cluster");
  }

  if (blink_count_in_cluster_ > 1 && blink_count_in_cluster_ <= 4) {
    // In cluster: quick succession blinks
    base_interval = 1500;
    variation = 800;
    Serial.printf("[Expression] -> Cluster blink %d\n", blink_count_in_cluster_);
  } else if (blink_count_in_cluster_ > 4) {
    // After cluster: LONG gap before next blink
    base_interval = base_interval * 2.5;  // 2.5x longer gap
    variation = variation * 1.5;
    blink_count_in_cluster_ = 0;  // Reset cluster
    Serial.println("[Expression] -> Post-cluster long gap");
  }

  // Occasional very long stares (5% chance) - humans sometimes go 15-30 seconds
  if (random(100) < 5) {
    base_interval = base_interval * 3;
    Serial.println("[Expression] -> Extended stare (no blink for a while)");
  }

  // Add random variation
  int32_t random_offset = random(-variation, variation);
  uint32_t interval = base_interval + random_offset;

  // Ensure minimum interval
  if (interval < 1200) interval = 1200;

  // Cap maximum interval (don't stare TOO long)
  if (interval > 25000) interval = 25000;

  Serial.printf("[Expression] Next blink in %lu ms (base: %lu ±%lu)\n",
                interval, base_interval, variation);

  return interval;
}

// ============================================================================
// Shape Drawing Functions (inspired by chopsticks1 face controller)
// ============================================================================

void EyeExpressionEngine::drawEyeShape(int16_t x, int16_t y, uint16_t radius,
                                        EyeShape shape, uint16_t color, float blink_amount) {
  TFT_eSPI& tft = driver_->getTFT();

  // Clear area around eye first (prevent ghosting)
  int16_t clear_size = (radius * 2) + 8;
  tft.fillRect(x - (clear_size / 2), y - (clear_size / 2), clear_size, clear_size, color_background_);

  // When fully blinking (blink_amount > 0.9), compress to thin horizontal line
  // This is the chopsticks1 style - eyes compress to 1/8 height
  if (blink_amount > 0.9) {
    // Draw thin horizontal line (closed eye)
    int16_t line_width = radius * 2;
    int16_t line_height = 4;  // Very thin line
    tft.fillRoundRect(x - (line_width / 2), y - (line_height / 2),
                      line_width, line_height, 2, color);
    return;
  }

  // Calculate blink compression (compress vertically as blink progresses)
  // Chopsticks1 style: linearly compress height from 100% to 12.5% (1/8)
  float height_scale = 1.0 - (blink_amount * 0.875);  // 1.0 -> 0.125

  switch (shape) {
    case SHAPE_ROUNDED_RECT: {
      // Vector-style rounded rectangles
      int16_t rect_size_x = radius * 2;
      int16_t rect_size_y = (radius * 2) * height_scale;  // Compress vertically during blink
      uint8_t corner_radius = radius / 3;

      tft.fillRoundRect(x - radius, y - (rect_size_y / 2),
                       rect_size_x, rect_size_y, corner_radius, color);
      break;
    }

    case SHAPE_CIRCLE: {
      // Circles (curious, excited, scared)
      if (blink_amount < 0.05) {
        // Full circle when not blinking
        tft.fillCircle(x, y, radius, color);
      } else {
        // Compress to ellipse during blink
        int16_t ellipse_height = radius * height_scale;
        // Draw compressed circle using fillRect with rounded ends
        for (int16_t dy = -ellipse_height; dy <= ellipse_height; dy++) {
          int16_t dx = sqrt(radius * radius - (dy * dy / height_scale / height_scale));
          tft.drawFastHLine(x - dx, y + dy, dx * 2, color);
        }
      }
      break;
    }

    case SHAPE_HAPPY_ARC: {
      // Upward arcs (smiling eyes) - chopsticks1 inspired
      drawHappyArc(x, y, radius, color, blink_amount);
      break;
    }

    case SHAPE_SAD_ARC: {
      // Downward arcs (frowning eyes) - chopsticks1 inspired
      drawSadArc(x, y, radius, color, blink_amount);
      break;
    }

    case SHAPE_LINE: {
      // Horizontal line (confused/squinting)
      int16_t line_width = radius * 2;
      int16_t line_height = max(4, (int)(radius / 4));  // Thin line, gets thinner during blink
      line_height = line_height * height_scale;
      if (line_height < 2) line_height = 2;

      tft.fillRoundRect(x - (line_width / 2), y - (line_height / 2),
                       line_width, line_height, 2, color);
      break;
    }

    default:
      // Fallback to rounded rect
      tft.fillRoundRect(x - radius, y - radius,
                       radius * 2, radius * 2, radius / 3, color);
      break;
  }
}

void EyeExpressionEngine::drawHappyArc(int16_t x, int16_t y, uint16_t radius,
                                        uint16_t color, float blink_amount) {
  // Upward arc (smiling eye shape) - draw as thick curved crescent
  TFT_eSPI& tft = driver_->getTFT();

  // Calculate arc compression during blink
  float height_scale = 1.0 - (blink_amount * 0.875);

  // Draw upward-facing arc (like a smile)
  int16_t arc_width = radius * 2;
  int16_t arc_height = radius * 0.8 * height_scale;
  int16_t arc_thickness = radius / 4;

  // Draw filled arc using horizontal scanlines
  // Use ellipse equation for smooth curve
  for (int16_t dy = 0; dy < arc_height; dy++) {
    float t = (float)dy / arc_height;  // 0 to 1
    // Elliptical curve
    float curve = sqrt(1.0 - t * t);
    int16_t half_width = arc_width * 0.5 * curve;

    // Draw thick line at this height
    for (int16_t thick = 0; thick < arc_thickness; thick++) {
      tft.drawFastHLine(x - half_width, y + dy + thick, half_width * 2, color);
    }
  }
}

void EyeExpressionEngine::drawSadArc(int16_t x, int16_t y, uint16_t radius,
                                      uint16_t color, float blink_amount) {
  // Downward arc (frowning eye shape) - draw as thick curved crescent facing down
  TFT_eSPI& tft = driver_->getTFT();

  // Calculate arc compression during blink
  float height_scale = 1.0 - (blink_amount * 0.875);

  // Draw downward-facing arc (like a frown)
  int16_t arc_width = radius * 2;
  int16_t arc_height = radius * 0.8 * height_scale;
  int16_t arc_thickness = radius / 4;

  // Draw filled arc using horizontal scanlines (inverted from happy)
  for (int16_t dy = 0; dy < arc_height; dy++) {
    float t = (float)dy / arc_height;  // 0 to 1
    // Inverted elliptical curve
    float curve = sqrt(1.0 - (1.0 - t) * (1.0 - t));
    int16_t half_width = arc_width * 0.5 * curve;

    // Draw thick line at this height
    for (int16_t thick = 0; thick < arc_thickness; thick++) {
      tft.drawFastHLine(x - half_width, y - dy - thick, half_width * 2, color);
    }
  }
}
