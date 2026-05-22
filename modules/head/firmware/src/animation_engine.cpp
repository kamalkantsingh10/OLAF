/**
 * animation_engine.cpp - Kawaii Eye Animation Engine
 * Story 1.3 (firmware) / Story 7.1a (eye visual language re-spec)
 *
 * Renders to a 200x200 TFT_eSprite in PSRAM, then pushes the
 * complete frame to each display. Zero flicker. Each LCD shows ONE
 * eye; asymmetric emotions differ between viewer-left / viewer-right.
 *
 * Re-spec 2026-05-19 (Kamal, .ai/notes review + 35-expression ref
 * sheet): kawaii cartoon model — light eyeball (optionally cropped)
 * + black pupil + catchlight, with a strict 3-LEVEL intensity:
 *   L1  base shape, monochrome
 *   L2  + per-emotion colour tint + static described marks
 *   L3  + animated anime FX
 * Incoming intensity 3-5 clamps to L3 (map re-authored separately).
 */

#include "animation_engine.h"
#include <math.h>

// ============================================================================
// Palette (RGB565) — tuned further on hardware
// ============================================================================

constexpr uint16_t EYE_COLOR     = 0xDEFB;  // #ddd light-gray eyeball
constexpr uint16_t EYE_BLOOM     = 0x6B4D;  // dim halo behind the eye
constexpr uint16_t PUPIL_COLOR   = TFT_BLACK;
constexpr uint16_t CATCH_COLOR   = TFT_WHITE;

// L2 per-emotion tints — colour now EVIDENT (Kamal 2026-05-19),
// neutral stays the only monochrome (calm datum).
constexpr uint16_t TINT_HAPPY    = 0xFFFB;  // #fdffdd barely-noticeable yellow
constexpr uint16_t TINT_CONTENT  = 0xEF9B;  // #edf2d8 (Kamal-picked)
constexpr uint16_t TINT_EXCITED  = 0xFFF9;  // #fcffcd (Kamal-picked, L2+L3)
constexpr uint16_t TINT_SAD      = 0x6EBF;  // evident pale-blue
constexpr uint16_t TINT_MELAN    = 0x445F;  // strong cold-blue
constexpr uint16_t TINT_SYMP     = 0xFDA8;  // soft warm
constexpr uint16_t TINT_ANGRY_L2 = 0xFD94;  // subtle red (#fab0a0)
constexpr uint16_t TINT_ANGRY_L3 = 0xFB08;  // noticeable red (#ff6040)
constexpr uint16_t TINT_FRUST_L2 = 0xFE94;  // hinted orange (#ffd0a0)
constexpr uint16_t TINT_FRUST_L3 = 0xFD0C;  // defined orange, not deep (#ffa060)
constexpr uint16_t TINT_SCARED   = 0x8EFF;  // pale-blue (fear)
constexpr uint16_t TINT_CURIOUS  = 0x5FFF;  // light cyan
constexpr uint16_t TINT_SURPR    = 0xCEFF;  // pale alert
constexpr uint16_t TINT_WINK     = 0xCE99;  // muted sly
constexpr uint16_t TINT_FLIRTY   = 0xFC7B;  // pink
constexpr uint16_t TINT_SLEEPY   = 0x7BCF;  // dim

// Motif colours
constexpr uint16_t WATER_CYAN    = 0x1E9F;  // sad tear / stream
constexpr uint16_t PURPLE_LINE   = 0x8838;  // Concerned purple slashes
constexpr uint16_t SPARK_YELLOW  = 0xFEE0;  // sparkle yellow
constexpr uint16_t SPARK_DEEP_Y  = 0xFD60;  // deep yellow (happy L3 / bg)
constexpr uint16_t FLIRT_PINK    = 0xFC3A;  // motion line / heart
constexpr uint16_t FLIRT_CYAN    = 0x2FFF;  // motion line
constexpr uint16_t STEAM_GRAY    = 0xAD55;  // frustrated/angry steam
constexpr uint16_t VEIN_RED      = 0xF904;  // anger cross-vein 💢
constexpr uint16_t BLUSH_PINK    = 0xFBB6;  // happy/content cheeks
constexpr uint16_t GLOOM_BLUE    = 0x3B7A;  // depression gloom lines
constexpr uint16_t SWEAT_BLUE    = 0x6E5F;  // anxiety sweat drop
constexpr uint16_t SHADOW_DARK   = 0x2124;  // fear vertical shadow
constexpr uint16_t MARK_WHITE    = 0xFFFF;  // ? / ! / glints

constexpr float    BASE_HALF     = 86.0f;   // big screen-filling eye

constexpr uint32_t TRANSITION_MS = 200;
constexpr uint32_t BLINK_MS      = 250;
constexpr uint32_t WAKE_MS       = 250;
constexpr uint32_t SLEEP_MS      = 600;

static float easeOut(float t)   { return sinf(t * PI * 0.5f); }
static float easeInOut(float t) { return 0.5f * (1.0f - cosf(t * PI)); }

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
                  SPRITE_SIZE, SPRITE_SIZE, psramFound() ? "yes" : "no");
  } else {
    Serial.println("[Eyes] ERROR: Sprite allocation failed!");
  }

  // Boot default = asleep (EXPR_SLEEPY, heavy half-lid) at L3, NOT
  // neutral. Until the first I2C set_expression lands this is what
  // shows — including during the Pi/ESP32 boot-order race window
  // (Story 9.1). Overridden the moment an expression is set over I2C.
  current_expression_ = EXPR_SLEEPY;
  current_intensity_  = 3;
  target_expression_  = EXPR_SLEEPY;
  target_intensity_   = 3;
  current_params_     = {EXPR_SLEEPY, 3};
  target_params_      = current_params_;

  look_x_ = 0;
  look_y_ = 0;
  transition_progress_ = 1.0f;
  last_update_millis_  = millis();

  blink_state_            = {false, 0, BLINK_MS};
  last_blink_millis_      = millis();
  next_blink_interval_ms_ = calculateNextBlinkInterval();
  double_blink_pending_   = false;

  system_status_    = 0;
  wake_level_       = 0.0f;
  wake_target_      = 0.0f;
  wake_start_level_ = 0.0f;
  wake_start_ms_    = millis();
  wake_duration_ms_ = WAKE_MS;
  auto_look_x_      = 0;
  auto_look_y_      = 0;
  bloom_pulse_      = 1.0f;

  driver_->clearBothEyes();
  renderFrame();

  Serial.println("[Eyes] Engine ready — asleep");
}

// ============================================================================
// System Status  (unchanged machinery)
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

uint8_t EyeExpressionEngine::getSystemStatus()      { return system_status_; }
float   EyeExpressionEngine::getWakeLevel()         { return wake_level_; }
uint8_t EyeExpressionEngine::getCurrentExpression() { return current_expression_; }
uint8_t EyeExpressionEngine::getCurrentIntensity()  { return current_intensity_; }
bool    EyeExpressionEngine::isBlinking()           { return blink_state_.active; }

// ============================================================================
// Expression Control
// ============================================================================

void EyeExpressionEngine::setExpression(uint8_t type, uint8_t intensity) {
  if (type >= EXPR_COUNT || intensity < 1 || intensity > 5) return;

  bool changed         = (type != current_expression_);
  target_expression_   = type;
  target_intensity_    = intensity;
  target_params_       = {type, intensity};
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

  // Per-emotion blink DURATION (close→open ms), spec-row midpoint
  // ± jitter ("The 12 speech-emotion specs").
  uint16_t dur;
  switch (current_expression_) {
    case EXPR_HAPPY:       dur = 115 + random(-25, 26); break;
    case EXPR_CONTENT:     dur = 135 + random(-25, 26); break;
    case EXPR_EXCITED:     dur =  90 + random(-20, 21); break;
    case EXPR_SAD:         dur = 260 + random(-60, 61); break;
    case EXPR_MELANCHOLIC: dur = 320 + random(-60, 61); break;
    case EXPR_SYMPATHETIC: dur = 175 + random(-25, 26); break;
    case EXPR_ANGRY:       dur = 100 + random(-20, 21); break;
    case EXPR_FRUSTRATED:  dur = 110 + random(-20, 21); break;
    case EXPR_SCARED:      dur =  75 + random(-15, 16); break;
    case EXPR_CURIOUS:     dur = 100 + random(-20, 21); break;
    case EXPR_SURPRISED:   dur = 110 + random(-20, 21); break;
    case EXPR_SLEEPY:      dur = 320 + random(-50, 51); break;
    case EXPR_NEUTRAL:     dur = 125 + random(-25, 26); break;
    default:               dur = 200 + random(-30, 31); break;
  }

  blink_state_.active       = true;
  blink_state_.start_millis = millis();
  blink_state_.duration_ms  = dur;
}

// ============================================================================
// Frame Update  (unchanged machinery)
// ============================================================================

uint32_t EyeExpressionEngine::update() {
  driver_->startFrameTiming();

  uint32_t now      = millis();
  uint32_t delta_ms = now - last_update_millis_;
  last_update_millis_ = now;

  updateWakeTransition();
  updateAutoLook();

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
      interpolateParams(transition_progress_);
    }
  }

  if (wake_level_ > 0.8f && !blink_state_.active &&
      (now - last_blink_millis_ >= next_blink_interval_ms_)) {
    if (double_blink_pending_) {
      triggerBlink();
      double_blink_pending_   = false;
      next_blink_interval_ms_ = calculateNextBlinkInterval();
    } else {
      triggerBlink();
      if (random(100) < 20) {
        double_blink_pending_   = true;
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
// Wake / Sleep / Auto-look / Closedness  (unchanged machinery)
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
    if (system_status_ == 5 && wake_level_ <= 0.01f) system_status_ = 0;
  }
}

void EyeExpressionEngine::updateAutoLook() {
  if (system_status_ != 3) { auto_look_x_ = 0; auto_look_y_ = 0; return; }
  float t      = millis() / 3000.0f;
  auto_look_x_ = (int8_t)(sinf(t * 2.0f * PI) * 40);
  auto_look_y_ = (int8_t)(sinf(t * 4.0f * PI) * 15);
}

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

uint8_t EyeExpressionEngine::levelOf(uint8_t intensity) {
  // Strict 3-level model; incoming 3-5 clamp to L3.
  if (intensity <= 1) return 1;
  if (intensity == 2) return 2;
  return 3;
}

void EyeExpressionEngine::renderFrame() {
  float   closedness = computeClosedness();
  uint8_t level      = levelOf(current_intensity_);

  // is_right == false  →  selectEye(LEFT)  →  viewer's RIGHT LCD
  // (driver enum is OLAF-anatomical; viewer POV is flipped — Kamal
  // 2026-05-20). renderEye() resolves viewer_left from is_right.
  sprite_->fillSprite(TFT_BLACK);
  renderEye(SPRITE_CENTER, current_expression_, level, closedness, false);
  driver_->selectEye(LEFT);
  sprite_->pushSprite(SPRITE_OFFSET, SPRITE_OFFSET);

  sprite_->fillSprite(TFT_BLACK);
  renderEye(SPRITE_CENTER, current_expression_, level, closedness, true);
  driver_->selectEye(RIGHT);
  sprite_->pushSprite(SPRITE_OFFSET, SPRITE_OFFSET);
}

// ============================================================================
// buildEye — per-emotion, per-side geometry (SCREEN coords: +x right,
// +y down). "Left/right eye" = viewer POV (ref sheet). nasal = toward
// the centre of the head (between the two LCDs).
// ============================================================================

void EyeExpressionEngine::buildEye(uint8_t type, bool viewer_left,
                                   EyeGeom& g) {
  int nasal = viewer_left ? +1 : -1;   // screen-x toward head centre

  // Defaults: a big round kawaii eye, medium pupil, catchlight.
  g.cx_off     = 0.0f;
  g.halfW      = BASE_HALF;
  g.halfH      = BASE_HALF;
  g.crop       = CROP_NONE;
  g.cropFrac   = 0.0f;
  g.hasPupil   = true;
  g.pupR       = BASE_HALF * 0.30f;
  g.pupX       = 0.0f;
  g.pupY       = 0.0f;
  g.catchlight = true;

  switch (type) {
    case EXPR_NEUTRAL:
      // Good as-is: −5% size, pupils a bit nasal (closer to head).
      g.halfW = g.halfH = BASE_HALF * 0.95f;
      g.pupR  = g.halfW * 0.30f;
      g.pupX  = nasal * g.halfW * 0.16f;
      break;

    case EXPR_HAPPY:
      // (Kamal 2026-05-20) Bottom-slanted crop — outer corner of the
      // visible eye sits HIGHER than the inner (smile-arch lower
      // lid). Small centred pupil sits low, touching the slant at
      // centre. Tint #fdffdd at L2/L3 + L3 right-LCD-edge sparkle
      // unchanged.
      g.halfW    = BASE_HALF;
      g.halfH    = BASE_HALF * 0.92f;
      g.crop     = CROP_BOTTOM_CURVED_SL;  // curved + slanted (Kamal R3)
      g.cropFrac = 0.30f;
      g.pupR     = g.halfW * 0.15f;        // 50% smaller (locked earlier)
      g.pupX     = 0.0f;
      // Pupil low, sitting just above the curved+slanted chord at
      // centre (the curve raises the chord by ~0.20·halfH there).
      g.pupY     = g.halfH - g.cropFrac * 2.0f * g.halfH
                            - 0.20f * g.halfH - g.pupR;
      break;

    case EXPR_CONTENT:
      // Pleased (ref R1C5): oval with ~30% top cut, bigger pupil,
      // noticeable colour tint (L2).
      g.halfW    = BASE_HALF * 1.05f;
      g.halfH    = BASE_HALF * 0.95f;
      g.crop     = CROP_TOP;
      g.cropFrac = 0.30f;
      g.pupR     = g.halfW * 0.36f;        // ~10% bigger than excited
      g.pupX     = nasal * g.halfW * 0.22f; // pull closer to each other
      g.pupY     = g.halfH * 0.18f;
      break;

    case EXPR_EXCITED:
      // Excited (ref R7C4): big round eye, pupil +50%, BIG black
      // 4-pt sparkle centred on the pupil (drawn in applyEmotionFX
      // at L1+). NO catchlight — Kamal 2026-05-20: the catchlight
      // looked like a leftover white dot.
      g.halfW = g.halfH = BASE_HALF;
      g.pupR  = g.halfW * 0.48f;           // ~+50% vs default 0.30
      g.pupY  = -g.halfH * 0.05f;
      g.catchlight = false;
      break;

    case EXPR_SAD:
      // (Kamal 2026-05-20) Both eyes: same curved-from-top crop
      // (same radius as the old left top crop). Asymmetry now lives
      // in the L2/L3 marks, not the geometry. Pupils pulled toward
      // the eyeline.
      g.halfW    = BASE_HALF * 0.96f;
      g.halfH    = BASE_HALF * 0.98f;
      g.crop     = CROP_TOP_CURVED;
      g.cropFrac = 0.20f;
      g.pupR     = g.halfW * 0.30f;
      g.pupX     = nasal * g.halfW * 0.24f;
      g.pupY     = g.halfH * 0.22f;
      break;

    case EXPR_MELANCHOLIC:
      // Cold (ref R6C4): two part-circles cropped from the top,
      // slightly different sizes, eyeballs pushed to SCREEN-right.
      g.halfW = BASE_HALF * (viewer_left ? 1.00f : 0.86f);
      g.halfH = g.halfW;
      g.crop     = CROP_TOP;
      g.cropFrac = 0.26f;
      g.pupR     = g.halfW * 0.28f;
      g.pupX     = +g.halfW * 0.34f;       // screen-right for both
      g.pupY     = g.halfH * 0.10f;
      break;

    case EXPR_SYMPATHETIC:
      // Serious (ref R2C3): heavy half-lid (flat top), right eye a
      // bit smaller. Soft warm tint (L2).
      g.halfW    = BASE_HALF * (viewer_left ? 1.00f : 0.86f);
      g.halfH    = g.halfW * 1.05f;
      g.crop     = CROP_TOP;
      g.cropFrac = 0.40f;
      g.pupR     = g.halfW * 0.26f;
      g.pupY     = g.halfH * 0.10f;
      break;

    case EXPR_ANGRY:
      // (Kamal 2026-05-20) Renamed from old Irritated style: angry
      // now uses the diagonal-crop shape that was frustrated —
      // viewer-LEFT cropped from top at an angle, viewer-RIGHT from
      // the bottom at an angle. Pupil kept "as it is now". No extra
      // shapes/marks/animation; only colour shifts by level
      // (subtle-red at L2, noticeable-red at L3).
      g.halfW = BASE_HALF * 0.98f;
      g.halfH = BASE_HALF * 0.96f;
      g.crop     = viewer_left ? CROP_DIAG_TOP : CROP_DIAG_BOT;
      g.cropFrac = 0.34f;
      g.pupR     = g.halfW * 0.28f;
      break;

    case EXPR_FRUSTRATED:
      // (Kamal 2026-05-20) Visual identity = ref-sheet "Silly"
      // (R2C4): round eye with a flat top crop, small pupils
      // glancing UP-OUTWARD. SHAPE + pupil placement only — no tint
      // and no marks at any level (those will be defined later).
      g.halfW    = BASE_HALF;
      g.halfH    = BASE_HALF;
      g.crop     = CROP_TOP;
      g.cropFrac = 0.28f;
      g.pupR     = g.halfW * 0.18f;
      g.pupX     = -nasal * g.halfW * 0.30f;
      g.pupY     = -g.halfH * 0.25f;
      break;

    case EXPR_SCARED:
      // Scared (Kamal 2026-05-20): plain round eyes (the left-edge
      // crop is gone). Big tense eye, small pupil, slightly raised.
      // Pale-blue tint at L2.
      g.halfW = g.halfH = BASE_HALF * 1.04f;
      g.pupR  = g.halfW * 0.18f;           // wide eye, small pupil
      g.pupY  = -g.halfH * 0.06f;
      break;

    case EXPR_CURIOUS:
      // Curious (ref R5C1): asymmetric — one eye bigger, pupils up.
      g.halfW = BASE_HALF * (viewer_left ? 1.05f : 0.80f);
      g.halfH = g.halfW;
      g.pupR  = g.halfW * 0.32f;
      g.pupX  = nasal * g.halfW * 0.10f;
      g.pupY  = -g.halfH * 0.26f;          // inquisitive "huh?"
      break;

    case EXPR_SURPRISED:
      // Good as-is: big round wide, raised, small pupil. + Flirty
      // colour motion lines on the left eye (L2).
      g.halfW = g.halfH = BASE_HALF * 1.06f;
      g.pupR  = g.halfW * 0.20f;
      g.pupY  = -g.halfH * 0.04f;
      break;

    case EXPR_SLEEPY:
      // Tired (ref R3C2): heavy droopy lid — large top crop, low.
      g.halfW    = BASE_HALF * 1.05f;
      g.halfH    = BASE_HALF * 0.95f;
      g.crop     = CROP_TOP;
      g.cropFrac = 0.55f;
      g.pupR     = g.halfW * 0.24f;
      g.pupY     = g.halfH * 0.28f;
      g.catchlight = false;
      break;

    case EXPR_WINK:
      // Devious (ref R7C2) (Kamal 2026-05-20): viewer-right eye is
      // SMALLER, both eyes use a curved-top crop with a slight
      // SLANT so the OUTER edge sits higher than the inner. Pupils
      // glance outward as before. Sly half-lidded look.
      g.halfW    = BASE_HALF * (viewer_left ? 1.02f : 0.85f);
      g.halfH    = g.halfW * 0.94f;
      g.crop     = CROP_TOP_CURVED_SLANT;
      g.cropFrac = 0.45f;
      g.pupR     = g.halfW * 0.24f;
      g.pupX     = -nasal * g.halfW * 0.34f;  // glance outward
      g.pupY     = g.halfH * 0.10f;
      break;

    case EXPR_FLIRTY:
      // Flirty (ref R2C5) "with small": a small coy eye — raised
      // lower lid (bottom crop), slight outward-up glance, + the
      // colour motion lines (L2). Device-only extra.
      g.halfW    = BASE_HALF * 0.64f;
      g.halfH    = BASE_HALF * 0.62f;
      g.crop     = CROP_BOTTOM;
      g.cropFrac = 0.30f;
      g.pupR     = g.halfW * 0.32f;
      g.pupX     = -nasal * g.halfW * 0.18f;  // glance outward
      g.pupY     = -g.halfH * 0.10f;
      break;

    default:
      break;
  }
}

// ============================================================================
// drawEyeball — one filled (optionally cropped) kawaii eyeball.
// Column rasteriser: per x-column, ellipse half-height `ey`; the crop
// trims the filled [yTop,yBot] band with a chord/curve/slope.
// ============================================================================

void EyeExpressionEngine::drawEyeball(int16_t cx, int16_t cy,
                                      const EyeGeom& g, bool viewer_left,
                                      uint16_t color) {
  int16_t halfW = (int16_t)g.halfW;
  int16_t halfH = (int16_t)g.halfH;
  if (halfW < 2 || halfH < 2) return;

  int   nasal = viewer_left ? +1 : -1;
  float frac  = g.cropFrac;

  for (int16_t dx = -halfW; dx <= halfW; dx++) {
    float u  = (float)dx / (float)halfW;            // -1..1
    float k  = 1.0f - u * u;
    if (k < 0.0f) k = 0.0f;
    float ey = halfH * sqrtf(k);

    int16_t yTop = cy - (int16_t)ey;
    int16_t yBot = cy + (int16_t)ey;

    switch (g.crop) {
      case CROP_TOP: {
        int16_t cut = cy - halfH + (int16_t)(frac * 2.0f * halfH);
        if (yTop < cut) yTop = cut;
        break;
      }
      case CROP_BOTTOM: {
        int16_t cut = cy + halfH - (int16_t)(frac * 2.0f * halfH);
        if (yBot > cut) yBot = cut;
        break;
      }
      case CROP_TOPBOT: {
        int16_t ct = cy - halfH + (int16_t)(frac * 2.0f * halfH);
        int16_t cb = cy + halfH - (int16_t)(frac * 2.0f * halfH);
        if (yTop < ct) yTop = ct;
        if (yBot > cb) yBot = cb;
        break;
      }
      case CROP_TOP_CURVED: {
        // Cut bulges DOWN at the centre (a curved upper lid).
        float   bulge = frac * 2.0f * halfH + halfH * 0.30f * k;
        int16_t cut   = cy - halfH + (int16_t)bulge;
        if (yTop < cut) yTop = cut;
        break;
      }
      case CROP_TOPBOT_CURV: {
        float   bulge = frac * 2.0f * halfH + halfH * 0.30f * k;
        int16_t ct    = cy - halfH + (int16_t)bulge;
        int16_t cb    = cy + halfH - (int16_t)(frac * 2.0f * halfH);
        if (yTop < ct) yTop = ct;
        if (yBot > cb) yBot = cb;
        break;
      }
      case CROP_LEFT: {
        // Vertical chop near the OUTER (temporal) edge.
        float edge = -1.0f + 2.0f * frac;            // u threshold
        int   temporal = -nasal;                     // outer side
        if (temporal < 0) { if (u < edge) continue; }
        else              { if (u > -edge) continue; }
        break;
      }
      case CROP_DIAG_TOP: {
        // Sloped cut descending from the top across the eye.
        int16_t cut = cy - halfH + (int16_t)((0.10f + 0.55f * (u + 1.0f) * 0.5f) * 2.0f * halfH);
        if (yTop < cut) yTop = cut;
        break;
      }
      case CROP_DIAG_BOT: {
        int16_t cut = cy + halfH - (int16_t)((0.10f + 0.55f * (1.0f - (u + 1.0f) * 0.5f)) * 2.0f * halfH);
        if (yBot > cut) yBot = cut;
        break;
      }
      case CROP_BASE_CURVED: {
        // Top = full ellipse top (ey). Base curves UP at centre by
        // `frac * halfH * k` — produces the "smile-arch" happy eye.
        int16_t cut = cy - (int16_t)(frac * halfH * k);
        if (yBot > cut) yBot = cut;
        break;
      }
      case CROP_TOP_CURVED_SLANT: {
        // Curved top (downward-bulge at centre) PLUS a linear slant
        // so the OUTER edge sits higher than the inner edge. Outer
        // side in screen-x is `-nasal`, so adding `slant·u·nasal`
        // raises (-) the chord on the outer side and lowers (+) it
        // on the inner side. Used by wink (Devious sly half-lid).
        float bulge = frac * 2.0f * halfH + halfH * 0.30f * k;
        float slant = halfH * 0.18f * u * (float)nasal;
        int16_t cut = cy - halfH + (int16_t)(bulge + slant);
        if (yTop < cut) yTop = cut;
        break;
      }
      case CROP_BOTTOM_SLANT: {
        // Straight bottom crop, slanted so the OUTER corner of the
        // visible eye sits HIGHER than the inner corner (smile-arch
        // lower lid). At the outer side u·nasal < 0 → smaller cut y
        // → MORE cropped; at the inner side → LESS cropped. Happy.
        float base  = frac * 2.0f * halfH;
        float slant = halfH * 0.22f * u * (float)nasal;
        int16_t cut = cy + halfH - (int16_t)base + (int16_t)slant;
        if (yBot > cut) yBot = cut;
        break;
      }
      case CROP_BOTTOM_CURVED_SL: {
        // CURVED + slanted bottom crop: the slant raises the outer
        // corner above the inner, AND the chord BULGES UP at the
        // centre (k = 1−u²) for a smile-arch lower lid. Happy v2.
        float base  = frac * 2.0f * halfH;
        float slant = halfH * 0.22f * u * (float)nasal;
        float curve = halfH * 0.20f * k;          // centre rise
        int16_t cut = cy + halfH - (int16_t)base + (int16_t)slant
                                    - (int16_t)curve;
        if (yBot > cut) yBot = cut;
        break;
      }
      default: break;
    }

    if (yBot <= yTop) continue;
    sprite_->drawFastVLine(cx + dx, yTop, yBot - yTop + 1, color);
  }
}

void EyeExpressionEngine::drawPupil(int16_t cx, int16_t cy,
                                    const EyeGeom& g, bool viewer_left) {
  if (!g.hasPupil) return;
  int16_t pr = (int16_t)g.pupR;
  if (pr < 3) return;
  int16_t px = cx + (int16_t)g.pupX;
  int16_t py = cy + (int16_t)g.pupY;
  sprite_->fillCircle(px, py, pr, PUPIL_COLOR);
  if (g.catchlight) {
    int   nasal = viewer_left ? +1 : -1;
    int16_t hr  = pr / 3;
    if (hr >= 2)
      sprite_->fillCircle(px - nasal * pr / 3, py - pr / 3, hr, CATCH_COLOR);
  }
}

// ============================================================================
// Eye Rendering — compose eyeball + pupil + tint + motifs
// ============================================================================

void EyeExpressionEngine::renderEye(int16_t cx, uint8_t type, uint8_t level,
                                    float closedness, bool is_right) {
  // Kamal 2026-05-20: the driver's LEFT/RIGHT enum maps to the
  // OPPOSITE LCD from viewer POV — selectEye(LEFT) drives the
  // viewer's RIGHT LCD. So `is_right==false` (the LEFT-selected eye)
  // is actually the VIEWER-RIGHT eye. Inverted assignment fixes
  // diverging pupils + flips asymmetric cells to the correct side.
  bool viewer_left = is_right;

  EyeGeom g;
  buildEye(type, viewer_left, g);

  // Blink / wake collapses the eye to a thin closed line.
  float openf = 1.0f - (closedness * 0.94f);
  g.halfH *= openf;
  int16_t ecx = cx + (int16_t)g.cx_off;
  int16_t ecy = SPRITE_CENTER;

  if (closedness > 0.85f || g.halfH < 3.0f) {
    drawClosedLine(ecx, ecy, (int16_t)g.halfW);
    return;
  }

  // L3 scared: the whole eye trembles (manpu fear shake).
  if (type == EXPR_SCARED && level >= 3) {
    ecx += random(-3, 4);
    ecy += random(-3, 4);
  }

  // L2+ : per-emotion eyeball colour tint. Neutral is the ONLY
  // monochrome expression (calm datum).
  uint16_t eyeCol = EYE_COLOR;
  if (level >= 2) {
    switch (type) {
      case EXPR_HAPPY:       eyeCol = TINT_HAPPY;   break;
      case EXPR_CONTENT:     eyeCol = TINT_CONTENT; break;
      case EXPR_EXCITED:     eyeCol = TINT_EXCITED; break;
      case EXPR_SAD:         eyeCol = TINT_SAD;     break;
      case EXPR_MELANCHOLIC: eyeCol = TINT_MELAN;   break;
      case EXPR_SYMPATHETIC: eyeCol = TINT_SYMP;    break;
      case EXPR_ANGRY:
        // Subtle red at L2, noticeable red at L3 (Kamal 2026-05-20).
        eyeCol = (level >= 3) ? TINT_ANGRY_L3 : TINT_ANGRY_L2;
        break;
      case EXPR_FRUSTRATED:
        // Hinted orange at L2, more defined (not deep) at L3.
        eyeCol = (level >= 3) ? TINT_FRUST_L3 : TINT_FRUST_L2;
        break;
      case EXPR_SCARED:      eyeCol = TINT_SCARED;  break;
      case EXPR_CURIOUS:     eyeCol = TINT_CURIOUS; break;
      case EXPR_SURPRISED:   eyeCol = TINT_SURPR;   break;
      case EXPR_WINK:        eyeCol = TINT_WINK;    break;
      case EXPR_FLIRTY:      eyeCol = TINT_FLIRTY;  break;
      case EXPR_SLEEPY:      eyeCol = TINT_SLEEPY;  break;
      default: break;  // neutral → EYE_COLOR
    }
  }

  // L3 soft-glow pulse (content / sympathetic): breathe the bloom.
  bloom_pulse_ = 1.0f;
  if (level >= 3 && (type == EXPR_CONTENT || type == EXPR_SYMPATHETIC))
    bloom_pulse_ = 3.0f + 4.0f * (0.5f + 0.5f * sinf(millis() / 600.0f));

  // Soft bloom halo, then the eyeball, then the pupil.
  EyeGeom bg = g;
  bg.halfW += 3.0f * bloom_pulse_;
  bg.halfH += 3.0f * bloom_pulse_;
  drawEyeball(ecx, ecy, bg, viewer_left, EYE_BLOOM);
  drawEyeball(ecx, ecy, g,  viewer_left, eyeCol);
  drawPupil(ecx, ecy, g, viewer_left);

  // ---- L2 static marks + L3 anime FX ----
  applyEmotionFX(type, level, ecx, ecy, g, viewer_left);
}

// ============================================================================
// applyEmotionFX — per-emotion L2 (static colour mark) + L3 (anime FX)
// ============================================================================

void EyeExpressionEngine::applyEmotionFX(uint8_t type, uint8_t level,
                                         int16_t ecx, int16_t ecy,
                                         const EyeGeom& g,
                                         bool viewer_left) {
  bool fx = (level >= 3);
  int16_t hW = (int16_t)g.halfW, hH = (int16_t)g.halfH;
  int   nasal = viewer_left ? +1 : -1;

  // ── Baselines that exist at L1 too ──
  if (type == EXPR_EXCITED) {
    // A WHITE 4-pt sparkle centred on the pupil, sized to FIT
    // INSIDE the pupil (Kamal 2026-05-20). Static at every level —
    // L3 background sparkles carry the animation, the central
    // sparkle does not pulse.
    int16_t pcx = ecx + (int16_t)g.pupX;
    int16_t pcy = ecy + (int16_t)g.pupY;
    int16_t sR  = (int16_t)(g.pupR * 0.60f);     // fits inside pupil
    drawSparkle(pcx, pcy, sR, MARK_WHITE, false);
  }

  if (level < 2) return;                         // L1 is otherwise pure shape

  switch (type) {
    case EXPR_HAPPY:
      // L2: only the yellow tint (no sparkles / no blush).
      // L3: a single LARGE deep-yellow sparkle on the VIEWER-RIGHT
      //     LCD, hugged to the right edge of the sprite.
      if (fx && !viewer_left) {
        int16_t sR = (int16_t)(hW * 0.50f);          // bigger (Kamal R2)
        int16_t sx = SPRITE_SIZE - sR - 6;           // hugged to right edge
        int16_t sy = SPRITE_CENTER;
        drawSparkle(sx, sy, sR, SPARK_DEEP_Y, true);
      }
      break;

    case EXPR_CONTENT:
      // L2: tint only (no static dot/blush).
      // L3: the soft outer glow-pulse — handled by bloom_pulse_ in
      // renderEye(). No extra marks.
      break;

    case EXPR_EXCITED:
      // L2: tint only (the BLACK pupil sparkle is already drawn as
      // the L1+ baseline above).
      // L3: deep-yellow animated background sparkles, toned down
      // (count + frequency reduced per Kamal "animation is too much").
      if (fx) drawBackgroundSparkles(SPARK_DEEP_Y, 6);
      break;

    case EXPR_SAD:
      // L2 ONLY: 3 purple slashes by the viewer-RIGHT eye. No tear.
      if (level == 2 && !viewer_left)
        drawPurpleLines(ecx, ecy, hW);
      // L3: animated tear stream off the viewer-LEFT eye + gloom
      // lines. No purple at L3.
      if (fx) {
        if (viewer_left) drawWaterDrop(ecx, ecy + hH, hW, hH, true);
        drawGloomLines(ecx, ecy, hW, hH, true);
      }
      break;

    case EXPR_MELANCHOLIC:
      drawGloomLines(ecx, ecy, hW, hH, fx);
      break;

    case EXPR_SYMPATHETIC: {
      // Gentle eye-shine (static), L3 glow via bloom_pulse_.
      int16_t sx = ecx - nasal * (int16_t)(hW * 0.30f);
      int16_t sy = ecy - (int16_t)(hH * 0.25f);
      sprite_->fillCircle(sx, sy, hW / 9, MARK_WHITE);
      break;
    }

    case EXPR_ANGRY:
      // (Kamal 2026-05-20) No additional shapes or animation —
      // colour-only signal (subtle-red L2 → noticeable-red L3).
      break;

    case EXPR_FRUSTRATED:
      // PLACEHOLDER — TBD. No marks until owner redefines.
      break;

    case EXPR_SCARED:
      drawShadowLines(ecx, ecy, hW, hH);
      drawSweatDrop(ecx, ecy, hW, viewer_left, fx);
      break;

    case EXPR_CURIOUS: {
      int16_t qx = ecx + nasal * (int16_t)(hW * 0.55f);
      int16_t qy = ecy - (int16_t)(hH * 0.95f);
      if (fx) qy += (int16_t)(sinf(millis() / 250.0f) * 6.0f);
      drawGlyph("?", qx, qy, 6, TINT_CURIOUS);
      break;
    }

    case EXPR_SURPRISED: {
      int16_t ex = ecx + nasal * (int16_t)(hW * 0.55f);
      int16_t ey = ecy - (int16_t)(hH * 0.95f);
      drawGlyph("!", ex, ey, 6, MARK_WHITE);
      if (viewer_left)
        drawFlirtyLines(ecx, ecy, hW, hH, viewer_left);
      if (fx) drawImpactLines(ecx, ecy, (int16_t)(hW * 1.25f));
      break;
    }

    case EXPR_SLEEPY: {
      int16_t zx = ecx + nasal * (int16_t)(hW * 0.55f);
      int16_t zy = ecy - (int16_t)(hH * 0.75f);
      if (fx) {
        float ph = (millis() % 2600) / 2600.0f;
        for (int i = 0; i < 3; i++) {
          float f = ph - i * 0.3f; if (f < 0) f += 1.0f;
          drawGlyph("Z", zx + i * 14 + (int16_t)(f * 10),
                    zy - (int16_t)(f * 40), 4 - (i ? 2 : 0),
                    EYE_COLOR);
        }
      } else {
        drawGlyph("Z", zx,      zy,      4, EYE_COLOR);
        drawGlyph("Z", zx + 16, zy - 16, 2, EYE_COLOR);
      }
      break;
    }

    case EXPR_WINK: {
      int16_t gx = ecx - nasal * (int16_t)(hW * 0.30f);
      int16_t gy = ecy - (int16_t)(hH * 0.25f);
      drawSparkle(gx, gy, hW / 7, MARK_WHITE, fx);
      break;
    }

    case EXPR_FLIRTY:
      drawFlirtyLines(ecx, ecy, hW, hH, viewer_left);
      drawHearts(ecx, ecy, hW, hH, fx);
      break;

    default: break;
  }
}

// ============================================================================
// Closed Line — blink / wink / asleep collapse to a thin line
// ============================================================================

void EyeExpressionEngine::drawClosedLine(int16_t x, int16_t y,
                                         int16_t halfW) {
  int16_t w = (int16_t)(halfW * 1.6f);
  if (w < 10) w = 10;
  sprite_->fillRoundRect(x - w / 2, y - 3, w, 6, 3, EYE_COLOR);
}

// ============================================================================
// Motif library  (L2 static / L3 animated — finite reusable tokens)
// ============================================================================

void EyeExpressionEngine::strokeLine(int16_t x0, int16_t y0,
                                     int16_t x1, int16_t y1,
                                     int16_t thick, uint16_t color) {
  // Thick line as a row of stamped circles (sprite-safe; avoids the
  // version-dependent TFT_eSprite::drawWideLine).
  int16_t r  = thick / 2; if (r < 1) r = 1;
  int16_t dx = x1 - x0, dy = y1 - y0;
  int   steps = (int)(sqrtf((float)(dx * dx + dy * dy)) / (r > 1 ? r : 1)) + 1;
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    sprite_->fillCircle(x0 + (int16_t)(dx * t),
                        y0 + (int16_t)(dy * t), r, color);
  }
}

void EyeExpressionEngine::drawPurpleLines(int16_t cx, int16_t cy,
                                          int16_t halfW) {
  // Three short diagonal slashes off the upper-outer corner.
  int16_t x0 = cx + (int16_t)(halfW * 0.55f);
  int16_t y0 = cy - (int16_t)(halfW * 0.85f);
  for (int i = 0; i < 3; i++) {
    int16_t ox = x0 + i * 14;
    strokeLine(ox, y0, ox + 18, y0 + 34, 4, PURPLE_LINE);
  }
}

void EyeExpressionEngine::drawFlirtyLines(int16_t cx, int16_t cy,
                                          int16_t halfW, int16_t halfH,
                                          bool viewer_left) {
  // Curved colour motion lines off the lower-outer edge of the eye.
  int   temporal = viewer_left ? -1 : +1;
  int16_t bx = cx + temporal * (int16_t)(halfW * 0.70f);
  int16_t by = cy + (int16_t)(halfH * 0.55f);
  uint16_t cols[2] = {FLIRT_PINK, FLIRT_CYAN};
  for (int i = 0; i < 2; i++) {
    int16_t off = i * 16;
    strokeLine(bx + temporal * off, by,
               bx + temporal * (off + 30), by + 26, 4, cols[i]);
  }
}

void EyeExpressionEngine::drawWaterDrop(int16_t cx, int16_t cy,
                                        int16_t halfW, int16_t halfH,
                                        bool animated) {
  int16_t dx = cx - (int16_t)(halfW * 0.10f);
  if (!animated) {
    // A single hanging cyan drop just below the eye.
    sprite_->fillCircle(dx, cy + 14, 9, WATER_CYAN);
    sprite_->fillTriangle(dx - 7, cy + 14, dx + 7, cy + 14,
                          dx, cy + 2, WATER_CYAN);
    return;
  }
  // L3: a flowing stream — a ribbon of falling drops, phase-animated.
  float ph = (millis() % 1100) / 1100.0f;
  for (int i = 0; i < 5; i++) {
    float f  = ph + i * 0.2f;
    if (f > 1.0f) f -= 1.0f;
    int16_t y = cy + (int16_t)(f * (halfH * 1.4f));
    int16_t r = 9 - (int16_t)(f * 4.0f);
    if (r < 3) r = 3;
    sprite_->fillCircle(dx + (int16_t)(sinf(f * 6.0f) * 4.0f), y, r,
                        WATER_CYAN);
  }
}

void EyeExpressionEngine::drawSparkle(int16_t cx, int16_t cy, int16_t r,
                                      uint16_t color, bool animated) {
  if (r < 4) r = 4;
  float pulse = 1.0f;
  if (animated) pulse = 0.7f + 0.3f * sinf(millis() / 110.0f);
  int16_t a = (int16_t)(r * pulse);
  int16_t b = (int16_t)(a * 0.34f);
  // 4-point star (two crossed diamonds).
  sprite_->fillTriangle(cx, cy - a, cx - b, cy, cx + b, cy, color);
  sprite_->fillTriangle(cx, cy + a, cx - b, cy, cx + b, cy, color);
  sprite_->fillTriangle(cx - a, cy, cx, cy - b, cx, cy + b, color);
  sprite_->fillTriangle(cx + a, cy, cx, cy - b, cx, cy + b, color);
}

void EyeExpressionEngine::drawSteam(int16_t cx, int16_t cy,
                                    int16_t halfW, int16_t halfH) {
  // Two rising wavy steam puffs above the eye (frustration vent).
  float ph = (millis() % 1400) / 1400.0f;
  for (int s = -1; s <= 1; s += 2) {
    int16_t bx = cx + s * (int16_t)(halfW * 0.45f);
    for (int i = 0; i < 4; i++) {
      float f = ph + i * 0.25f;
      if (f > 1.0f) f -= 1.0f;
      int16_t y = cy - (int16_t)(f * halfH * 0.9f);
      int16_t x = bx + (int16_t)(sinf(f * 8.0f + s) * 8.0f);
      int16_t r = 7 - (int16_t)(f * 4.0f);
      if (r < 2) r = 2;
      sprite_->fillCircle(x, y, r, STEAM_GRAY);
    }
  }
}

void EyeExpressionEngine::drawBlush(int16_t cx, int16_t cy,
                                    int16_t halfW, int16_t halfH,
                                    bool viewer_left) {
  // A soft pink cheek oval low on the outer side (manpu happiness).
  int   temporal = viewer_left ? -1 : +1;
  int16_t bx = cx + temporal * (int16_t)(halfW * 0.55f);
  int16_t by = cy + (int16_t)(halfH * 0.78f);
  for (int16_t dx = -halfW / 4; dx <= halfW / 4; dx++) {
    float u  = (float)dx / (halfW / 4);
    int16_t h = (int16_t)((halfH / 8) * sqrtf(fmaxf(0.0f, 1.0f - u * u)));
    sprite_->drawFastVLine(bx + dx, by - h, 2 * h + 1, BLUSH_PINK);
  }
}

void EyeExpressionEngine::drawTwinkles(int16_t cx, int16_t cy,
                                       int16_t halfW, int16_t halfH,
                                       bool animated) {
  // A few small sparkle stars orbiting the eye (manpu joy).
  static const float ax[4] = {-0.85f, 0.80f, 0.55f, -0.60f};
  static const float ay[4] = {-0.75f, -0.55f, 0.70f, 0.65f};
  float t = animated ? (millis() / 220.0f) : 0.0f;
  for (int i = 0; i < 4; i++) {
    float s = animated ? (0.6f + 0.4f * sinf(t + i)) : 1.0f;
    int16_t r = (int16_t)(halfW * 0.10f * s);
    drawSparkle(cx + (int16_t)(ax[i] * halfW),
                cy + (int16_t)(ay[i] * halfH), r, SPARK_YELLOW, false);
  }
}

void EyeExpressionEngine::drawStarBurst(int16_t cx, int16_t cy,
                                        int16_t r) {
  // Radiating short rays (excited star-burst), phase-rotated.
  float base = millis() / 400.0f;
  for (int i = 0; i < 8; i++) {
    float a  = base + i * (PI / 4.0f);
    int16_t x0 = cx + (int16_t)(cosf(a) * r * 0.78f);
    int16_t y0 = cy + (int16_t)(sinf(a) * r * 0.78f);
    int16_t x1 = cx + (int16_t)(cosf(a) * r * 1.08f);
    int16_t y1 = cy + (int16_t)(sinf(a) * r * 1.08f);
    strokeLine(x0, y0, x1, y1, 4, SPARK_YELLOW);
  }
}

void EyeExpressionEngine::drawGloomLines(int16_t cx, int16_t cy,
                                         int16_t halfW, int16_t halfH,
                                         bool animated) {
  // Vertical blue depression bars hanging over the top of the eye.
  int n = 5;
  for (int i = 0; i < n; i++) {
    int16_t x = cx - halfW * 0.6f + i * (int16_t)(halfW * 1.2f / (n - 1));
    int16_t len = (int16_t)(halfH * 0.55f);
    int16_t y0  = cy - (int16_t)(halfH * 0.95f);
    if (animated) len += (int16_t)(sinf(millis() / 500.0f + i) * 5.0f);
    sprite_->drawFastVLine(x,     y0, len, GLOOM_BLUE);
    sprite_->drawFastVLine(x + 1, y0, len, GLOOM_BLUE);
  }
}

void EyeExpressionEngine::drawCrossVein(int16_t cx, int16_t cy,
                                        int16_t s, bool animated) {
  // The 💢 anger mark: four rounded "bulge" arcs meeting at a cross.
  if (s < 8) s = 8;
  if (animated) s += (int16_t)(sinf(millis() / 90.0f) * (s * 0.18f));
  int16_t t = s / 4; if (t < 3) t = 3;
  // Horizontal + vertical bars (the cross).
  sprite_->fillRoundRect(cx - s, cy - t, 2 * s, 2 * t, t, VEIN_RED);
  sprite_->fillRoundRect(cx - t, cy - s, 2 * t, 2 * s, t, VEIN_RED);
  // Corner knuckle bulges.
  sprite_->fillCircle(cx - s, cy - s, t, VEIN_RED);
  sprite_->fillCircle(cx + s, cy - s, t, VEIN_RED);
  sprite_->fillCircle(cx - s, cy + s, t, VEIN_RED);
  sprite_->fillCircle(cx + s, cy + s, t, VEIN_RED);
}

void EyeExpressionEngine::drawSweatDrop(int16_t cx, int16_t cy,
                                        int16_t halfW, bool viewer_left,
                                        bool animated) {
  // A single anxiety sweat bead by the inner-upper corner; L3 slides.
  int   nasal = viewer_left ? +1 : -1;
  int16_t x = cx + nasal * (int16_t)(halfW * 0.45f);
  int16_t y = cy - (int16_t)(halfW * 0.55f);
  if (animated) y += (int16_t)(((millis() % 900) / 900.0f) * halfW * 0.7f);
  sprite_->fillCircle(x, y + 8, 9, SWEAT_BLUE);
  sprite_->fillTriangle(x - 6, y + 8, x + 6, y + 8, x, y - 4, SWEAT_BLUE);
}

void EyeExpressionEngine::drawShadowLines(int16_t cx, int16_t cy,
                                          int16_t halfW, int16_t halfH) {
  // Dark vertical fear shading down the upper face.
  for (int i = -3; i <= 3; i++) {
    int16_t x = cx + i * (int16_t)(halfW * 0.22f);
    sprite_->drawFastVLine(x, cy - (int16_t)(halfH * 1.05f),
                           (int16_t)(halfH * 0.5f), SHADOW_DARK);
  }
}

void EyeExpressionEngine::drawGlyph(const char* s, int16_t x, int16_t y,
                                    uint8_t font, uint16_t color) {
  sprite_->setTextColor(color);
  sprite_->setTextDatum(MC_DATUM);
  sprite_->drawString(s, x, y, font);
  sprite_->setTextDatum(TL_DATUM);
}

void EyeExpressionEngine::drawHeart(int16_t cx, int16_t cy, int16_t s,
                                    uint16_t color) {
  if (s < 3) s = 3;
  int16_t r = s / 2;
  sprite_->fillCircle(cx - r, cy - r / 2, r, color);
  sprite_->fillCircle(cx + r, cy - r / 2, r, color);
  sprite_->fillTriangle(cx - s, cy - r / 4, cx + s, cy - r / 4,
                        cx, cy + s, color);
}

void EyeExpressionEngine::drawHearts(int16_t cx, int16_t cy,
                                     int16_t halfW, int16_t halfH,
                                     bool animated) {
  // L2: a couple of small static hearts by the eye. L3: a drift of
  // floating pink hearts in the background (Kamal-requested).
  if (!animated) {
    drawHeart(cx + (int16_t)(halfW * 0.7f), cy - (int16_t)(halfH * 0.6f),
              (int16_t)(halfW * 0.16f), FLIRT_PINK);
    drawHeart(cx - (int16_t)(halfW * 0.8f), cy + (int16_t)(halfH * 0.2f),
              (int16_t)(halfW * 0.12f), FLIRT_PINK);
    return;
  }
  float ph = (millis() % 2400) / 2400.0f;
  for (int i = 0; i < 6; i++) {
    float f  = ph + i * (1.0f / 6.0f);
    if (f > 1.0f) f -= 1.0f;
    int16_t bx = SPRITE_CENTER + (int16_t)(((i * 53) % 160) - 80);
    int16_t by = SPRITE_CENTER + 90 - (int16_t)(f * 190.0f);
    int16_t sz = 6 + (int16_t)((1.0f - f) * 10.0f);
    bx += (int16_t)(sinf(f * 6.0f + i) * 12.0f);
    drawHeart(bx, by, sz, FLIRT_PINK);
  }
}

void EyeExpressionEngine::drawBackgroundSparkles(uint16_t color,
                                                 int count) {
  // A deterministic-ish scatter of small sparkles across the sprite,
  // time-animated (twinkle phase per index). Excited L3.
  uint32_t t = millis();
  for (int i = 0; i < count; i++) {
    // Pseudo-random positions (cheap LCG-ish hash on i).
    int16_t bx = 10 + (int16_t)(((i * 73 + 17) * 31) % 180);
    int16_t by = 10 + (int16_t)(((i * 113 + 41) * 19) % 180);
    float ph = sinf((t / 380.0f) + i * 0.9f);     // slow twinkle
    int16_t r = 8 + (int16_t)((0.5f + 0.5f * ph) * 6.0f); // bigger (Kamal R2)
    drawSparkle(bx, by, r, color, false);
  }
}

void EyeExpressionEngine::drawImpactLines(int16_t cx, int16_t cy,
                                          int16_t r) {
  // Radiating shock burst (manpu surprise) — short outward strokes.
  for (int i = 0; i < 10; i++) {
    float a = i * (2.0f * PI / 10.0f);
    int16_t x0 = cx + (int16_t)(cosf(a) * r * 0.85f);
    int16_t y0 = cy + (int16_t)(sinf(a) * r * 0.85f);
    int16_t x1 = cx + (int16_t)(cosf(a) * r * 1.15f);
    int16_t y1 = cy + (int16_t)(sinf(a) * r * 1.15f);
    strokeLine(x0, y0, x1, y1, 3, MARK_WHITE);
  }
}

// ============================================================================
// Expression Parameters / Interpolation
// ============================================================================

ExpressionParams EyeExpressionEngine::calculateExpressionParams(
    uint8_t type, uint8_t intensity) {
  return {type, intensity};
}

void EyeExpressionEngine::interpolateParams(float progress) {
  // A blink (triggered on change when awake) masks the swap; snap the
  // discrete emotion at the midpoint. No geometric tween needed —
  // each emotion is a bespoke composition.
  if (progress > 0.5f) {
    current_expression_ = target_expression_;
    current_intensity_  = target_intensity_;
    current_params_     = target_params_;
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
  uint32_t base = 4500, vary = 1500;
  switch (current_expression_) {
    case EXPR_HAPPY:       base = 3500; vary = 1500; break;
    case EXPR_CONTENT:     base = 4500; vary = 1500; break;
    case EXPR_EXCITED:     base = 2500; vary = 1000; break;
    case EXPR_SAD:         base = 6000; vary = 2000; break;
    case EXPR_MELANCHOLIC: base = 7500; vary = 2500; break;
    case EXPR_SYMPATHETIC: base = 4500; vary = 1500; break;
    case EXPR_ANGRY:       base = 5500; vary = 1500; break;
    case EXPR_FRUSTRATED:  base = 4500; vary = 1500; break;
    case EXPR_SCARED:      base = 7000; vary = 2000; break;
    case EXPR_CURIOUS:     base = 3250; vary = 1250; break;
    case EXPR_SURPRISED:   base = 8000; vary = 2000; break;
    case EXPR_SLEEPY:      base = 2500; vary = 800;  break;
    case EXPR_WINK:        base = 4000; vary = 2000; break;
    default: break;
  }
  float f = 1.4f - (current_intensity_ * 0.08f);
  base = (uint32_t)(base * f);
  return constrain(base + random(-(int32_t)vary, (int32_t)vary),
                   1200UL, 11000UL);
}
