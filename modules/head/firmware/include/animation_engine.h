/**
 * animation_engine.h - Eye Expression & Status Animation Engine
 * Story 1.3 (firmware) / Story 7.1a (eye visual language rewrite)
 *
 * Sprite-based rendering: all drawing goes to an off-screen buffer,
 * then pushes the complete frame to the display. Zero flicker.
 *
 * Story 7.1a (re-spec 2026-05-19, Kamal): the device eye is a
 * **kawaii cartoon** model — a light eyeball shape (optionally
 * cropped: top / bottom / both / left / diagonal / partial-circle)
 * + a black pupil + a small catchlight. Each of the two LCDs is an
 * independent eye (asymmetric emotions differ left vs right).
 *
 * Intensity is a 3-LEVEL model (strict; incoming 3-5 clamp to L3):
 *   L1 — base shape, monochrome (no tint, no FX)
 *   L2 — + per-emotion colour tint + static described marks
 *   L3 — + animated anime FX (tears stream, sparkle burst, …)
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

// How the eyeball silhouette is cut (Story 7.1a kawaii model). The
// crop is applied as a straight (or curved/diagonal) chord on the
// filled ellipse — matching the reference-sheet cells.
enum CropMode {
  CROP_NONE        = 0,  // full ellipse
  CROP_TOP         = 1,  // flat horizontal cut off the top
  CROP_BOTTOM      = 2,  // flat horizontal cut off the bottom
  CROP_TOPBOT      = 3,  // both top and bottom (narrow band)
  CROP_TOP_CURVED  = 4,  // top cut is a downward-bulging curve
  CROP_TOPBOT_CURV = 5,  // curved top + flat bottom (sad/Concerned L)
  CROP_LEFT        = 6,  // vertical cut off the outer (left) side
  CROP_DIAG_TOP    = 7,  // sloped cut from the top (Disgusted L)
  CROP_DIAG_BOT    = 8,  // sloped cut from the bottom (Disgusted R)
  CROP_BASE_CURVED = 9,  // top is full ellipse, BASE curves upward
                         // at centre — "smile-arch" / upper-arc happy
  CROP_TOP_CURVED_SLANT = 10,  // curved top + linear slant
                               // (outer edge higher than inner) — wink
  CROP_BOTTOM_SLANT     = 11,  // straight bottom crop with slant
                               // (outer corner higher) — happy v1
  CROP_BOTTOM_CURVED_SL = 12,  // bottom crop, slanted + curved
                               // (smile-arch rise at centre) — happy v2
};

// One eye's full geometry, built per-emotion per-side at render time.
struct EyeGeom {
  float    cx_off;     // horiz centre shift from sprite centre (+inner)
  float    halfW;
  float    halfH;
  uint8_t  crop;       // CropMode
  float    cropFrac;   // fraction of total height/width removed
  bool     hasPupil;
  float    pupR;       // pupil radius px
  float    pupX;       // pupil centre x offset (+ = nasal/inner)
  float    pupY;       // pupil centre y offset (+ = down)
  bool     catchlight;
};

// Kept for the blink-masked transition machinery (snap at midpoint).
// Rendering reads current_expression_ / current_intensity_ directly.
struct ExpressionParams {
  uint8_t type;
  uint8_t intensity;
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

  // Look direction (-100 to +100) — accepted over I2C, moves nothing
  // (spec: head/neck/ears carry 100% of gaze motion).
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
  void renderEye(int16_t cx, uint8_t type, uint8_t level,
                 float closedness, bool is_right);
  void interpolateParams(float progress);
  void updateWakeTransition();
  void updateAutoLook();
  void updateBlinkAnimation();
  float    computeClosedness();
  uint32_t calculateNextBlinkInterval();

  // Story 7.1a kawaii primitives
  uint8_t  levelOf(uint8_t intensity);
  void     buildEye(uint8_t type, bool viewer_left, EyeGeom& g);
  void     drawEyeball(int16_t cx, int16_t cy, const EyeGeom& g,
                       bool viewer_left, uint16_t color);
  void     drawPupil(int16_t cx, int16_t cy, const EyeGeom& g,
                     bool viewer_left);
  void     drawClosedLine(int16_t x, int16_t y, int16_t halfW);
  void     strokeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                      int16_t thick, uint16_t color);

  // ---- Motif library: L2 static colour marks + L3 anime FX ----
  // (manpu: vein 💢, sweat, gloom lines, sparkles, hearts, ZZZ, ?/!)
  void applyEmotionFX(uint8_t type, uint8_t level, int16_t ecx,
                      int16_t ecy, const EyeGeom& g, bool viewer_left);
  void drawPurpleLines(int16_t cx, int16_t cy, int16_t halfW);
  void drawFlirtyLines(int16_t cx, int16_t cy, int16_t halfW,
                       int16_t halfH, bool viewer_left);
  void drawWaterDrop(int16_t cx, int16_t cy, int16_t halfW,
                     int16_t halfH, bool animated);
  void drawSparkle(int16_t cx, int16_t cy, int16_t r,
                   uint16_t color, bool animated);
  void drawSteam(int16_t cx, int16_t cy, int16_t halfW, int16_t halfH);
  void drawBlush(int16_t cx, int16_t cy, int16_t halfW, int16_t halfH,
                 bool viewer_left);
  void drawTwinkles(int16_t cx, int16_t cy, int16_t halfW,
                    int16_t halfH, bool animated);
  void drawStarBurst(int16_t cx, int16_t cy, int16_t r);
  void drawGloomLines(int16_t cx, int16_t cy, int16_t halfW,
                      int16_t halfH, bool animated);
  void drawCrossVein(int16_t cx, int16_t cy, int16_t s, bool animated);
  void drawSweatDrop(int16_t cx, int16_t cy, int16_t halfW,
                     bool viewer_left, bool animated);
  void drawShadowLines(int16_t cx, int16_t cy, int16_t halfW,
                       int16_t halfH);
  void drawGlyph(const char* s, int16_t x, int16_t y, uint8_t font,
                 uint16_t color);
  void drawHearts(int16_t cx, int16_t cy, int16_t halfW, int16_t halfH,
                  bool animated);
  void drawImpactLines(int16_t cx, int16_t cy, int16_t r);
  void drawHeart(int16_t cx, int16_t cy, int16_t s, uint16_t color);
  void drawBackgroundSparkles(uint16_t color, int count);

  // Bloom-pulse factor (content/sympathetic L3 soft glow)
  float bloom_pulse_;
};
