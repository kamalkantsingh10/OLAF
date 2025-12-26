/**
 * Torso Module - Animation Demo
 *
 * Demonstrates 3 animation states with smooth transitions:
 * - HEARTBEAT: Steady pulse (normal)
 * - BREATHING: Slow fade (calm/idle)
 * - EXCITED: Fast pulse with color shift
 *
 * Uses sprite double-buffering for flicker-free rendering
 * Press 'h', 'b', or 'e' via serial to switch animations
 */

#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);  // Sprite for double buffering

// Animation states
enum AnimationState {
  HEARTBEAT,  // Steady pulse
  BREATHING,  // Slow fade
  EXCITED     // Fast pulse + color shift
};

AnimationState currentState = HEARTBEAT;
unsigned long lastStateChange = 0;
const unsigned long STATE_DURATION = 5000;  // 5 seconds per state

// Animation parameters
struct AnimParams {
  int scale;
  int dir;
  int minScale;
  int maxScale;
  int speed;
  uint16_t color;
  int hue;  // For color cycling
};

AnimParams anim = {50, 1, 40, 60, 2, TFT_RED, 0};

// Forward declarations
void drawHeart(TFT_eSprite* spr, int x, int y, int size, uint16_t color);
void updateAnimation();
void handleSerialInput();
uint16_t hsvToRgb565(int h, int s, int v);

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== OLAF Torso Animation Demo ===");
  Serial.println("Commands:");
  Serial.println("  h - Heartbeat animation");
  Serial.println("  b - Breathing animation");
  Serial.println("  e - Excited animation");
  Serial.println("  a - Auto-cycle (every 5s)");
  Serial.println();

  // Initialize display
  tft.init();
  tft.setRotation(2);  // Landscape mode (320×240)
  tft.fillScreen(TFT_BLACK);

  // Create sprite buffer (320x240x16bit = ~150KB, fits in 8MB PSRAM)
  sprite.createSprite(320, 240);
  sprite.setSwapBytes(true);  // Color byte order fix

  Serial.println("Display initialized (320x240)");
  Serial.println("Sprite buffer created in PSRAM");
  Serial.println("Starting animations...\n");

  lastStateChange = millis();
}

void loop() {
  handleSerialInput();
  updateAnimation();

  // Draw to sprite (off-screen buffer)
  sprite.fillSprite(TFT_BLACK);

  // Draw state label
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.setTextSize(2);
  sprite.setCursor(10, 10);

  switch(currentState) {
    case HEARTBEAT:
      sprite.print("HEARTBEAT");
      break;
    case BREATHING:
      sprite.print("BREATHING");
      break;
    case EXCITED:
      sprite.print("EXCITED!");
      break;
  }

  // Draw animated heart (centered)
  drawHeart(&sprite, 160, 120, anim.scale, anim.color);

  // Push complete frame to display (DMA transfer, no flicker)
  sprite.pushSprite(0, 0);

  delay(33);  // ~30 FPS
}

void updateAnimation() {
  switch(currentState) {
    case HEARTBEAT: {
      // Steady pulse (normal heartbeat)
      anim.scale += anim.dir * anim.speed;
      if (anim.scale >= anim.maxScale || anim.scale <= anim.minScale) {
        anim.dir *= -1;
      }
      anim.color = TFT_RED;
      break;
    }

    case BREATHING: {
      // Slow fade (calm breathing)
      anim.scale += anim.dir * anim.speed;
      if (anim.scale >= anim.maxScale || anim.scale <= anim.minScale) {
        anim.dir *= -1;
      }
      // Fade color intensity
      int brightness = map(anim.scale, anim.minScale, anim.maxScale, 60, 255);
      anim.color = tft.color565(brightness, 0, 0);
      break;
    }

    case EXCITED: {
      // Fast pulse with color cycling
      anim.scale += anim.dir * anim.speed;
      if (anim.scale >= anim.maxScale || anim.scale <= anim.minScale) {
        anim.dir *= -1;
      }
      // Cycle through hue (red → pink → red)
      anim.hue = (anim.hue + 2) % 360;
      anim.color = hsvToRgb565(anim.hue, 255, 255);
      break;
    }
  }
}

void handleSerialInput() {
  if (Serial.available()) {
    char cmd = Serial.read();

    switch(cmd) {
      case 'h':
        currentState = HEARTBEAT;
        anim = {50, 1, 40, 60, 2, TFT_RED, 0};
        Serial.println("→ HEARTBEAT mode");
        break;

      case 'b':
        currentState = BREATHING;
        anim = {50, 1, 30, 70, 1, TFT_RED, 0};
        Serial.println("→ BREATHING mode");
        break;

      case 'e':
        currentState = EXCITED;
        anim = {50, 1, 35, 65, 4, TFT_RED, 0};
        Serial.println("→ EXCITED mode");
        break;

      case 'a':
        Serial.println("→ AUTO-CYCLE mode");
        lastStateChange = millis();
        break;
    }
  }

  // Auto-cycle through states (demo mode)
  if (millis() - lastStateChange > STATE_DURATION) {
    lastStateChange = millis();

    // Cycle to next state
    switch(currentState) {
      case HEARTBEAT:
        currentState = BREATHING;
        anim = {50, 1, 30, 70, 1, TFT_RED, 0};
        Serial.println("Auto-cycle → BREATHING");
        break;
      case BREATHING:
        currentState = EXCITED;
        anim = {50, 1, 35, 65, 4, TFT_RED, 0};
        Serial.println("Auto-cycle → EXCITED");
        break;
      case EXCITED:
        currentState = HEARTBEAT;
        anim = {50, 1, 40, 60, 2, TFT_RED, 0};
        Serial.println("Auto-cycle → HEARTBEAT");
        break;
    }
  }
}

void drawHeart(TFT_eSprite* spr, int x, int y, int size, uint16_t color) {
  // Simple heart shape using circles and triangle (draw to sprite)
  spr->fillCircle(x - size/3, y - size/4, size/3, color);
  spr->fillCircle(x + size/3, y - size/4, size/3, color);
  spr->fillTriangle(
    x - size*2/3, y - size/6,
    x + size*2/3, y - size/6,
    x, y + size,
    color
  );
}

// Convert HSV to RGB565 for color cycling
uint16_t hsvToRgb565(int h, int s, int v) {
  h = h % 360;
  s = constrain(s, 0, 255);
  v = constrain(v, 0, 255);

  int region = h / 60;
  int remainder = (h % 60) * 6;

  int p = (v * (255 - s)) >> 8;
  int q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  int t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  int r, g, b;
  switch(region) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
  }

  // Convert to RGB565
  return tft.color565(r, g, b);
}
