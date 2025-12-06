/**
 * gc9a01_driver_spi.cpp - Dual GC9A01 Round TFT Display Driver Implementation
 * Story 1.4: Head Module ESP32 Firmware - Eye Expressions
 *
 * Implementation Details:
 *   - Uses TFT_eSPI library configured for GC9A01 driver
 *   - Manual CS pin control for dual display support
 *   - Optimized for 60 FPS rendering (both eyes must render in <16.67ms)
 *
 * SPI Communication:
 *   - Clock speed: 20 MHz (balance between speed and reliability)
 *   - Mode: SPI_MODE0 (default for GC9A01)
 *   - Bit order: MSBFIRST
 *
 * Thread Safety:
 *   - Not thread-safe (main loop only, no ISR access)
 *   - All methods assume single-threaded access
 */

#include "../firmware/gc9a01_driver_spi.h"

/**
 * Initialize both displays
 *
 * Steps:
 *   1. Configure CS pins as outputs
 *   2. Set both CS HIGH (inactive)
 *   3. Initialize TFT_eSPI library (handles SPI, DC, RST pins)
 *   4. Set both CS LOW to init both displays simultaneously
 *   5. Configure rotation (0 = no rotation for round displays)
 *   6. Clear displays to black
 *
 * @return true if initialization successful, false on error
 */
bool GC9A01DualDriver::begin(uint8_t left_cs_pin, uint8_t right_cs_pin) {
  left_cs_pin_ = left_cs_pin;
  right_cs_pin_ = right_cs_pin;

  // Configure CS pins as outputs
  // CS is active LOW (LOW = selected, HIGH = deselected)
  pinMode(left_cs_pin_, OUTPUT);
  pinMode(right_cs_pin_, OUTPUT);

  // Deselect both displays initially (CS HIGH)
  // This prevents any accidental SPI traffic from affecting displays during init
  digitalWrite(left_cs_pin_, HIGH);
  digitalWrite(right_cs_pin_, HIGH);

  Serial.println("[Display] Initializing dual GC9A01 displays...");
  Serial.printf("[Display]   Left eye CS: GPIO%d\n", left_cs_pin_);
  Serial.printf("[Display]   Right eye CS: GPIO%d\n", right_cs_pin_);

  // Initialize TFT_eSPI library
  // Note: TFT_eSPI is configured via build flags in platformio.ini
  // This handles SPI init, DC pin, RST pin, backlight, etc.
  try {
    // Select BOTH eyes for simultaneous init (faster, ensures sync)
    digitalWrite(left_cs_pin_, LOW);
    digitalWrite(right_cs_pin_, LOW);

    tft_.init();  // Initialize display controller

    // Deselect both eyes after init
    digitalWrite(left_cs_pin_, HIGH);
    digitalWrite(right_cs_pin_, HIGH);

    // Set different rotations for each eye due to physical mounting orientation
    // Left eye: 90° rotation (rotation value 1)
    digitalWrite(left_cs_pin_, LOW);
    tft_.setRotation(1);  // 90° clockwise
    digitalWrite(left_cs_pin_, HIGH);

    // Right eye: 270° rotation / -90° (rotation value 3)
    digitalWrite(right_cs_pin_, LOW);
    tft_.setRotation(3);  // 270° clockwise / -90° counter-clockwise
    digitalWrite(right_cs_pin_, HIGH);

    Serial.println("[Display] ✓ Dual displays initialized");

    // Clear both displays to black
    clearBothEyes();

    Serial.println("[Display] ✓ Displays cleared");

    // Initialize timing
    frame_start_micros_ = 0;
    last_frame_time_ms_ = 0;

    return true;

  } catch (...) {
    Serial.println("[Display] ERROR: Display initialization failed!");
    return false;
  }
}

/**
 * Select which eye(s) to draw on
 *
 * CS Pin Logic:
 *   - CS LOW = Display is selected and will respond to SPI
 *   - CS HIGH = Display is deselected and ignores SPI
 *
 * For LEFT eye: Set left CS LOW, right CS HIGH
 * For RIGHT eye: Set right CS LOW, left CS HIGH
 * For BOTH eyes: Set both CS LOW (used only for init and synchronized operations)
 *
 * @param eye Which eye to select
 */
void GC9A01DualDriver::selectEye(Eye eye) {
  switch (eye) {
    case LEFT:
      // Select left eye only
      digitalWrite(left_cs_pin_, LOW);   // Activate left
      digitalWrite(right_cs_pin_, HIGH); // Deactivate right
      break;

    case RIGHT:
      // Select right eye only
      digitalWrite(left_cs_pin_, HIGH);  // Deactivate left
      digitalWrite(right_cs_pin_, LOW);  // Activate right
      break;

    case BOTH:
      // Select both eyes (for simultaneous operations like clear screen)
      // WARNING: Both eyes will show IDENTICAL content in this mode
      digitalWrite(left_cs_pin_, LOW);
      digitalWrite(right_cs_pin_, LOW);
      break;
  }
}

/**
 * Get TFT_eSPI instance for direct drawing
 *
 * Allows access to full TFT_eSPI API for custom drawing.
 * Remember to call selectEye() first!
 *
 * @return Reference to TFT_eSPI instance
 */
TFT_eSPI& GC9A01DualDriver::getTFT() {
  return tft_;
}

/**
 * Clear both eyes to black
 *
 * Optimization: Use BOTH mode to clear simultaneously instead of
 * clearing each eye individually (2x faster).
 */
void GC9A01DualDriver::clearBothEyes() {
  selectEye(BOTH);  // Select both displays
  tft_.fillScreen(TFT_BLACK);
  deselectAllEyes();  // Deselect after operation
}

/**
 * Fill both eyes with specified color
 *
 * @param color RGB565 color value (e.g., TFT_WHITE, TFT_BLUE)
 */
void GC9A01DualDriver::fillBothEyes(uint16_t color) {
  selectEye(BOTH);
  tft_.fillScreen(color);
  deselectAllEyes();
}

/**
 * Draw filled circle on currently selected eye
 *
 * Must call selectEye() before calling this method.
 *
 * @param x X coordinate (0-239, center of display is 120)
 * @param y Y coordinate (0-239, center of display is 120)
 * @param radius Circle radius in pixels
 * @param color RGB565 color value
 *
 * Example - Draw white pupil on left eye:
 *   driver.selectEye(LEFT);
 *   driver.drawFilledCircle(120, 120, 40, TFT_WHITE);
 */
void GC9A01DualDriver::drawFilledCircle(int16_t x, int16_t y, uint16_t radius, uint16_t color) {
  tft_.fillCircle(x, y, radius, color);
}

/**
 * Get last frame render time
 *
 * Returns duration of last frame in milliseconds.
 * Use to monitor performance and ensure 60 FPS target (<16.67ms).
 *
 * @return Frame time in milliseconds
 */
uint32_t GC9A01DualDriver::getLastFrameTimeMs() {
  return last_frame_time_ms_;
}

/**
 * Start frame timing measurement
 *
 * Call this at the beginning of your render loop.
 *
 * Example:
 *   driver.startFrameTiming();
 *   renderLeftEye();
 *   renderRightEye();
 *   driver.endFrameTiming();
 *   Serial.printf("Frame time: %lums\n", driver.getLastFrameTimeMs());
 */
void GC9A01DualDriver::startFrameTiming() {
  frame_start_micros_ = micros();  // Record start time in microseconds
}

/**
 * End frame timing measurement
 *
 * Call this at the end of your render loop.
 * Calculates elapsed time since startFrameTiming() and stores in last_frame_time_ms_.
 */
void GC9A01DualDriver::endFrameTiming() {
  uint32_t elapsed_micros = micros() - frame_start_micros_;
  last_frame_time_ms_ = elapsed_micros / 1000;  // Convert to milliseconds

  // Warn if frame took too long (>16.67ms for 60 FPS)
  if (last_frame_time_ms_ > 17) {
    Serial.printf("[Display] WARNING: Frame took %lums (target <17ms for 60 FPS)\n",
                  last_frame_time_ms_);
  }
}

/**
 * Deselect all eyes
 *
 * Internal helper to ensure clean state.
 * Sets both CS pins HIGH (inactive).
 */
void GC9A01DualDriver::deselectAllEyes() {
  digitalWrite(left_cs_pin_, HIGH);
  digitalWrite(right_cs_pin_, HIGH);
}
