/**
 * gc9a01_driver_spi.h - Dual GC9A01 Round TFT Display Driver
 * Story 1.4: Head Module ESP32 Firmware - Eye Expressions
 *
 * Purpose:
 *   Provides clean interface for controlling dual GC9A01 round displays (240×240)
 *   as synchronized "eyes". Abstracts away CS pin switching and ensures both
 *   displays stay synchronized within ±1 frame.
 *
 * Hardware Configuration:
 *   - Shared SPI bus: SCK (GPIO12), MOSI (GPIO11), DC (GPIO2), RST (GPIO4)
 *   - Individual CS pins: Left eye (GPIO5), Right eye (GPIO15)
 *   - SPI speed: 20 MHz (optimal for 60 FPS dual display rendering)
 *
 * Design Philosophy:
 *   - High-level API: renderBothEyes(leftPixels, rightPixels)
 *   - CS switching handled internally
 *   - Frame synchronization guaranteed
 *   - Optimized for 60 FPS (16.67ms per frame total)
 *
 * Performance Target:
 *   - 60 FPS on BOTH displays
 *   - <16ms total render time per frame
 *   - ±1 frame synchronization tolerance
 */

#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>

// Eye selection enum for clarity
enum Eye {
  LEFT,    // Left eye display
  RIGHT,   // Right eye display
  BOTH     // Both eyes simultaneously (for synchronized init)
};

// Display dimensions
constexpr uint16_t DISPLAY_WIDTH = 240;
constexpr uint16_t DISPLAY_HEIGHT = 240;
constexpr uint16_t DISPLAY_CENTER_X = 120;
constexpr uint16_t DISPLAY_CENTER_Y = 120;

/**
 * Dual GC9A01 Display Driver
 *
 * Manages two round TFT displays as synchronized "eyes".
 * Handles CS pin switching and ensures frame synchronization.
 */
class GC9A01DualDriver {
public:
  /**
   * Initialize both displays
   *
   * Configures SPI pins, initializes both GC9A01 displays, clears screens.
   *
   * @param left_cs_pin GPIO pin for left eye chip select
   * @param right_cs_pin GPIO pin for right eye chip select
   * @return true if initialization successful, false on error
   *
   * Note: Must be called in setup() before any drawing operations
   */
  bool begin(uint8_t left_cs_pin, uint8_t right_cs_pin);

  /**
   * Select which eye(s) to draw on
   *
   * Sets CS pins to select left eye, right eye, or both.
   * All subsequent TFT_eSPI draw calls will affect selected eye(s).
   *
   * @param eye Which eye to select (LEFT, RIGHT, or BOTH)
   *
   * Design Note: BOTH mode is only for initialization. For normal rendering,
   * draw to LEFT, then RIGHT separately to allow different content per eye.
   */
  void selectEye(Eye eye);

  /**
   * Get TFT_eSPI instance for direct drawing
   *
   * After calling selectEye(), use this to access TFT_eSPI drawing functions.
   *
   * @return Reference to TFT_eSPI instance
   *
   * Example:
   *   driver.selectEye(LEFT);
   *   driver.getTFT().fillCircle(120, 120, 50, TFT_WHITE);
   */
  TFT_eSPI& getTFT();

  /**
   * Clear both eyes to black
   *
   * Convenience method to clear both displays simultaneously.
   * Useful for resetting displays before new animation frame.
   */
  void clearBothEyes();

  /**
   * Fill both eyes with specified color
   *
   * @param color RGB565 color value
   */
  void fillBothEyes(uint16_t color);

  /**
   * Draw filled circle on selected eye
   *
   * Convenience wrapper around TFT_eSPI fillCircle.
   * Must call selectEye() first to choose which display.
   *
   * @param x X coordinate (0-239)
   * @param y Y coordinate (0-239)
   * @param radius Circle radius in pixels
   * @param color RGB565 color value
   */
  void drawFilledCircle(int16_t x, int16_t y, uint16_t radius, uint16_t color);

  /**
   * Get frame timing info for performance monitoring
   *
   * Returns time taken to render last frame in milliseconds.
   * Use to verify 60 FPS target (<16.67ms per frame).
   *
   * @return Last frame render time in milliseconds
   */
  uint32_t getLastFrameTimeMs();

  /**
   * Start frame timing measurement
   *
   * Call at beginning of render loop to measure frame time.
   */
  void startFrameTiming();

  /**
   * End frame timing measurement
   *
   * Call at end of render loop to calculate frame time.
   */
  void endFrameTiming();

private:
  TFT_eSPI tft_;                    // TFT_eSPI instance (shared between eyes)
  uint8_t left_cs_pin_;             // GPIO pin for left eye CS
  uint8_t right_cs_pin_;            // GPIO pin for right eye CS

  // Frame timing for performance monitoring
  uint32_t frame_start_micros_;     // Frame start time (microseconds)
  uint32_t last_frame_time_ms_;     // Last frame duration (milliseconds)

  /**
   * Internal helper: Deselect all eyes (set CS pins HIGH)
   *
   * Called internally to ensure only one display active at a time.
   */
  void deselectAllEyes();
};
