/**
 * display_driver.h - Dual GC9A01 Round TFT Display Driver
 * Story 1.3: Develop Head ESP32 Firmware
 *
 * Purpose:
 *   Controls dual GC9A01 round displays (240x240) as synchronized "eyes".
 *   Handles CS pin switching for independent eye content.
 *
 * Hardware (per wiring-guide.md - J3 Module-Specific header):
 *   - Shared SPI: CLK (GPIO38), MOSI (GPIO39)
 *   - Shared Control: DC (GPIO40), RST (GPIO41)
 *   - Left Eye CS: GPIO2
 *   - Right Eye CS: GPIO1
 *   - Backlight: Connected to 3.3V (always on)
 *   - SPI speed: 20 MHz for 30+ FPS
 */

#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "config.h"

// Eye selection enum
enum Eye {
  LEFT,
  RIGHT,
  BOTH
};

// Display dimensions (also in config.h)
constexpr uint16_t DISPLAY_WIDTH = 240;
constexpr uint16_t DISPLAY_HEIGHT = 240;
constexpr uint16_t DISPLAY_CENTER_X = 120;
constexpr uint16_t DISPLAY_CENTER_Y = 120;

/**
 * Dual GC9A01 Display Driver
 */
class GC9A01DualDriver {
public:
  /**
   * Initialize both displays
   *
   * @param left_cs_pin GPIO for left eye chip select
   * @param right_cs_pin GPIO for right eye chip select
   * @return true if successful
   */
  bool begin(uint8_t left_cs_pin, uint8_t right_cs_pin);

  /**
   * Select which eye(s) to draw on
   *
   * @param eye LEFT, RIGHT, or BOTH
   */
  void selectEye(Eye eye);

  /**
   * Get TFT_eSPI instance for direct drawing
   */
  TFT_eSPI& getTFT();

  /**
   * Clear both eyes to black
   */
  void clearBothEyes();

  /**
   * Fill both eyes with color
   *
   * @param color RGB565 color value
   */
  void fillBothEyes(uint16_t color);

  /**
   * Draw filled circle on selected eye
   */
  void drawFilledCircle(int16_t x, int16_t y, uint16_t radius, uint16_t color);

  /**
   * Frame timing for performance monitoring
   */
  uint32_t getLastFrameTimeMs();
  void startFrameTiming();
  void endFrameTiming();

private:
  TFT_eSPI tft_;
  uint8_t left_cs_pin_;
  uint8_t right_cs_pin_;

  uint32_t frame_start_micros_;
  uint32_t last_frame_time_ms_;

  void deselectAllEyes();
};
