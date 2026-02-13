/**
 * display_driver.cpp - Dual GC9A01 Display Driver Implementation
 * Story 1.3: Develop Head ESP32 Firmware
 *
 * Handles dual round TFT displays via manual CS pin switching.
 * TFT_eSPI configured via platformio.ini build flags.
 *
 * Pin assignments (per wiring-guide.md):
 *   SPI: CLK=GPIO38, MOSI=GPIO39, DC=GPIO40, RST=GPIO41 (all shared)
 *   CS: Left=GPIO2, Right=GPIO1
 */

#include "display_driver.h"

bool GC9A01DualDriver::begin(uint8_t left_cs_pin, uint8_t right_cs_pin) {
  left_cs_pin_ = left_cs_pin;
  right_cs_pin_ = right_cs_pin;

  Serial.println("[Display] === Display Driver Init Start ===");
  Serial.printf("[Display] Left CS pin: GPIO%d\n", left_cs_pin_);
  Serial.printf("[Display] Right CS pin: GPIO%d\n", right_cs_pin_);
  Serial.printf("[Display] TFT_eSPI config: MOSI=%d, SCLK=%d, DC=%d, RST=%d\n",
                TFT_MOSI, TFT_SCLK, TFT_DC, TFT_RST);

  // Configure CS pins as outputs (active LOW)
  pinMode(left_cs_pin_, OUTPUT);
  pinMode(right_cs_pin_, OUTPUT);

  // Deselect both initially
  digitalWrite(left_cs_pin_, HIGH);
  digitalWrite(right_cs_pin_, HIGH);

  Serial.println("[Display] Initializing dual GC9A01 displays...");

  try {
    // Select BOTH eyes for simultaneous init
    digitalWrite(left_cs_pin_, LOW);
    digitalWrite(right_cs_pin_, LOW);

    tft_.init();
    Serial.println("[Display] tft_.init() completed");

    // Deselect after init
    digitalWrite(left_cs_pin_, HIGH);
    digitalWrite(right_cs_pin_, HIGH);

    // Set rotation for each eye (adjust based on physical mounting)
    // Left eye: 90 degrees
    digitalWrite(left_cs_pin_, LOW);
    tft_.setRotation(1);
    digitalWrite(left_cs_pin_, HIGH);

    // Right eye: 90 degrees (180° rotated from left for physical mounting)
    digitalWrite(right_cs_pin_, LOW);
    tft_.setRotation(3);
    digitalWrite(right_cs_pin_, HIGH);

    Serial.println("[Display] Displays initialized, rotations set");

    // Clear both displays to black
    clearBothEyes();
    Serial.println("[Display] Displays cleared");

    // Initialize timing
    frame_start_micros_ = 0;
    last_frame_time_ms_ = 0;

    Serial.println("[Display] === Display Driver Init SUCCESS ===");
    return true;

  } catch (...) {
    Serial.println("[Display] ERROR: Display initialization FAILED!");
    return false;
  }
}

void GC9A01DualDriver::selectEye(Eye eye) {
  switch (eye) {
    case LEFT:
      digitalWrite(left_cs_pin_, LOW);
      digitalWrite(right_cs_pin_, HIGH);
      tft_.setRotation(3);  // 270°
      break;

    case RIGHT:
      digitalWrite(left_cs_pin_, HIGH);
      digitalWrite(right_cs_pin_, LOW);
      tft_.setRotation(1);  // 90°
      break;

    case BOTH:
      digitalWrite(left_cs_pin_, LOW);
      digitalWrite(right_cs_pin_, LOW);
      tft_.setRotation(3);  // 270°
      break;
  }
}

TFT_eSPI& GC9A01DualDriver::getTFT() {
  return tft_;
}

void GC9A01DualDriver::clearBothEyes() {
  selectEye(BOTH);
  tft_.fillScreen(TFT_BLACK);
  deselectAllEyes();
}

void GC9A01DualDriver::fillBothEyes(uint16_t color) {
  selectEye(BOTH);
  tft_.fillScreen(color);
  deselectAllEyes();
}

void GC9A01DualDriver::drawFilledCircle(int16_t x, int16_t y, uint16_t radius, uint16_t color) {
  tft_.fillCircle(x, y, radius, color);
}

uint32_t GC9A01DualDriver::getLastFrameTimeMs() {
  return last_frame_time_ms_;
}

void GC9A01DualDriver::startFrameTiming() {
  frame_start_micros_ = micros();
}

void GC9A01DualDriver::endFrameTiming() {
  uint32_t elapsed_micros = micros() - frame_start_micros_;
  last_frame_time_ms_ = elapsed_micros / 1000;

  if (last_frame_time_ms_ > 33) {  // Target 30 FPS = 33ms
    Serial.printf("[Display] WARNING: Frame took %lums (target <33ms)\n", last_frame_time_ms_);
  }
}

void GC9A01DualDriver::deselectAllEyes() {
  digitalWrite(left_cs_pin_, HIGH);
  digitalWrite(right_cs_pin_, HIGH);
}
