/**
 * Display Driver Implementation
 *
 * OLED HUD optimized for PID balancing feedback.
 * 128x32 SSD1306 layout:
 *   Row 0: Large pitch angle + state + IMU accuracy dot
 *   Row 1: Horizontal pitch bar (visual tilt indicator)
 *   Row 2: Left/right motor velocities + PID output
 */

#include "display_driver.h"
#include <Adafruit_GFX.h>
#include <Wire.h>

DisplayDriver oled;

bool DisplayDriver::begin(uint8_t width, uint8_t height, uint8_t addr) {
    display_ = new Adafruit_SSD1306(width, height, &Wire, -1);
    ok_ = display_->begin(SSD1306_SWITCHCAPVCC, addr);
    if (ok_) {
        display_->setTextColor(SSD1306_WHITE);
    } else {
        Serial.println("[Display] OLED init failed");
    }
    return ok_;
}

void DisplayDriver::showSplash(const char* line1, const char* line2) {
    if (!ok_) return;
    display_->clearDisplay();
    display_->setTextSize(2);
    display_->setCursor(0, 0);
    display_->println(line1);
    display_->println(line2);
    display_->display();
}

void DisplayDriver::showMessage(const char* line1, const char* line2) {
    if (!ok_) return;
    display_->clearDisplay();
    display_->setTextSize(1);
    display_->setCursor(0, 0);
    display_->println(line1);
    if (line2) display_->println(line2);
    display_->display();
}

void DisplayDriver::updateHUD(float pitch_deg, float roll_deg,
                               AppState state, float m0_vel, float m1_vel,
                               uint8_t imu_accuracy) {
    if (!ok_) return;

    display_->clearDisplay();
    display_->setTextColor(SSD1306_WHITE);

    // --- Row 0 (y=0): Large pitch + state + IMU dot ---
    // Pitch angle in large text (size 2 = 12x16 px)
    display_->setTextSize(2);
    display_->setCursor(0, 0);
    display_->printf("%+.1f", pitch_deg);

    // State label (size 1, top-right area)
    display_->setTextSize(1);
    display_->setCursor(88, 0);
    display_->print(stateLabel(state));

    // IMU accuracy dot — top-right corner
    // Filled = high accuracy (3), hollow = low
    if (imu_accuracy >= 3) {
        display_->fillCircle(124, 3, 3, SSD1306_WHITE);
    } else if (imu_accuracy >= 2) {
        display_->drawCircle(124, 3, 3, SSD1306_WHITE);
        display_->drawPixel(124, 3, SSD1306_WHITE);
    } else {
        display_->drawCircle(124, 3, 3, SSD1306_WHITE);
    }

    // --- Row 1 (y=17): Pitch bar — visual tilt indicator ---
    constexpr int16_t bar_y = 17;
    constexpr int16_t bar_h = 5;
    constexpr int16_t bar_left = 4;
    constexpr int16_t bar_right = 123;
    constexpr int16_t bar_w = bar_right - bar_left;
    constexpr int16_t bar_center = bar_left + bar_w / 2;

    // Bar outline
    display_->drawRect(bar_left, bar_y, bar_w, bar_h, SSD1306_WHITE);

    // Center tick mark
    display_->drawFastVLine(bar_center, bar_y - 1, bar_h + 2, SSD1306_WHITE);

    // Fill bar proportional to pitch (±20° = full bar)
    constexpr float bar_max_deg = 20.0f;
    float clamped = constrain(pitch_deg, -bar_max_deg, bar_max_deg);
    int16_t fill_px = (int16_t)(clamped / bar_max_deg * (bar_w / 2));

    if (fill_px > 0) {
        display_->fillRect(bar_center, bar_y + 1, fill_px, bar_h - 2, SSD1306_WHITE);
    } else if (fill_px < 0) {
        display_->fillRect(bar_center + fill_px, bar_y + 1, -fill_px, bar_h - 2, SSD1306_WHITE);
    }

    // --- Row 2 (y=25): Motor velocities + PID output ---
    display_->setTextSize(1);
    display_->setCursor(0, 25);
    display_->printf("L%+.1f R%+.1f", m0_vel, m1_vel);

    // PID output on right side
    float pid_out = m0_vel;  // Approximate — both wheels get same balance vel
    display_->setCursor(88, 25);
    display_->printf("P%+.1f", pitch_deg);

    display_->display();
}

bool DisplayDriver::isOk() {
    return ok_;
}

const char* DisplayDriver::stateLabel(AppState state) {
    switch (state) {
        case STATE_IDLE:       return "IDLE";
        case STATE_BALANCING:  return "BAL";
        case STATE_ERROR:      return "ERR!";
        default:               return "???";
    }
}
