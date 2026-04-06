/**
 * Display Driver — OLED HUD for Base Module
 *
 * SSD1306 128x32 optimized for PID balancing feedback.
 * Shows pitch angle, tilt bar, motor velocities, and state.
 */

#pragma once

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

// Application state for HUD display
enum AppState { STATE_IDLE, STATE_BALANCING, STATE_ERROR };

class DisplayDriver {
public:
    bool begin(uint8_t width, uint8_t height, uint8_t addr);

    /** Show a 1-2 line text message (clears screen) */
    void showMessage(const char* line1, const char* line2 = nullptr);

    /** Show splash screen (large text) */
    void showSplash(const char* line1, const char* line2);

    /**
     * Update balancing HUD. Call at 10Hz max.
     * Layout:
     *   Row 0: Large pitch angle + state + IMU dot
     *   Row 1: Horizontal pitch bar (center = balanced)
     *   Row 2: L/R motor velocities + pitch readout
     */
    void updateHUD(float pitch_deg, float roll_deg,
                   AppState state, float m0_vel, float m1_vel,
                   uint8_t imu_accuracy);

    bool isOk();

private:
    Adafruit_SSD1306* display_;
    bool ok_;

    const char* stateLabel(AppState state);
};

extern DisplayDriver oled;
