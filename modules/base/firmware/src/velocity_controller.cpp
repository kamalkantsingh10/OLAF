/**
 * Velocity Outer Loop Controller Implementation
 *
 * Reads wheel velocity from ODrive "f <axis>" command at 50Hz.
 * Runs PI loop to output pitch target offset for the balance PID.
 */

#include "velocity_controller.h"
#include "odrive_uart.h"

VelocityController velController;

// Live-tunable gains (web sliders update these)
float tuneVelKp = kVelKp;
float tuneVelKi = kVelKi;
float tuneVelAlpha = kVelFilterAlpha;

void VelocityController::begin() {
    enabled_ = false;  // Disabled by default — enable via web dashboard
    desiredVelocityMps_ = 0.0f;
    measuredVelocity_ = 0.0f;
    velIntegral_ = 0.0f;
    pitchOffset_ = 0.0f;
    lastUpdateMs_ = 0;

    Serial.printf("[VelCtrl] Initialized — Kp=%.1f Ki=%.1f, interval=%dms\n",
                  tuneVelKp, tuneVelKi, kVelIntervalMs);
}

void VelocityController::reset() {
    velIntegral_ = 0.0f;
    pitchOffset_ = 0.0f;
    measuredVelocity_ = 0.0f;
    filteredVel0_ = 0.0f;
    filteredVel1_ = 0.0f;
    lastUpdateMs_ = millis();
}

void VelocityController::update() {
    if (!enabled_) {
        pitchOffset_ = 0.0f;
        return;
    }

    uint32_t now = millis();
    if (now - lastUpdateMs_ < kVelIntervalMs) return;

    float dt = (now - lastUpdateMs_) / 1000.0f;
    lastUpdateMs_ = now;

    // Clamp dt to prevent huge integral jumps after pauses
    if (dt > 0.1f) dt = 0.1f;

    // Read ONE axis per cycle to minimize UART blocking time (~10ms not ~40ms)
    // Alternate between axis 0 and 1 each update
    static uint8_t nextAxis = 0;

    float alpha = tuneVelAlpha;
    if (nextAxis == 0) {
        float rawVel0 = odriveUart.readVelocity(0) * kAxis0Direction;
        filteredVel0_ = alpha * filteredVel0_ + (1.0f - alpha) * rawVel0;
        nextAxis = 1;
    } else {
        float rawVel1 = odriveUart.readVelocity(1) * kAxis1Direction;
        filteredVel1_ = alpha * filteredVel1_ + (1.0f - alpha) * rawVel1;
        nextAxis = 0;
    }
    measuredVelocity_ = (filteredVel0_ + filteredVel1_) * 0.5f;

    // Convert desired velocity from m/s to turns/s
    float desiredVelTurns = desiredVelocityMps_ / kWheelCircumferenceM;

    // Velocity PI — output is pitch offset added to balance target
    float velError = desiredVelTurns - measuredVelocity_;

    // P term
    float p = tuneVelKp * velError;

    // I term with anti-windup
    velIntegral_ += velError * dt;
    velIntegral_ = constrain(velIntegral_, -kVelMaxIntegral, kVelMaxIntegral);
    float i = tuneVelKi * velIntegral_;

    // Output: pitch offset in degrees, clamped to safe range
    pitchOffset_ = constrain(p + i, -kVelOutputMaxDeg, kVelOutputMaxDeg);

    if (kEnablePIDDebug) {
        Serial.printf("[VelCtrl] fv0=%.3f fv1=%.3f avg=%.3f err=%.3f offset=%.2f° alpha=%.2f\n",
                      filteredVel0_, filteredVel1_, measuredVelocity_, velError, pitchOffset_,
                      tuneVelAlpha);
    }
}
