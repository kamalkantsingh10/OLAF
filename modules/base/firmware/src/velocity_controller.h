/**
 * Velocity Outer Loop Controller
 *
 * Reads wheel velocity from ODrive, runs PI loop at 50Hz.
 * Outputs a pitch target offset (degrees) consumed by BalancingController.
 *
 * Architecture (cascade control):
 *   desired_vel (0) → [Velocity PID] → pitch offset → [Balance PID] → current → motors
 *
 * When stationary (desired_vel=0):
 *   Drift backward → negative velocity → positive pitch offset → lean forward → stop
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Live-tunable velocity gains (initialized from config.h defaults)
extern float tuneVelKp;
extern float tuneVelKi;
extern float tuneVelAlpha;  // Exponential smoothing factor (0.5–0.99)

class VelocityController {
public:
    void begin();

    /**
     * Run one velocity loop iteration (self-timed at 50Hz).
     * Reads ODrive velocity via UART — call every main loop, returns early if not time yet.
     */
    void update();

    /** Reset PID state (call when balance starts/stops) */
    void reset();

    /** Get current pitch offset in degrees (add to balance target) */
    float getPitchOffset() const { return pitchOffset_; }

    /** Get last measured forward velocity in turns/s (for telemetry) */
    float getMeasuredVelocity() const { return measuredVelocity_; }

    /**
     * Set desired forward velocity.
     * @param mps Velocity in m/s (0 = stationary, positive = forward)
     */
    void setDesiredVelocity(float mps) { desiredVelocityMps_ = mps; }

    /** Enable/disable the velocity loop (disabled = pitch offset stays 0) */
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

private:
    bool enabled_;
    float desiredVelocityMps_;     // Target velocity in m/s
    float measuredVelocity_;       // Measured forward velocity in turns/s (filtered)
    float velIntegral_;            // PI integral accumulator
    float pitchOffset_;            // Output: pitch target offset in degrees
    uint32_t lastUpdateMs_;        // Last update timestamp

    // Exponential smoothing filter state for noisy Hall encoder velocity
    float filteredVel0_ = 0.0f;    // Filtered velocity axis 0 (turns/s)
    float filteredVel1_ = 0.0f;    // Filtered velocity axis 1 (turns/s)
};

extern VelocityController velController;
