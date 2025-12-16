/**
 * Balancing Controller - 200Hz PID Loop
 * Base Module - Self-Balancing Control
 *
 * Implements PID control for self-balancing two-wheel robot.
 * Runs at 200Hz (5ms period) for stable, responsive control.
 *
 * Control Flow:
 * 1. Read pitch angle from IMU (how far tilted)
 * 2. Calculate error: target (0°) - current pitch
 * 3. Run PID: output = Kp*error + Ki*integral + Kd*derivative
 * 4. Send output velocity to motors via ODrive
 * 5. Repeat every 5ms
 *
 * PID Tuning (from config.h):
 * - Kp = 20-35: Proportional gain (main correction)
 * - Ki = 0-0.2: Integral gain (eliminate steady-state error)
 * - Kd = 5-8: Derivative gain (dampen oscillations)
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include "imu_fusion.h"
#include "odrive_uart.h"

enum BalanceMode {
    MODE_DISABLED,      // Motors off, no balancing
    MODE_BALANCING,     // Active balancing enabled
    MODE_EMERGENCY_STOP // Emergency stop triggered
};

class BalancingController {
public:
    /**
     * Initialize balancing controller
     */
    void begin();

    /**
     * Enable balancing mode
     */
    void enable();

    /**
     * Disable balancing (relax)
     */
    void disable();

    /**
     * Update PID loop - MUST be called at 200Hz (every 5ms)
     * @return true if balancing active, false if disabled/emergency
     */
    bool update();

    /**
     * Set target velocity (from Raspberry Pi commands)
     * @param linear_mps Linear velocity in m/s
     * @param angular_radps Angular velocity in rad/s
     */
    void setTargetVelocity(float linear_mps, float angular_radps);

    /**
     * Get current balance mode
     */
    BalanceMode getMode();

    /**
     * Check if emergency stop is active
     * @return true if emergency condition detected
     */
    bool isEmergency();

    /**
     * Reset PID integral term (call after large disturbances)
     */
    void resetPID();

    /**
     * Get current PID output (for debugging)
     */
    float getPIDOutput();

private:
    BalanceMode mode_;
    bool initialized_;

    // Target velocities (from Pi or manual control)
    float targetLinearVel_;
    float targetAngularVel_;

    // PID state
    float lastError_;
    float integral_;
    uint32_t lastUpdateMicros_;

    // Current PID output
    float pidOutput_;

    // Emergency detection
    uint32_t emergencyStartTime_;
    bool emergencyTriggered_;

    /**
     * Run PID calculation
     * @param current_pitch Current pitch angle (degrees)
     * @param dt Time step (seconds)
     * @return Motor velocity command (rev/s)
     */
    float calculatePID(float current_pitch, float dt);

    /**
     * Check for emergency conditions
     * @param pitch Current pitch angle
     * @return true if emergency detected
     */
    bool checkEmergency(float pitch);

    /**
     * Handle emergency stop condition
     */
    void handleEmergency();
};

// Global instance
extern BalancingController balancer;
