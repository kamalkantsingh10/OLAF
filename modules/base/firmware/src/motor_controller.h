/**
 * Motor Controller — High-Level Drive Interface
 *
 * Abstraction over ODriveUART for PID, navigation, and manual control.
 * Combines balance + linear + turn velocities into per-wheel commands.
 * Handles safety cutoff, direction correction, velocity clamping.
 *
 * Usage:
 *   motors.begin();
 *   motors.enable();
 *   motors.setBalanceVelocity(pid_output);   // PID calls this
 *   motors.setTurnRate(0.5);                  // Navigation/phone joystick
 *   motors.update(current_pitch_deg);         // Call every loop — applies velocities + safety
 */

#pragma once

#include <Arduino.h>
#include "config.h"

class MotorController {
public:
    /** Initialize ODrive UART and reset state */
    void begin();

    /** Enable motors (closed-loop control). Returns false if ODrive has errors. */
    bool enable();

    /** Disable motors (idle mode) */
    void disable();

    /** Zero velocity + disable — immediate */
    void emergencyStop();

    /**
     * Set balance current — PID pitch correction (torque mode).
     * Applied equally to both wheels (same direction).
     * @param amps Current in Amps (positive = forward torque)
     */
    void setBalanceCurrent(float amps);

    /**
     * Set linear velocity — forward/backward movement.
     * For navigation or manual control (phone joystick forward/back).
     * @param mps Velocity in m/s (positive = forward)
     */
    void setLinearVelocity(float mps);

    /**
     * Set turn rate — differential steering.
     * For navigation or manual control (phone joystick left/right).
     * @param radps Angular velocity in rad/s (positive = turn right)
     */
    void setTurnRate(float radps);

    /**
     * Apply combined velocities to motors + check safety.
     * MUST be called every loop iteration.
     * @param current_pitch_deg Current pitch from IMU (for safety cutoff)
     */
    void update(float current_pitch_deg);

    /** Check if ODrive has errors. Reads error registers. */
    bool hasError(int& err0, int& err1);

    /** Are motors in closed-loop control? */
    bool isEnabled();

    /** No errors, ready to enable */
    bool isReady();

    /** Get last commanded velocity for an axis (for telemetry) */
    float getAxisVelocity(uint8_t axis);

    /**
     * Full calibrate both axes (blocking ~30-60s).
     * Clears errors, runs calibration, saves pre_calibrated.
     * @return true if both axes calibrated successfully
     */
    bool calibrate();

    /** True if safety cutoff was triggered (auto-recovers when level) */
    bool isSafetyTripped();

private:
    bool enabled_;
    bool safetyTripped_;
    uint32_t recoverStartMs_;  // When pitch first dropped below recover threshold

    // Current/torque inputs (set independently, combined in update)
    float balanceCurrent_; // Amps — from PID (torque mode)
    float yawCurrent_;     // Amps — differential for turning

    // Last commanded per-wheel currents
    float leftCurrent_;    // Amps
    float rightCurrent_;   // Amps

    /** Combine balance + yaw currents and send to ODrive */
    void applyCurrents();

    /** Send current command for one axis with direction correction */
    void sendAxisCurrent(uint8_t axis, float amps);

    /** Run calibration on one axis (blocking) */
    bool calibrateAxis(uint8_t axis, uint32_t timeout_ms = 30000);
};

extern MotorController motors;
