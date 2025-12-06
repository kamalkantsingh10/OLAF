/**
 * ODrive UART Communication Handler
 * Base Module - ODrive Motor Controller Interface
 *
 * Communicates with ODrive v3.6 via UART using ASCII protocol.
 * Controls two hoverboard motors (Axis 0 = Left, Axis 1 = Right).
 *
 * Commands used:
 * - "w axis0.requested_state 8\n" - Enable motor (CLOSED_LOOP_CONTROL)
 * - "w axis0.requested_state 1\n" - Disable motor (IDLE)
 * - "v 0 2.5\n" - Set axis 0 velocity to 2.5 rev/s
 * - "r axis0.encoder.vel_estimate\n" - Read velocity feedback
 */

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"

class ODriveUART {
public:
    /**
     * Initialize UART connection to ODrive
     */
    void begin();

    /**
     * Enable both motors (CLOSED_LOOP_CONTROL mode)
     * @return true if successful
     */
    bool enableMotors();

    /**
     * Disable both motors (IDLE mode)
     */
    void disableMotors();

    /**
     * Set velocity for differential drive
     * @param linear_mps Linear velocity in m/s
     * @param angular_radps Angular velocity in rad/s
     */
    void setDifferentialVelocity(float linear_mps, float angular_radps);

    /**
     * Set raw velocity for one motor axis
     * @param axis Motor axis (0 = left, 1 = right)
     * @param velocity_revs Velocity in rev/s
     */
    void setAxisVelocity(uint8_t axis, float velocity_revs);

    /**
     * Emergency stop - immediately set all motors to 0 velocity
     */
    void emergencyStop();

    /**
     * Check if motors are enabled
     * @return true if in closed-loop mode
     */
    bool areMotorsEnabled();

    /**
     * Get estimated velocity for axis (requires reading from ODrive)
     * @param axis Motor axis (0 or 1)
     * @return Velocity in rev/s (requires parseResponse() call)
     */
    float getAxisVelocity(uint8_t axis);

    /**
     * Update - process any incoming UART data
     */
    void update();

private:
    HardwareSerial odriveSerial_;
    bool motorsEnabled_;
    float lastLinearVel_;
    float lastAngularVel_;

    // Current velocities (rev/s)
    float axis0Velocity_;
    float axis1Velocity_;

    /**
     * Send raw command string to ODrive
     * @param command ASCII command string (must end with \n)
     */
    void sendCommand(const char* command);

    /**
     * Send formatted velocity command
     * @param axis Axis number (0 or 1)
     * @param velocity Velocity in rev/s
     */
    void sendVelocityCommand(uint8_t axis, float velocity);

    /**
     * Convert linear/angular velocities to wheel velocities
     * @param linear_mps Linear velocity (m/s)
     * @param angular_radps Angular velocity (rad/s)
     * @param left_revs Output: left wheel velocity (rev/s)
     * @param right_revs Output: right wheel velocity (rev/s)
     */
    void differentialToWheels(float linear_mps, float angular_radps,
                               float& left_revs, float& right_revs);
};

// Global instance
extern ODriveUART odrive;
