/**
 * I2C Slave Communication Handler
 * Base Module - I2C Address 0x0B
 *
 * Handles communication with Raspberry Pi orchestrator.
 * Receives high-level commands and reports status/odometry.
 *
 * Register Map:
 * - 0x00: STATUS (R) - Module status byte
 * - 0x01: COMMAND (W) - Command register
 * - 0x10: LINEAR_VEL (W) - Target linear velocity (float, m/s)
 * - 0x11: ANGULAR_VEL (W) - Target angular velocity (float, rad/s)
 * - 0x20: ODOM_X (R) - Odometry X position (float, m)
 * - 0x21: ODOM_Y (R) - Odometry Y position (float, m)
 * - 0x22: ODOM_THETA (R) - Odometry heading (float, rad)
 * - 0x30: PITCH (R) - Current pitch angle (float, degrees)
 * - 0x31: BATTERY_VOLTAGE (R) - Battery voltage (float, V)
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// I2C Register Map
enum I2CRegister {
    REG_STATUS = 0x00,           // Module status (1 byte)
    REG_COMMAND = 0x01,          // Command register (1 byte)

    REG_LINEAR_VEL = 0x10,       // Target linear velocity (4 bytes float)
    REG_ANGULAR_VEL = 0x11,      // Target angular velocity (4 bytes float)

    REG_ODOM_X = 0x20,           // Odometry X (4 bytes float)
    REG_ODOM_Y = 0x21,           // Odometry Y (4 bytes float)
    REG_ODOM_THETA = 0x22,       // Odometry heading (4 bytes float)

    REG_PITCH = 0x30,            // Current pitch angle (4 bytes float)
    REG_BATTERY_VOLTAGE = 0x31   // Battery voltage (4 bytes float)
};

// Status byte bits
enum StatusBits {
    STATUS_OK = 0x00,
    STATUS_BALANCING = 0x01,     // Bit 0: Currently balancing
    STATUS_MOTORS_ENABLED = 0x02, // Bit 1: Motors enabled
    STATUS_ERROR = 0x80          // Bit 7: Error flag
};

// Command bytes
enum Commands {
    CMD_ENABLE_BALANCE = 0x01,   // Enable balancing mode
    CMD_DISABLE_BALANCE = 0x02,  // Disable balancing (relax)
    CMD_RESET = 0x03,            // Reset module
    CMD_CALIBRATE_IMU = 0x04     // Calibrate IMU
};

class I2CSlave {
public:
    /**
     * Initialize I2C slave on given pins and address
     */
    void begin();

    /**
     * Process I2C events (call from main loop)
     */
    void update();

    /**
     * Set status byte value
     */
    void setStatus(uint8_t status);

    /**
     * Get last received command
     * @return Command byte (0 if none)
     */
    uint8_t getCommand();

    /**
     * Clear command after processing
     */
    void clearCommand();

    /**
     * Get target linear velocity from Pi
     * @return Linear velocity in m/s
     */
    float getLinearVelocity();

    /**
     * Get target angular velocity from Pi
     * @return Angular velocity in rad/s
     */
    float getAngularVelocity();

    /**
     * Update odometry values (for Pi to read)
     */
    void setOdometry(float x, float y, float theta);

    /**
     * Update current pitch angle (for Pi to read)
     */
    void setPitch(float pitch_deg);

    /**
     * Update battery voltage (for Pi to read)
     */
    void setBatteryVoltage(float voltage);

private:
    static volatile uint8_t currentRegister_;
    static volatile uint8_t statusByte_;
    static volatile uint8_t commandByte_;
    static volatile float linearVelocity_;
    static volatile float angularVelocity_;
    static volatile float odomX_;
    static volatile float odomY_;
    static volatile float odomTheta_;
    static volatile float pitch_;
    static volatile float batteryVoltage_;

    // I2C interrupt handlers (must be static)
    static void onReceive(int numBytes);
    static void onRequest();

    // Helper to convert bytes to float
    static float bytesToFloat(uint8_t* bytes);
    static void floatToBytes(float value, uint8_t* bytes);
};

// Global instance
extern I2CSlave i2cSlave;
