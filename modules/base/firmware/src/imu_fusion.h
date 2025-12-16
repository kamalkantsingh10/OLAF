/**
 * IMU Sensor Fusion - MPU6050 with Complementary Filter
 * Base Module - Tilt Angle Sensing for Balancing
 *
 * Uses complementary filter to fuse gyroscope and accelerometer data:
 * - Gyroscope: Fast, accurate short-term, but drifts over time
 * - Accelerometer: Slow, noisy, but accurate long-term (gravity reference)
 * - Complementary filter: 98% gyro + 2% accel = best of both worlds
 *
 * Formula:
 *   angle = alpha * (angle + gyro*dt) + (1-alpha) * accel_angle
 *   where alpha = 0.98 (from config.h)
 */

#pragma once

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"

class IMUFusion {
public:
    /**
     * Initialize MPU6050 sensor
     * @return true if successful
     */
    bool begin();

    /**
     * Calibrate IMU (robot must be perfectly level)
     * Call this once at startup or when commanded
     */
    void calibrate();

    /**
     * Update sensor readings and apply complementary filter
     * Should be called at 200Hz (every 5ms)
     */
    void update();

    /**
     * Get current pitch angle (forward/backward tilt)
     * @return Pitch angle in degrees (positive = tilted forward)
     */
    float getPitchDeg();

    /**
     * Get current roll angle (left/right tilt)
     * @return Roll angle in degrees (positive = tilted right)
     */
    float getRollDeg();

    /**
     * Get gyro rate (for D-term in PID)
     * @return Pitch rate in degrees/second
     */
    float getPitchRateDegPerSec();

    /**
     * Check if IMU is initialized and working
     * @return true if ready
     */
    bool isReady();

private:
    Adafruit_MPU6050 mpu_;
    bool initialized_;

    // Current fused angles (degrees)
    float pitchDeg_;
    float rollDeg_;

    // Calibration offsets
    float gyroXOffset_;
    float gyroYOffset_;
    float gyroZOffset_;
    float accelXOffset_;
    float accelYOffset_;

    // Timing for integration
    uint32_t lastUpdateMicros_;

    // Raw sensor data
    float gyroX_, gyroY_, gyroZ_;  // rad/s
    float accelX_, accelY_, accelZ_;  // m/s²

    /**
     * Calculate pitch angle from accelerometer
     * @return Pitch from gravity vector (degrees)
     */
    float calculateAccelPitch();

    /**
     * Calculate roll angle from accelerometer
     * @return Roll from gravity vector (degrees)
     */
    float calculateAccelRoll();

    /**
     * Apply complementary filter
     * @param gyro_angle Integrated gyro angle
     * @param accel_angle Instantaneous accel angle
     * @param dt Time step (seconds)
     * @return Fused angle
     */
    float complementaryFilter(float gyro_angle, float accel_angle, float dt);
};

// Global instance
extern IMUFusion imu;
