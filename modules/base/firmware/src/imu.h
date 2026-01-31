/**
 * IMU Driver - Raw register access for MPU-6500/9250/9265
 *
 * Reads accel, gyro, and magnetometer (if present) via I2C.
 * No external library dependencies.
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

class IMU {
public:
    /**
     * Initialize IMU on specified I2C pins
     * @return true if sensor found and initialized
     */
    bool begin(uint8_t sda, uint8_t scl);

    /**
     * Read all sensor data
     * @return true if read successful
     */
    bool update();

    // Accelerometer (g)
    float getAccX() { return ax * acc_scale; }
    float getAccY() { return ay * acc_scale; }
    float getAccZ() { return az * acc_scale; }

    // Gyroscope (deg/s)
    float getGyroX() { return gx * gyro_scale; }
    float getGyroY() { return gy * gyro_scale; }
    float getGyroZ() { return gz * gyro_scale; }

    // Magnetometer (raw counts, only if hasMag)
    int16_t getMagX() { return mx; }
    int16_t getMagY() { return my; }
    int16_t getMagZ() { return mz; }

    bool hasMagnetometer() { return hasMag; }
    uint8_t getWhoAmI() { return whoami; }

private:
    // Raw sensor data
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;
    int16_t mx = 0, my = 0, mz = 0;

    // Conversion scales (default: ±2g, ±250°/s)
    const float acc_scale = 2.0 / 32768.0;
    const float gyro_scale = 250.0 / 32768.0;

    bool hasMag = false;
    uint8_t whoami = 0;

    // Register addresses
    static constexpr uint8_t MPU_ADDR = 0x68;
    static constexpr uint8_t PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t INT_PIN_CFG = 0x37;
    static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t AK8963_ADDR = 0x0C;
    static constexpr uint8_t AK8963_CNTL1 = 0x0A;
    static constexpr uint8_t AK8963_HXL = 0x03;
};

// Global instance
extern IMU imu;
