/**
 * IMU Driver Implementation - Raw register access
 */

#include "imu.h"

// Global instance
IMU imu;

bool IMU::begin(uint8_t sda, uint8_t scl) {
    Wire.begin(sda, scl);
    Wire.setClock(50000);  // 50kHz - very slow for 10K pull-ups

    // Check if MPU is present
    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    // Read WHO_AM_I
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
    if (Wire.available()) {
        whoami = Wire.read();
    }

    // Wake up MPU (clear sleep bit)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);

    // Enable I2C bypass to access magnetometer
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(INT_PIN_CFG);
    Wire.write(0x02);
    Wire.endTransmission();
    delay(10);

    // Check for magnetometer (AK8963)
    Wire.beginTransmission(AK8963_ADDR);
    if (Wire.endTransmission() == 0) {
        hasMag = true;

        // Set continuous measurement mode (16-bit, 100Hz)
        Wire.beginTransmission(AK8963_ADDR);
        Wire.write(AK8963_CNTL1);
        Wire.write(0x16);
        Wire.endTransmission();
    }

    return true;
}

bool IMU::update() {
    // Read accel + gyro (14 bytes)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) {
        return false;
    }

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // Skip temp
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();

    // Read magnetometer if present
    if (hasMag) {
        Wire.beginTransmission(AK8963_ADDR);
        Wire.write(AK8963_HXL);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)7);

        mx = Wire.read() | (Wire.read() << 8);  // Little-endian
        my = Wire.read() | (Wire.read() << 8);
        mz = Wire.read() | (Wire.read() << 8);
        Wire.read();  // Status2
    }

    delay(500);
    return true;
}
