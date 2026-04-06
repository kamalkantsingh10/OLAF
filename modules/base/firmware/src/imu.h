/**
 * BNO085 IMU Driver — On-chip AHRS (Hillcrest SH-2)
 *
 * Outputs euler angles and quaternions directly — no complementary filter needed.
 * I2C address: 0x4A on I2C0 (GPIO 4/5).
 * INT pin (GPIO 6) signals data-ready for 200Hz operation.
 * RST pin (GPIO 7) for hardware reset recovery.
 *
 * Library: Adafruit_BNO08x
 */

#pragma once

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

class IMU {
public:
    /**
     * Initialize BNO085 on I2C bus (must call Wire.begin() first).
     * @param int_pin GPIO for data-ready interrupt (active low), -1 to disable
     * @param rst_pin GPIO for hardware reset (active low), -1 to disable
     * @return true if sensor found and rotation vector report enabled
     */
    bool begin(int8_t int_pin = BNO085_INT, int8_t rst_pin = BNO085_RST);

    /**
     * Check for new sensor data and update euler angles.
     * @return true if new data was available and read
     */
    bool update();

    // Euler angles from on-chip AHRS (degrees)
    float getPitch() const { return pitch_deg_; }
    float getRoll() const { return roll_deg_; }
    float getYaw() const { return yaw_deg_; }

    // Gyro rate (rad/s) — useful for PID D-term
    float getGyroX() const { return gyro_x_; }
    float getGyroY() const { return gyro_y_; }
    float getGyroZ() const { return gyro_z_; }

    // Quaternion (if needed)
    float getQR() const { return qr_; }
    float getQI() const { return qi_; }
    float getQJ() const { return qj_; }
    float getQK() const { return qk_; }

    // Calibration accuracy (0=unreliable, 1=low, 2=medium, 3=high)
    uint8_t getAccuracy() const { return accuracy_; }

    // Save current calibration to BNO085 flash
    void saveCalibration();

    // Tare — set current orientation as zero reference
    void tare();

    // True if data-ready interrupt fired (cleared after update)
    volatile bool dataReady() const { return data_ready_; }

    // Attach ISR to INT pin for interrupt-driven reads
    void attachInterrupt();

private:
    Adafruit_BNO08x bno_;
    sh2_SensorValue_t sensor_value_;

    // Euler angles (degrees)
    float pitch_deg_ = 0.0f;
    float roll_deg_ = 0.0f;
    float yaw_deg_ = 0.0f;

    // Quaternion components
    float qr_ = 1.0f, qi_ = 0.0f, qj_ = 0.0f, qk_ = 0.0f;

    // Gyro rates (rad/s)
    float gyro_x_ = 0.0f, gyro_y_ = 0.0f, gyro_z_ = 0.0f;

    uint8_t accuracy_ = 0;
    int8_t int_pin_ = -1;

    volatile bool data_ready_ = false;

    bool enableReports();
    void quaternionToEuler();

    static void IRAM_ATTR isrHandler();
    static IMU* instance_;  // For ISR callback
};

// Global instance
extern IMU imu;
