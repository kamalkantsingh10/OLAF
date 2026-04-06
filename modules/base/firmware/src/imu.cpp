/**
 * BNO085 IMU Driver Implementation
 *
 * Uses Adafruit_BNO08x library for on-chip AHRS sensor fusion.
 * Outputs euler angles directly — no complementary filter needed.
 */

#include "imu.h"

// Global instance
IMU imu;

// Static instance pointer for ISR
IMU* IMU::instance_ = nullptr;

void IRAM_ATTR IMU::isrHandler() {
    if (instance_) {
        instance_->data_ready_ = true;
    }
}

bool IMU::begin(int8_t int_pin, int8_t rst_pin) {
    instance_ = this;
    int_pin_ = int_pin;

    // Initialize BNO085 on default Wire bus (must be initialized already)
    if (!bno_.begin_I2C(0x4A, &Wire, 0)) {
        Serial.println("[ERROR] Base IMU: BNO085 not found at 0x4A");
        return false;
    }

    Serial.println("[INFO] Base IMU: BNO085 found at 0x4A");

    if (!enableReports()) {
        Serial.println("[ERROR] Base IMU: Failed to enable sensor reports");
        return false;
    }

    Serial.println("[INFO] Base IMU: Rotation vector + gyro reports enabled at 200Hz");

    // Attach data-ready interrupt if INT pin specified
    if (int_pin_ >= 0) {
        attachInterrupt();
    }

    return true;
}

bool IMU::enableReports() {
    // Game rotation vector at 200Hz (5000µs interval) — no magnetometer needed
    // Better for balancing: lower latency, unaffected by motor magnetic interference
    if (!bno_.enableReport(SH2_GAME_ROTATION_VECTOR, 5000)) {
        Serial.println("[WARN] Base IMU: Could not enable game rotation vector report");
        return false;
    }

    // Gyro (calibrated) at 200Hz — for PID D-term (cleaner than differentiating position)
    if (!bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000)) {
        Serial.println("[WARN] Base IMU: Could not enable gyro report");
        // Non-fatal — PID can use position differentiation as fallback
    }

    return true;
}

void IMU::attachInterrupt() {
    if (int_pin_ < 0) return;

    pinMode(int_pin_, INPUT_PULLUP);
    ::attachInterrupt(digitalPinToInterrupt(int_pin_), isrHandler, FALLING);
    Serial.printf("[INFO] Base IMU: Data-ready interrupt on GPIO %d\n", int_pin_);
}

bool IMU::update() {
    bool got_data = false;

    // Read all available sensor events
    while (bno_.getSensorEvent(&sensor_value_)) {
        switch (sensor_value_.sensorId) {
            case SH2_GAME_ROTATION_VECTOR:
                qr_ = sensor_value_.un.gameRotationVector.real;
                qi_ = sensor_value_.un.gameRotationVector.i;
                qj_ = sensor_value_.un.gameRotationVector.j;
                qk_ = sensor_value_.un.gameRotationVector.k;
                accuracy_ = sensor_value_.status & 0x03;
                quaternionToEuler();
                got_data = true;
                break;

            case SH2_GYROSCOPE_CALIBRATED:
                gyro_x_ = sensor_value_.un.gyroscope.x;
                gyro_y_ = sensor_value_.un.gyroscope.y;
                gyro_z_ = sensor_value_.un.gyroscope.z;
                break;
        }
    }

    data_ready_ = false;
    return got_data;
}

void IMU::quaternionToEuler() {
    // Quaternion to euler (ZYX convention)
    // Pitch (Y-axis rotation) — primary axis for balancing
    float sinp = 2.0f * (qr_ * qj_ - qk_ * qi_);
    if (fabsf(sinp) >= 1.0f) {
        pitch_deg_ = copysignf(90.0f, sinp);  // Gimbal lock
    } else {
        pitch_deg_ = asinf(sinp) * RAD_TO_DEG;
    }

    // Roll (X-axis rotation)
    float sinr_cosp = 2.0f * (qr_ * qi_ + qj_ * qk_);
    float cosr_cosp = 1.0f - 2.0f * (qi_ * qi_ + qj_ * qj_);
    roll_deg_ = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Yaw (Z-axis rotation)
    float siny_cosp = 2.0f * (qr_ * qk_ + qi_ * qj_);
    float cosy_cosp = 1.0f - 2.0f * (qj_ * qj_ + qk_ * qk_);
    yaw_deg_ = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

void IMU::saveCalibration() {
    // Save Dynamic Calibration Data to BNO085 flash
    // Call after calibration accuracy reaches 3 (high)
    sh2_saveDcdNow();
    Serial.println("[INFO] Base IMU: Calibration saved to BNO085 flash");
}

void IMU::tare() {
    // Set current orientation as zero reference
    sh2_setTareNow(SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z, SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
    Serial.println("[INFO] Base IMU: Tare set — current orientation is now zero");
}
