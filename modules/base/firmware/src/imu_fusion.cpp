/**
 * IMU Sensor Fusion Implementation
 */

#include "imu_fusion.h"

// Global instance
IMUFusion imu;

bool IMUFusion::begin() {
    Serial.println("[IMU] Initializing MPU6050...");

    // Initialize MPU6050
    if (!mpu_.begin(kMPU6050Address, &Wire)) {
        Serial.println("[IMU] Failed to find MPU6050!");
        initialized_ = false;
        return false;
    }

    // Configure MPU6050 settings
    mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);  // ±8g range
    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);       // ±500°/s range
    mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);    // 21Hz low-pass filter

    // Initialize state
    pitchDeg_ = 0.0;
    rollDeg_ = 0.0;
    gyroXOffset_ = 0.0;
    gyroYOffset_ = 0.0;
    gyroZOffset_ = 0.0;
    accelXOffset_ = 0.0;
    accelYOffset_ = 0.0;
    lastUpdateMicros_ = micros();

    initialized_ = true;

    Serial.println("[IMU] MPU6050 initialized");
    Serial.println("[IMU] Calibrating... keep robot level!");

    // Auto-calibrate on startup
    calibrate();

    return true;
}

void IMUFusion::calibrate() {
    if (!initialized_) return;

    Serial.println("[IMU] Calibration starting (2 seconds)...");

    const int numSamples = 400;  // 2 seconds at 200Hz
    float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    float accelXSum = 0, accelYSum = 0;

    for (int i = 0; i < numSamples; i++) {
        sensors_event_t accel, gyro, temp;
        mpu_.getEvent(&accel, &gyro, &temp);

        gyroXSum += gyro.gyro.x;
        gyroYSum += gyro.gyro.y;
        gyroZSum += gyro.gyro.z;
        accelXSum += accel.acceleration.x;
        accelYSum += accel.acceleration.y;

        delay(5);  // 200Hz sampling
    }

    // Calculate averages (offsets)
    gyroXOffset_ = gyroXSum / numSamples;
    gyroYOffset_ = gyroYSum / numSamples;
    gyroZOffset_ = gyroZSum / numSamples;
    accelXOffset_ = accelXSum / numSamples;
    accelYOffset_ = accelYSum / numSamples;

    Serial.println("[IMU] Calibration complete!");
    Serial.printf("[IMU] Gyro offsets: X=%.4f, Y=%.4f, Z=%.4f rad/s\n",
                  gyroXOffset_, gyroYOffset_, gyroZOffset_);
    Serial.printf("[IMU] Accel offsets: X=%.2f, Y=%.2f m/s²\n",
                  accelXOffset_, accelYOffset_);

    // Initialize angles from accelerometer
    sensors_event_t accel, gyro, temp;
    mpu_.getEvent(&accel, &gyro, &temp);
    accelX_ = accel.acceleration.x - accelXOffset_;
    accelY_ = accel.acceleration.y - accelYOffset_;
    accelZ_ = accel.acceleration.z;

    pitchDeg_ = calculateAccelPitch();
    rollDeg_ = calculateAccelRoll();

    lastUpdateMicros_ = micros();
}

void IMUFusion::update() {
    if (!initialized_) return;

    // Read raw sensor data
    sensors_event_t accel, gyro, temp;
    mpu_.getEvent(&accel, &gyro, &temp);

    // Apply calibration offsets
    gyroX_ = gyro.gyro.x - gyroXOffset_;
    gyroY_ = gyro.gyro.y - gyroYOffset_;
    gyroZ_ = gyro.gyro.z - gyroZOffset_;
    accelX_ = accel.acceleration.x - accelXOffset_;
    accelY_ = accel.acceleration.y - accelYOffset_;
    accelZ_ = accel.acceleration.z;

    // Calculate time step
    uint32_t now = micros();
    float dt = (now - lastUpdateMicros_) / 1000000.0;  // Convert to seconds
    lastUpdateMicros_ = now;

    // Clamp dt to reasonable values (handle first call or overflow)
    if (dt <= 0 || dt > 0.1) {
        dt = 0.005;  // 5ms default (200Hz)
    }

    // Integrate gyro (rate → angle)
    // MPU6050: gyro.y = pitch rate (rad/s), convert to deg/s
    float gyro_pitch_rate_deg = gyroY_ * RAD_TO_DEG;
    float gyro_pitch_delta = gyro_pitch_rate_deg * dt;

    // Calculate instantaneous angle from accelerometer
    float accel_pitch = calculateAccelPitch();

    // Apply complementary filter
    pitchDeg_ = complementaryFilter(pitchDeg_ + gyro_pitch_delta, accel_pitch, dt);

    // Optional: Roll calculation (not critical for forward/backward balancing)
    float gyro_roll_rate_deg = gyroX_ * RAD_TO_DEG;
    float gyro_roll_delta = gyro_roll_rate_deg * dt;
    float accel_roll = calculateAccelRoll();
    rollDeg_ = complementaryFilter(rollDeg_ + gyro_roll_delta, accel_roll, dt);

    // Debug output (if enabled)
    if (kEnableIMUDebug) {
        Serial.printf("[IMU] Pitch: %.2f°, Roll: %.2f°, dt: %.4fs\n",
                      pitchDeg_, rollDeg_, dt);
    }
}

float IMUFusion::getPitchDeg() {
    return pitchDeg_;
}

float IMUFusion::getRollDeg() {
    return rollDeg_;
}

float IMUFusion::getPitchRateDegPerSec() {
    return gyroY_ * RAD_TO_DEG;
}

bool IMUFusion::isReady() {
    return initialized_;
}

float IMUFusion::calculateAccelPitch() {
    // Pitch from accelerometer (using atan2 for gravity vector)
    // Pitch = atan2(accelX, sqrt(accelY² + accelZ²))
    float pitch_rad = atan2(accelX_, sqrt(accelY_ * accelY_ + accelZ_ * accelZ_));
    return pitch_rad * RAD_TO_DEG;
}

float IMUFusion::calculateAccelRoll() {
    // Roll from accelerometer
    // Roll = atan2(accelY, accelZ)
    float roll_rad = atan2(accelY_, accelZ_);
    return roll_rad * RAD_TO_DEG;
}

float IMUFusion::complementaryFilter(float gyro_angle, float accel_angle, float dt) {
    // Complementary filter: alpha * gyro + (1-alpha) * accel
    // alpha = kComplementaryFilterAlpha (0.98 from config.h)
    //
    // High alpha (0.98) = trust gyro more (responsive, but drifts)
    // Low alpha (0.02) = trust accel more (stable, but slow/noisy)
    //
    // The gyro_angle is already integrated: previous_angle + gyro_rate*dt
    // So we just blend it with the instant accel measurement

    return kComplementaryFilterAlpha * gyro_angle +
           (1.0 - kComplementaryFilterAlpha) * accel_angle;
}
