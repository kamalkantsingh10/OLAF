/**
 * Balancing Controller Implementation
 */

#include "balancing_controller.h"

// Global instance
BalancingController balancer;

void BalancingController::begin() {
    mode_ = MODE_DISABLED;
    initialized_ = false;

    targetLinearVel_ = 0.0;
    targetAngularVel_ = 0.0;

    lastError_ = 0.0;
    integral_ = 0.0;
    lastUpdateMicros_ = micros();
    pidOutput_ = 0.0;

    emergencyTriggered_ = false;
    emergencyStartTime_ = 0;

    initialized_ = true;

    Serial.println("[Balancer] Controller initialized");
    Serial.printf("[Balancer] PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
                  kPitchKp, kPitchKi, kPitchKd);
}

void BalancingController::enable() {
    if (!initialized_) {
        Serial.println("[Balancer] ERROR: Not initialized!");
        return;
    }

    Serial.println("[Balancer] Enabling balancing mode...");

    // Reset PID state
    resetPID();

    // Enable motors via ODrive
    if (!odrive.enableMotors()) {
        Serial.println("[Balancer] ERROR: Failed to enable motors!");
        return;
    }

    mode_ = MODE_BALANCING;
    emergencyTriggered_ = false;

    Serial.println("[Balancer] Balancing ENABLED");
}

void BalancingController::disable() {
    Serial.println("[Balancer] Disabling balancing mode...");

    mode_ = MODE_DISABLED;

    // Stop motors
    odrive.disableMotors();

    Serial.println("[Balancer] Balancing DISABLED");
}

bool BalancingController::update() {
    if (!initialized_ || mode_ == MODE_DISABLED) {
        return false;
    }

    // Calculate time step
    uint32_t now = micros();
    float dt = (now - lastUpdateMicros_) / 1000000.0;  // Convert to seconds
    lastUpdateMicros_ = now;

    // Clamp dt to reasonable values
    if (dt <= 0 || dt > 0.02) {  // Max 20ms (50Hz minimum)
        dt = 0.005;  // 5ms default (200Hz)
    }

    // Get current pitch from IMU
    float current_pitch = imu.getPitchDeg();

    // Check for emergency conditions
    if (checkEmergency(current_pitch)) {
        handleEmergency();
        return false;
    }

    // Calculate PID output
    pidOutput_ = calculatePID(current_pitch, dt);

    // Add target velocity from Pi commands
    // (This allows Pi to command movement while balancing)
    float combined_linear = pidOutput_ + targetLinearVel_;
    float combined_angular = targetAngularVel_;

    // Send to motors via differential drive
    odrive.setDifferentialVelocity(combined_linear, combined_angular);

    // Debug output (if enabled)
    if (kEnablePIDDebug) {
        Serial.printf("[PID] Pitch: %.2f°, Error: %.2f°, Output: %.2f rev/s\n",
                      current_pitch, kTargetPitchDeg - current_pitch, pidOutput_);
    }

    return true;
}

void BalancingController::setTargetVelocity(float linear_mps, float angular_radps) {
    targetLinearVel_ = linear_mps;
    targetAngularVel_ = angular_radps;

    if (kEnableSerialDebug) {
        Serial.printf("[Balancer] Target velocity: linear=%.2f m/s, angular=%.2f rad/s\n",
                      linear_mps, angular_radps);
    }
}

BalanceMode BalancingController::getMode() {
    return mode_;
}

bool BalancingController::isEmergency() {
    return emergencyTriggered_;
}

void BalancingController::resetPID() {
    lastError_ = 0.0;
    integral_ = 0.0;
    pidOutput_ = 0.0;

    Serial.println("[Balancer] PID state reset");
}

float BalancingController::getPIDOutput() {
    return pidOutput_;
}

float BalancingController::calculatePID(float current_pitch, float dt) {
    // Calculate error (target - current)
    float error = kTargetPitchDeg - current_pitch;

    // Check if within deadband (ignore tiny errors)
    if (abs(error) < kBalanceDeadbandDeg) {
        error = 0.0;
    }

    // Proportional term
    float p_term = kPitchKp * error;

    // Integral term (accumulate error over time)
    integral_ += error * dt;

    // Anti-windup: Clamp integral to prevent runaway
    float max_integral = 10.0;  // Arbitrary limit (tune as needed)
    integral_ = constrain(integral_, -max_integral, max_integral);

    float i_term = kPitchKi * integral_;

    // Derivative term (rate of change of error)
    // Use gyro rate directly for better noise rejection
    float pitch_rate = imu.getPitchRateDegPerSec();
    float d_term = kPitchKd * (-pitch_rate);  // Negative because we want to oppose rate

    // Alternative: calculate derivative from error change
    // float derivative = (error - lastError_) / dt;
    // float d_term = kPitchKd * derivative;

    lastError_ = error;

    // Sum all terms
    float output = p_term + i_term + d_term;

    // Clamp output to motor limits
    output = constrain(output, kPIDOutputMin, kPIDOutputMax);

    return output;
}

bool BalancingController::checkEmergency(float pitch) {
    // Check if tilt exceeds safe threshold
    if (abs(pitch) > kMaxTiltDeg) {
        if (!emergencyTriggered_) {
            emergencyStartTime_ = millis();
            emergencyTriggered_ = true;
        }
        return true;
    }

    // TODO: Add battery voltage check
    // TODO: Add watchdog timeout check

    return false;
}

void BalancingController::handleEmergency() {
    if (mode_ != MODE_EMERGENCY_STOP) {
        Serial.println("[Balancer] ⚠ EMERGENCY STOP TRIGGERED!");
        Serial.printf("[Balancer] Pitch: %.2f° (max: %.2f°)\n",
                      imu.getPitchDeg(), kMaxTiltDeg);

        mode_ = MODE_EMERGENCY_STOP;

        // Emergency stop motors
        odrive.emergencyStop();

        // TODO: Deploy kickstand (if implemented)
    }
}
