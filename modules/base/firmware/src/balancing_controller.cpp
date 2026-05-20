/**
 * Balancing Controller Implementation
 *
 * PID loop using IMU pitch + gyro rate, outputs to MotorController.
 * Runs at 200Hz (5ms period).
 */

#include "balancing_controller.h"
#include "imu.h"
#include "motor_controller.h"
#include "velocity_controller.h"
#include "web_tuner.h"

// Tighter output clamp during auto-tune to prevent violent movements
constexpr float kAutoTuneOutputMax = 1.5f;  // 1.5 Amps max during auto-tune

// Global instance
BalancingController balancer;

void BalancingController::begin() {
    mode_ = MODE_DISABLED;
    initialized_ = false;

    targetLinearVel_ = 0.0f;
    targetAngularVel_ = 0.0f;

    lastError_ = 0.0f;
    integral_ = 0.0f;
    lastUpdateMicros_ = micros();
    pidOutput_ = 0.0f;

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

    // Enable motors via MotorController
    if (!motors.enable()) {
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
    pidOutput_ = 0.0f;
    motors.setBalanceCurrent(0.0f);
    motors.disable();

    Serial.println("[Balancer] Balancing DISABLED");
}

bool BalancingController::update() {
    if (!initialized_ || mode_ == MODE_DISABLED) {
        return false;
    }

    // Calculate time step
    uint32_t now = micros();
    float dt = (now - lastUpdateMicros_) / 1000000.0f;
    lastUpdateMicros_ = now;

    // Clamp dt to reasonable range
    if (dt <= 0.0f || dt > 0.02f) {
        dt = kPIDIntervalS;
    }

    // Get current pitch from IMU
    float current_pitch = imu.getPitch();

    // Check for emergency conditions
    if (checkEmergency(current_pitch)) {
        handleEmergency();
        return false;
    }

    // Calculate PID output
    pidOutput_ = calculatePID(current_pitch, dt);

    // Send balance velocity to motor controller
    motors.setBalanceCurrent(pidOutput_);

    // Apply navigation velocity targets (from Pi, when available)
    motors.setLinearVelocity(targetLinearVel_);
    motors.setTurnRate(targetAngularVel_);

    // Motor controller handles combining balance + nav + safety in its update()

    if (kEnablePIDDebug) {
        Serial.printf("[PID] Pitch: %.2f, Error: %.2f, Output: %.2f rev/s\n",
                      current_pitch, kTargetPitchDeg - current_pitch, pidOutput_);
    }

    return true;
}

void BalancingController::setTargetVelocity(float linear_mps, float angular_radps) {
    targetLinearVel_ = linear_mps;
    targetAngularVel_ = angular_radps;
}

BalanceMode BalancingController::getMode() {
    return mode_;
}

bool BalancingController::isEmergency() {
    return emergencyTriggered_;
}

void BalancingController::resetPID() {
    lastError_ = 0.0f;
    integral_ = 0.0f;
    pidOutput_ = 0.0f;
    lastUpdateMicros_ = micros();

    Serial.println("[Balancer] PID state reset");
}

float BalancingController::getPIDOutput() {
    return pidOutput_;
}

float BalancingController::calculatePID(float current_pitch, float dt) {
    // Use live-tunable gains from web UI
    // Positive pitch = leaning backward. Target ~4.2° = CG offset.
    // Velocity outer loop adds pitch offset to correct drift.
    float effectiveTarget = tuneTarget + velController.getPitchOffset();
    float error = effectiveTarget - current_pitch;

    // P term
    float p_term = tuneKp * error;

    // I term with anti-windup
    integral_ += error * dt;
    integral_ = constrain(integral_, -kMaxIntegral, kMaxIntegral);
    float i_term = tuneKi * integral_;

    // D term — opposes rate of tilt change
    float gyro_rate = imu.getGyroY();  // rad/s
    float d_term = tuneKd * (-gyro_rate);

    lastError_ = error;

    // Sum and clamp — tighter limit during auto-tune
    float output = p_term + i_term + d_term;
    float limit = webTuner.isAutoTuning() ? kAutoTuneOutputMax : kPIDOutputMax;
    output = constrain(output, -limit, limit);

    return output;
}

bool BalancingController::checkEmergency(float pitch) {
    if (fabsf(pitch) > kMaxTiltDeg) {
        if (!emergencyTriggered_) {
            emergencyStartTime_ = millis();
            emergencyTriggered_ = true;
        }
        return true;
    }
    return false;
}

void BalancingController::handleEmergency() {
    if (mode_ != MODE_EMERGENCY_STOP) {
        Serial.printf("[Balancer] EMERGENCY STOP — pitch=%.1f (max=%.0f)\n",
                      imu.getPitch(), kMaxTiltDeg);

        mode_ = MODE_EMERGENCY_STOP;
        pidOutput_ = 0.0f;
        motors.emergencyStop();
    }
}
