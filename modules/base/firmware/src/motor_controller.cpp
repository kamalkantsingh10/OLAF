/**
 * Motor Controller Implementation — Torque (Current) Control Mode
 *
 * PID outputs current (Amps) directly to ODrive via "c <axis> <amps>" command.
 * No intermediate velocity PID loop — direct torque for responsive balancing.
 */

#include "motor_controller.h"
#include "odrive_uart.h"

MotorController motors;

void MotorController::begin() {
    odriveUart.begin();

    enabled_ = false;
    safetyTripped_ = false;
    recoverStartMs_ = 0;
    balanceCurrent_ = 0.0f;
    yawCurrent_ = 0.0f;
    leftCurrent_ = 0.0f;
    rightCurrent_ = 0.0f;
}

bool MotorController::enable() {
    // Check for errors first
    int err0, err1;
    if (hasError(err0, err1)) {
        Serial.printf("[Motors] Cannot enable — errors: Axis0=%d Axis1=%d\n", err0, err1);
        return false;
    }

    // Switch to torque (current) control mode
    odriveUart.sendCommand(ODRIVE_SET_TORQUE_MODE_AXIS0);
    delay(50);
    odriveUart.sendCommand(ODRIVE_SET_INPUT_PASSTHROUGH_AXIS0);
    delay(50);
    odriveUart.sendCommand(ODRIVE_SET_TORQUE_MODE_AXIS1);
    delay(50);
    odriveUart.sendCommand(ODRIVE_SET_INPUT_PASSTHROUGH_AXIS1);
    delay(50);

    // Enter closed-loop control
    odriveUart.sendCommand(ODRIVE_ENABLE_AXIS0);
    delay(100);
    odriveUart.sendCommand(ODRIVE_ENABLE_AXIS1);
    delay(100);

    enabled_ = true;
    safetyTripped_ = false;
    Serial.println("[Motors] Enabled (torque/current mode)");
    return true;
}

void MotorController::disable() {
    // Zero current first
    balanceCurrent_ = 0.0f;
    yawCurrent_ = 0.0f;
    sendAxisCurrent(0, 0.0f);
    sendAxisCurrent(1, 0.0f);
    delay(50);

    odriveUart.sendCommand(ODRIVE_DISABLE_AXIS0);
    delay(50);
    odriveUart.sendCommand(ODRIVE_DISABLE_AXIS1);
    delay(50);

    enabled_ = false;
    Serial.println("[Motors] Disabled");
}

void MotorController::emergencyStop() {
    Serial.println("[Motors] EMERGENCY STOP");
    balanceCurrent_ = 0.0f;
    yawCurrent_ = 0.0f;
    sendAxisCurrent(0, 0.0f);
    sendAxisCurrent(1, 0.0f);
    disable();
}

void MotorController::setBalanceCurrent(float amps) {
    balanceCurrent_ = constrain(amps, kPIDOutputMin, kPIDOutputMax);
}

void MotorController::setLinearVelocity(float mps) {
    // In torque mode, linear velocity control requires an outer loop (future).
    // For now, this is a no-op. Navigation will add a drive PID later.
    (void)mps;
}

void MotorController::setTurnRate(float radps) {
    // Convert turn rate to differential current
    // Simple proportional: radps → amps differential
    float turn_current = radps * 0.5f;  // Tune this scale factor later
    yawCurrent_ = constrain(turn_current, -kMaxCurrentAmps, kMaxCurrentAmps);
}

void MotorController::update(float current_pitch_deg) {
    odriveUart.update();

    // Auto-recover
    if (safetyTripped_) {
        if (fabsf(current_pitch_deg) < kRecoverTiltDeg) {
            if (recoverStartMs_ == 0) {
                recoverStartMs_ = millis();
            } else if (millis() - recoverStartMs_ >= kRecoverHoldMs) {
                Serial.printf("[Motors] Auto-recover — pitch=%.1f within %.0f deg\n",
                              current_pitch_deg, kRecoverTiltDeg);
                safetyTripped_ = false;
                recoverStartMs_ = 0;
                enable();
            }
        } else {
            recoverStartMs_ = 0;
        }
        return;
    }

    if (!enabled_) return;

    // Safety cutoff
    if (fabsf(current_pitch_deg) > kMaxTiltDeg) {
        Serial.printf("[Motors] SAFETY — pitch=%.1f exceeds %.0f deg\n",
                      current_pitch_deg, kMaxTiltDeg);
        emergencyStop();
        safetyTripped_ = true;
        recoverStartMs_ = 0;
        return;
    }

    applyCurrents();
}

void MotorController::applyCurrents() {
    // Balance current same on both wheels + yaw differential
    float left = balanceCurrent_ + yawCurrent_;
    float right = balanceCurrent_ - yawCurrent_;

    // Clamp to safety limit
    left = constrain(left, -kMaxCurrentAmps, kMaxCurrentAmps);
    right = constrain(right, -kMaxCurrentAmps, kMaxCurrentAmps);

    leftCurrent_ = left;
    rightCurrent_ = right;

    sendAxisCurrent(kLeftMotorAxis, left);
    sendAxisCurrent(kRightMotorAxis, right);
}

void MotorController::sendAxisCurrent(uint8_t axis, float amps) {
    float dir = (axis == 0) ? kAxis0Direction : kAxis1Direction;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ODRIVE_CURRENT_CMD_FMT, axis, amps * dir);
    odriveUart.sendCommand(cmd);
}

bool MotorController::hasError(int& err0, int& err1) {
    err0 = (int)odriveUart.readFloat("r axis0.error\n");
    err1 = (int)odriveUart.readFloat("r axis1.error\n");
    return (err0 != 0 || err1 != 0);
}

bool MotorController::isEnabled() {
    return enabled_;
}

bool MotorController::isReady() {
    int err0, err1;
    return !hasError(err0, err1);
}

bool MotorController::isSafetyTripped() {
    return safetyTripped_;
}

float MotorController::getAxisVelocity(uint8_t axis) {
    // Now returns current (Amps), not velocity — name kept for telemetry compat
    if (axis == 0) return leftCurrent_;
    if (axis == 1) return rightCurrent_;
    return 0.0f;
}

bool MotorController::calibrateAxis(uint8_t axis, uint32_t timeout_ms) {
    char cmd[48];
    snprintf(cmd, sizeof(cmd), "w axis%d.requested_state 3\n", axis);
    odriveUart.sendCommand(cmd);

    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        delay(1000);
        char readCmd[48];
        snprintf(readCmd, sizeof(readCmd), "r axis%d.current_state\n", axis);
        float state = odriveUart.readFloat(readCmd, 1000);
        Serial.printf("[Motors] Axis %d state: %d\n", axis, (int)state);

        if ((int)state == 1) {
            snprintf(readCmd, sizeof(readCmd), "r axis%d.error\n", axis);
            float err = odriveUart.readFloat(readCmd, 1000);
            snprintf(readCmd, sizeof(readCmd), "r axis%d.motor.error\n", axis);
            float merr = odriveUart.readFloat(readCmd, 1000);
            snprintf(readCmd, sizeof(readCmd), "r axis%d.encoder.error\n", axis);
            float eerr = odriveUart.readFloat(readCmd, 1000);
            Serial.printf("[Motors] Axis %d: err=%d motor=%d enc=%d\n",
                          axis, (int)err, (int)merr, (int)eerr);
            return ((int)err == 0 && (int)merr == 0 && (int)eerr == 0);
        }
    }
    Serial.printf("[Motors] Axis %d calibration TIMEOUT\n", axis);
    return false;
}

bool MotorController::calibrate() {
    Serial.println("[Motors] Full calibration — wheels must be free!");

    for (uint8_t i = 0; i < 2; i++) {
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "w axis%d.error 0\n", i);
        odriveUart.sendCommand(cmd); delay(50);
        snprintf(cmd, sizeof(cmd), "w axis%d.motor.error 0\n", i);
        odriveUart.sendCommand(cmd); delay(50);
        snprintf(cmd, sizeof(cmd), "w axis%d.encoder.error 0\n", i);
        odriveUart.sendCommand(cmd); delay(50);
        snprintf(cmd, sizeof(cmd), "w axis%d.motor.config.pre_calibrated 0\n", i);
        odriveUart.sendCommand(cmd); delay(50);
        snprintf(cmd, sizeof(cmd), "w axis%d.encoder.config.pre_calibrated 0\n", i);
        odriveUart.sendCommand(cmd); delay(50);
    }

    for (uint8_t i = 0; i < 2; i++) {
        Serial.printf("[Motors] Calibrating Axis %d...\n", i);
        delay(1000);
        if (!calibrateAxis(i)) {
            Serial.printf("[Motors] Axis %d FAILED\n", i);
            return false;
        }
        Serial.printf("[Motors] Axis %d OK\n", i);
    }

    for (uint8_t i = 0; i < 2; i++) {
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "w axis%d.motor.config.pre_calibrated 1\n", i);
        odriveUart.sendCommand(cmd); delay(50);
        snprintf(cmd, sizeof(cmd), "w axis%d.encoder.config.pre_calibrated 1\n", i);
        odriveUart.sendCommand(cmd); delay(50);
    }
    odriveUart.sendCommand("ss\n"); delay(100);

    safetyTripped_ = false;
    Serial.println("[Motors] Calibration saved");
    return true;
}
