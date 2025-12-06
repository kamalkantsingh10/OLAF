/**
 * ODrive UART Communication Handler Implementation
 */

#include "odrive_uart.h"

// Global instance
ODriveUART odrive;

void ODriveUART::begin() {
    // Initialize UART2 for ODrive communication
    odriveSerial_.begin(kODriveBaudRate, SERIAL_8N1, kODriveRxPin, kODriveTxPin);

    motorsEnabled_ = false;
    lastLinearVel_ = 0.0;
    lastAngularVel_ = 0.0;
    axis0Velocity_ = 0.0;
    axis1Velocity_ = 0.0;

    Serial.println("[ODrive] UART initialized");
    Serial.printf("[ODrive] RX: GPIO%d, TX: GPIO%d, Baud: %d\n",
                  kODriveRxPin, kODriveTxPin, kODriveBaudRate);

    // Wait for ODrive to boot
    delay(kStartupDelayMs);
}

bool ODriveUART::enableMotors() {
    Serial.println("[ODrive] Enabling motors...");

    // Send enable commands
    sendCommand(ODRIVE_ENABLE_AXIS0);
    delay(100);
    sendCommand(ODRIVE_ENABLE_AXIS1);
    delay(100);

    motorsEnabled_ = true;

    Serial.println("[ODrive] Motors enabled");
    return true;
}

void ODriveUART::disableMotors() {
    Serial.println("[ODrive] Disabling motors...");

    // Stop motors first
    setAxisVelocity(0, 0.0);
    setAxisVelocity(1, 0.0);
    delay(100);

    // Send disable commands
    sendCommand(ODRIVE_DISABLE_AXIS0);
    delay(50);
    sendCommand(ODRIVE_DISABLE_AXIS1);
    delay(50);

    motorsEnabled_ = false;

    Serial.println("[ODrive] Motors disabled");
}

void ODriveUART::setDifferentialVelocity(float linear_mps, float angular_radps) {
    lastLinearVel_ = linear_mps;
    lastAngularVel_ = angular_radps;

    // Convert to wheel velocities
    float left_revs, right_revs;
    differentialToWheels(linear_mps, angular_radps, left_revs, right_revs);

    // Clamp to maximum velocity
    left_revs = constrain(left_revs, -kMaxVelocityRevS, kMaxVelocityRevS);
    right_revs = constrain(right_revs, -kMaxVelocityRevS, kMaxVelocityRevS);

    // Send to motors
    setAxisVelocity(kLeftMotorAxis, left_revs);
    setAxisVelocity(kRightMotorAxis, right_revs);

    axis0Velocity_ = left_revs;
    axis1Velocity_ = right_revs;
}

void ODriveUART::setAxisVelocity(uint8_t axis, float velocity_revs) {
    // Clamp velocity
    velocity_revs = constrain(velocity_revs, -kMaxVelocityRevS, kMaxVelocityRevS);

    // Send velocity command
    sendVelocityCommand(axis, velocity_revs);

    // Store for reference
    if (axis == 0) {
        axis0Velocity_ = velocity_revs;
    } else if (axis == 1) {
        axis1Velocity_ = velocity_revs;
    }
}

void ODriveUART::emergencyStop() {
    Serial.println("[ODrive] EMERGENCY STOP!");

    // Immediately set all velocities to zero
    setAxisVelocity(0, 0.0);
    setAxisVelocity(1, 0.0);

    // Optionally disable motors
    // disableMotors();
}

bool ODriveUART::areMotorsEnabled() {
    return motorsEnabled_;
}

float ODriveUART::getAxisVelocity(uint8_t axis) {
    // Return last commanded velocity
    // TODO: Implement actual velocity reading from ODrive
    if (axis == 0) {
        return axis0Velocity_;
    } else if (axis == 1) {
        return axis1Velocity_;
    }
    return 0.0;
}

void ODriveUART::update() {
    // Process any incoming UART data from ODrive
    while (odriveSerial_.available()) {
        char c = odriveSerial_.read();

        // TODO: Parse responses if needed
        // For now, just discard (ASCII protocol sends confirmations)
        if (kEnableODriveEcho) {
            Serial.print(c);
        }
    }
}

void ODriveUART::sendCommand(const char* command) {
    odriveSerial_.print(command);

    if (kEnableSerialDebug) {
        Serial.printf("[ODrive TX] %s", command);
    }
}

void ODriveUART::sendVelocityCommand(uint8_t axis, float velocity) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), ODRIVE_VELOCITY_CMD_FMT, axis, velocity);
    sendCommand(cmd);
}

void ODriveUART::differentialToWheels(float linear_mps, float angular_radps,
                                       float& left_revs, float& right_revs) {
    // Convert linear m/s to wheel rev/s
    float base_vel_revs = linear_mps / kWheelCircumferenceM;

    // Calculate differential from turning rate
    // angular_radps * wheelbase/2 gives the differential linear speed at each wheel
    float diff_mps = (angular_radps * kWheelBaseM / 2.0);
    float diff_revs = diff_mps / kWheelCircumferenceM;

    // Left wheel gets base + differential
    // Right wheel gets base - differential
    // (Positive angular = turn right, so left wheel goes faster)
    left_revs = base_vel_revs + diff_revs;
    right_revs = base_vel_revs - diff_revs;
}
