/**
 * OLAF Base Module Configuration
 *
 * Central configuration for all base module subsystems.
 * Pin assignments, constants, PID defaults, safety thresholds.
 * Values from modules/base/wiring.md and ODrive calibration.
 */

#pragma once

#include <Arduino.h>

// --- ODrive UART ---
constexpr uint8_t kODriveTxPin = ODRIVE_TX;   // GPIO 39 — ESP32 TX → ODrive RX
constexpr uint8_t kODriveRxPin = ODRIVE_RX;   // GPIO 38 — ESP32 RX ← ODrive TX
constexpr uint32_t kODriveBaudRate = ODRIVE_BAUD;  // 115200
constexpr uint32_t kStartupDelayMs = 2000;     // Wait for ODrive boot

// --- Motor physical parameters ---
constexpr float kWheelDiameterM = 0.17f;       // 6.5" hoverboard wheel
constexpr float kWheelCircumferenceM = kWheelDiameterM * PI;  // ~0.534m
constexpr float kWheelBaseM = 0.40f;           // Distance between wheel centers (m)

// --- Control mode ---
// Torque (current) control for balance — direct, no ODrive velocity PID fighting us
// ODrive ASCII: "c <axis> <current_amps>\n"
constexpr float kMaxCurrentAmps = 10.0f;       // Max current per motor (safety clamp)
constexpr float kMaxLinearVelocityMps = 1.0f;  // Max forward/backward speed (navigation/joystick)
constexpr float kMaxTurnRateRadps = 2.0f;      // Max turn rate (navigation/joystick)

// --- Motor axis mapping ---
constexpr uint8_t kLeftMotorAxis = 0;
constexpr uint8_t kRightMotorAxis = 1;

// --- Motor direction correction ---
// +1 = positive velocity = forward, -1 = flip direction
// Axis 0 spins backward with positive velocity (confirmed 2026-03-30)
constexpr float kAxis0Direction = -1.0f;
constexpr float kAxis1Direction =  1.0f;

// --- I2C1 Slave (Pi communication) ---
constexpr uint8_t kI2CSlaveAddress = 0x0B;    // Base module I2C address
constexpr uint8_t kI2CSdaPin = 21;             // J3 right header
constexpr uint8_t kI2CSclPin = 47;             // J3 right header
constexpr uint32_t kI2CFrequency = 400000;     // 400kHz fast mode

// --- PID balancing defaults (torque/current mode) ---
// Output is current (Amps) directly to motors. Values from mjbots hoverbot.
constexpr float kPitchKp = 0.2f;              // Conservative start — feel the response first
constexpr float kPitchKi = 0.0f;              // No integral yet
constexpr float kPitchKd = 0.05f;             // Damping
constexpr float kTargetPitchDeg = 1.5f;       // CG offset — adjust on slider once balancing works
constexpr float kPIDOutputMin = -kMaxCurrentAmps;
constexpr float kPIDOutputMax = kMaxCurrentAmps;
constexpr float kMaxIntegral = 5.0f;          // Anti-windup clamp

// --- PID loop timing ---
constexpr uint32_t kPIDIntervalUs = 5000;      // 5ms = 200Hz
constexpr float kPIDIntervalS = kPIDIntervalUs / 1000000.0f;

// --- Safety thresholds ---
constexpr float kMaxTiltDeg = 45.0f;           // Emergency stop if tilt exceeds this
constexpr float kRecoverTiltDeg = 30.0f;       // Auto-recover when pitch drops below this
constexpr uint32_t kRecoverHoldMs = 1000;      // Must stay below recover tilt for this long
constexpr uint32_t kWatchdogTimeoutMs = 2000;  // Watchdog timeout
constexpr uint32_t kI2CTimeoutMs = 5000;       // Pi communication timeout

// --- Debug flags ---
constexpr bool kEnableSerialDebug = true;
constexpr bool kEnableODriveEcho = false;
constexpr bool kEnablePIDDebug = false;        // Verbose PID output (disable for 200Hz)

// --- ODrive ASCII protocol commands ---
#define ODRIVE_ENABLE_AXIS0   "w axis0.requested_state 8\n"
#define ODRIVE_ENABLE_AXIS1   "w axis1.requested_state 8\n"
#define ODRIVE_DISABLE_AXIS0  "w axis0.requested_state 1\n"
#define ODRIVE_DISABLE_AXIS1  "w axis1.requested_state 1\n"
#define ODRIVE_VELOCITY_CMD_FMT "v %d %.3f\n"
#define ODRIVE_CURRENT_CMD_FMT  "c %d %.3f\n"

// Switch ODrive to torque (current) control mode before enabling
// control_mode 1 = torque control, input_mode 1 = passthrough
#define ODRIVE_SET_TORQUE_MODE_AXIS0 "w axis0.controller.config.control_mode 1\n"
#define ODRIVE_SET_TORQUE_MODE_AXIS1 "w axis1.controller.config.control_mode 1\n"
#define ODRIVE_SET_INPUT_PASSTHROUGH_AXIS0 "w axis0.controller.config.input_mode 1\n"
#define ODRIVE_SET_INPUT_PASSTHROUGH_AXIS1 "w axis1.controller.config.input_mode 1\n"
