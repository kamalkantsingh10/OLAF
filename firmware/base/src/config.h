/**
 * Base Module Configuration - ESP32-S3 Pin Definitions
 * Story 6.3: ODrive Motor Controller Setup
 *
 * Hardware: ESP32-S3-DevKitC-1 (ESP32-S3-WROOM-2 N16R8)
 * Purpose: Self-balancing base with ODrive motor control
 *
 * This file defines all GPIO pin assignments, I2C configuration,
 * UART settings for ODrive communication, and motor control parameters.
 */

#pragma once

#include <Arduino.h>

// ============================================================================
// MODULE IDENTIFICATION
// ============================================================================

constexpr uint8_t kI2CSlaveAddress = 0x0B;  // Base module I2C address
constexpr char kModuleName[] = "Base";

// ============================================================================
// UART CONFIGURATION (ODrive Communication)
// ============================================================================

// ODrive UART pins (ESP32-S3 UART2)
constexpr uint8_t kODriveRxPin = 16;  // GPIO16 → ODrive GPIO1 (TX)
constexpr uint8_t kODriveTxPin = 17;  // GPIO17 → ODrive GPIO2 (RX)
constexpr uint32_t kODriveBaudRate = 115200;

// ODrive protocol settings
constexpr uint32_t kODriveCommandTimeoutMs = 100;  // Timeout for ODrive responses
constexpr uint8_t kODriveMaxRetries = 3;           // Max command retry attempts

// ============================================================================
// I2C CONFIGURATION (Raspberry Pi Communication)
// ============================================================================

// I2C pins (ESP32-S3 default I2C bus)
constexpr uint8_t kI2CSdaPin = 8;   // GPIO8 - I2C Data
constexpr uint8_t kI2CSclPin = 9;   // GPIO9 - I2C Clock
constexpr uint32_t kI2CFrequency = 400000;  // 400kHz (fast mode)

// I2C buffer size
constexpr size_t kI2CBufferSize = 32;  // Bytes for I2C register communication

// ============================================================================
// IMU CONFIGURATION (MPU6050)
// ============================================================================

// MPU6050 6-axis IMU (gyro + accel) - per architecture docs
// Shares I2C bus with Pi (separate I2C address)
constexpr uint8_t kMPU6050Address = 0x68;  // MPU6050 default address (AD0=LOW)
constexpr uint16_t kIMUUpdateRateHz = 200;  // 200Hz PID loop frequency
constexpr uint32_t kIMUUpdateIntervalMs = 1000 / kIMUUpdateRateHz;  // 5ms

// Complementary filter coefficient (gyro vs accel fusion)
constexpr float kComplementaryFilterAlpha = 0.98;  // 98% gyro, 2% accel

// ============================================================================
// MOTOR CONTROL PARAMETERS
// ============================================================================

// Wheel and chassis dimensions
constexpr float kWheelRadiusM = 0.0825;      // 6.5" wheels = 8.25cm radius
constexpr float kWheelBaseM = 0.30;          // Distance between wheels (30cm)
constexpr float kWheelCircumferenceM = 2.0 * PI * kWheelRadiusM;  // ~0.518m

// Velocity limits (rev/s)
constexpr float kMaxVelocityRevS = 10.0;     // Max motor speed (10 rev/s = 600 RPM)
constexpr float kMaxLinearVelocityMps = 2.0; // Max linear speed (2 m/s)
constexpr float kMaxAngularVelocityRadps = 2.0;  // Max turn rate (2 rad/s)

// Acceleration limits (rev/s²)
constexpr float kMaxAccelerationRevS2 = 20.0;  // Prevent jerky starts
constexpr float kRampRateRevSPerMs = kMaxAccelerationRevS2 / 1000.0;  // Per millisecond

// ============================================================================
// BALANCING CONTROL (PID Parameters)
// ============================================================================

// PID gains (initial values - will be tuned in Story 6.7)
constexpr float kPitchKp = 20.0;  // Proportional gain
constexpr float kPitchKi = 0.0;   // Integral gain (initially disabled)
constexpr float kPitchKd = 5.0;   // Derivative gain

// Balancing constraints
constexpr float kTargetPitchDeg = 0.0;      // Target angle (vertical)
constexpr float kMaxTiltDeg = 45.0;         // Emergency stop threshold
constexpr float kBalanceDeadbandDeg = 0.5;  // Ignore tiny oscillations

// PID output limits
constexpr float kPIDOutputMin = -kMaxVelocityRevS;
constexpr float kPIDOutputMax = kMaxVelocityRevS;

// ============================================================================
// KICKSTAND SERVO CONFIGURATION
// ============================================================================

// Kickstand servo (Feetech STS3215 via shared neck servo controller)
constexpr uint8_t kKickstandServoID = 4;     // Servo ID on bus
constexpr uint16_t kKickstandDeployedPos = 2048;   // Position when down
constexpr uint16_t kKickstandRetractedPos = 1024;  // Position when up
constexpr uint16_t kKickstandServoSpeed = 2000;    // Movement speed

// Kickstand trigger logic
constexpr float kKickstandDeployTiltDeg = 30.0;  // Auto-deploy if tilt exceeds this

// ============================================================================
// WATCHDOG TIMER
// ============================================================================

constexpr uint32_t kWatchdogTimeoutSec = 2;  // 2 second timeout
// Watchdog protects against firmware hangs during balancing
// If PID loop hangs, motors must stop immediately

// ============================================================================
// STATUS LED (Optional Debug LED)
// ============================================================================

constexpr uint8_t kStatusLedPin = 48;  // Built-in LED on ESP32-S3-DevKitC-1
constexpr uint32_t kLedBlinkIntervalMs = 500;  // Heartbeat blink rate

// ============================================================================
// ODRIVE ASCII COMMAND TEMPLATES
// ============================================================================

// Pre-formatted command strings for ODrive UART protocol
// Use with sprintf() or String concatenation

// Enable motors (CLOSED_LOOP_CONTROL = state 8)
#define ODRIVE_ENABLE_AXIS0 "w axis0.requested_state 8\n"
#define ODRIVE_ENABLE_AXIS1 "w axis1.requested_state 8\n"

// Disable motors (IDLE = state 1)
#define ODRIVE_DISABLE_AXIS0 "w axis0.requested_state 1\n"
#define ODRIVE_DISABLE_AXIS1 "w axis1.requested_state 1\n"

// Velocity command template: "v <axis> <velocity>\n"
// Example: sprintf(cmd, "v 0 %.2f\n", velocity);
#define ODRIVE_VELOCITY_CMD_FMT "v %d %.2f\n"

// Read velocity estimate: "r axis0.encoder.vel_estimate\n"
#define ODRIVE_READ_VEL0 "r axis0.encoder.vel_estimate\n"
#define ODRIVE_READ_VEL1 "r axis1.encoder.vel_estimate\n"

// Emergency stop (all motors idle)
#define ODRIVE_EMERGENCY_STOP "w axis0.requested_state 1\nw axis1.requested_state 1\n"

// ============================================================================
// MOTOR AXIS ASSIGNMENTS
// ============================================================================

constexpr uint8_t kLeftMotorAxis = 0;   // ODrive Axis 0 = Left motor
constexpr uint8_t kRightMotorAxis = 1;  // ODrive Axis 1 = Right motor

// ============================================================================
// TIMING CONSTANTS
// ============================================================================

constexpr uint32_t kSerialDebugBaudRate = 115200;  // USB serial for debugging
constexpr uint32_t kStartupDelayMs = 2000;         // Wait for ODrive boot
constexpr uint32_t kMainLoopRateHz = 200;          // Match IMU update rate
constexpr uint32_t kMainLoopIntervalMs = 1000 / kMainLoopRateHz;  // 5ms

// ============================================================================
// POWER SYSTEM CONFIGURATION
// ============================================================================

// Battery and power rails
constexpr float kBatteryNominalVoltageV = 36.0;   // 10S Li-ion (salvaged hoverboard)
constexpr float kBatteryCutoffVoltageV = 30.0;    // Emergency shutdown (3.0V/cell)
constexpr float kBatteryWarningVoltageV = 32.0;   // Low battery warning (3.2V/cell)
constexpr float kBatteryFullVoltageV = 42.0;      // Fully charged (4.2V/cell)

// Buck converter outputs
constexpr float k12VRailVoltageV = 12.0;  // Buck 36V→12V @ 10A (servos, peripherals)
constexpr float k5VRailVoltageV = 5.0;    // Buck 36V→5V @ 10A (ESP32, IMU, logic)

// Current limits
constexpr float kMotorCurrentLimitA = 20.0;       // Max current per motor
constexpr float k12VRailCurrentLimitA = 10.0;     // Buck converter limit
constexpr float k5VRailCurrentLimitA = 10.0;      // Buck converter limit

// ============================================================================
// DEBUG FLAGS (Set to false for production)
// ============================================================================

constexpr bool kEnableSerialDebug = true;    // Print debug messages to USB serial
constexpr bool kEnableODriveEcho = false;    // Echo ODrive responses (verbose)
constexpr bool kEnableIMUDebug = false;      // Print IMU readings every loop
constexpr bool kEnablePIDDebug = false;      // Print PID calculations

// ============================================================================
// COMPILE-TIME CHECKS
// ============================================================================

// Verify critical parameters at compile time
static_assert(kIMUUpdateRateHz == kMainLoopRateHz,
              "IMU and main loop rates must match for real-time control");
static_assert(kMaxVelocityRevS > 0, "Max velocity must be positive");
static_assert(kWheelBaseM > 0, "Wheel base must be positive");
static_assert(kWatchdogTimeoutSec >= 1, "Watchdog timeout too short");

// ============================================================================
// END OF CONFIGURATION
// ============================================================================
