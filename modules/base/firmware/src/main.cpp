/**
 * OLAF Base Module - Main Firmware
 * ESP32-S3 Self-Balancing Controller
 *
 * Epic 6: Self-Balancing Base
 * Story 6.5: Base Module ESP32 Firmware
 *
 * Architecture:
 * - I2C Slave (0x0B) - Receives commands from Raspberry Pi
 * - UART1 (GPIO16/17) - Controls ODrive motor controller
 * - I2C Master (GPIO8/9) - Reads MPU6050 IMU
 * - 200Hz PID Loop - Self-balancing control
 *
 * Control Flow:
 * 1. setup(): Initialize all hardware modules
 * 2. loop(): Run 200Hz balancing loop + process I2C commands
 *
 * Author: Gilfoyle Bertram (Dev Agent)
 * Date: 2025-12-02
 */

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "i2c_slave.h"
#include "odrive_uart.h"
#include "imu_fusion.h"
#include "balancing_controller.h"

// Status LED control
uint32_t lastLedBlinkMs = 0;
bool ledState = false;

// Main loop timing
uint32_t lastLoopMicros = 0;
uint32_t loopCounter = 0;

// Statistics
uint32_t lastStatsMs = 0;
float averageLoopTimeUs = 0.0;
float maxLoopTimeUs = 0.0;

/**
 * Initialize all hardware modules
 */
void setup() {
    // Initialize USB serial for debugging
    Serial.begin(kSerialDebugBaudRate);
    delay(1000);  // Wait for serial to initialize

    Serial.println("\n\n========================================");
    Serial.println("  OLAF Base Module - Self-Balancing");
    Serial.println("  Epic 6: Story 6.5");
    Serial.println("========================================\n");

    // Initialize status LED
    pinMode(kStatusLedPin, OUTPUT);
    digitalWrite(kStatusLedPin, LOW);

    // Initialize I2C master for IMU (must be before i2cSlave.begin())
    Wire.begin(kI2CSdaPin, kI2CSclPin, kI2CFrequency);
    Serial.println("[INIT] I2C master initialized for IMU");

    // Initialize IMU (MPU6050)
    Serial.println("\n[INIT] Step 1: Initialize IMU...");
    if (!imu.begin()) {
        Serial.println("[INIT] FATAL: IMU initialization failed!");
        while (1) {
            // Blink LED rapidly to indicate error
            digitalWrite(kStatusLedPin, !digitalRead(kStatusLedPin));
            delay(100);
        }
    }

    // Initialize ODrive UART
    Serial.println("\n[INIT] Step 2: Initialize ODrive UART...");
    odrive.begin();

    // Initialize I2C slave for Pi communication
    Serial.println("\n[INIT] Step 3: Initialize I2C Slave...");
    i2cSlave.begin();
    i2cSlave.setStatus(STATUS_OK);

    // Initialize balancing controller
    Serial.println("\n[INIT] Step 4: Initialize Balancing Controller...");
    balancer.begin();

    // Initialization complete
    Serial.println("\n========================================");
    Serial.println("  ✅ Initialization Complete!");
    Serial.println("========================================\n");
    Serial.println("Waiting for commands from Raspberry Pi...");
    Serial.println("Send CMD_ENABLE_BALANCE (0x01) to start balancing\n");

    // Blink LED 3 times to indicate ready
    for (int i = 0; i < 3; i++) {
        digitalWrite(kStatusLedPin, HIGH);
        delay(200);
        digitalWrite(kStatusLedPin, LOW);
        delay(200);
    }

    lastLoopMicros = micros();
    lastStatsMs = millis();
}

/**
 * Main loop - runs continuously
 * Target: 200Hz (5ms period)
 */
void loop() {
    uint32_t loopStartMicros = micros();

    // ========================================================================
    // TIMING: Enforce 200Hz loop rate (5000 microseconds = 5ms)
    // ========================================================================
    uint32_t elapsedMicros = loopStartMicros - lastLoopMicros;

    if (elapsedMicros < kMainLoopIntervalMs * 1000) {
        // Too early, wait
        delayMicroseconds((kMainLoopIntervalMs * 1000) - elapsedMicros);
        loopStartMicros = micros();
        elapsedMicros = loopStartMicros - lastLoopMicros;
    }

    lastLoopMicros = loopStartMicros;

    // ========================================================================
    // STEP 1: Update IMU (read pitch angle)
    // ========================================================================
    imu.update();

    // ========================================================================
    // STEP 2: Process I2C commands from Raspberry Pi
    // ========================================================================
    uint8_t command = i2cSlave.getCommand();
    if (command != 0) {
        handleI2CCommand(command);
        i2cSlave.clearCommand();
    }

    // Update target velocity from I2C
    if (balancer.getMode() == MODE_BALANCING) {
        float linear_vel = i2cSlave.getLinearVelocity();
        float angular_vel = i2cSlave.getAngularVelocity();
        balancer.setTargetVelocity(linear_vel, angular_vel);
    }

    // ========================================================================
    // STEP 3: Run balancing PID loop
    // ========================================================================
    bool balancing_active = balancer.update();

    // Update status byte for Pi to read
    uint8_t status = STATUS_OK;
    if (balancing_active) {
        status |= STATUS_BALANCING | STATUS_MOTORS_ENABLED;
    }
    if (balancer.isEmergency()) {
        status |= STATUS_ERROR;
    }
    i2cSlave.setStatus(status);

    // ========================================================================
    // STEP 4: Update odometry and telemetry for Pi
    // ========================================================================
    // TODO: Calculate odometry from motor encoders
    // For now, just report current pitch
    i2cSlave.setPitch(imu.getPitchDeg());
    i2cSlave.setBatteryVoltage(36.0);  // TODO: Read actual voltage

    // ========================================================================
    // STEP 5: Update ODrive (process UART responses)
    // ========================================================================
    odrive.update();

    // ========================================================================
    // STEP 6: Status LED heartbeat
    // ========================================================================
    uint32_t now = millis();
    if (now - lastLedBlinkMs >= kLedBlinkIntervalMs) {
        lastLedBlinkMs = now;
        ledState = !ledState;
        digitalWrite(kStatusLedPin, ledState);
    }

    // ========================================================================
    // STATISTICS: Track loop timing
    // ========================================================================
    uint32_t loopEndMicros = micros();
    float loopTimeUs = loopEndMicros - loopStartMicros;

    averageLoopTimeUs = (averageLoopTimeUs * loopCounter + loopTimeUs) / (loopCounter + 1);
    if (loopTimeUs > maxLoopTimeUs) {
        maxLoopTimeUs = loopTimeUs;
    }
    loopCounter++;

    // Print statistics every second
    if (now - lastStatsMs >= 1000) {
        lastStatsMs = now;

        if (kEnableSerialDebug) {
            Serial.printf("[STATS] Loop rate: %d Hz, Avg: %.1f us, Max: %.1f us, Pitch: %.2f°\n",
                          loopCounter, averageLoopTimeUs, maxLoopTimeUs, imu.getPitchDeg());
        }

        loopCounter = 0;
        averageLoopTimeUs = 0.0;
        maxLoopTimeUs = 0.0;
    }

    // Warn if loop is running slow
    if (loopTimeUs > 6000) {  // > 6ms (should be 5ms)
        Serial.printf("[WARNING] Slow loop: %.1f us (target: 5000 us)\n", loopTimeUs);
    }
}

/**
 * Handle I2C commands from Raspberry Pi
 */
void handleI2CCommand(uint8_t command) {
    Serial.printf("[I2C] Received command: 0x%02X\n", command);

    switch (command) {
        case CMD_ENABLE_BALANCE:
            Serial.println("[I2C] Command: ENABLE_BALANCE");
            balancer.enable();
            break;

        case CMD_DISABLE_BALANCE:
            Serial.println("[I2C] Command: DISABLE_BALANCE");
            balancer.disable();
            break;

        case CMD_RESET:
            Serial.println("[I2C] Command: RESET");
            balancer.disable();
            balancer.resetPID();
            break;

        case CMD_CALIBRATE_IMU:
            Serial.println("[I2C] Command: CALIBRATE_IMU");
            balancer.disable();
            imu.calibrate();
            break;

        default:
            Serial.printf("[I2C] Unknown command: 0x%02X\n", command);
            break;
    }
}
