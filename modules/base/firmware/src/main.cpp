/**
 * OLAF Base Module — Main Application
 *
 * Boot sequence, state machine, serial/web commands.
 * Delegates to: DisplayDriver (OLED), MotorController (ODrive),
 *               IMU (BNO085), BalancingController (PID), WebTuner.
 *
 * Control via web dashboard at http://olaf-base.local
 * Serial commands: CALIBRATE | BALANCE | TARE | STOP
 */

#include <Arduino.h>
#include <Wire.h>
#include <OLAFota.h>
#include "imu.h"
#include "motor_controller.h"
#include "display_driver.h"
#include "balancing_controller.h"
#include "web_tuner.h"

#define OTA_HOSTNAME "olaf-base"
#define OTA_PASSWORD "olaf-base-ota-2024"

constexpr uint8_t OLED_ADDR = 0x3C;

// OLED refresh throttle — 10Hz
constexpr uint32_t OLED_REFRESH_MS = 100;
uint32_t last_oled_update = 0;

// Application state
AppState appState = STATE_IDLE;

// Telemetry throttle — 20Hz
uint32_t lastTelemetryPrint = 0;
constexpr uint32_t kTelemetryPrintMs = 50;

// PID loop timing
uint32_t lastPIDLoop = 0;

// Serial command buffer
char serialCmdBuf[32];
uint8_t serialCmdIdx = 0;

// --- Shared command handlers (used by serial and web) ---

void startBalance() {
    Serial.println("[BALANCE] Starting PID balancing");
    Serial.printf("[BALANCE] Kp=%.1f, Ki=%.2f, Kd=%.1f\n", tuneKp, tuneKi, tuneKd);
    oled.showMessage("BALANCE", "PID active");
    balancer.enable();
    appState = STATE_BALANCING;
    lastPIDLoop = micros();
    lastTelemetryPrint = millis();
}

void stopBalance() {
    balancer.disable();
    motors.disable();
    appState = STATE_IDLE;
    Serial.println("[STOP] Motors off");
    oled.showMessage("STOPPED", "Motors off");
}

void handleSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            serialCmdBuf[serialCmdIdx] = '\0';
            if (strcmp(serialCmdBuf, "CALIBRATE") == 0) {
                oled.showMessage("CALIBRATING", "Stand clear!");
                delay(2000);
                bool ok = motors.calibrate();
                if (ok) {
                    oled.showMessage("CAL OK", "Saved!");
                    appState = STATE_IDLE;
                } else {
                    oled.showMessage("CAL FAILED!", "Check motors");
                    appState = STATE_ERROR;
                }
                delay(2000);
            } else if (strcmp(serialCmdBuf, "BALANCE") == 0) {
                startBalance();
            } else if (strcmp(serialCmdBuf, "TARE") == 0) {
                imu.tare();
                oled.showMessage("TARE", "Pitch zeroed");
                delay(1000);
            } else if (strcmp(serialCmdBuf, "STOP") == 0) {
                stopBalance();
            }
            serialCmdIdx = 0;
        } else if (serialCmdIdx < sizeof(serialCmdBuf) - 1) {
            serialCmdBuf[serialCmdIdx++] = c;
        }
    }
}

void handleWebCommands() {
    WebCommand cmd = webTuner.getCommand();
    switch (cmd) {
        case WEB_CMD_BALANCE: startBalance(); break;
        case WEB_CMD_STOP:    stopBalance(); break;
        case WEB_CMD_TARE:
            imu.tare();
            oled.showMessage("TARE", "Pitch zeroed");
            break;
        case WEB_CMD_AUTOTUNE:
            // Auto-tune needs balancing to be active
            if (appState != STATE_BALANCING) {
                startBalance();
            }
            oled.showMessage("AUTO-TUNE", "Hold steady!");
            break;
        default: break;
    }
}

void setup() {
    delay(500);

    Serial.begin(115200);
    Serial.println("[INFO] Base: Starting...");

    // 1. OTA
    OLAFota::begin(OTA_HOSTNAME, OTA_PASSWORD);

    // 2. I2C bus for IMU + OLED
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    // 3. OLED display
    if (oled.begin(128, 32, OLED_ADDR)) {
        oled.showSplash("OLAF", "Base");
        delay(1000);
    }

    // I2C scan
    oled.showMessage("I2C Scan...");
    Serial.println("[INFO] Base: I2C scan:");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            const char* label = "???";
            if (addr == OLED_ADDR) label = "OLED";
            else if (addr == 0x4A) label = "IMU";
            Serial.printf("  0x%02X %s\n", addr, label);
            found++;
        }
    }
    Serial.printf("[INFO] Base: %d device(s) found\n", found);
    delay(1000);

    // 4. BNO085 IMU
    oled.showMessage("IMU init...");
    if (!imu.begin(BNO085_INT, BNO085_RST)) {
        oled.showMessage("IMU FAIL!", "Check wiring");
        Serial.println("[ERROR] Base: IMU init failed — halting (OTA still active)");
        while (1) {
            OLAFota::handle();
            delay(1000);
        }
    }

    oled.showMessage("IMU OK", "BNO085 0x4A");
    Serial.println("[INFO] Base: BNO085 initialized");
    delay(1000);

    // 5. Motor controller
    oled.showMessage("Motors init...");
    motors.begin();

    // 6. Balancing controller
    balancer.begin();

    // 7. Web tuner dashboard
    webTuner.begin();

    // 8. Check for motor errors
    int err0, err1;
    if (motors.hasError(err0, err1)) {
        char buf[32];
        snprintf(buf, sizeof(buf), "E0:%d E1:%d", err0, err1);
        oled.showMessage("MOTOR ERROR", buf);
        Serial.printf("[ERROR] Motor errors — Axis0:%d Axis1:%d\n", err0, err1);
        Serial.println("[INFO] Send 'CALIBRATE' or use web UI");
        appState = STATE_ERROR;
    } else {
        oled.showMessage("Motors OK", "olaf-base.local");
        Serial.println("[INFO] Motors OK — use web UI or serial to start");
        appState = STATE_IDLE;
    }
}

void loop() {
    OLAFota::handle();
    if (OLAFota::isUpdating()) return;

    // Web tuner — handle HTTP + WebSocket
    webTuner.update();

    // Read IMU
    imu.update();
    float pitch = imu.getPitch();
    float roll = imu.getRoll();
    uint8_t accuracy = imu.getAccuracy();

    // Handle commands from serial and web
    handleSerialCommands();
    handleWebCommands();

    uint32_t now = millis();

    // PID balancing at 200Hz
    if (appState == STATE_BALANCING) {
        uint32_t now_us = micros();
        if (now_us - lastPIDLoop >= kPIDIntervalUs) {
            lastPIDLoop = now_us;

            // Auto-tune runs at PID rate if active
            if (webTuner.isAutoTuning()) {
                webTuner.autoTuneStep(pitch);
            }

            balancer.update();
            motors.update(pitch);

            // Log at full 200Hz when recording
            if (webTuner.isLogging()) {
                webTuner.logSample(now, pitch, balancer.getPIDOutput(),
                                   motors.getAxisVelocity(0), motors.getAxisVelocity(1),
                                   imu.getGyroX(), imu.getGyroY(), imu.getGyroZ());
            }

            if (motors.isSafetyTripped()) {
                if (appState != STATE_ERROR) {
                    Serial.printf("[BALANCE] SAFETY STOP — pitch=%.1f\n", pitch);
                    oled.showMessage("TILT STOP!", "Recovering...");
                }
                appState = STATE_ERROR;
            }

            // Telemetry at 20Hz — serial + web
            if (now - lastTelemetryPrint >= kTelemetryPrintMs) {
                lastTelemetryPrint = now;
                float pidOut = balancer.getPIDOutput();
                float m0 = motors.getAxisVelocity(0);
                float m1 = motors.getAxisVelocity(1);

                Serial.printf("%.0f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
                              (float)now, pitch, roll, pidOut, m0, m1);

                webTuner.sendTelemetry(pitch, roll, pidOut, m0, m1, accuracy,
                                       imu.getGyroX(), imu.getGyroY(), imu.getGyroZ());
            }
        }
    }
    // Error — auto-recover via MotorController
    else if (appState == STATE_ERROR) {
        motors.update(pitch);
        if (!motors.isSafetyTripped()) {
            Serial.println("[INFO] Recovered — returning to idle");
            oled.showMessage("RECOVERED", "Use web/serial");
            appState = STATE_IDLE;
        }
    }
    else {
        motors.update(pitch);

        // Still send telemetry when idle (so web UI shows pitch)
        if (now - lastTelemetryPrint >= kTelemetryPrintMs) {
            lastTelemetryPrint = now;
            webTuner.sendTelemetry(pitch, roll, 0, 0, 0, accuracy,
                                   imu.getGyroX(), imu.getGyroY(), imu.getGyroZ());
        }
    }

    // OLED HUD at 10Hz
    if (oled.isOk() && (now - last_oled_update >= OLED_REFRESH_MS)) {
        last_oled_update = now;
        oled.updateHUD(pitch, roll, appState,
                       motors.getAxisVelocity(0), motors.getAxisVelocity(1),
                       accuracy);
    }
}
