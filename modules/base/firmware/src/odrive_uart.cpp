/**
 * ODrive UART — Raw ASCII Protocol Layer
 */

#include "odrive_uart.h"

ODriveUART odriveUart;

void ODriveUART::begin() {
    serial_.begin(kODriveBaudRate, SERIAL_8N1, kODriveRxPin, kODriveTxPin);
    Serial.println("[ODrive] UART initialized");
    Serial.printf("[ODrive] RX: GPIO%d, TX: GPIO%d, Baud: %d\n",
                  kODriveRxPin, kODriveTxPin, kODriveBaudRate);
    delay(kStartupDelayMs);
}

void ODriveUART::sendCommand(const char* command) {
    serial_.print(command);
    if (kEnableSerialDebug) {
        Serial.printf("[ODrive TX] %s", command);
    }
}

float ODriveUART::readFloat(const char* command, uint32_t timeout_ms) {
    // Flush pending input
    while (serial_.available()) serial_.read();

    sendCommand(command);

    char buf[64];
    uint8_t idx = 0;
    uint32_t start = millis();

    while (millis() - start < timeout_ms) {
        if (serial_.available()) {
            char c = serial_.read();
            if (c == '\n') {
                buf[idx] = '\0';
                return atof(buf);
            }
            if (idx < sizeof(buf) - 1) {
                buf[idx++] = c;
            }
        }
    }

    Serial.println("[ODrive] readFloat timeout");
    return -999.0f;
}

float ODriveUART::readVelocity(uint8_t axis) {
    // Flush pending input
    while (serial_.available()) serial_.read();

    char cmd[16];
    snprintf(cmd, sizeof(cmd), "f %d\n", axis);
    serial_.print(cmd);

    // Response format: "<pos> <vel>\n"
    char buf[64];
    uint8_t idx = 0;
    uint32_t start = millis();
    constexpr uint32_t kTimeoutMs = 10;  // Short timeout — minimize balance PID starvation

    while (millis() - start < kTimeoutMs) {
        if (serial_.available()) {
            char c = serial_.read();
            if (c == '\n') {
                buf[idx] = '\0';
                // Find the space separating pos and vel
                char* space = strchr(buf, ' ');
                if (space) {
                    return atof(space + 1);  // velocity in turns/s
                }
                return 0.0f;
            }
            if (idx < sizeof(buf) - 1) {
                buf[idx++] = c;
            }
        }
    }
    return 0.0f;  // Timeout — return 0 (safe default)
}

void ODriveUART::update() {
    while (serial_.available()) {
        char c = serial_.read();
        if (kEnableODriveEcho) {
            Serial.print(c);
        }
    }
}
