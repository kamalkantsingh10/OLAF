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

void ODriveUART::update() {
    while (serial_.available()) {
        char c = serial_.read();
        if (kEnableODriveEcho) {
            Serial.print(c);
        }
    }
}
