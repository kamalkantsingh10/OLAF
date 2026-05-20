/**
 * ODrive UART — Raw ASCII Protocol Layer
 *
 * Low-level UART communication with ODrive v3.6.
 * Sends ASCII commands and reads responses. No motor logic.
 * Motor control logic lives in MotorController.
 */

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"

class ODriveUART {
public:
    void begin();

    /** Send raw ASCII command (must end with \n) */
    void sendCommand(const char* command);

    /** Send command and read float response (blocking, with timeout) */
    float readFloat(const char* command, uint32_t timeout_ms = 500);

    /**
     * Read wheel velocity from ODrive feedback.
     * Sends "f <axis>\n", parses response "<pos> <vel>\n".
     * @param axis Motor axis (0 or 1)
     * @return Velocity in turns/s, or 0 on timeout
     */
    float readVelocity(uint8_t axis);

    /** Drain incoming UART buffer */
    void update();

private:
    HardwareSerial serial_{1};  // UART1 on ESP32-S3
};

extern ODriveUART odriveUart;
