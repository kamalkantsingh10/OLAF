/**
 * FeetechServo - Generic Serial Bus Servo Library
 *
 * Wrapper around Waveshare SCServo library for Feetech/Waveshare serial bus servos.
 * Supports both STS and SCS servo types.
 *
 * Compatible Hardware:
 *   - ST Series: ST3215, ST3032, etc. (Waveshare)
 *   - STS Series: STS3215, STS3032, etc. (Feetech)
 *   - SCS Series: SCS0009, SCS15, etc. (Feetech)
 *
 * Usage:
 *   FeetechServo controller;
 *   controller.begin(&Serial2, 1000000, 17, 18);  // TX, RX pins (1MHz baud)
 *   controller.addServo(2, FeetechServo::STS);    // Add servo ID 2
 *   controller.setPosition(2, 0, 1500);           // Center position, 1500ms duration
 */

#pragma once

#include <Arduino.h>
#include <SCServo.h>

class FeetechServo {
public:
    // Servo types (maps to STSServoDriver ServoType)
    enum ServoType {
        UNKNOWN = 0,
        STS = 1,      // STS series (STS3215, STS3032, etc.)
        SCS = 2       // SCS series (SCS0009, SCS15, etc.)
    };

    // Servo operating modes
    enum ServoMode {
        POSITION = 0,   // Position control (default)
        VELOCITY = 1,   // Velocity control
        STEP = 3        // Step mode
    };

    // Constructor
    FeetechServo();
    ~FeetechServo();

    /**
     * Initialize servo controller
     * @param serial Hardware serial port (e.g., &Serial2)
     * @param baudRate Baud rate (typically 115200 or 1000000)
     * @param txPin TX pin number
     * @param rxPin RX pin number
     * @return true if initialization successful
     */
    bool begin(HardwareSerial* serial, long baudRate, int8_t txPin, int8_t rxPin);

    /**
     * Register a servo with this controller
     * @param servoId Servo ID (1-253, usually 1-10)
     * @param type Servo type (STS or SCS)
     * @return true if servo responds to ping
     */
    bool addServo(uint8_t servoId, ServoType type = STS);

    /**
     * Scan for servos on the bus
     * @param startId Start scanning from this ID (default 1)
     * @param endId End scanning at this ID (default 10)
     * @return Number of servos found
     */
    uint8_t scanServos(uint8_t startId = 1, uint8_t endId = 10);

    /**
     * Set target position for a servo
     * @param servoId Servo ID
     * @param degrees Target position in degrees (-180 to +180, 0 = center)
     * @param durationMs Movement duration in milliseconds (0 = max speed)
     * @return true if command sent successfully
     */
    bool setPosition(uint8_t servoId, float degrees, uint16_t durationMs = 1000);

    /**
     * Set target position in raw units (2048 = center, ~14 units per degree)
     * @param servoId Servo ID
     * @param rawPosition Raw position (0-4095 for most models)
     * @param durationMs Movement duration in milliseconds
     * @return true if command sent successfully
     */
    bool setPositionRaw(uint8_t servoId, int16_t rawPosition, uint16_t durationMs = 1000);

    /**
     * Get current position in degrees
     * @param servoId Servo ID
     * @return Current position in degrees, or NaN if read failed
     */
    float getPosition(uint8_t servoId);

    /**
     * Get current position in raw units
     * @param servoId Servo ID
     * @return Raw position, or -1 if read failed
     */
    int16_t getPositionRaw(uint8_t servoId);

    /**
     * Get current speed
     * @param servoId Servo ID
     * @return Current speed in RPM, or NaN if read failed
     */
    float getSpeed(uint8_t servoId);

    /**
     * Get current temperature
     * @param servoId Servo ID
     * @return Temperature in Celsius, or NaN if read failed
     */
    float getTemperature(uint8_t servoId);

    /**
     * Get current draw
     * @param servoId Servo ID
     * @return Current in mA, or NaN if read failed
     */
    float getCurrent(uint8_t servoId);

    /**
     * Check if servo is currently moving
     * @param servoId Servo ID
     * @return true if moving, false if stopped or error
     */
    bool isMoving(uint8_t servoId);

    /**
     * Enable/disable torque
     * @param servoId Servo ID
     * @param enable true to enable torque, false to disable (limp mode)
     * @return true if command sent successfully
     */
    bool setTorque(uint8_t servoId, bool enable);

    /**
     * Ping a servo to check if it's responding
     * @param servoId Servo ID
     * @return true if servo responds
     */
    bool ping(uint8_t servoId);

    /**
     * Set servo operating mode
     * @param servoId Servo ID
     * @param mode Operating mode (POSITION, VELOCITY, STEP)
     * @return true if command sent successfully
     */
    bool setMode(uint8_t servoId, ServoMode mode);

    /**
     * Get the underlying SMS_STS instance (for advanced usage)
     * @return Pointer to driver, or nullptr if not initialized
     */
    SMS_STS* getDriver();

private:
    struct ServoInfo {
        uint8_t id;
        ServoType type;
        bool initialized;
    };

    static constexpr uint8_t kMaxServos = 16;           // Max servos per controller
    static constexpr int16_t kServoCenter = 2048;       // Center position in raw units
    static constexpr float kRawToDegreeSTS = 14.0f;     // ~14 raw units per degree (STS series)
    static constexpr float kRawToDegreeSCS = 14.0f;     // ~14 raw units per degree (SCS series, adjust if needed)
    static constexpr uint16_t kDefaultSpeed = 3400;     // Default speed for WritePosEx
    static constexpr uint8_t kDefaultAccel = 50;        // Default acceleration for WritePosEx

    ServoInfo _servos[kMaxServos];
    uint8_t _servoCount;
    HardwareSerial* _serial;
    long _baudRate;
    int8_t _txPin;
    int8_t _rxPin;
    bool _initialized;
    SMS_STS _driver;  // SCServo library driver instance

    // Helper: Find servo info by ID
    ServoInfo* findServo(uint8_t servoId);

    // Helper: Convert degrees to raw position based on servo type
    int16_t degreesToRaw(float degrees, ServoType type);

    // Helper: Convert raw position to degrees based on servo type
    float rawToDegrees(int16_t raw, ServoType type);
};
