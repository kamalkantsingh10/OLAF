/**
 * FeetechServo - Generic Serial Bus Servo Library
 * Implementation
 */

#include "FeetechServo.h"

FeetechServo::FeetechServo()
    : _servoCount(0),
      _serial(nullptr),
      _baudRate(0),
      _txPin(-1),
      _rxPin(-1),
      _initialized(false) {
    // Initialize servo array
    for (uint8_t i = 0; i < kMaxServos; i++) {
        _servos[i].id = 0;
        _servos[i].type = UNKNOWN;
        _servos[i].initialized = false;
    }
}

FeetechServo::~FeetechServo() {
    // SMS_STS is a value type, no need to delete
}

bool FeetechServo::begin(HardwareSerial* serial, long baudRate, int8_t txPin, int8_t rxPin) {
    if (serial == nullptr) {
        return false;
    }

    _serial = serial;
    _baudRate = baudRate;
    _txPin = txPin;
    _rxPin = rxPin;

    // Initialize serial port (Note: ESP32 HardwareSerial uses (baud, mode, rxPin, txPin))
    _serial->begin(_baudRate, SERIAL_8N1, _rxPin, _txPin);
    delay(100);  // Give serial time to stabilize

    // Assign serial port to SMS_STS driver
    _driver.pSerial = _serial;

    _initialized = true;
    return true;
}

bool FeetechServo::addServo(uint8_t servoId, ServoType type) {
    if (!_initialized || _servoCount >= kMaxServos) {
        return false;
    }

    // Check if servo already registered
    if (findServo(servoId) != nullptr) {
        return false;  // Already exists
    }

    // Test communication with servo (Ping returns ID if successful, -1 if failed)
    int pingResult = _driver.Ping(servoId);
    if (pingResult == -1) {
        return false;
    }

    // Register servo
    _servos[_servoCount].id = servoId;
    _servos[_servoCount].type = type;
    _servos[_servoCount].initialized = true;
    _servoCount++;

    return true;
}

uint8_t FeetechServo::scanServos(uint8_t startId, uint8_t endId) {
    if (!_initialized) {
        return 0;
    }

    uint8_t foundCount = 0;

    for (uint8_t id = startId; id <= endId; id++) {
        int pingResult = _driver.Ping(id);
        if (pingResult != -1) {
            foundCount++;
        }
        delay(10);  // Small delay between pings
    }

    return foundCount;
}

bool FeetechServo::setPosition(uint8_t servoId, float degrees, uint16_t durationMs) {
    ServoInfo* servo = findServo(servoId);
    if (servo == nullptr) {
        servo = &_servos[0];  // Use default type if not registered
        servo->type = STS;
    }

    int16_t raw = degreesToRaw(degrees, servo->type);
    return setPositionRaw(servoId, raw, durationMs);
}

bool FeetechServo::setPositionRaw(uint8_t servoId, int16_t rawPosition, uint16_t durationMs) {
    if (!_initialized) {
        return false;
    }

    // Calculate speed based on duration (approximate)
    // For SMS_STS: WritePosEx(ID, Position, Speed, Acceleration)
    // Speed range is typically 0-4095 steps/sec
    uint16_t speed = (durationMs > 0) ? (4095 * 1000 / durationMs) : kDefaultSpeed;
    if (speed > 4095) speed = 4095;
    if (speed < 1) speed = 1;

    // WritePosEx(ID, Position, Speed, Acceleration)
    _driver.WritePosEx(servoId, rawPosition, speed, kDefaultAccel);
    return true;  // WritePosEx doesn't return success/fail
}

float FeetechServo::getPosition(uint8_t servoId) {
    int16_t raw = getPositionRaw(servoId);
    if (raw == -1) {
        return NAN;
    }

    ServoInfo* servo = findServo(servoId);
    ServoType type = (servo != nullptr) ? servo->type : STS;
    return rawToDegrees(raw, type);
}

int16_t FeetechServo::getPositionRaw(uint8_t servoId) {
    if (!_initialized) {
        return -1;
    }

    // ReadPos returns current position, -1 on error
    return _driver.ReadPos(servoId);
}

float FeetechServo::getSpeed(uint8_t servoId) {
    if (!_initialized) {
        return NAN;
    }

    // ReadSpeed returns current speed, -1 on error
    int16_t speed = _driver.ReadSpeed(servoId);
    return (speed == -1) ? NAN : static_cast<float>(speed);
}

float FeetechServo::getTemperature(uint8_t servoId) {
    if (!_initialized) {
        return NAN;
    }

    // ReadTemper returns temperature in Celsius, -1 on error
    int8_t temp = _driver.ReadTemper(servoId);
    return (temp == -1) ? NAN : static_cast<float>(temp);
}

float FeetechServo::getCurrent(uint8_t servoId) {
    if (!_initialized) {
        return NAN;
    }

    // ReadCurrent returns current in mA, -1 on error
    int16_t current = _driver.ReadLoad(servoId);  // Load/Current
    return (current == -1) ? NAN : static_cast<float>(current);
}

bool FeetechServo::isMoving(uint8_t servoId) {
    if (!_initialized) {
        return false;
    }

    // ReadMove returns 0 if stopped, 1 if moving
    int moving = _driver.ReadMove(servoId);
    return (moving == 1);
}

bool FeetechServo::setTorque(uint8_t servoId, bool enable) {
    if (!_initialized) {
        return false;
    }

    // Enable/disable torque
    // EnableTorque(ID, 1=on, 0=off)
    _driver.EnableTorque(servoId, enable ? 1 : 0);
    return true;  // EnableTorque doesn't return success/fail
}

bool FeetechServo::ping(uint8_t servoId) {
    if (!_initialized) {
        return false;
    }

    // Ping returns ID if successful, -1 if failed
    int pingResult = _driver.Ping(servoId);
    return (pingResult != -1);
}

bool FeetechServo::setMode(uint8_t servoId, ServoMode mode) {
    if (!_initialized) {
        return false;
    }

    // Mode switching: SMS_STS uses WriteByte to set register 33 (mode register)
    // Mode: 0=Position, 1=Velocity, 3=Step
    // For now, position mode is default and most common - mode switching rarely needed
    // TODO: Implement if needed via: _driver.WriteByte(servoId, 33, static_cast<uint8_t>(mode));
    return true;  // Return true for position mode (default)
}

SMS_STS* FeetechServo::getDriver() {
    return _initialized ? &_driver : nullptr;
}

// Private helper methods

FeetechServo::ServoInfo* FeetechServo::findServo(uint8_t servoId) {
    for (uint8_t i = 0; i < _servoCount; i++) {
        if (_servos[i].id == servoId && _servos[i].initialized) {
            return &_servos[i];
        }
    }
    return nullptr;
}

int16_t FeetechServo::degreesToRaw(float degrees, ServoType type) {
    float rawPerDegree = (type == SCS) ? kRawToDegreeSCS : kRawToDegreeSTS;
    return kServoCenter + static_cast<int16_t>(degrees * rawPerDegree);
}

float FeetechServo::rawToDegrees(int16_t raw, ServoType type) {
    float rawPerDegree = (type == SCS) ? kRawToDegreeSCS : kRawToDegreeSTS;
    return static_cast<float>(raw - kServoCenter) / rawPerDegree;
}
