/**
 * I2C Slave Communication Handler Implementation
 * Base Module - I2C Address 0x0B
 */

#include "i2c_slave.h"

// Static member initialization
volatile uint8_t I2CSlave::currentRegister_ = 0;
volatile uint8_t I2CSlave::statusByte_ = STATUS_OK;
volatile uint8_t I2CSlave::commandByte_ = 0;
volatile float I2CSlave::linearVelocity_ = 0.0;
volatile float I2CSlave::angularVelocity_ = 0.0;
volatile float I2CSlave::odomX_ = 0.0;
volatile float I2CSlave::odomY_ = 0.0;
volatile float I2CSlave::odomTheta_ = 0.0;
volatile float I2CSlave::pitch_ = 0.0;
volatile float I2CSlave::batteryVoltage_ = 0.0;

// Global instance
I2CSlave i2cSlave;

void I2CSlave::begin() {
    // Initialize I2C slave on configured pins
    Wire.begin(kI2CSlaveAddress, kI2CSdaPin, kI2CSclPin, kI2CFrequency);

    // Register interrupt handlers
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);

    Serial.println("[I2C] Slave initialized");
    Serial.printf("[I2C] Address: 0x%02X, SDA: GPIO%d, SCL: GPIO%d\n",
                  kI2CSlaveAddress, kI2CSdaPin, kI2CSclPin);
}

void I2CSlave::update() {
    // Currently no polling needed - interrupts handle everything
    // Future: Add timeout detection, error handling
}

void I2CSlave::setStatus(uint8_t status) {
    statusByte_ = status;
}

uint8_t I2CSlave::getCommand() {
    return commandByte_;
}

void I2CSlave::clearCommand() {
    commandByte_ = 0;
}

float I2CSlave::getLinearVelocity() {
    return linearVelocity_;
}

float I2CSlave::getAngularVelocity() {
    return angularVelocity_;
}

void I2CSlave::setOdometry(float x, float y, float theta) {
    odomX_ = x;
    odomY_ = y;
    odomTheta_ = theta;
}

void I2CSlave::setPitch(float pitch_deg) {
    pitch_ = pitch_deg;
}

void I2CSlave::setBatteryVoltage(float voltage) {
    batteryVoltage_ = voltage;
}

/**
 * I2C Receive Interrupt Handler
 * Called when Raspberry Pi writes data to this slave
 */
void I2CSlave::onReceive(int numBytes) {
    if (numBytes < 1) return;

    // First byte is always the register address
    currentRegister_ = Wire.read();
    numBytes--;

    // Handle writes based on register
    switch (currentRegister_) {
        case REG_COMMAND:
            if (numBytes >= 1) {
                commandByte_ = Wire.read();
            }
            break;

        case REG_LINEAR_VEL:
            if (numBytes >= 4) {
                uint8_t bytes[4];
                for (int i = 0; i < 4; i++) {
                    bytes[i] = Wire.read();
                }
                linearVelocity_ = bytesToFloat(bytes);
            }
            break;

        case REG_ANGULAR_VEL:
            if (numBytes >= 4) {
                uint8_t bytes[4];
                for (int i = 0; i < 4; i++) {
                    bytes[i] = Wire.read();
                }
                angularVelocity_ = bytesToFloat(bytes);
            }
            break;

        default:
            // Unknown register - discard remaining bytes
            while (Wire.available()) {
                Wire.read();
            }
            break;
    }
}

/**
 * I2C Request Interrupt Handler
 * Called when Raspberry Pi reads data from this slave
 */
void I2CSlave::onRequest() {
    uint8_t bytes[4];

    switch (currentRegister_) {
        case REG_STATUS:
            Wire.write(statusByte_);
            break;

        case REG_ODOM_X:
            floatToBytes(odomX_, bytes);
            Wire.write(bytes, 4);
            break;

        case REG_ODOM_Y:
            floatToBytes(odomY_, bytes);
            Wire.write(bytes, 4);
            break;

        case REG_ODOM_THETA:
            floatToBytes(odomTheta_, bytes);
            Wire.write(bytes, 4);
            break;

        case REG_PITCH:
            floatToBytes(pitch_, bytes);
            Wire.write(bytes, 4);
            break;

        case REG_BATTERY_VOLTAGE:
            floatToBytes(batteryVoltage_, bytes);
            Wire.write(bytes, 4);
            break;

        default:
            // Unknown register - send zeros
            Wire.write((uint8_t)0x00);
            break;
    }
}

/**
 * Convert 4 bytes to float (little-endian)
 */
float I2CSlave::bytesToFloat(uint8_t* bytes) {
    float value;
    memcpy(&value, bytes, 4);
    return value;
}

/**
 * Convert float to 4 bytes (little-endian)
 */
void I2CSlave::floatToBytes(float value, uint8_t* bytes) {
    memcpy(bytes, &value, 4);
}
