/**
 * i2c_slave.cpp - I2C Slave Communication Implementation
 * Story 1.3: Develop Head ESP32 Firmware
 *
 * Implements register-based I2C slave protocol for receiving commands
 * from Raspberry Pi orchestrator.
 */

#include "i2c_slave.h"

// Global instance
I2CSlave i2c_slave;

// Static member initialization
I2CSlave* I2CSlave::instance_ = nullptr;

void I2CSlave::begin(uint8_t address, uint8_t sda_pin, uint8_t scl_pin) {
  i2c_address_ = address;
  instance_ = this;

  // Initialize command state with safe defaults
  current_command_.expression_type = EXPR_NEUTRAL;
  current_command_.intensity = 2;
  current_command_.blink_requested = false;
  current_command_.look_x = 0;
  current_command_.look_y = 0;
  current_command_.status = STATUS_READY;
  current_command_.error_code = 0;
  current_command_.system_status = 0;  // IDLE
  current_command_.led_overlay = 0;    // NONE (separate event)

  // Initialize volatile state
  register_address_ = 0;
  new_command_received_ = false;
  write_buffer_index_ = 0;

  // Configure I2C pins
  Wire.setPins(sda_pin, scl_pin);
  Wire.begin(address);

  // Register interrupt handlers
  Wire.onReceive(I2CSlave::onReceive);
  Wire.onRequest(I2CSlave::onRequest);
  Wire.setTimeout(500);

  Serial.printf("[I2C] Slave initialized at 0x%02X\n", address);
  Serial.printf("[I2C]   SDA: GPIO%d, SCL: GPIO%d\n", sda_pin, scl_pin);
}

bool I2CSlave::hasNewCommand() {
  if (new_command_received_) {
    new_command_received_ = false;
    return true;
  }
  return false;
}

I2CCommand I2CSlave::getCommand() {
  return current_command_;
}

void I2CSlave::setStatus(uint8_t status_flags) {
  current_command_.status = status_flags;
}

void I2CSlave::setError(uint8_t error_code) {
  current_command_.error_code = error_code;
  current_command_.status |= STATUS_ERROR;
}

void I2CSlave::clearBlinkRequest() {
  current_command_.blink_requested = false;
}

void I2CSlave::processRegisterWrite(uint8_t reg, uint8_t value) {
  switch (reg) {
    case REG_EXPRESSION_TYPE:
      if (value < EXPR_COUNT) {
        current_command_.expression_type = value;
        new_command_received_ = true;
        Serial.printf("[I2C] Expression type: %d\n", value);
      } else {
        Serial.printf("[I2C] ERROR: Invalid expression type %d\n", value);
        setError(0x01);
      }
      break;

    case REG_EXPRESSION_INTENSITY:
      if (value >= 1 && value <= 5) {
        current_command_.intensity = value;
        new_command_received_ = true;
        Serial.printf("[I2C] Intensity: %d\n", value);
      } else {
        Serial.printf("[I2C] ERROR: Invalid intensity %d (must be 1-5)\n", value);
        setError(0x01);
      }
      break;

    case REG_BLINK_TRIGGER:
      current_command_.blink_requested = true;
      new_command_received_ = true;
      Serial.println("[I2C] Blink triggered");
      break;

    case REG_LOOK_X:
      // Treat as signed byte (-100 to +100)
      current_command_.look_x = (int8_t)value;
      new_command_received_ = true;
      Serial.printf("[I2C] Look X: %d\n", current_command_.look_x);
      break;

    case REG_LOOK_Y:
      current_command_.look_y = (int8_t)value;
      new_command_received_ = true;
      Serial.printf("[I2C] Look Y: %d\n", current_command_.look_y);
      break;

    case REG_SYSTEM_STATUS:
      if (value <= 5) {
        current_command_.system_status = value;
        new_command_received_ = true;
        Serial.printf("[I2C] System status: %d\n", value);
      } else {
        Serial.printf("[I2C] ERROR: Invalid system status %d\n", value);
        setError(0x02);
      }
      break;

    case REG_LED_OVERLAY:
      // SEPARATE event — independent of expression_type & system_status.
      if (value <= LED_OVERLAY_MAX) {
        current_command_.led_overlay = value;
        new_command_received_ = true;
        Serial.printf("[I2C] LED overlay: %d\n", value);
      } else {
        Serial.printf("[I2C] ERROR: Invalid LED overlay %d\n", value);
        setError(0x03);
      }
      break;

    case REG_COMMAND:
      Serial.printf("[I2C] Command register: 0x%02X\n", value);
      break;

    default:
      Serial.printf("[I2C] WARNING: Write to unknown register 0x%02X\n", reg);
      break;
  }
}

uint8_t I2CSlave::readRegister(uint8_t reg) {
  switch (reg) {
    case REG_STATUS:
      return current_command_.status;

    case REG_MODULE_ID:
      return i2c_address_;

    case REG_FIRMWARE_VER:
      return FIRMWARE_VERSION;

    case REG_ERROR_CODE:
      return current_command_.error_code;

    case REG_EXPRESSION_TYPE:
      return current_command_.expression_type;

    case REG_EXPRESSION_INTENSITY:
      return current_command_.intensity;

    case REG_LOOK_X:
      return (uint8_t)current_command_.look_x;

    case REG_LOOK_Y:
      return (uint8_t)current_command_.look_y;

    case REG_LED_OVERLAY:
      return current_command_.led_overlay;

    default:
      Serial.printf("[I2C] WARNING: Read from unknown register 0x%02X\n", reg);
      return 0xFF;
  }
}

void I2CSlave::onReceive(int num_bytes) {
  if (!instance_) return;
  if (num_bytes < 1) return;

  uint8_t reg = Wire.read();
  instance_->register_address_ = reg;

  if (num_bytes > 1) {
    uint8_t value = Wire.read();
    instance_->processRegisterWrite(reg, value);

    while (Wire.available()) {
      Wire.read();
    }
  }
}

void I2CSlave::onRequest() {
  if (!instance_) return;

  uint8_t value = instance_->readRegister(instance_->register_address_);
  Wire.write(value);
}
