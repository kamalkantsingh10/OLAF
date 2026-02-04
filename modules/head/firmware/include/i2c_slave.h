/**
 * i2c_slave.h - I2C Slave Communication for Head Module
 * Story 1.3: Develop Head ESP32 Firmware
 *
 * I2C Register Map:
 *   0x00: STATUS (R) - Module status (0=OK, 1=ERROR)
 *   0x01: COMMAND (W) - Reserved for future commands
 *   0x10: EXPRESSION_TYPE (W) - Emotion type (0-6)
 *   0x11: EXPRESSION_INTENSITY (W) - Intensity (1-5)
 *   0x12: BLINK_TRIGGER (W) - Write 1 to trigger blink
 *   0x20: LOOK_X (W) - Look direction X (-100 to +100)
 *   0x21: LOOK_Y (W) - Look direction Y (-100 to +100)
 *
 * Thread Safety:
 *   - ISR handlers set flags and copy data to volatile buffers
 *   - Main loop processes commands from buffers
 *   - Minimal processing in ISR context (<10us)
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// ============================================================================
// I2C Register Map
// ============================================================================

// Standard Registers (Read-only status)
constexpr uint8_t REG_STATUS         = 0x00;  // Module status byte
constexpr uint8_t REG_COMMAND        = 0x01;  // Command register (future use)
constexpr uint8_t REG_MODULE_ID      = 0x02;  // Module identification
constexpr uint8_t REG_FIRMWARE_VER   = 0x03;  // Firmware version
constexpr uint8_t REG_ERROR_CODE     = 0x04;  // Last error code

// Expression Registers
constexpr uint8_t REG_EXPRESSION_TYPE      = 0x10;  // Emotion type (0-6)
constexpr uint8_t REG_EXPRESSION_INTENSITY = 0x11;  // Intensity (1-5)
constexpr uint8_t REG_BLINK_TRIGGER        = 0x12;  // Write any value to blink

// Look Direction Registers
constexpr uint8_t REG_LOOK_X = 0x20;  // Signed: -100 (left) to +100 (right)
constexpr uint8_t REG_LOOK_Y = 0x21;  // Signed: -100 (down) to +100 (up)

// ============================================================================
// Status Flags
// ============================================================================

constexpr uint8_t STATUS_OK    = 0x00;
constexpr uint8_t STATUS_READY = 0x01;
constexpr uint8_t STATUS_BUSY  = 0x02;
constexpr uint8_t STATUS_ERROR = 0x04;

// ============================================================================
// Expression Types (per story spec)
// ============================================================================

constexpr uint8_t EXPR_NEUTRAL   = 0;  // Calm, attentive baseline
constexpr uint8_t EXPR_HAPPY     = 1;  // Joyful, positive
constexpr uint8_t EXPR_SAD       = 2;  // Disappointed, melancholy
constexpr uint8_t EXPR_SURPRISED = 3;  // Startled, wide-eyed
constexpr uint8_t EXPR_ANGRY     = 4;  // Frustrated, intense
constexpr uint8_t EXPR_SLEEPY    = 5;  // Tired, droopy
constexpr uint8_t EXPR_WINK      = 6;  // Playful wink (one eye)

constexpr uint8_t EXPR_COUNT = 7;  // Total number of expressions

// ============================================================================
// I2C Command Structure
// ============================================================================

struct I2CCommand {
  uint8_t expression_type;
  uint8_t intensity;
  bool blink_requested;
  int8_t look_x;   // -100 to +100
  int8_t look_y;   // -100 to +100
  uint8_t status;
  uint8_t error_code;
};

// ============================================================================
// I2C Slave Class
// ============================================================================

class I2CSlave {
public:
  /**
   * Initialize I2C slave with address and pin configuration
   */
  void begin(uint8_t address, uint8_t sda_pin, uint8_t scl_pin);

  /**
   * Check if new command received from I2C master
   */
  bool hasNewCommand();

  /**
   * Get the current command state
   */
  I2CCommand getCommand();

  /**
   * Update module status
   */
  void setStatus(uint8_t status_flags);

  /**
   * Report error code
   */
  void setError(uint8_t error_code);

  /**
   * Clear blink request flag
   */
  void clearBlinkRequest();

  /**
   * I2C interrupt handlers (static for ISR registration)
   */
  static void onReceive(int num_bytes);
  static void onRequest();

private:
  static I2CSlave* instance_;

  I2CCommand current_command_;

  volatile uint8_t register_address_;
  volatile bool new_command_received_;
  volatile uint8_t write_buffer_[4];
  volatile uint8_t write_buffer_index_;

  uint8_t i2c_address_;

  void processRegisterWrite(uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t reg);
};

// Global instance
extern I2CSlave i2c_slave;
