/**
 * i2c_slave.h - I2C Slave Communication for Head Module
 * Story 1.4: Head Module ESP32 Firmware - Eye Expressions
 *
 * I2C Register Map Implementation:
 *   0x00: Module ID (read-only, returns 0x08)
 *   0x01: Firmware Version (read-only)
 *   0x02: Status byte (read-only: READY/BUSY/ERROR)
 *   0x03: Error code (read-only)
 *   0x04: Command register (write-only)
 *   0x10: Expression type (write: 0-6)
 *   0x11: Expression intensity (write: 1-5)
 *   0x12: Blink trigger (write: any value triggers blink)
 *
 * Thread Safety:
 *   - ISR handlers set flags and copy data to volatile buffers
 *   - Main loop processes commands from buffers
 *   - Minimal processing in ISR context (<10μs)
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// I2C Register Map - Standard Registers (All Modules)
// ============================================================================

#define REG_MODULE_ID           0x00  // Read-only: Module identification
#define REG_FIRMWARE_VERSION    0x01  // Read-only: Firmware version
#define REG_STATUS              0x02  // Read-only: Module status byte
#define REG_ERROR_CODE          0x03  // Read-only: Last error code
#define REG_COMMAND             0x04  // Write: Command trigger

// Status byte bit flags
#define STATUS_READY            0x01
#define STATUS_BUSY             0x02
#define STATUS_ERROR            0x04

// ============================================================================
// Head Module Specific Registers (0x10-0x1F)
// ============================================================================

#define REG_EXPRESSION_TYPE     0x10  // Emotion type (0x00-0x06)
#define REG_EXPRESSION_INTENSITY 0x11 // Intensity level (1-5)
#define REG_BLINK_TRIGGER       0x12  // Write any value to trigger blink

// Expression types
#define EXPR_NEUTRAL  0x00
#define EXPR_HAPPY    0x01
#define EXPR_CURIOUS  0x02
#define EXPR_THINKING 0x03
#define EXPR_CONFUSED 0x04
#define EXPR_SAD      0x05
#define EXPR_EXCITED  0x06

// Firmware version
#define FIRMWARE_VERSION 0x01  // v0.1

// ============================================================================
// I2C Slave Command Structure
// ============================================================================

struct I2CCommand {
  uint8_t expression_type;
  uint8_t intensity;
  bool blink_requested;
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
   * Call this in main loop to process commands
   */
  bool hasNewCommand();

  /**
   * Get the current command state
   * Only valid after hasNewCommand() returns true
   */
  I2CCommand getCommand();

  /**
   * Update module status (called by main loop)
   */
  void setStatus(uint8_t status_flags);

  /**
   * Report error code
   */
  void setError(uint8_t error_code);

  /**
   * Clear blink request flag (after processing)
   */
  void clearBlinkRequest();

  /**
   * I2C interrupt handlers (must be public for ISR registration)
   */
  static void onReceive(int num_bytes);
  static void onRequest();

private:
  // Singleton instance pointer for ISR access
  static I2CSlave* instance_;

  // Current command state (accessed by main loop)
  I2CCommand current_command_;

  // Volatile data shared between ISR and main loop
  volatile uint8_t register_address_;
  volatile bool new_command_received_;
  volatile uint8_t write_buffer_[4];  // Buffer for multi-byte writes
  volatile uint8_t write_buffer_index_;

  // I2C configuration
  uint8_t i2c_address_;

  // Internal helper methods
  void processRegisterWrite(uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t reg);
};

// Global instance (accessible to ISR)
extern I2CSlave i2c_slave;
