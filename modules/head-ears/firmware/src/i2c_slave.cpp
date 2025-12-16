/**
 * i2c_slave.cpp - I2C Slave Communication Implementation
 * Story 1.4: Head Module ESP32 Firmware - Eye Expressions
 *
 * Purpose:
 *   Implements I2C slave protocol for receiving commands from Raspberry Pi
 *   orchestrator. The Pi sends high-level commands (e.g., "show happy expression
 *   with intensity 3") and the ESP32 translates these into display animations.
 *
 * Design Philosophy:
 *   - ISR handlers are FAST (<10μs) - only copy data and set flags
 *   - Main loop processes commands asynchronously
 *   - Volatile variables for ISR/main loop data sharing
 *   - Defensive programming: validate all inputs, never crash on bad data
 *
 * I2C Protocol:
 *   Write: [register_address] [value]
 *   Read:  [register_address] then request byte(s)
 */

#include "../firmware/i2c_slave.h"

// Global instance - accessible to both ISR and main loop
I2CSlave i2c_slave;

// Static member initialization - required for singleton pattern
I2CSlave* I2CSlave::instance_ = nullptr;

// ============================================================================
// Public Methods
// ============================================================================

/**
 * Initialize I2C slave interface
 *
 * @param address I2C slave address (0x08 for head module)
 * @param sda_pin GPIO pin for I2C data line
 * @param scl_pin GPIO pin for I2C clock line
 *
 * Sets up ESP32 as I2C slave, registers interrupt handlers, and initializes
 * default state (neutral expression, medium intensity).
 */
void I2CSlave::begin(uint8_t address, uint8_t sda_pin, uint8_t scl_pin) {
  i2c_address_ = address;
  instance_ = this;  // Set singleton instance for ISR access

  // Initialize command state with safe defaults
  // Start with neutral expression to avoid surprising animations on boot
  current_command_.expression_type = EXPR_NEUTRAL;
  current_command_.intensity = 2;  // Default medium intensity (range 1-5)
  current_command_.blink_requested = false;
  current_command_.status = STATUS_READY;  // Module ready to receive commands
  current_command_.error_code = 0;         // No errors

  // Initialize volatile state shared between ISR and main loop
  register_address_ = 0;
  new_command_received_ = false;
  write_buffer_index_ = 0;

  // Configure I2C pins (ESP32-S3 allows flexible pin mapping, unlike fixed hardware I2C on some MCUs)
  Wire.setPins(sda_pin, scl_pin);

  // Begin as I2C slave at specified address
  // Unlike master mode, slave mode only takes address (no frequency parameter)
  Wire.begin(address);

  // Register interrupt handlers
  // onReceive: Called when master writes data to us
  // onRequest: Called when master requests data from us
  Wire.onReceive(I2CSlave::onReceive);
  Wire.onRequest(I2CSlave::onRequest);

  // Set I2C timeout to prevent hanging if communication fails
  Wire.setTimeout(500);  // 500ms timeout

  Serial.printf("[I2C] Slave initialized at 0x%02X - Head Module\n", address);
  Serial.printf("[I2C]   SDA: GPIO%d, SCL: GPIO%d\n", sda_pin, scl_pin);
}

/**
 * Check if new command received from I2C master
 *
 * Returns true if a new command has been received and is ready for processing.
 * This is a non-blocking check - call frequently from main loop.
 *
 * @return true if new command available, false otherwise
 *
 * Thread Safety: Clears the new_command_received_ flag atomically
 */
bool I2CSlave::hasNewCommand() {
  if (new_command_received_) {
    new_command_received_ = false;  // Clear flag for next command
    return true;
  }
  return false;
}

/**
 * Get the current command state
 *
 * Returns a copy of the current command structure containing:
 *   - expression_type: Which emotion to display (0-6)
 *   - intensity: How exaggerated the expression should be (1-5)
 *   - blink_requested: Whether a blink animation should trigger
 *   - status: Module status flags (READY/BUSY/ERROR)
 *   - error_code: Last error code (0 = no error)
 *
 * Only call this after hasNewCommand() returns true.
 *
 * @return Copy of current command state
 */
I2CCommand I2CSlave::getCommand() {
  return current_command_;
}

/**
 * Update module status
 *
 * Called by main loop to report current state back to I2C master.
 * Status flags can be:
 *   - STATUS_READY: Idle, ready for new commands
 *   - STATUS_BUSY: Processing animation, may reject new commands
 *   - STATUS_ERROR: Error occurred, check error_code
 *
 * @param status_flags Bitwise OR of status flags
 *
 * Example: setStatus(STATUS_BUSY) during long animation
 */
void I2CSlave::setStatus(uint8_t status_flags) {
  current_command_.status = status_flags;
}

/**
 * Report error code
 *
 * Called by main loop when an error occurs (e.g., display init failure).
 * Automatically sets STATUS_ERROR flag.
 *
 * @param error_code Application-specific error code
 *
 * Error codes:
 *   0x00: No error
 *   0x01: Invalid parameter
 *   0x02: Display error
 *   0x03: Hardware error
 */
void I2CSlave::setError(uint8_t error_code) {
  current_command_.error_code = error_code;
  current_command_.status |= STATUS_ERROR;  // Set error bit
}

/**
 * Clear blink request flag
 *
 * Called by main loop after processing a blink animation.
 * Prevents blink from repeating every loop iteration.
 */
void I2CSlave::clearBlinkRequest() {
  current_command_.blink_requested = false;
}

// ============================================================================
// Private Methods
// ============================================================================

/**
 * Process register write from I2C master
 *
 * Called by ISR when master writes to a register.
 * Validates input and updates internal state.
 *
 * @param reg Register address (0x00-0xFF)
 * @param value Value to write (0x00-0xFF)
 *
 * Design Note: Keep validation simple in ISR context. Complex validation
 * should happen in main loop if needed.
 */
void I2CSlave::processRegisterWrite(uint8_t reg, uint8_t value) {
  switch (reg) {
    case REG_EXPRESSION_TYPE:
      // Expression type must be in valid range (0-6)
      // Invalid values are rejected to prevent undefined behavior in animation engine
      if (value <= EXPR_EXCITED) {  // Valid expression type (0-6)
        current_command_.expression_type = value;
        new_command_received_ = true;  // Signal main loop to process
        Serial.printf("[I2C] Expression type: %d\n", value);
      } else {
        Serial.printf("[I2C] ERROR: Invalid expression type %d\n", value);
        setError(0x01);  // Error code 0x01: Invalid parameter
      }
      break;

    case REG_EXPRESSION_INTENSITY:
      // Intensity range is 1-5 (1=subtle, 5=exaggerated)
      // Zero is invalid (no expression), >5 is undefined
      if (value >= 1 && value <= 5) {  // Valid intensity (1-5)
        current_command_.intensity = value;
        new_command_received_ = true;
        Serial.printf("[I2C] Intensity: %d\n", value);
      } else {
        Serial.printf("[I2C] ERROR: Invalid intensity %d (must be 1-5)\n", value);
        setError(0x01);  // Error code 0x01: Invalid parameter
      }
      break;

    case REG_BLINK_TRIGGER:
      // Any write to blink register triggers a blink animation
      // Value doesn't matter - it's a trigger, not a state
      current_command_.blink_requested = true;
      new_command_received_ = true;
      Serial.println("[I2C] Blink triggered");
      break;

    case REG_COMMAND:
      // Generic command register for future expansion
      // Could be used for: reset, calibration, self-test, etc.
      Serial.printf("[I2C] Command register: 0x%02X\n", value);
      break;

    default:
      // Unknown register write - log but don't crash
      // Allows forward compatibility if new registers added later
      Serial.printf("[I2C] WARNING: Write to unknown register 0x%02X\n", reg);
      break;
  }
}

/**
 * Read register value for I2C master request
 *
 * Called by ISR when master requests data from a register.
 * Returns current state of requested register.
 *
 * @param reg Register address to read
 * @return Register value (0x00-0xFF)
 *
 * Design Note: Read-only registers (0x00-0x04) provide status info.
 * Write-only registers can also be read to verify last written value.
 */
uint8_t I2CSlave::readRegister(uint8_t reg) {
  switch (reg) {
    case REG_MODULE_ID:
      // Return module I2C address as identification
      // Useful for master to verify it's talking to correct module
      return i2c_address_;  // 0x08 for head module

    case REG_FIRMWARE_VERSION:
      // Firmware version for compatibility checking
      // Format: 0xXY where X=major, Y=minor (e.g., 0x11 = v1.1)
      return FIRMWARE_VERSION;

    case REG_STATUS:
      // Current module status (READY/BUSY/ERROR)
      // Master can poll this before sending commands
      return current_command_.status;

    case REG_ERROR_CODE:
      // Last error code (0 = no error)
      // Only valid when STATUS_ERROR bit is set
      return current_command_.error_code;

    case REG_EXPRESSION_TYPE:
      // Echo back current expression type
      // Allows master to verify command was received correctly
      return current_command_.expression_type;

    case REG_EXPRESSION_INTENSITY:
      // Echo back current intensity
      return current_command_.intensity;

    default:
      // Unknown register read - return 0xFF (common I2C convention for "no data")
      Serial.printf("[I2C] WARNING: Read from unknown register 0x%02X\n", reg);
      return 0xFF;  // Return 0xFF for unknown registers
  }
}

// ============================================================================
// I2C Interrupt Handlers (Static)
// ============================================================================

/**
 * I2C Receive Interrupt Handler
 *
 * Called automatically by Wire library when I2C master writes data.
 * Runs in ISR context - MUST be fast (<10μs).
 *
 * Protocol:
 *   Byte 0: Register address
 *   Byte 1+: Data to write
 *
 * @param num_bytes Number of bytes received
 *
 * CRITICAL: Keep this handler MINIMAL
 *   - No Serial.print (slow)
 *   - No delays
 *   - No complex processing
 *   - Just copy data and set flags
 */
void I2CSlave::onReceive(int num_bytes) {
  if (!instance_) return;  // Safety check - instance must be initialized

  // ISR - Keep minimal! Just copy data and set flags.
  if (num_bytes < 1) return;  // Need at least register address

  // First byte is always register address
  uint8_t reg = Wire.read();
  instance_->register_address_ = reg;

  // If additional bytes present, this is a write operation
  if (num_bytes > 1) {
    uint8_t value = Wire.read();  // Read data byte
    instance_->processRegisterWrite(reg, value);  // Validate and store

    // Discard any additional bytes (protocol is single-byte writes)
    // This prevents buffer overflow if master sends too many bytes
    while (Wire.available()) {
      Wire.read();
    }
  }
  // If only 1 byte (register address), master will follow with a request operation
}

/**
 * I2C Request Interrupt Handler
 *
 * Called automatically by Wire library when I2C master requests data.
 * Runs in ISR context - MUST be fast (<10μs).
 *
 * Protocol:
 *   Master previously sent register address via onReceive()
 *   Now master requests data from that register
 *
 * CRITICAL: Keep this handler MINIMAL
 *   - No Serial.print
 *   - No delays
 *   - Just read and respond
 */
void I2CSlave::onRequest() {
  if (!instance_) return;  // Safety check

  // ISR - Master is requesting data from previously set register address
  uint8_t value = instance_->readRegister(instance_->register_address_);
  Wire.write(value);  // Send single byte response
}
