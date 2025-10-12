// OLAF I2C Register Map - Common Definitions
//
// This header defines the common I2C register protocol that all ESP32 modules
// implement. Each module acts as an I2C slave with a unique address (0x08-0x0C)
// and responds to these standard registers.
//
// Architecture: Raspberry Pi (I2C master) ↔ ESP32 modules (I2C slaves)
// Communication latency target: <100ms per operation
//
// Module I2C Addresses:
//   - Head Module:      0x08
//   - Ears Module:      0x09
//   - Neck Module:      0x0A
//   - Projector Module: 0x0B
//   - Base Module:      0x0C

#pragma once

// =============================================================================
// Common Registers (All Modules)
// =============================================================================

// REG_MODULE_ID (Read-only)
// Returns module identifier to verify correct I2C communication
// Values: 0x08 (Head), 0x09 (Ears), 0x0A (Neck), 0x0B (Projector), 0x0C (Base)
#define REG_MODULE_ID           0x00

// REG_FIRMWARE_VERSION (Read-only)
// Returns firmware version for OTA update tracking
// Format: Major.Minor (e.g., 0x12 = v1.2)
#define REG_FIRMWARE_VERSION    0x01

// REG_STATUS (Read-only)
// Returns current module status byte (bitfield)
// See STATUS_* flags below for bit definitions
#define REG_STATUS              0x02

// REG_COMMAND (Write-only)
// Command register for high-level module operations
// Module-specific command codes defined in per-module headers
#define REG_COMMAND             0x04

// =============================================================================
// Status Register Flags (REG_STATUS bitfield)
// =============================================================================

// STATUS_READY - Module initialized and ready to receive commands
// Set after successful setup(), cleared during error conditions
#define STATUS_READY            0x01

// STATUS_BUSY - Module currently processing command or animation
// Set during active operations, cleared when ready for next command
// Orchestrator should poll this before sending time-sensitive commands
#define STATUS_BUSY             0x02

// STATUS_ERROR - Error condition detected (check module serial logs)
// Set on hardware init failures, I2C errors, or invalid commands
// Requires module reset or power cycle to clear
#define STATUS_ERROR            0x04

// =============================================================================
// Usage Example (Orchestrator Side)
// =============================================================================
//
// Python example (orchestrator/ros2_nodes/hardware_drivers/):
//
//   from smbus2 import SMBus
//
//   HEAD_MODULE_ADDR = 0x08
//   bus = SMBus(1)  # I2C bus 1 on Raspberry Pi
//
//   # Read module ID to verify connection
//   module_id = bus.read_byte_data(HEAD_MODULE_ADDR, REG_MODULE_ID)
//   assert module_id == HEAD_MODULE_ADDR, "Module ID mismatch!"
//
//   # Check if module is ready
//   status = bus.read_byte_data(HEAD_MODULE_ADDR, REG_STATUS)
//   if status & STATUS_READY and not (status & STATUS_BUSY):
//       # Send command (module-specific)
//       bus.write_byte_data(HEAD_MODULE_ADDR, REG_COMMAND, 0x10)
//
// =============================================================================
// Usage Example (ESP32 Module Side)
// =============================================================================
//
// C++ example (modules/head/firmware/head_controller.ino):
//
//   #include "i2c_registers.h"
//   #include <Wire.h>
//
//   uint8_t register_map[256] = {0};
//
//   void setup() {
//     register_map[REG_MODULE_ID] = 0x08;  // Head module
//     register_map[REG_FIRMWARE_VERSION] = 0x10;  // v1.0
//     register_map[REG_STATUS] = STATUS_READY;
//
//     Wire.begin(0x08);  // I2C slave address
//     Wire.onReceive(onI2CReceive);
//     Wire.onRequest(onI2CRequest);
//   }
//
// =============================================================================
