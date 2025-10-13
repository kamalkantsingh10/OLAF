// OLAF I2C Register Map - Common Definitions
//
// This header defines the common I2C register protocol that all ESP32 modules
// implement. Each module acts as an I2C slave with a unique address (0x08-0x0B)
// and responds to these standard registers.
//
// Architecture: Raspberry Pi (I2C master) ↔ ESP32 modules (I2C slaves)
// Communication latency target: <100ms per operation
//
// Module I2C Addresses (4-Module Architecture):
//   - Head Module:      0x08  (Eyes, mmWave sensor, I2S mics, speaker)
//   - Ears+Neck Module: 0x09  (4× ear servos + 3× neck servos, 2 UART controllers)
//   - Body Module:      0x0A  (Heart LCD, projector relay, WS2812B LEDs)
//   - Base Module:      0x0B  (Self-balancing: MPU6050, ODrive, kickstand servo)

#pragma once

// =============================================================================
// Module I2C Address Definitions
// =============================================================================

// MODULE_ID_HEAD (0x08)
// Hardware: ESP32-S3-WROOM-2 (N8R8)
// Components: 2× GC9A01 Round TFT (eyes), DFRobot SEN0395 mmWave sensor,
//             2× INMP441 I2S microphones, PAM8403 amplifier + speaker
// Responsibilities: Eye animations (60 FPS), presence detection, audio I/O
#define MODULE_ID_HEAD        0x08

// MODULE_ID_EARS_NECK (0x09)
// Hardware: ESP32-S3-WROOM-2 (N8R8)
// Components: Controller A (UART1): 4× Feetech SCS0009 servos (ears, 2-DOF × 2)
//             Controller B (UART2): 3× Feetech STS3215 servos (neck pan/tilt/roll)
// Responsibilities: Coordinated upper-body gestures, ear expressions, head tracking
#define MODULE_ID_EARS_NECK   0x09

// MODULE_ID_BODY (0x0A)
// Hardware: ESP32-S3-WROOM-2 (N8R8)
// Components: 1× GC9A01 Round TFT (heart), relay module (12V projector power),
//             WS2812B RGB LED strip (5-10 LEDs)
// Responsibilities: Heart animation (60 FPS, 50-120 BPM), projector control, status LEDs
#define MODULE_ID_BODY        0x0A

// MODULE_ID_BASE (0x0B)
// Hardware: ESP32-S3-WROOM-2 (N8R8)
// Components: MPU6050 IMU (200Hz PID), ODrive v3.6 (UART1, 2× 350W motors),
//             1× Feetech STS3215 kickstand servo (UART2)
// Responsibilities: Self-balancing (200Hz), fall detection, kickstand deployment
#define MODULE_ID_BASE        0x0B

// =============================================================================
// Common Registers (All Modules)
// =============================================================================

// REG_MODULE_ID (Read-only)
// Returns module identifier to verify correct I2C communication
// Values: 0x08 (Head), 0x09 (Ears+Neck), 0x0A (Body), 0x0B (Base)
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
