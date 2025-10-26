// config.h - Head Module Hardware Configuration
// ESP32-S3-WROOM-2 (N16R8) Pin Assignments
// Story 1.3: Head Module - Eye LCD Hardware Assembly

#pragma once

// ============================================================================
// I2C SLAVE CONFIGURATION (Communication with Raspberry Pi)
// ============================================================================

// I2C Slave Address - Head Module
#define I2C_SLAVE_ADDRESS 0x08  // Head module address on I2C bus

// I2C Pins (to Raspberry Pi)
constexpr uint8_t kI2cSdaPin = 8;   // GPIO8 - I2C SDA (flexible mapping on ESP32-S3)
constexpr uint8_t kI2cSclPin = 9;   // GPIO9 - I2C SCL (flexible mapping on ESP32-S3)

// I2C Speed
constexpr uint32_t kI2cFrequency = 400000;  // 400 kHz (Fast Mode)

// ============================================================================
// SPI DISPLAY CONFIGURATION (Dual GC9A01 Eye Displays)
// ============================================================================

// SPI Shared Pins (Both Eyes)
constexpr uint8_t kSpiSckPin  = 12;  // GPIO12 - SPI Clock (ESP32-S3 default)
constexpr uint8_t kSpiMosiPin = 11;  // GPIO11 - SPI MOSI (ESP32-S3 default)
constexpr uint8_t kSpiDcPin   = 2;   // GPIO2  - Data/Command (A0 on some boards)
constexpr uint8_t kSpiRstPin  = 4;   // GPIO4  - Reset (shared for both displays)

// SPI Chip Select Pins (Individual Eye Control)
constexpr uint8_t kLeftEyeCsPin  = 5;   // GPIO5  - Left Eye Chip Select
constexpr uint8_t kRightEyeCsPin = 15;  // GPIO15 - Right Eye Chip Select

// Optional: Backlight Control
constexpr uint8_t kDisplayLedPin = 10;  // GPIO10 - Backlight PWM (optional)

// Display Specifications
constexpr uint16_t kDisplayWidth  = 240;  // Pixels
constexpr uint16_t kDisplayHeight = 240;  // Pixels

// SPI Speed
constexpr uint32_t kSpiFrequency = 20000000;  // 20 MHz (optimal for 60 FPS dual displays)

// Target Frame Rate
constexpr uint8_t kTargetFps = 60;  // Frames per second
constexpr uint32_t kFrameTimeMs = 1000 / kTargetFps;  // 16ms per frame

// ============================================================================
// POWER CONFIGURATION
// ============================================================================

// Power Supply Notes:
// - ESP32-S3: Separate USB power (5V 2A recommended)
// - Raspberry Pi: Separate power supply (5V 3A minimum)
// - CRITICAL: Connect Pi GND to ESP32 GND for common ground reference

// ============================================================================
// FUTURE EXPANSION (Story 1.4+)
// ============================================================================

// Placeholder for future sensors on second I2C bus (master mode)
// constexpr uint8_t kI2cMasterSdaPin = 6;   // GPIO6 - Future sensor bus
// constexpr uint8_t kI2cMasterSclPin = 7;   // GPIO7 - Future sensor bus

// ============================================================================
// PIN USAGE SUMMARY
// ============================================================================
/*
 * GPIO2  - SPI DC (Data/Command)
 * GPIO4  - SPI RST (Display Reset)
 * GPIO5  - SPI CS Left Eye
 * GPIO8  - I2C SDA (to Pi)
 * GPIO9  - I2C SCL (to Pi)
 * GPIO10 - Display LED (Backlight PWM, optional)
 * GPIO11 - SPI MOSI
 * GPIO12 - SPI SCK
 * GPIO15 - SPI CS Right Eye
 *
 * Available for future use:
 * GPIO6, GPIO7 (I2C master to sensors)
 * GPIO13 (SPI MISO if needed)
 * Many others depending on dev board variant
 */
