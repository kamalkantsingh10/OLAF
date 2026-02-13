/**
 * config.h - Head Module Hardware Configuration
 * ESP32-S3-DevKitC-1 (N16R8) Pin Assignments
 * Story 1.3: Develop Head ESP32 Firmware
 *
 * IMPORTANT: Pin assignments must match platformio.ini TFT_eSPI build flags
 */

#pragma once

#include <Arduino.h>

// ============================================================================
// FIRMWARE VERSION
// ============================================================================

#ifndef FIRMWARE_VERSION_STR
#define FIRMWARE_VERSION_STR "1.0.0"
#endif

constexpr uint8_t FIRMWARE_VERSION_MAJOR = 1;
constexpr uint8_t FIRMWARE_VERSION_MINOR = 0;
constexpr uint8_t FIRMWARE_VERSION_PATCH = 0;

// Combined version byte for I2C register (0xXY = major.minor)
constexpr uint8_t FIRMWARE_VERSION = 0x10;  // v1.0

// ============================================================================
// I2C SLAVE CONFIGURATION (Communication with Raspberry Pi)
// Via J1 Backplane header (LEFT SIDE of ESP32)
// ============================================================================

// I2C Slave Address - Head Module
constexpr uint8_t I2C_SLAVE_ADDRESS = 0x08;  // Head module address per wiring-guide.md

// I2C Pins (to Raspberry Pi via J1 Backplane)
constexpr uint8_t kI2cSdaPin = 8;   // GPIO8 - I2C SDA (J1-12)
constexpr uint8_t kI2cSclPin = 9;   // GPIO9 - I2C SCL (J1-15)

// I2C Speed
constexpr uint32_t kI2cFrequency = 400000;  // 400 kHz (Fast Mode)

// ============================================================================
// SPI DISPLAY CONFIGURATION (Dual GC9A01 Eye Displays)
// Via J3 Module-Specific header (RIGHT SIDE of ESP32)
// All 6 display control GPIOs grouped together (Physical pins 8-11, 13-14)
// ============================================================================

// SPI Shared Pins (Both Eyes) - RIGHT SIDE GPIOs
constexpr uint8_t kSpiSckPin  = 38;  // GPIO38 - SPI Clock (J3 Pin 8)
constexpr uint8_t kSpiMosiPin = 39;  // GPIO39 - SPI MOSI (J3 Pin 9)

// Shared DC and RST (Both Eyes) - RIGHT SIDE GPIOs
constexpr uint8_t kDisplayDcPin  = 40;  // GPIO40 - Data/Command SHARED (J3 Pin 10)
constexpr uint8_t kDisplayRstPin = 41;  // GPIO41 - Reset SHARED (J3 Pin 11)

// Individual Chip Select Pins - RIGHT SIDE GPIOs
constexpr uint8_t kLeftEyeCsPin  = 2;   // GPIO2 - Left Eye Chip Select (J3 Pin 13)
constexpr uint8_t kRightEyeCsPin = 1;   // GPIO1 - Right Eye Chip Select (J3 Pin 14)

// Backlight: Connected directly to 3.3V (always on, no GPIO control)

// Display Specifications
constexpr uint16_t kDisplayWidth  = 240;  // Pixels
constexpr uint16_t kDisplayHeight = 240;  // Pixels
constexpr uint16_t kDisplayCenterX = 120;
constexpr uint16_t kDisplayCenterY = 120;

// SPI Speed
constexpr uint32_t kSpiFrequency = 20000000;  // 20 MHz for 30-60 FPS

// Target Frame Rate
constexpr uint8_t kTargetFps = 30;  // Minimum frames per second
constexpr uint32_t kFrameTimeMs = 1000 / kTargetFps;  // ~33ms per frame

// ============================================================================
// WS2812 LED STRIP CONFIGURATION (Face Status Indicator)
// 8 LEDs arranged symmetrically: [0][1][2][3] | [4][5][6][7]
// Animations radiate from center (indices 3,4) to edges (indices 0,7)
// ============================================================================

constexpr uint8_t kLedStripPin = 4;          // GPIO4 - WS2812 data line
constexpr uint8_t kLedStripCount = 8;        // Number of LEDs
constexpr uint8_t kLedDefaultBrightness = 10; // Default brightness (0-255)
constexpr uint8_t kLedMaxBrightness = 80;    // Max brightness for effects

// LED strip center indices (for symmetrical animations)
constexpr uint8_t kLedCenterLeft = 3;        // Left side center
constexpr uint8_t kLedCenterRight = 4;       // Right side center

// Animation timing
constexpr uint32_t kLedUpdateIntervalMs = 20; // 50 FPS for smooth animations

// ============================================================================
// OTA CONFIGURATION
// ============================================================================

constexpr const char* kOtaHostname = "olaf-head";
constexpr const char* kOtaPassword = "olaf-ota-head";
constexpr uint16_t kOtaPort = 3232;

// ============================================================================
// RESERVED GPIOs (DO NOT USE)
// ============================================================================
/*
 * GPIO 26-32: Internal flash
 * GPIO 33-37: Internal PSRAM
 * GPIO 0, 3, 45, 46: Strapping pins
 */

// ============================================================================
// PIN USAGE SUMMARY (per wiring-guide.md backplane architecture)
// ============================================================================
/*
 * LEFT SIDE - J1 Backplane (Standard):
 *   GPIO8  - I2C SDA (J1-12, to Pi GPIO2)
 *   GPIO9  - I2C SCL (J1-15, to Pi GPIO3)
 *   GPIO17 - UART TX (J1-10, for ear servos - future)
 *   GPIO18 - UART RX (J1-11, for ear servos - future)
 *
 * RIGHT SIDE - J3 Module-Specific (Eye Displays):
 *   GPIO38 - SPI CLK (Pin 8, shared)
 *   GPIO39 - SPI MOSI (Pin 9, shared)
 *   GPIO40 - SPI DC (Pin 10, shared)
 *   GPIO41 - SPI RST (Pin 11, shared)
 *   GPIO2  - CS Left Eye (Pin 13)
 *   GPIO1  - CS Right Eye (Pin 14)
 *
 * Backlight: Connected directly to 3.3V (always on)
 */
