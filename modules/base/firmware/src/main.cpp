/**
 * OLAF Base Module - Incremental Build
 *
 * Phase 1: MPU-6500 IMU Only
 *
 * ============================================================================
 * WIRING DIAGRAM - MPU to Keyestudio ESP32 PLUS
 * ============================================================================
 *
 *   MPU-6500          ESP32 I2C Header
 *   --------          ----------------
 *   VCC  ──────────── 3.3V
 *   GND  ──────────── GND
 *   SDA  ──────────── GPIO 21
 *   SCL  ──────────── GPIO 22
 *   AD0  ──────────── GND     (I2C address = 0x68)
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <OLAFota.h>
#include <Logger.h>
#include "imu.h"

// ============================================================================
// CONFIG
// ============================================================================
#define OTA_HOSTNAME  "olaf-base"
#define OTA_PASSWORD  "olaf-base-ota-2024"
#define LOG_SERVER_IP "192.168.118.30"

// I2C pins - Keyestudio ESP32 PLUS (standard ESP32 I2C)
#ifndef IMU_SDA
#define IMU_SDA 21
#endif
#ifndef IMU_SCL
#define IMU_SCL 22
#endif

// ============================================================================
// GLOBALS
// ============================================================================
Logger logger("BASE");

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(2000);

    // OTA + WiFi
    bool wifi_ok = OLAFota::begin(OTA_HOSTNAME, OTA_PASSWORD);

    // Logger
    if (wifi_ok) {
        logger.begin(LOG_SERVER_IP);
        logger.setLevel(Logger::DEBUG);
        delay(100);
    }

    Serial.println("========================================");
    Serial.println("  OLAF Base Module - Phase 1: IMU");
    Serial.println("========================================");

    // I2C scan for debugging
    Serial.printf("I2C pins: SDA=%d, SCL=%d\n", IMU_SDA, IMU_SCL);
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(100000);  // 100kHz - slower is more reliable
    Serial.println("Scanning I2C...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  Found device at 0x%02X\n", addr);
        }
    }
    Serial.println("Scan done");

    // Initialize IMU
    if (!imu.begin(IMU_SDA, IMU_SCL)) {
        Serial.println("ERROR: IMU init FAILED!");
        while (1) {
            OLAFota::handle();
            delay(1000);
        }
    }

    Serial.printf("IMU ready! WHO_AM_I: 0x%02X\n", imu.getWhoAmI());
    if (imu.hasMagnetometer()) {
        Serial.println("Magnetometer: YES");
    } else {
        Serial.println("Magnetometer: NO (MPU-6500)");
    }
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
    OLAFota::handle();
    if (OLAFota::isUpdating()) return;

    if (!imu.update()) {
        Serial.println("IMU read error");
        delay(100);
        return;
    }

    if (imu.hasMagnetometer()) {
        Serial.printf("A:%5.2f %5.2f %5.2f G:%6.1f %6.1f %6.1f M:%5d %5d %5d\n",
                     imu.getAccX(), imu.getAccY(), imu.getAccZ(),
                     imu.getGyroX(), imu.getGyroY(), imu.getGyroZ(),
                     imu.getMagX(), imu.getMagY(), imu.getMagZ());
    } else {
        Serial.printf("A:%5.2f %5.2f %5.2f G:%6.1f %6.1f %6.1f\n",
                     imu.getAccX(), imu.getAccY(), imu.getAccZ(),
                     imu.getGyroX(), imu.getGyroY(), imu.getGyroZ());
    }

    delay(100);
}
