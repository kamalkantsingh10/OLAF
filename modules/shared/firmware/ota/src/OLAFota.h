/**
 * OLAFota.h - OTA Update Library for OLAF Modules
 *
 * Handles WiFi connection, mDNS, and OTA updates.
 * Pure networking - no LED dependencies.
 * Just call begin() in setup() and handle() in loop().
 */

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

class OLAFota {
public:
  /**
   * Initialize OTA system
   *
   * @param hostname Module hostname (e.g., "olaf-neck")
   * @param password OTA password (must match platformio.ini)
   * @return true if WiFi connected, false if all networks failed
   */
  static bool begin(const char* hostname, const char* password);

  /**
   * Handle OTA requests - call this in loop()
   */
  static void handle();

  /**
   * Check if OTA update is in progress
   * Use this to skip other operations during update
   */
  static bool isUpdating();

private:
  static bool _ota_in_progress;

  // Helper: Try connecting to a WiFi network
  static bool tryConnectWiFi(const char* ssid, const char* password, int timeout_sec);
};
