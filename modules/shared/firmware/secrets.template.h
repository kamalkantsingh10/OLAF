/**
 * secrets.template.h - WiFi Credentials for All Modules
 *
 * Setup: cp secrets.template.h secrets.h
 * Then edit secrets.h with your actual WiFi credentials.
 *
 * ESP32 tries networks in order until one connects.
 * Leave unused slots as "" to skip.
 */

#pragma once

// WiFi Networks (tries in order)
#define WIFI_SSID_1 "YourHomeWiFi"
#define WIFI_PASSWORD_1 "YourPassword"

#define WIFI_SSID_2 "YourWorkshopWiFi"  // Optional
#define WIFI_PASSWORD_2 "WorkshopPassword"

#define WIFI_SSID_3 ""  // Optional - use phone hotspot as backup
#define WIFI_PASSWORD_3 ""

// OTA Configuration
#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "olaf-module"  // Modules override this
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD "olaf-ota-2024"  // Match platformio.ini --auth flag
#endif
