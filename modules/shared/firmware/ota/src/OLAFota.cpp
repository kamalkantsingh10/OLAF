/**
 * OLAFota.cpp - OTA Update Library Implementation
 * Pure WiFi + OTA functionality, no LED dependencies
 */

#include "OLAFota.h"
#include "secrets.h"  // WiFi credentials

// Static member initialization
bool OLAFota::_ota_in_progress = false;

bool OLAFota::tryConnectWiFi(const char* ssid, const char* password, int timeout_sec) {
  if (ssid == nullptr || strlen(ssid) == 0) {
    return false;
  }

  Serial.printf("      Trying: %s (password: %s)... ", ssid, password);

  // Disconnect from previous attempt
  WiFi.disconnect(true);
  delay(100);

  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start < timeout_sec * 1000)) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" ✓ Connected!");
    return true;
  } else {
    // Show detailed error
    Serial.print(" ✗ Failed");
    wl_status_t status = WiFi.status();
    switch(status) {
      case WL_NO_SSID_AVAIL:
        Serial.println(" (Network not found - check SSID or 2.4GHz)");
        break;
      case WL_CONNECT_FAILED:
        Serial.println(" (Wrong password)");
        break;
      case WL_IDLE_STATUS:
        Serial.println(" (Timeout)");
        break;
      case WL_DISCONNECTED:
        Serial.println(" (Disconnected)");
        break;
      default:
        Serial.printf(" (Status: %d)\n", status);
    }
    return false;
  }
}

bool OLAFota::begin(const char* hostname, const char* password) {
  Serial.println();
  Serial.println("[OTA] Connecting to WiFi (multi-network fallback)...");

  WiFi.mode(WIFI_STA);

  // Scan for available networks
  Serial.println("      Scanning for WiFi networks...");
  int n = WiFi.scanNetworks();
  if (n > 0) {
    Serial.printf("      Found %d networks:\n", n);
    for (int i = 0; i < n && i < 10; i++) {  // Show max 10
      Serial.printf("        %d: %s (RSSI: %d dBm, Ch: %d)\n",
                    i+1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
    }
    Serial.println();
  } else {
    Serial.println("      ⚠ No networks found!");
  }

  bool connected = false;

  // Try each WiFi network in priority order
  connected = tryConnectWiFi(WIFI_SSID_1, WIFI_PASSWORD_1, 10);
  if (!connected) {
    connected = tryConnectWiFi(WIFI_SSID_2, WIFI_PASSWORD_2, 10);
  }
  if (!connected) {
    connected = tryConnectWiFi(WIFI_SSID_3, WIFI_PASSWORD_3, 10);
  }

  if (connected) {
    Serial.println();
    Serial.println("      ✓ WiFi connected!");
    Serial.printf("      IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("      Hostname: %s.local\n", hostname);
    Serial.printf("      RSSI: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println();
    Serial.println("      ✗✗✗ ALL WiFi networks FAILED! ✗✗✗");
    Serial.println("      → OTA will NOT be available!");
    Serial.println("      → Check credentials in shared/firmware/secrets.h");
    Serial.println();
    return false;
  }

  // Setup mDNS
  Serial.println();
  Serial.println("[OTA] Starting mDNS responder...");
  if (MDNS.begin(hostname)) {
    Serial.printf("      ✓ mDNS responder started: %s.local\n", hostname);
    MDNS.addService("arduino", "tcp", 3232);
  } else {
    Serial.println("      ✗ mDNS setup failed (not critical)");
  }

  // Setup OTA
  Serial.println();
  Serial.println("[OTA] Configuring OTA updates...");

  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(password);

  ArduinoOTA.onStart([]() {
    _ota_in_progress = true;
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("\n[OTA] Update started: " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Update complete!");
    _ota_in_progress = false;
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int percent = (progress / (total / 100));
    Serial.printf("[OTA] Progress: %u%%\r", percent);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    _ota_in_progress = false;
  });

  ArduinoOTA.begin();
  Serial.println("      ✓ OTA updates enabled");
  Serial.println();

  return true;
}

void OLAFota::handle() {
  ArduinoOTA.handle();
}

bool OLAFota::isUpdating() {
  return _ota_in_progress;
}
