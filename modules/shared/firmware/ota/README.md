# OLAFota Library

Pure WiFi + OTA library for OLAF modules. No LED dependencies.

## Features

- Multi-WiFi fallback (tries 3 networks)
- Automatic rollback on failed updates
- Simple API: 2 functions

---

## Usage

```cpp
#include "ota/OLAFota.h"

#define OTA_HOSTNAME "olaf-neck"
#define OTA_PASSWORD "olaf-neck-ota-2024"

void setup() {
  Serial.begin(115200);

  // Initialize OTA
  OLAFota::begin(OTA_HOSTNAME, OTA_PASSWORD);
}

void loop() {
  // Handle OTA requests
  OLAFota::handle();

  // Skip your code during OTA
  if (OLAFota::isUpdating()) {
    return;
  }

  // Your code here
}
```

---

## API

### `OLAFota::begin(hostname, password)`

Initialize WiFi, mDNS, and OTA.

**Returns:** `true` if WiFi connected, `false` if all networks failed

### `OLAFota::handle()`

Process OTA requests. Call in `loop()`.

### `OLAFota::isUpdating()`

Returns `true` if OTA update in progress.

---

## WiFi Configuration

Edit `modules/shared/firmware/secrets.h`:

```cpp
#define WIFI_SSID_1 "HomeWiFi"
#define WIFI_PASSWORD_1 "password"

#define WIFI_SSID_2 "WorkshopWiFi"  // Optional
#define WIFI_PASSWORD_2 "password"

#define WIFI_SSID_3 ""  // Leave empty to skip
#define WIFI_PASSWORD_3 ""
```

ESP32 tries networks in order until one connects.

---

## Visual Feedback

Library has **no LED code**. Handle your own visual feedback in your module code if needed.

**Example with WS2812:**

```cpp
void loop() {
  OLAFota::handle();

  if (OLAFota::isUpdating()) {
    // Show blue progress bar on YOUR LEDs
    fill_solid(leds, LED_COUNT, CRGB::Blue);
    FastLED.show();
    return;
  }

  // Normal LED patterns
}
```
