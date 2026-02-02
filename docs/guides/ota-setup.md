# OTA Setup - Flash Firmware Over WiFi

You screwed up your PCB soldering and can't access the programming pins anymore? Me too. That's why we have OTA.

## How It Works

Flash firmware once via USB, then never touch that cable again. ESP32 downloads updates over WiFi instead.

**The secret:** ESP32 has two app slots. Run from slot A, download new firmware to slot B, reboot and swap. If slot B crashes 3 times, auto-rollback to slot A. You literally cannot brick it.

---

## Setup (One Time)

### 1. Configure WiFi

All modules share one WiFi config. Edit once, flash everywhere.

```bash
cd modules/shared/firmware
cp secrets.template.h secrets.h
nano secrets.h
```

Add your WiFi networks (it tries them in order until one works):

```cpp
#define WIFI_SSID_1 "HomeWiFi"
#define WIFI_PASSWORD_1 "password123"

#define WIFI_SSID_2 "WorkshopWiFi"  // Optional
#define WIFI_PASSWORD_2 "password456"

#define WIFI_SSID_3 "PhoneHotspot"  // Optional - emergency backup
#define WIFI_PASSWORD_3 "hotspot789"
```

Leave unused slots as `""` to skip.

### 2. First Flash (USB)

```bash
cd modules/neck/firmware  # or head/torso/base
pio run -t upload
pio device monitor
```

**Write down the IP address** that appears:
```
IP Address: 192.168.1.105  ← This one
```

### 3. Switch to OTA

Edit `modules/neck/firmware/platformio.ini` (lines 82-84), uncomment and set IP:

```ini
; Upload: Serial (first time) or OTA (after first flash)
; upload_port = /dev/ttyUSB0

; For OTA: uncomment these and set IP from Serial Monitor
upload_protocol = espota
upload_port = 192.168.1.105
upload_flags = --auth=olaf-neck-ota-2024
```

Done. Never touch USB again.

---

## Daily Use

### Adding New LED Patterns

All OTA complexity is hidden in `OLAFota` library. Just focus on your LED code.

Edit `main.cpp` in the marked section:

```cpp
void loop() {
  OLAFota::handle();  // Handles OTA - don't touch
  if (OLAFota::isUpdating()) return;  // Skip during updates

  // ===== YOUR LED PATTERNS GO HERE =====

  // Add your patterns here
  fill_solid(leds, LED_COUNT, CRGB::Purple);
  FastLED.show();
}
```

### Upload Changes

```bash
pio run -t upload
```

LEDs fill blue during upload (progress bar). Reboots automatically when done.

If upload fails, check ESP32 is powered on and IP hasn't changed (`ping 192.168.1.105`).

---

## Multi-Module Setup

Each module gets a unique hostname in its `main.cpp`:

```cpp
#define OTA_HOSTNAME "olaf-neck"   // Neck
#define OTA_HOSTNAME "olaf-head"   // Head
#define OTA_HOSTNAME "olaf-torso"  // Torso
#define OTA_HOSTNAME "olaf-base"   // Base
```

Then create environments in `platformio.ini`:

```ini
[env:neck-ota]
extends = env:esp32s3
upload_protocol = espota
upload_port = 192.168.1.105
upload_flags = --auth=olaf-neck-ota-2024

[env:head-ota]
extends = env:esp32s3
upload_protocol = espota
upload_port = 192.168.1.106
upload_flags = --auth=olaf-head-ota-2024
```

Upload to specific module:
```bash
pio run -e neck-ota -t upload
pio run -e head-ota -t upload
```

---

## Troubleshooting

**Can't connect:** IP changed. Check router DHCP or serial monitor.

**Auth failed:** Password mismatch. Check `OTA_PASSWORD` in code matches `--auth` in `platformio.ini`.

**All WiFi failed:** LEDs flash red 10 times at boot. Fix credentials in `shared/firmware/secrets.h` and reflash via USB.

**Module offline:** Power cycle it. OTA needs ESP32 running to receive updates.

---

## Why Multi-WiFi Instead of Access Point?

Old design: ESP32 creates its own WiFi network if yours fails.

New design: ESP32 tries 3 of your networks in order.

**Better because:**
- Works at home, workshop, and with phone hotspot
- All modules on same network (no switching)
- Shared credentials across all modules
- Simpler

**Trade-off:** If all 3 networks fail, need USB reflash. Use phone hotspot as #3 to avoid this.

---

## Technical Details

- **Partition scheme:** `default_16MB.csv` splits flash into 2× 6.5MB app slots + bootloader + NVS
- **OTA port:** 3232 (TCP)
- **Protocol:** ArduinoOTA library (built into ESP-IDF)
- **Security:** Password-protected (not encrypted - use trusted networks only)
- **Auto-rollback:** 3 failed boots → revert to previous firmware
- **LED feedback:** Handle in your module code (OTA library has no LED dependencies)

---

That's it. One USB flash per module, lifetime of WiFi updates.

---

## How PlatformIO Handles OTA

**Important:** You use the **same command** for both USB and OTA uploads:
```bash
pio run -t upload
```

**PlatformIO automatically detects** which method to use:
- If `upload_protocol = espota` is set → Uses WiFi OTA
- If commented out → Uses USB serial

**No special commands needed.** Just uncomment 3 lines in platformio.ini and PlatformIO handles everything.

**What happens during OTA:**
1. PlatformIO compiles firmware
2. Connects to ESP32 IP address (from `upload_port`)
3. Authenticates with password (from `upload_flags`)
4. Sends firmware over WiFi
5. ESP32 reboots with new firmware

**Visual feedback:**
- Terminal: `Uploading: [=========>] 67%`
- Serial Monitor: `[OTA] Progress: 67%`

**Switching back to USB:** Comment out the OTA lines in platformio.ini. Next upload uses USB automatically.
