/**
 * main.cpp - Minimal Display Test with OTA
 * Story 1.3: Develop Head ESP32 Firmware
 */

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <OLAFota.h>

// OTA Configuration
#define OTA_HOSTNAME "olaf-head"
#define OTA_PASSWORD "olaf-head-ota-2024"

// CS Pins for dual displays
#define LEFT_CS  2
#define RIGHT_CS 1

TFT_eSPI tft = TFT_eSPI();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== MINIMAL DISPLAY TEST ===\n");

  // OTA init
  if (OLAFota::begin(OTA_HOSTNAME, OTA_PASSWORD)) {
    Serial.println("[OTA] Ready");
  } else {
    Serial.println("[OTA] Failed");
  }

  // Setup CS pins
  pinMode(LEFT_CS, OUTPUT);
  pinMode(RIGHT_CS, OUTPUT);
  digitalWrite(LEFT_CS, HIGH);
  digitalWrite(RIGHT_CS, HIGH);

  Serial.println("[Display] Initializing...");
  Serial.printf("  MOSI=%d, SCLK=%d, DC=%d, RST=%d\n", TFT_MOSI, TFT_SCLK, TFT_DC, TFT_RST);
  Serial.printf("  Left CS=%d, Right CS=%d\n", LEFT_CS, RIGHT_CS);

  // Init with BOTH displays selected
  digitalWrite(LEFT_CS, LOW);
  digitalWrite(RIGHT_CS, LOW);
  tft.init();
  Serial.println("[Display] tft.init() done");

  // Fill RED
  Serial.println("[Display] Filling RED...");
  tft.fillScreen(TFT_RED);
  delay(1000);

  // Fill GREEN
  Serial.println("[Display] Filling GREEN...");
  tft.fillScreen(TFT_GREEN);
  delay(1000);

  // Fill BLUE
  Serial.println("[Display] Filling BLUE...");
  tft.fillScreen(TFT_BLUE);
  delay(1000);

  // Fill WHITE
  Serial.println("[Display] Filling WHITE...");
  tft.fillScreen(TFT_WHITE);
  delay(1000);

  // Draw a circle
  Serial.println("[Display] Drawing circle...");
  tft.fillScreen(TFT_BLACK);
  tft.fillCircle(120, 120, 80, TFT_YELLOW);

  // Deselect
  digitalWrite(LEFT_CS, HIGH);
  digitalWrite(RIGHT_CS, HIGH);

  Serial.println("\n=== SETUP COMPLETE ===");
  Serial.println("You should see a yellow circle on black background");
}

void loop() {
  OLAFota::handle();
  delay(100);
}
