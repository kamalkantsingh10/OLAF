/**
 * Story 1.3 - Task 5: Dual GC9A01 Eye Display Test (WORKING VERSION)
 * TFT_eSPI Library - ESP32-S3
 *
 * This version: Both displays work and show test patterns
 */

#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  OLAF Story 1.3 - Dual Eye Display Test");
  Serial.println("  TFT_eSPI + GC9A01 (240x240)");
  Serial.println("========================================\n");

  // Initialize TFT
  Serial.println("Initializing displays...");
  tft.init();
  tft.setRotation(0);
  Serial.println("✓ TFT initialized\n");

  // Test sequence
  Serial.println("========================================");
  Serial.println("Starting test sequence...\n");
  delay(1000);

  // Test 1: Fill RED
  Serial.println("TEST 1: Fill RED");
  tft.fillScreen(TFT_RED);
  delay(2000);

  // Test 2: Fill GREEN
  Serial.println("TEST 2: Fill GREEN");
  tft.fillScreen(TFT_GREEN);
  delay(2000);

  // Test 3: Fill BLUE
  Serial.println("TEST 3: Fill BLUE");
  tft.fillScreen(TFT_BLUE);
  delay(2000);

  // Test 4: Draw circles
  Serial.println("TEST 4: Draw concentric circles");
  tft.fillScreen(TFT_BLACK);

  int centerX = 120;
  int centerY = 120;
  uint16_t colors[] = {TFT_RED, TFT_YELLOW, TFT_GREEN, TFT_CYAN, TFT_BLUE, TFT_MAGENTA};

  for (int r = 100; r > 0; r -= 20) {
    tft.drawCircle(centerX, centerY, r, colors[(r / 20) % 6]);
  }
  delay(3000);

  // Test 5: Draw text
  Serial.println("TEST 5: Draw text");
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(3);
  tft.drawString("OLAF", 120, 120);
  delay(3000);

  Serial.println("\nTest sequence complete!");
  Serial.println("Entering animation loop...\n");
}

void loop() {
  static uint32_t frameCount = 0;
  static uint32_t lastFPSTime = 0;
  static int animPhase = 0;

  // Color animation cycling
  uint16_t colors[] = {TFT_RED, TFT_GREEN, TFT_BLUE, TFT_YELLOW, TFT_CYAN, TFT_MAGENTA, TFT_WHITE};
  int colorCount = 7;

  tft.fillScreen(colors[animPhase % colorCount]);

  // FPS calculation
  frameCount++;
  uint32_t now = millis();
  if (now - lastFPSTime >= 1000) {
    float fps = frameCount * 1000.0 / (now - lastFPSTime);
    Serial.printf("FPS: %.1f | Color: %d\n", fps, colors[animPhase % colorCount]);
    frameCount = 0;
    lastFPSTime = now;
  }

  animPhase++;
  delay(500);  // Change color every 500ms
}
