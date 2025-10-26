/**
 * Story 1.3 - Task 5: Dual GC9A01 Independent Display Test
 * ESP32-S3 with Manual CS Control
 *
 * Tests independent control of left and right eye displays
 * Left eye shows "LEFT" text, right eye shows "RIGHT" text
 */

#include <Arduino.h>
#include <TFT_eSPI.h>

// CS pin definitions (Story 1.3)
constexpr uint8_t CS_LEFT_EYE = 5;   // GPIO5 - Left eye chip select
constexpr uint8_t CS_RIGHT_EYE = 15; // GPIO15 - Right eye chip select

TFT_eSPI tft = TFT_eSPI();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  OLAF - Dual Eye Display Test");
  Serial.println("  Independent CS Control");
  Serial.println("========================================\n");

  // Configure CS pins as outputs
  pinMode(CS_LEFT_EYE, OUTPUT);
  pinMode(CS_RIGHT_EYE, OUTPUT);

  // Set both CS HIGH (inactive) initially
  digitalWrite(CS_LEFT_EYE, HIGH);
  digitalWrite(CS_RIGHT_EYE, HIGH);

  Serial.println("Initializing displays...");

  // Initialize both displays together
  // Both CS LOW during init
  digitalWrite(CS_LEFT_EYE, LOW);
  digitalWrite(CS_RIGHT_EYE, LOW);
  tft.init();
  tft.setRotation(0);

  // Deactivate both after init
  digitalWrite(CS_LEFT_EYE, HIGH);
  digitalWrite(CS_RIGHT_EYE, HIGH);

  Serial.println("✓ Displays initialized\n");
  Serial.println("Drawing LEFT on left eye...");

  // Draw on LEFT EYE
  digitalWrite(CS_LEFT_EYE, LOW);   // Activate left eye
  digitalWrite(CS_RIGHT_EYE, HIGH); // Ensure right eye inactive

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);  // Middle center
  tft.setTextSize(3);
  tft.drawString("LEFT", 120, 120);

  digitalWrite(CS_LEFT_EYE, HIGH);  // Deactivate left eye

  Serial.println("✓ LEFT eye drawn");
  delay(500);

  Serial.println("Drawing RIGHT on right eye...");

  // Draw on RIGHT EYE
  digitalWrite(CS_LEFT_EYE, HIGH);  // Ensure left eye inactive
  digitalWrite(CS_RIGHT_EYE, LOW);  // Activate right eye

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);  // Middle center
  tft.setTextSize(3);
  tft.drawString("RIGHT", 120, 120);

  digitalWrite(CS_RIGHT_EYE, HIGH); // Deactivate right eye

  Serial.println("✓ RIGHT eye drawn");
  Serial.println("\n========================================");
  Serial.println("Setup complete!");
  Serial.println("Both eyes should show different text");
  Serial.println("========================================\n");
}

void loop() {
  // Static display - nothing in loop
  delay(1000);
}
