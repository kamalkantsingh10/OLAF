/**
 * Story 1.3 - Task 5 & 6: Dual GC9A01 Display + I2C Slave Test
 * ESP32-S3 with Manual CS Control + I2C Communication
 *
 * Features:
 * - Dual eye displays (left/right) with independent CS control
 * - I2C slave at address 0x08
 * - Displays text received from Raspberry Pi via I2C
 *
 * I2C Wiring:
 *   Pi GPIO2 (SDA) → ESP32 GPIO8
 *   Pi GPIO3 (SCL) → ESP32 GPIO9
 *   Pi GND → ESP32 GND (CRITICAL!)
 */

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include "config.h"  // Contains I2C_SLAVE_ADDRESS, pins

// CS pin definitions (Story 1.3)
constexpr uint8_t CS_LEFT_EYE = 5;   // GPIO5 - Left eye chip select
constexpr uint8_t CS_RIGHT_EYE = 15; // GPIO15 - Right eye chip select

// I2C receive buffer
constexpr size_t I2C_BUFFER_SIZE = 64;
volatile char i2cReceivedText[I2C_BUFFER_SIZE] = {0};
volatile bool newTextReceived = false;

TFT_eSPI tft = TFT_eSPI();

/**
 * Helper: Display text on both eyes
 */
void displayTextOnBothEyes(const char* text) {
  // Draw on LEFT EYE
  digitalWrite(CS_LEFT_EYE, LOW);
  digitalWrite(CS_RIGHT_EYE, HIGH);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString(text, 120, 120);

  digitalWrite(CS_LEFT_EYE, HIGH);

  // Draw on RIGHT EYE
  digitalWrite(CS_LEFT_EYE, HIGH);
  digitalWrite(CS_RIGHT_EYE, LOW);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString(text, 120, 120);

  digitalWrite(CS_RIGHT_EYE, HIGH);
}

/**
 * I2C Receive Handler
 * Called when Pi sends data to ESP32
 */
void onI2CReceive(int numBytes) {
  Serial.printf("[I2C RX] Received %d bytes: ", numBytes);

  int index = 0;
  while (Wire.available() && index < I2C_BUFFER_SIZE - 1) {
    char c = Wire.read();
    i2cReceivedText[index++] = c;
    Serial.print(c);
  }
  i2cReceivedText[index] = '\0';  // Null terminate
  Serial.println();

  newTextReceived = true;
}

/**
 * I2C Request Handler
 * Called when Pi requests data from ESP32
 */
void onI2CRequest() {
  // Simple acknowledgment: send module ID
  Wire.write(I2C_SLAVE_ADDRESS);
  Serial.println("[I2C TX] Sent module ID");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  OLAF Head Module");
  Serial.println("  Dual Eyes + I2C Slave");
  Serial.println("========================================\n");

  // Configure CS pins as outputs
  pinMode(CS_LEFT_EYE, OUTPUT);
  pinMode(CS_RIGHT_EYE, OUTPUT);

  // Set both CS HIGH (inactive) initially
  digitalWrite(CS_LEFT_EYE, HIGH);
  digitalWrite(CS_RIGHT_EYE, HIGH);

  Serial.println("Initializing displays...");

  // Initialize both displays together
  digitalWrite(CS_LEFT_EYE, LOW);
  digitalWrite(CS_RIGHT_EYE, LOW);
  tft.init();
  tft.setRotation(0);
  digitalWrite(CS_LEFT_EYE, HIGH);
  digitalWrite(CS_RIGHT_EYE, HIGH);

  Serial.println("✓ Displays initialized");

  // Initialize I2C as slave
  Serial.printf("\nInitializing I2C slave...\n");
  Serial.printf("  Address: 0x%02X\n", I2C_SLAVE_ADDRESS);
  Serial.printf("  SDA Pin: GPIO%d\n", kI2cSdaPin);
  Serial.printf("  SCL Pin: GPIO%d\n", kI2cSclPin);

  // ESP32-S3: First set pins, then begin as slave
  Wire.setPins(kI2cSdaPin, kI2cSclPin);
  Wire.begin(I2C_SLAVE_ADDRESS);  // Slave address only
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  Wire.setTimeout(500);  // 500ms timeout

  Serial.println("✓ I2C slave initialized\n");

  // Display initial message
  displayTextOnBothEyes("READY");

  Serial.println("========================================");
  Serial.println("Setup complete!");
  Serial.println("Waiting for I2C commands from Pi...");
  Serial.println("Test: sudo i2cdetect -y 1");
  Serial.println("Send text: i2cset -y 1 0x08 <bytes>");
  Serial.println("========================================\n");
}

void loop() {
  // Check if new text received via I2C
  if (newTextReceived) {
    newTextReceived = false;

    // Display received text on both eyes
    Serial.printf("[Display] Showing text: %s\n", (const char*)i2cReceivedText);
    displayTextOnBothEyes((const char*)i2cReceivedText);
  }

  delay(10);
}
