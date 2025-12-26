/*
 * SERVO ID SETTER - Change servo ID via UART
 * Based on FTServo ProgramEprom example
 *
 * CRITICAL: Connect ONLY ONE servo at a time!
 *
 * Usage:
 * 1. Connect single servo to controller
 * 2. Edit OLD_ID and NEW_ID below
 * 3. Upload and run
 * 4. LED will blink when done
 * 5. Power cycle servo
 * 6. Disconnect and repeat for next servo
 */

#include <Arduino.h>
#include <SCServo.h>

#define SERVO_TX 17
#define SERVO_RX 18
#define SERVO_BAUD 1000000
#define LED_PIN 48

// ============================================
// CONFIGURE THESE VALUES:
// ============================================
#define OLD_ID 3       // Current servo ID (factory default = 1)
#define NEW_ID 4        // New servo ID to set (2 = tilt_left)
// ============================================

SMS_STS sms_sts;
HardwareSerial ServoSerial(1);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  delay(1000);

  Serial.println("\n=== SERVO ID SETTER ===\n");
  Serial.print("Changing servo ID: ");
  Serial.print(OLD_ID);
  Serial.print(" → ");
  Serial.println(NEW_ID);
  Serial.println("\n⚠️  WARNING: Connect ONLY ONE servo!");
  Serial.println();

  // Init servo bus
  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX, SERVO_TX);
  sms_sts.pSerial = &ServoSerial;

  delay(1000);

  // Verify servo exists
  Serial.print("Checking for servo at ID ");
  Serial.print(OLD_ID);
  Serial.print("... ");

  int ID = sms_sts.Ping(OLD_ID);
  if (sms_sts.getLastError()) {
    Serial.println("NOT FOUND!");
    Serial.println("\nTroubleshooting:");
    Serial.println("  - Check servo power (12V)");
    Serial.println("  - Check wiring (TX/RX on GPIO 17/18)");
    Serial.println("  - Verify OLD_ID is correct");
    Serial.println("  - Only ONE servo connected?");

    // Flash LED rapidly = error
    while(1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }

  Serial.println("FOUND!");

  // Change ID
  Serial.print("\nChanging ID to ");
  Serial.print(NEW_ID);
  Serial.print("... ");

  digitalWrite(LED_PIN, LOW);

  // Unlock EPROM
  sms_sts.unLockEprom(OLD_ID);
  delay(10);

  // Write new ID
  sms_sts.writeByte(OLD_ID, SMS_STS_ID, NEW_ID);
  delay(10);

  // Lock EPROM
  sms_sts.LockEprom(NEW_ID);
  delay(10);

  Serial.println("DONE!");

  // Verify new ID
  Serial.print("Verifying new ID... ");
  delay(500);  // Wait for servo to update

  int newID = sms_sts.Ping(NEW_ID);
  if (!sms_sts.getLastError()) {
    Serial.println("SUCCESS! ✓");
    Serial.println();
    Serial.print("Servo now has ID: ");
    Serial.println(NEW_ID);
    Serial.println();
    Serial.println("Next steps:");
    Serial.println("  1. Power cycle the servo");
    Serial.println("  2. Disconnect this servo");
    Serial.println("  3. Connect next servo and repeat");

    // Slow blink = success
    while(1) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  } else {
    Serial.println("FAILED!");
    Serial.println("Try power cycling servo and run again.");

    // Fast blink = error
    while(1) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }
}

void loop() {
  // Nothing - all work done in setup()
}
