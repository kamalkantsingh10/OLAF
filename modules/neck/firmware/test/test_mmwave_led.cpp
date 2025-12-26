/*
NECK MODULE - mmWave Sensor + LED Strip Test
- HLK-LD2450 mmWave sensor
- 8× WS2812B LED strip
*/

#include <Arduino.h>
#include <LD2450.h>
#include <FastLED.h>

// mmWave Sensor
#define MMWAVE_TX 38
#define MMWAVE_RX 39
#define MMWAVE_BAUD 256000

// LED Strip
#define LED_DATA_PIN 1
#define LED_COUNT 8

const int ledPin = 48; // Onboard LED

// HARDWARE SERIAL FOR ESP32-S3
HardwareSerial RadarSerial(2); // UART2

// INSTANCES
LD2450 ld2450;
CRGB leds[LED_COUNT];

void setup()
{
  // SERIAL FOR HOST / DEBUG MESSAGES
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  Serial.println("SETUP_STARTED");

  // BUILD-IN LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // SETUP LED STRIP
  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, LED_COUNT);
  FastLED.setBrightness(50); // 50/255 brightness
  fill_solid(leds, LED_COUNT, CRGB::Blue); // Startup: blue
  FastLED.show();
  Serial.println("LED strip initialized (8 LEDs)");

  // SETUP SENSOR
  RadarSerial.begin(MMWAVE_BAUD, SERIAL_8N1, MMWAVE_RX, MMWAVE_TX);
  ld2450.begin(RadarSerial, false);

  if(!ld2450.waitForSensorMessage()){
    Serial.println("SENSOR CONNECTION SEEMS OK");
    fill_solid(leds, LED_COUNT, CRGB::Green); // Sensor OK: green
    FastLED.show();
  }else{
    Serial.println("SENSOR TEST: GOT NO VALID SENSORDATA - PLEASE CHECK CONNECTION!");
    fill_solid(leds, LED_COUNT, CRGB::Red); // Sensor error: red
    FastLED.show();
  }

  delay(1000);
  fill_solid(leds, LED_COUNT, CRGB::Black); // Clear LEDs
  FastLED.show();

  Serial.println("SETUP_FINISHED");
}

void loop()
{
  const int sensor_got_valid_targets = ld2450.read();

  if (sensor_got_valid_targets > 0)
  {
    Serial.println("--- CORRECTED DATA ---");

    for (int i = 0; i < ld2450.getSensorSupportedTargetCount(); i++)
    {
      LD2450::RadarTarget t = ld2450.getTarget(i);

      if (!t.valid) continue; // Skip invalid targets

      // CORRECTION: Add 32768 to negative values (signed/unsigned interpretation issue)
      int16_t corrected_x = t.x;
      int16_t corrected_y = t.y;
      int16_t corrected_speed = t.speed;

      if (t.x < 0) corrected_x = (t.x + 32768)*-1;
      if (t.y < 0) corrected_y = (t.y + 32768)*-1;
      if (t.speed < 0) corrected_speed = (t.speed + 32768)*-1;

      // Recalculate distance with corrected values
      uint16_t corrected_dist = (uint16_t)sqrt((double)corrected_x * corrected_x + (double)corrected_y * corrected_y);

      Serial.print("Target ");
      Serial.print(i + 1);
      Serial.print(": x=");
      Serial.print(corrected_x);
      Serial.print("mm y=");
      Serial.print(corrected_y);
      Serial.print("mm dist=");
      Serial.print(corrected_dist);
      Serial.print("mm speed=");
      Serial.print(corrected_speed);
      Serial.println("cm/s");

      // Onboard LED: on if moving
      if (abs(t.speed) > 0) {
        digitalWrite(ledPin, HIGH);
      } else {
        digitalWrite(ledPin, LOW);
      }
    }
    Serial.println();

    // LED Strip visualization: show distance with color gradient
    if (sensor_got_valid_targets > 0) {
      // Get first valid target for LED display
      LD2450::RadarTarget t = ld2450.getTarget(0);
      if (t.valid) {
        int16_t corrected_x = t.x;
        int16_t corrected_y = t.y;
        if (t.x < 0) corrected_x = (t.x + 32768) * -1;
        if (t.y < 0) corrected_y = (t.y + 32768) * -1;
        uint16_t distance = (uint16_t)sqrt((double)corrected_x * corrected_x + (double)corrected_y * corrected_y);

        // Map distance (0-6000mm) to LED count (1-8)
        int numLeds = map(distance, 0, 6000, 8, 1);
        numLeds = constrain(numLeds, 1, 8);

        // Color: Green (close) -> Yellow -> Red (far)
        CRGB color;
        if (distance < 1000) {
          color = CRGB::Green;
        } else if (distance < 3000) {
          color = CRGB::Yellow;
        } else {
          color = CRGB::Red;
        }

        // Light up LEDs
        fill_solid(leds, LED_COUNT, CRGB::Black);
        fill_solid(leds, numLeds, color);
        FastLED.show();
      }
    } else {
      // No targets: all LEDs off
      fill_solid(leds, LED_COUNT, CRGB::Black);
      FastLED.show();
    }
  } else {
    // No targets detected
    fill_solid(leds, LED_COUNT, CRGB::Black);
    FastLED.show();
  }

  delay(1500);
}
