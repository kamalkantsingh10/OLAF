/**
 * main.cpp - OLAF Head Module Firmware
 * Story 1.3: Develop Head ESP32 Firmware
 *
 * Integrates:
 *   - Dual GC9A01 eye displays (expression + status animations)
 *   - WS2812 LED strip (status animations)
 *   - I2C slave interface (Pi commands)
 *   - OTA firmware updates
 *
 * System status flow (driven by I2C REG_SYSTEM_STATUS or demo cycle):
 *   IDLE → WOKE_UP → LISTENING → PROCESSING → SPEAKING → GOING_IDLE → IDLE
 */

#include <Arduino.h>
#include <OLAFota.h>
#include "config.h"
#include "display_driver.h"
#include "animation_engine.h"
#include "i2c_slave.h"
#include "led_strip.h"

// OTA credentials
#define OTA_HOSTNAME "olaf-head"
#define OTA_PASSWORD "olaf-head-ota-2024"

// Components
GC9A01DualDriver display;
EyeExpressionEngine eyes;

// Frame timing
uint32_t frame_count = 0;
uint32_t fps_report_ms = 0;

// Demo mode: cycle through statuses for testing without Pi
bool demo_mode = true;
uint32_t demo_last_change_ms = 0;
uint8_t demo_status = 0;

// Demo timing per status (ms)
constexpr uint32_t DEMO_DURATIONS[] = {
    2000,   // 0: IDLE (asleep 2s)
    1000,   // 1: WOKE_UP (1s for animation to complete)
    4000,   // 2: LISTENING (4s attentive)
    4000,   // 3: PROCESSING (4s thinking)
    6000,   // 4: SPEAKING (6s with expression changes)
    1000,   // 5: GOING_IDLE (1s for animation)
};

// Expression cycle during SPEAKING demo
uint8_t demo_expression = 0;
uint32_t demo_expr_change_ms = 0;
constexpr uint32_t DEMO_EXPR_INTERVAL_MS = 1500;

void setStatus(uint8_t status) {
    eyes.setSystemStatus(status);
    ledStripSetState(static_cast<LedState>(status));
}

void updateDemoMode() {
    uint32_t now = millis();
    uint32_t elapsed = now - demo_last_change_ms;

    if (elapsed >= DEMO_DURATIONS[demo_status]) {
        demo_last_change_ms = now;

        // Advance to next status
        demo_status++;
        if (demo_status > 5) demo_status = 0;

        setStatus(demo_status);

        // Reset expression cycle for SPEAKING
        if (demo_status == 4) {
            demo_expression = 0;
            demo_expr_change_ms = now;
            eyes.setExpression(EXPR_HAPPY, 3);
        }
    }

    // Cycle expressions during SPEAKING demo
    if (demo_status == 4 && (now - demo_expr_change_ms >= DEMO_EXPR_INTERVAL_MS)) {
        demo_expr_change_ms = now;
        demo_expression++;
        if (demo_expression >= EXPR_COUNT) demo_expression = 0;

        // Vary intensity slightly
        uint8_t intensity = 2 + (demo_expression % 3);
        eyes.setExpression(demo_expression, intensity);
    }
}

void processI2CCommands() {
    if (!i2c_slave.hasNewCommand()) return;

    I2CCommand cmd = i2c_slave.getCommand();

    // Any I2C command disables demo mode
    if (demo_mode) {
        demo_mode = false;
        Serial.println("[HEAD] I2C command received — demo mode OFF");
    }

    // System status drives both eyes and LEDs
    static uint8_t last_system_status = 0xFF;
    if (cmd.system_status != last_system_status) {
        setStatus(cmd.system_status);
        last_system_status = cmd.system_status;
    }

    // LED emotional overlay — a SEPARATE event. Independent of
    // system_status and expression_type; only touches the LED wash.
    static uint8_t last_led_overlay = 0xFF;
    if (cmd.led_overlay != last_led_overlay) {
        ledStripSetOverlay(static_cast<LedOverlay>(cmd.led_overlay));
        last_led_overlay = cmd.led_overlay;
    }

    // Expression (only effective when status = SPEAKING)
    if (eyes.getSystemStatus() == 4) {
        eyes.setExpression(cmd.expression_type, cmd.intensity);
    }

    // Look direction
    eyes.setLookDirection(cmd.look_x, cmd.look_y);

    // Blink trigger
    if (cmd.blink_requested) {
        eyes.triggerBlink();
        i2c_slave.clearBlinkRequest();
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.printf("\n\n=== OLAF Head Module v%s ===\n\n", FIRMWARE_VERSION_STR);

    // OTA
    if (OLAFota::begin(OTA_HOSTNAME, OTA_PASSWORD)) {
        Serial.println("[OTA] Ready");
    } else {
        Serial.println("[OTA] Failed to initialize");
    }

    // LED strip
    ledStripInit();

    // Display driver
    if (!display.begin(kLeftEyeCsPin, kRightEyeCsPin)) {
        Serial.println("[HEAD] FATAL: Display init failed!");
        // Show error on LEDs and halt
        while (true) { delay(1000); }
    }

    // Animation engine
    eyes.begin(&display);

    // I2C slave
    i2c_slave.begin(I2C_SLAVE_ADDRESS, kI2cSdaPin, kI2cSclPin);
    i2c_slave.setStatus(STATUS_READY);

    // Start in IDLE (asleep), then immediately wake up
    setStatus(0);  // IDLE
    delay(200);
    setStatus(1);  // WOKE_UP

    demo_last_change_ms = millis();
    demo_status = 1;  // Currently in WOKE_UP

    Serial.println("[HEAD] Setup complete — starting in demo mode");
    Serial.println("[HEAD] Send any I2C command to switch to I2C control");

    fps_report_ms = millis();
}

void loop() {
    OLAFota::handle();

    // Process I2C commands
    processI2CCommands();

    // Demo mode auto-cycling
    if (demo_mode) {
        updateDemoMode();
    }

    // Update eye animation (frame-rate limited internally)
    eyes.update();

    // Update LED strip (timing managed internally)
    ledStripUpdate();

    // FPS reporting every 10 seconds
    frame_count++;
    uint32_t now = millis();
    if (now - fps_report_ms >= 10000) {
        float fps = frame_count * 1000.0f / (now - fps_report_ms);
        Serial.printf("[HEAD] FPS: %.1f | Wake: %.0f%% | Status: %d | Expr: %d\n",
                      fps, eyes.getWakeLevel() * 100.0f,
                      eyes.getSystemStatus(), eyes.getCurrentExpression());
        fps_report_ms = now;
        frame_count = 0;
    }
}
