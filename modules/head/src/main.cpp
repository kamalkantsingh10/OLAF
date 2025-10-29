/**
 * main.cpp - Head Module Main Controller
 * Story 1.4: Head Module ESP32 Firmware - Eye Expressions
 *
 * Purpose:
 *   Integrates I2C slave communication, dual display driver, and expression engine
 *   to create emotion-driven eye expressions on dual GC9A01 round TFT displays.
 *
 * Architecture:
 *   ┌─────────────────────────────────────────────────────────────┐
 *   │                   Raspberry Pi Orchestrator                  │
 *   │         (Sends I2C commands: expression type, intensity)     │
 *   └────────────────────────────┬────────────────────────────────┘
 *                                │ I2C (0x08)
 *   ┌────────────────────────────▼────────────────────────────────┐
 *   │                    ESP32-S3 Head Module                      │
 *   │  ┌────────────┐   ┌──────────────┐   ┌──────────────────┐  │
 *   │  │ I2C Slave  │──▶│ Expression   │──▶│ Display Driver   │  │
 *   │  │ (0x08)     │   │ Engine       │   │ (Dual GC9A01)    │  │
 *   │  └────────────┘   └──────────────┘   └──────────────────┘  │
 *   └─────────────────────────────┬───────────────┬───────────────┘
 *                                 │               │
 *                    ┌────────────▼───┐   ┌───────▼────────────┐
 *                    │  Left Eye LCD  │   │  Right Eye LCD     │
 *                    │  (240×240 GC9A01) │ (240×240 GC9A01)   │
 *                    └────────────────┘   └────────────────────┘
 *
 * Main Loop Flow:
 *   1. Check for I2C commands (non-blocking)
 *   2. Process commands (update expression, trigger blink)
 *   3. Update expression engine (render frame)
 *   4. Monitor performance (warn if <60 FPS)
 *   5. Repeat at 60 FPS
 *
 * I2C Command Protocol:
 *   - Write to REG_EXPRESSION_TYPE (0x10): Set emotion (0-6)
 *   - Write to REG_EXPRESSION_INTENSITY (0x11): Set intensity (1-5)
 *   - Write to REG_BLINK_TRIGGER (0x12): Trigger blink animation
 *
 * Performance Target:
 *   - 60 FPS animation (16.67ms per frame)
 *   - I2C response latency <10ms
 *   - Eye synchronization ±1 frame
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include "../firmware/config.h"
#include "../firmware/i2c_slave.h"
#include "../firmware/gc9a01_driver_spi.h"
#include "../firmware/eye_expression.h"

// ============================================================================
// Global Objects
// ============================================================================

GC9A01DualDriver display_driver;
EyeExpressionEngine expression_engine;
// i2c_slave is extern from i2c_slave.cpp

// ============================================================================
// Performance Monitoring
// ============================================================================

struct PerformanceStats {
  uint32_t frame_count;
  uint32_t total_frame_time_ms;
  uint32_t min_frame_time_ms;
  uint32_t max_frame_time_ms;
  uint32_t last_report_millis;
};

PerformanceStats perf_stats = {0, 0, 9999, 0, 0};

/**
 * Update performance statistics
 *
 * Tracks frame timing for monitoring 60 FPS target.
 * Prints report every 5 seconds.
 */
void updatePerformanceStats(uint32_t frame_time_ms) {
  perf_stats.frame_count++;
  perf_stats.total_frame_time_ms += frame_time_ms;

  if (frame_time_ms < perf_stats.min_frame_time_ms) {
    perf_stats.min_frame_time_ms = frame_time_ms;
  }
  if (frame_time_ms > perf_stats.max_frame_time_ms) {
    perf_stats.max_frame_time_ms = frame_time_ms;
  }

  // Report every 5 seconds
  uint32_t now = millis();
  if (now - perf_stats.last_report_millis >= 5000) {
    float avg_frame_time = (float)perf_stats.total_frame_time_ms / perf_stats.frame_count;
    float avg_fps = 1000.0 / avg_frame_time;

    Serial.println("\n========== Performance Report ==========");
    Serial.printf("Frames rendered: %lu\n", perf_stats.frame_count);
    Serial.printf("Avg frame time: %.2f ms (%.1f FPS)\n", avg_frame_time, avg_fps);
    Serial.printf("Min frame time: %lu ms\n", perf_stats.min_frame_time_ms);
    Serial.printf("Max frame time: %lu ms\n", perf_stats.max_frame_time_ms);

    if (avg_fps < 55.0) {
      Serial.println("⚠️  WARNING: Frame rate below target (60 FPS)");
    } else {
      Serial.println("✓ Frame rate target achieved");
    }

    Serial.println("========================================\n");

    // Reset stats for next period
    perf_stats.frame_count = 0;
    perf_stats.total_frame_time_ms = 0;
    perf_stats.min_frame_time_ms = 9999;
    perf_stats.max_frame_time_ms = 0;
    perf_stats.last_report_millis = now;
  }
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(1000);  // Wait for serial connection

  Serial.println("\n");
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║      OLAF Head Module - Eye Expressions    ║");
  Serial.println("║      Story 1.4: ESP32 Firmware             ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println();

  // ── Step 1: Initialize I2C Slave ──────────────────────────────────────
  Serial.println("┌─ Initializing I2C Slave ─────────────────────┐");
  i2c_slave.begin(I2C_SLAVE_ADDRESS, kI2cSdaPin, kI2cSclPin);
  Serial.println("└──────────────────────────────────────────────┘");
  Serial.println();

  // ── Step 2: Initialize Display Driver ─────────────────────────────────
  Serial.println("┌─ Initializing Dual Display Driver ──────────┐");
  if (!display_driver.begin(kLeftEyeCsPin, kRightEyeCsPin)) {
    Serial.println("└─ ERROR: Display initialization failed ──────┘");
    Serial.println();
    Serial.println("⚠️  FATAL ERROR: Cannot continue without displays!");
    Serial.println("   Check wiring and power supply.");

    // Report error via I2C
    i2c_slave.setError(0x02);  // Error code 0x02: Display error

    // Enter safe mode - blink LED error pattern
    while (true) {
      delay(500);
    }
  }
  Serial.println("└──────────────────────────────────────────────┘");
  Serial.println();

  // ── Step 3: Initialize Expression Engine ──────────────────────────────
  Serial.println("┌─ Initializing Expression Engine ────────────┐");
  expression_engine.begin(&display_driver);
  Serial.println("└──────────────────────────────────────────────┘");
  Serial.println();

  // ── Step 4: Set Initial State ─────────────────────────────────────────
  Serial.println("┌─ Setting Initial State ──────────────────────┐");
  expression_engine.setExpression(EXPR_NEUTRAL, 2);  // Neutral, medium intensity
  i2c_slave.setStatus(STATUS_READY);  // Signal ready to orchestrator
  Serial.println("  Expression: NEUTRAL (intensity 2)");
  Serial.println("  Status: READY");
  Serial.println("└──────────────────────────────────────────────┘");
  Serial.println();

  // ── Setup Complete ─────────────────────────────────────────────────────
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║           SETUP COMPLETE ✓                 ║");
  Serial.println("║                                            ║");
  Serial.println("║  Waiting for I2C commands from Pi...       ║");
  Serial.println("║                                            ║");
  Serial.println("║  Test Commands:                            ║");
  Serial.println("║    i2cdetect -y 1        (detect module)   ║");
  Serial.println("║    i2cset -y 1 0x08 0x10 0x01  (happy)     ║");
  Serial.println("║    i2cset -y 1 0x08 0x11 0x05  (max intensity) ║");
  Serial.println("║    i2cset -y 1 0x08 0x12 0x01  (blink)     ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println();

  perf_stats.last_report_millis = millis();
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  // ── Step 1: Check for I2C Commands ────────────────────────────────────
  if (i2c_slave.hasNewCommand()) {
    I2CCommand cmd = i2c_slave.getCommand();

    Serial.printf("[Main] I2C command received - Expression: %d, Intensity: %d, Blink: %s\n",
                  cmd.expression_type, cmd.intensity, cmd.blink_requested ? "YES" : "NO");

    // Process expression change
    if (cmd.expression_type != expression_engine.getCurrentExpression() ||
        cmd.intensity != expression_engine.getCurrentIntensity()) {

      Serial.printf("[Main] New expression: %d (intensity %d)\n",
                    cmd.expression_type, cmd.intensity);

      // Update expression engine
      expression_engine.setExpression(cmd.expression_type, cmd.intensity);

      // Set status to BUSY during expression transition
      i2c_slave.setStatus(STATUS_BUSY);
    }

    // Process blink request
    if (cmd.blink_requested) {
      Serial.println("[Main] ✓ BLINK REQUESTED - Triggering blink animation!");
      expression_engine.triggerBlink();
      i2c_slave.clearBlinkRequest();  // Clear flag after processing
    }
  }

  // ── Step 2: Update Expression Engine ──────────────────────────────────
  uint32_t frame_time_ms = expression_engine.update();

  // Update performance stats
  updatePerformanceStats(frame_time_ms);

  // ── Step 3: Update Status ──────────────────────────────────────────────
  // If not blinking and expression transition complete, set status to READY
  if (!expression_engine.isBlinking()) {
    i2c_slave.setStatus(STATUS_READY);
  }

  // ── Step 4: Frame Rate Control ────────────────────────────────────────
  // Vector-style: No frame rate limiting needed since eyes are mostly static
  // Only delay a bit to prevent busy-looping
  delay(10);  // 10ms delay = check for I2C commands ~100 times per second
}

// ============================================================================
// Additional Helper Functions
// ============================================================================

/**
 * Emergency stop handler (future use)
 *
 * Called on critical errors to enter safe mode.
 * For now, just logs error and halts.
 */
void emergencyStop(const char* reason) {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║         ⚠️  EMERGENCY STOP ⚠️              ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.printf("Reason: %s\n", reason);
  Serial.println("\nSystem halted. Reset ESP32 to restart.");

  // Clear displays
  display_driver.fillBothEyes(TFT_RED);

  // Report error via I2C
  i2c_slave.setError(0x03);  // Error code 0x03: Hardware error

  // Halt forever
  while (true) {
    delay(1000);
  }
}
