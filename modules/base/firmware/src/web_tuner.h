/**
 * Web-Based PID Tuner with Auto-Tune
 *
 * Serves a dashboard at http://olaf-base.local with:
 *   - Live charts: pitch, PID output, motor velocities
 *   - Sliders: Kp, Ki, Kd, target pitch
 *   - Buttons: BALANCE / STOP / TARE / AUTO-TUNE
 *   - Auto-tune: Ziegler-Nichols relay method
 *
 * WebSocket streams telemetry JSON at 20Hz.
 * WebSocket receives commands and PID gain changes from browser.
 */

#pragma once

#include <Arduino.h>
#include <Preferences.h>

// Live-tunable PID gains (initialized from config.h defaults)
extern float tuneKp;
extern float tuneKi;
extern float tuneKd;
extern float tuneTarget;

// Commands from web UI (consumed by main loop)
enum WebCommand {
    WEB_CMD_NONE = 0,
    WEB_CMD_BALANCE,
    WEB_CMD_STOP,
    WEB_CMD_TARE,
    WEB_CMD_AUTOTUNE
};

// Auto-tune state
enum AutoTuneState {
    AT_IDLE,
    AT_FINDING_KU,     // Increasing Kp until sustained oscillation
    AT_MEASURING,       // Measuring oscillation period
    AT_DONE             // Results ready
};

class WebTuner {
public:
    void begin();
    void update();

    void sendTelemetry(float pitch, float roll, float pid_out,
                       float m0_vel, float m1_vel, uint8_t accuracy,
                       float gyro_x = 0, float gyro_y = 0, float gyro_z = 0);

    WebCommand getCommand();

    /**
     * Run one step of auto-tune. Call at PID rate (200Hz) while auto-tuning.
     * @param pitch Current pitch angle in degrees
     * @return true while auto-tuning is in progress
     */
    bool autoTuneStep(float pitch);

    /** Is auto-tune currently running? */
    bool isAutoTuning() const { return atState_ != AT_IDLE; }

    volatile WebCommand pendingCmd_ = WEB_CMD_NONE;

    /** Save current PID gains to flash */
    void saveGains();

    /** Record one sample to the log buffer. Call at 200Hz when logging. */
    void logSample(uint32_t time_ms, float pitch, float pid_out,
                   float m0, float m1, float gx, float gy, float gz);

    /** Start/stop logging */
    void startLog();
    void stopLog();
    bool isLogging() const { return logging_; }

private:
    // Data log — ring buffer in PSRAM for full 200Hz recording
    // Each row: time_ms,pitch,pid,m0,m1,gx,gy,gz (8 floats = 32 bytes)
    // 10 seconds at 200Hz = 2000 rows = 64KB — fits easily in 8MB PSRAM
    static constexpr uint16_t kLogMaxRows = 4000;  // 20 seconds at 200Hz
    struct LogRow {
        uint32_t time_ms;
        float pitch;
        float pid_out;
        float m0;
        float m1;
        float gx;
        float gy;
        float gz;
    };
    LogRow* logBuf_ = nullptr;
    uint16_t logCount_ = 0;
    bool logging_ = false;
    uint32_t logStartMs_ = 0;
    // Auto-tune state
    AutoTuneState atState_ = AT_IDLE;
    float atKp_ = 0.0f;              // Current test Kp
    float atKpStep_ = 1.0f;          // Kp increment per step
    uint32_t atStepStartMs_ = 0;     // When current Kp step started
    uint32_t atStepDurationMs_ = 3000; // How long to test each Kp value

    // Oscillation detection
    float atPeaks_[20];              // Recent pitch peaks
    uint32_t atPeakTimes_[20];       // Timestamps of peaks
    uint8_t atPeakCount_ = 0;
    float atPrevPitch_ = 0.0f;
    float atPrevDelta_ = 0.0f;
    bool atRising_ = true;

    // Results
    float atKu_ = 0.0f;              // Critical gain
    float atTu_ = 0.0f;              // Oscillation period (seconds)

    Preferences prefs_;

    void autoTuneStart();
    void autoTuneApplyResults();
    void sendAutoTuneStatus(const char* msg);
    bool detectSustainedOscillation();
    void loadGains();
};

extern WebTuner webTuner;
