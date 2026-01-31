/**
 * Logger - UDP Syslog Logger for Multi-Module Debugging
 *
 * Broadcasts log messages via UDP to a PC listener for real-time monitoring
 * across all OLAF modules (Head, Neck, Torso, Base).
 *
 * Features:
 * - Drop-in replacement for Serial.print/println
 * - Multiple log levels (DEBUG, INFO, WARN, ERROR)
 * - Auto-includes module name in messages
 * - Graceful fallback when WiFi unavailable
 * - Minimal performance impact (fire-and-forget UDP)
 *
 * Usage:
 *   Logger logger("NECK");
 *   logger.begin(logServerIP);
 *   logger.info("Servo detected");
 *   logger.error("I2C timeout");
 *
 * On PC: nc -lu 514
 */

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

class Logger {
public:
    enum Level {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3
    };

    /**
     * Constructor
     * @param moduleName Module identifier (e.g., "NECK", "HEAD", "BASE")
     */
    Logger(const char* moduleName);

    /**
     * Initialize logger with remote syslog server
     * @param serverIP IP address of PC running UDP listener (e.g., "192.168.1.100")
     * @param port UDP port (default 514 for syslog)
     * @return true if initialization successful
     */
    bool begin(IPAddress serverIP, uint16_t port = 514);
    bool begin(const char* serverIP, uint16_t port = 514);

    /**
     * Set minimum log level (messages below this level are ignored)
     * Default: INFO
     */
    void setLevel(Level level);

    /**
     * Check if logger is active and WiFi connected
     */
    bool isActive() const;

    /**
     * Log messages at different levels
     */
    void debug(const char* msg);
    void info(const char* msg);
    void warn(const char* msg);
    void error(const char* msg);

    /**
     * Formatted logging (printf-style)
     */
    void debugf(const char* format, ...);
    void infof(const char* format, ...);
    void warnf(const char* format, ...);
    void errorf(const char* format, ...);

    /**
     * Generic log with explicit level
     */
    void log(Level level, const char* msg);
    void logf(Level level, const char* format, ...);

    /**
     * Print methods (Serial-compatible API)
     */
    size_t print(const char* msg);
    size_t println(const char* msg);
    size_t printf(const char* format, ...);

private:
    static constexpr size_t kMaxMessageSize = 512;
    static constexpr const char* kLevelNames[] = {"DEBUG", "INFO", "WARN", "ERROR"};

    const char* _moduleName;
    IPAddress _serverIP;
    uint16_t _port;
    Level _minLevel;
    bool _initialized;
    WiFiUDP _udp;

    void sendMessage(Level level, const char* msg);
    void formatAndSend(Level level, const char* format, va_list args);
};
