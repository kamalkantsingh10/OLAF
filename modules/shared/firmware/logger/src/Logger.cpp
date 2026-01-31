/**
 * Logger - UDP Syslog Logger Implementation
 */

#include "Logger.h"

constexpr const char* Logger::kLevelNames[];

Logger::Logger(const char* moduleName)
    : _moduleName(moduleName),
      _port(514),
      _minLevel(INFO),
      _initialized(false) {
}

bool Logger::begin(IPAddress serverIP, uint16_t port) {
    _serverIP = serverIP;
    _port = port;
    _initialized = true;
    return true;
}

bool Logger::begin(const char* serverIP, uint16_t port) {
    IPAddress ip;
    if (ip.fromString(serverIP)) {
        return begin(ip, port);
    }
    return false;
}

void Logger::setLevel(Level level) {
    _minLevel = level;
}

bool Logger::isActive() const {
    return _initialized && (WiFi.status() == WL_CONNECTED);
}

void Logger::debug(const char* msg) {
    log(DEBUG, msg);
}

void Logger::info(const char* msg) {
    log(INFO, msg);
}

void Logger::warn(const char* msg) {
    log(WARN, msg);
}

void Logger::error(const char* msg) {
    log(ERROR, msg);
}

void Logger::debugf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    formatAndSend(DEBUG, format, args);
    va_end(args);
}

void Logger::infof(const char* format, ...) {
    va_list args;
    va_start(args, format);
    formatAndSend(INFO, format, args);
    va_end(args);
}

void Logger::warnf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    formatAndSend(WARN, format, args);
    va_end(args);
}

void Logger::errorf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    formatAndSend(ERROR, format, args);
    va_end(args);
}

void Logger::log(Level level, const char* msg) {
    if (level < _minLevel) {
        return;  // Below minimum log level
    }
    sendMessage(level, msg);
}

void Logger::logf(Level level, const char* format, ...) {
    if (level < _minLevel) {
        return;
    }
    va_list args;
    va_start(args, format);
    formatAndSend(level, format, args);
    va_end(args);
}

size_t Logger::print(const char* msg) {
    log(INFO, msg);
    return strlen(msg);
}

size_t Logger::println(const char* msg) {
    log(INFO, msg);
    return strlen(msg) + 1;
}

size_t Logger::printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    formatAndSend(INFO, format, args);
    va_end(args);
    return 0;  // UDP doesn't track bytes sent
}

void Logger::sendMessage(Level level, const char* msg) {
    if (!isActive()) {
        return;  // WiFi not connected or logger not initialized
    }

    // Format: [MODULE][LEVEL] message\n
    char buffer[kMaxMessageSize];
    snprintf(buffer, kMaxMessageSize, "[%s][%s] %s\n",
             _moduleName, kLevelNames[level], msg);

    // Send via UDP (fire-and-forget)
    _udp.beginPacket(_serverIP, _port);
    _udp.write((uint8_t*)buffer, strlen(buffer));
    _udp.endPacket();
}

void Logger::formatAndSend(Level level, const char* format, va_list args) {
    if (!isActive() || level < _minLevel) {
        return;
    }

    char buffer[kMaxMessageSize];
    vsnprintf(buffer, kMaxMessageSize, format, args);
    sendMessage(level, buffer);
}
