#ifndef LOG_MAP_H
#define LOG_MAP_H

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration constants
#define MAX_LOG_ENTRIES 32
#define MAX_LOG_KEY_LEN 25
#define MAX_LOG_VALUE_LEN 80

// Core functions
void logInit(void);
void logUpdate(const char* key, const char* format, ...);
void logUpdateDouble(const char* key, double value);
void logUpdateInt(const char* key, int value);
void logDisplay(void);
void logRemove(const char* key);
void logClear(void);

// Display control functions
void logClearScreen(void);
void logMoveToTop(void);
void logSetDisplayMode(int bordered);

// Utility functions
int logGetCount(void);
const char* logGetValue(const char* key);
int logExists(const char* key);

// Throttled update macros (assumes millis() function exists)
#define LOG_THROTTLED_UPDATE(interval_ms, key, format, ...) do { \
    static uint32_t lastUpdate = 0; \
    uint32_t now = millis(); \
    if (now - lastUpdate >= (interval_ms)) { \
        logUpdate(key, format, ##__VA_ARGS__); \
        lastUpdate = now; \
    } \
} while(0)

#define LOG_THROTTLED_UPDATE_DOUBLE(interval_ms, key, value) do { \
    static uint32_t lastUpdate = 0; \
    uint32_t now = millis(); \
    if (now - lastUpdate >= (interval_ms)) { \
        logUpdateDouble(key, value); \
        lastUpdate = now; \
    } \
} while(0)

#define LOG_THROTTLED_UPDATE_INT(interval_ms, key, value) do { \
    static uint32_t lastUpdate = 0; \
    uint32_t now = millis(); \
    if (now - lastUpdate >= (interval_ms)) { \
        logUpdateInt(key, value); \
        lastUpdate = now; \
    } \
} while(0)

// Direct implementations instead of nested macros to avoid expansion issues
#define LOG_DISPLAY_50MS() do { \
    static uint32_t lastDisplay = 0; \
    uint32_t now = millis(); \
    if (now - lastDisplay >= 50) { \
        logDisplay(); \
        lastDisplay = now; \
    } \
} while(0)

#define LOG_DISPLAY_100MS() do { \
    static uint32_t lastDisplay = 0; \
    uint32_t now = millis(); \
    if (now - lastDisplay >= 100) { \
        logDisplay(); \
        lastDisplay = now; \
    } \
} while(0)

#define LOG_DISPLAY_THROTTLED(interval_ms) do { \
    static uint32_t lastDisplay = 0; \
    uint32_t now = millis(); \
    if (now - lastDisplay >= (interval_ms)) { \
        logDisplay(); \
        lastDisplay = now; \
    } \
} while(0)

// Convenience macros for common update intervals
#define LOG_UPDATE_100MS(key, format, ...) LOG_THROTTLED_UPDATE(100, key, format, ##__VA_ARGS__)
#define LOG_UPDATE_500MS(key, format, ...) LOG_THROTTLED_UPDATE(500, key, format, ##__VA_ARGS__)
#define LOG_UPDATE_1000MS(key, format, ...) LOG_THROTTLED_UPDATE(1000, key, format, ##__VA_ARGS__)

#define LOG_UPDATE_DOUBLE_100MS(key, value) LOG_THROTTLED_UPDATE_DOUBLE(100, key, value)
#define LOG_UPDATE_DOUBLE_500MS(key, value) LOG_THROTTLED_UPDATE_DOUBLE(500, key, value)
#define LOG_UPDATE_DOUBLE_1000MS(key, value) LOG_THROTTLED_UPDATE_DOUBLE(1000, key, value)

// Flight controller specific convenience macros
#define LOG_FLIGHT_VALUE(key, value) logUpdateDouble(key, (double)(value))
#define LOG_FLIGHT_VELOCITY(key, cmPerSec) logUpdateDouble(key, (double)((cmPerSec) / 100.0f) * 3.6)
#define LOG_FLIGHT_ALTITUDE(key, altCm) logUpdateDouble(key, (double)((altCm) / 100.0f))
#define LOG_FLIGHT_ANGLE(key, angle) logUpdateDouble(key, (double)(angle))

// Error codes
typedef enum {
    LOG_OK = 0,
    LOG_ERROR_FULL = -1,
    LOG_ERROR_NOT_FOUND = -2,
    LOG_ERROR_INVALID_KEY = -3
} LogResult;

#ifdef __cplusplus
}
#endif

#endif // LOG_MAP_H