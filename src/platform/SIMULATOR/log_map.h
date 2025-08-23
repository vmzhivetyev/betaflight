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

uint32_t millis(void);

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
#define LOG_UPDATE(key, format, ...) logUpdate(key, format, ##__VA_ARGS__);

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