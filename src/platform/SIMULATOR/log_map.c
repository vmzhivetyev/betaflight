#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "log_map.h"

typedef struct {
    char key[MAX_LOG_KEY_LEN];
    char value[MAX_LOG_VALUE_LEN];
    int valid;
} LogEntry;

typedef struct {
    LogEntry entries[MAX_LOG_ENTRIES];
    int count;
} LogMap;

static LogMap logMap = {0};

// Clear terminal and move cursor to top
void clearScreen(void) {
    printf("\033[2J\033[H");
}

// Move cursor to top without clearing
void moveToTop(void) {
    printf("\033[H");
}

// Comparison function for qsort
int compareEntries(const void* a, const void* b) {
    const LogEntry* entryA = (const LogEntry*)a;
    const LogEntry* entryB = (const LogEntry*)b;
    
    // Invalid entries go to the end
    if (!entryA->valid && !entryB->valid) return 0;
    if (!entryA->valid) return 1;
    if (!entryB->valid) return -1;
    
    return strcmp(entryA->key, entryB->key);
}

// Find or create entry by key
LogEntry* findOrCreateEntry(const char* key) {
    // First, try to find existing entry
    for (int i = 0; i < logMap.count; i++) {
        if (logMap.entries[i].valid && strcmp(logMap.entries[i].key, key) == 0) {
            return &logMap.entries[i];
        }
    }
    
    // If not found and we have space, create new entry
    if (logMap.count < MAX_LOG_ENTRIES) {
        LogEntry* entry = &logMap.entries[logMap.count];
        strncpy(entry->key, key, MAX_LOG_KEY_LEN - 1);
        entry->key[MAX_LOG_KEY_LEN - 1] = '\0';
        entry->valid = 1;
        logMap.count++;
        return entry;
    }
    
    return NULL; // No space available
}

// Update log entry
void logUpdate(const char* key, const char* format, ...) {
    LogEntry* entry = findOrCreateEntry(key);
    if (!entry) return;
    
    va_list args;
    va_start(args, format);
    vsnprintf(entry->value, MAX_LOG_VALUE_LEN, format, args);
    va_end(args);
}

// Update log entry with double value (formatted)
void logUpdateDouble(const char* key, double value) {
    logUpdate(key, "%+8.3f", value);
}

// Update log entry with int value
void logUpdateInt(const char* key, int value) {
    logUpdate(key, "%+8d", value);
}

// Display all log entries sorted by key
void logDisplay(void) {
    moveToTop();
    
    // Create a copy of entries for sorting
    LogEntry sortedEntries[MAX_LOG_ENTRIES];
    memcpy(sortedEntries, logMap.entries, sizeof(logMap.entries));
    
    // Sort the entries by key
    qsort(sortedEntries, logMap.count, sizeof(LogEntry), compareEntries);
    
    // Define column widths
    #define KEY_WIDTH MAX_LOG_KEY_LEN
    #define VALUE_WIDTH MAX_LOG_VALUE_LEN
    
    // Display header with correct border width
    printf("┌");
    for(int i = 0; i < KEY_WIDTH + 2; i++) printf("─");
    printf("┬");
    for(int i = 0; i < VALUE_WIDTH + 2; i++) printf("─");
    printf("┐\n");
    
    printf("│ %-*s │ %-*s │\n", KEY_WIDTH, "Key", VALUE_WIDTH, "Value");
    
    printf("├");
    for(int i = 0; i < KEY_WIDTH + 2; i++) printf("─");
    printf("┼");
    for(int i = 0; i < VALUE_WIDTH + 2; i++) printf("─");
    printf("┤\n");
    
    // Display sorted entries
    int validCount = 0;
    for (int i = 0; i < logMap.count; i++) {
        if (sortedEntries[i].valid) {
            // Truncate value if it's too long
            char truncated_value[VALUE_WIDTH + 1];
            strncpy(truncated_value, sortedEntries[i].value, VALUE_WIDTH);
            truncated_value[VALUE_WIDTH] = '\0';
            
            printf("│ %-*s │ %-*s │\n", 
                   KEY_WIDTH, sortedEntries[i].key,
                   VALUE_WIDTH, truncated_value);
            validCount++;
        }
    }
    
    printf("└");
    for(int i = 0; i < KEY_WIDTH + 2; i++) printf("─");
    printf("┴");
    for(int i = 0; i < VALUE_WIDTH + 2; i++) printf("─");
    printf("┘\n");
    
    // Fill remaining lines to prevent flicker
    int total_width = KEY_WIDTH + VALUE_WIDTH + 7; // 7 = borders + padding
    for (int i = validCount; i < MAX_LOG_ENTRIES; i++) {
        for(int j = 0; j < total_width; j++) printf(" ");
        printf("\n");
    }
    
    fflush(stdout);
}

// Initialize the log system (call once at startup)
void logInit(void) {
    clearScreen();
    memset(&logMap, 0, sizeof(LogMap));
}

// Remove entry by key
void logRemove(const char* key) {
    for (int i = 0; i < logMap.count; i++) {
        if (logMap.entries[i].valid && strcmp(logMap.entries[i].key, key) == 0) {
            // Shift all entries after this one back
            for (int j = i; j < logMap.count - 1; j++) {
                logMap.entries[j] = logMap.entries[j + 1];
            }
            logMap.count--;
            memset(&logMap.entries[logMap.count], 0, sizeof(LogEntry));
            break;
        }
    }
}
