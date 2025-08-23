/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "fc/runtime_config.h"
#include "io/beeper.h"

uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;

bool ARMING_ALWAYS_ENABLED = false;

static uint32_t enabledSensors = 0;

// Name must be no longer than OSD_WARNINGS_MAX_SIZE
// try to keep names within OSD_WARNINGS_PREFFERED_SIZE
const char *armingDisableFlagNames[]= {
    "NOGYRO",
    "FAILSAFE",
    "RXLOSS",
    "NOT_DISARMED",
    "BOXFAILSAFE",
    "RUNAWAY",
    "CRASH",
    "THROTTLE",
    "ANGLE",
    "BOOTGRACE",
    "NOPREARM",
    "LOAD",
    "CALIB",
    "CLI",
    "CMS",
    "BST",
    "MSP",
    "PARALYZE",
    "GPS",
    "RESCUE_SW",
    "DSHOT_TELEM",
    "REBOOT_REQD",
    "DSHOT_BBANG",
    "NO_ACC_CAL",
    "MOTOR_PROTO",
    "FLIP_SWITCH",
    "ALT_HOLD_SW",
    "POS_HOLD_SW",
    "ARM_SWITCH",
};
STATIC_ASSERT(ARRAYLEN(armingDisableFlagNames) == ARMING_DISABLE_FLAGS_COUNT, armingDisableFlagNames_size_mismatch);

static armingDisableFlags_e armingDisableFlags = 0;

void debug_log_armingDisableFlags(void) {
#ifdef SIMULATOR_BUILD
    char buffer[250];
    int len = 0;
    
    for (int i = 0; i < 32; i++) {
        armingDisableFlags_e currentFlag = 1 << i;
        bool isFlagSet = armingDisableFlags & currentFlag;
        
        if (isFlagSet) {
            // Check buffer space before writing
            int remainingSpace = sizeof(buffer) - len - 1; // -1 for null terminator
            if (remainingSpace <= 1) {
                break; // Not enough space for even a single character + null terminator
            }
            
            int written = snprintf(buffer + len, remainingSpace, "%s ", getArmingDisableFlagName(currentFlag));
            
            // snprintf returns number of chars that would have been written
            if (written > 0 && written < remainingSpace) {
                len += written;
            } else {
                // Not enough space, truncate gracefully
                break;
            }
        }
    }
    
    // Remove trailing space if present
    if (len > 0 && buffer[len - 1] == ' ') {
        buffer[len - 1] = '\0';
    } else {
        buffer[len] = '\0';
    }

    LOG_UPDATE("arming disable", buffer);
#endif // #ifdef SIMULATOR_BUILD
}

void setArmingDisabled(armingDisableFlags_e flag)
{
    if (!ARMING_ALWAYS_ENABLED) {
        armingDisableFlags = armingDisableFlags | flag;
    }
    debug_log_armingDisableFlags();
}

void unsetArmingDisabled(armingDisableFlags_e flag)
{
    armingDisableFlags = armingDisableFlags & ~flag;
    debug_log_armingDisableFlags();
}

bool isArmingDisabled(void)
{
    return armingDisableFlags && !ARMING_ALWAYS_ENABLED;
}

armingDisableFlags_e getArmingDisableFlags(void)
{
    if (ARMING_ALWAYS_ENABLED) {
        return 0;
    }
    return armingDisableFlags;
}

// return name for given flag
// will return first name (LSB) if multiple bits are passed
const char *getArmingDisableFlagName(armingDisableFlags_e flag)
{
    if (!flag) {
        return "NONE";
    }
    unsigned idx = ffs(flag & -flag) - 1;   // use LSB if there are multiple bits set
    return idx < ARRAYLEN(armingDisableFlagNames) ? armingDisableFlagNames[idx] : "UNKNOWN";
}

/**
 * Enables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint16_t enableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;

    flightModeFlags |= (mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

/**
 * Disables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint16_t disableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;

    flightModeFlags &= ~(mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}
