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

#pragma once

#include <stdbool.h>

#include "fc/rc_modes.h"

#include "pg/pg.h"

typedef enum {
    // RC Action values for EdgeTX Special Function configuration
    // These values correspond to EdgeTX "override" settings for AUX channels
    // 
    // Setup instructions:
    // 1. In EdgeTX, create a Special Function with "Override Channel" action
    // 2. Set the channel to your desired AUX channel (commonly AUX8)
    // 3. Ensure the AUX channel is at center position with no mixes applied
    // 4. Use the values shown in comments below as the override value
    // 5. Bind this Special Function to a switch/button to trigger the action
    // 
    // Note: Current values are negative, but future additions may use positive values
    //
    // Warning: This will only work with ELRS "Wide" Switching mode and Std Telemetry Ratio 
    // (to guarantee resolution of 128 for AUX8).
    
    RC_ACTION_NONE = 0,                    // No action (neutral position should be 0 which shows as 1503 in Betaflight Configurator "Receiver" tab)
    RC_ACTION_ADJUSTMENT_NEXT,             // -97: Cycle to next adjustment setting
    RC_ACTION_ADJUSTMENT_PREV,             // -96: Cycle to previous adjustment setting  
    RC_ACTION_ADJUSTMENT_INCREASE,         // -94: Increase current adjustment value
    RC_ACTION_ADJUSTMENT_DECREASE,         // -92: Decrease current adjustment value
    RC_ACTION_OSD_NEXT,                    // -91: Switch to next OSD screen/page
    RC_ACTION_OSD_PREV,                    // -89: Switch to previous OSD screen/page
    RC_ACTION_OSD_TOGGLE,                  // -88: Toggle OSD on/off
    RC_ACTION_VTX_TOGGLE,                  // -86: Toggle VTX power/channel settings
    RC_ACTION_BEEPER_TOGGLE,               // -85: Toggle beeper on/off
    RC_ACTION_LED_STRIP_TOGGLE,            // -83: Toggle LED strip on/off
    RC_ACTION_PID_PROFILE_NEXT,            // -82: Switch to next PID profile
    RC_ACTION_PID_PROFILE_PREV,            // -80: Switch to previous PID profile
    RC_ACTION_PID_PROFILE_SAVE,            // -79: Save current PID profile to EEPROM
    RC_ACTION_PID_PROFILE_RESET,           // -77: Reset current PID profile to defaults
    RC_ACTION_BB_TOGGLE,                   // -76: Toggle blackbox logging on/off
    RC_ACTION_COUNT,                       // Total number of actions (for validation)
} rcAction_e;

// this is limited by ELRS channel resolution
// see https://www.expresslrs.org/software/switch-config/#summary-of-switch-configuration-modes
// AUX8 has
// a) 64 or 128 steps resolution in WIDE mode,
//    128 most likely if you are using Std telemtry ratio (1:64 for 250Hz link, 1:128 for 500Hz link)
// b) 16 steps in Hybrid mode.
//
// Yes, since we have 64 here, actions range detection will not work properly in Hybrid mode.
#define RC_ACTION_AUX_RESOLUTION 128

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
const char* getOSDActivatedAction(void);
#endif

void processRCActionsAUXInput(controlRateConfig_t *controlRateConfig);
