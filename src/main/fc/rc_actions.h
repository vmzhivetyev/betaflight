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
    RC_ACTION_NONE = 0,
    RC_ACTION_ADJUSTMENT_NEXT,
    RC_ACTION_ADJUSTMENT_PREV,
    RC_ACTION_ADJUSTMENT_INCREASE,
    RC_ACTION_ADJUSTMENT_DECREASE,
    RC_ACTION_OSD_NEXT,
    RC_ACTION_OSD_PREV,
    RC_ACTION_OSD_TOGGLE,
    RC_ACTION_VTX_TOGGLE,
    RC_ACTION_BEEPER_TOGGLE,
    RC_ACTION_LED_STRIP_TOGGLE,
    RC_ACTION_PID_PROFILE_NEXT,
    RC_ACTION_PID_PROFILE_PREV,
    RC_ACTION_PID_PROFILE_SAVE,
    RC_ACTION_PID_PROFILE_RESET,
    RC_ACTION_BB_TOGGLE,
    RC_ACTION_COUNT,
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
