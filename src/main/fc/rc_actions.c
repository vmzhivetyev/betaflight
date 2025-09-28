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

#include <math.h>

#include "platform.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/printf.h"

#include "config/feature.h"

#include "drivers/time.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_adjustments.h"
#include "fc/rc.h"

#include "flight/pid.h"
#include "flight/pid_init.h"

#include "io/beeper.h"
#include "io/ledstrip.h"
#include "io/pidaudio.h"

#include "osd/osd.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "rc_actions.h"

#include "scheduler/scheduler.h"

#define VALUE_DISPLAY_LATENCY_MS 2000

// this just corresponds to last AUX8 value
static rcAction_e prevActiveRCAction = 0;

// triggered means it was actually performed
static rcAction_e lastTriggeredRCAction = 0;
static char lastTriggeredRCActionResult[10];
static timeMs_t lastTriggeredRCActionMs = 0;

static void blackboxLogInflightActionEvent(rcAction_e action, int32_t newValue)
{
#ifndef USE_BLACKBOX
    UNUSED(adjustmentFunction);
    UNUSED(newValue);
#else
    if (blackboxConfig()->device) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = 127 - action; // for adjustment function index we are capped to use only 7bits, let's use the high end of the 0...127 range for actions
        eventData.newValue = newValue;
        eventData.floatFlag = false;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}

static const char * const rcActionLabels[] = {
    "NO ACTION",
    "NEXT ADJ",
    "PREV ADJ",
    "ADJ++",
    "ADJ--",
    "NEXT OSD",
    "PREV OSD",
    "TOGGLE OSD",
    "TOGGLE VTX",
    "TOGGLE BPR",
    "TOGGLE LED",
    "NEXT PID",
    "PREV PID", 
    "SAVED PID",
    "RESET PID",
    "TOGGLE BB",
};

static void performAction(rcAction_e action, controlRateConfig_t *controlRateConfig)
{
    lastTriggeredRCActionResult[0] = '\0';

    switch (action) {
    case RC_ACTION_NONE:
        // No action
        break;

    case RC_ACTION_ADJUSTMENT_NEXT:
    case RC_ACTION_ADJUSTMENT_PREV:
        changeActiveAdjustmentIndex(action == RC_ACTION_ADJUSTMENT_NEXT);
        break;

    case RC_ACTION_ADJUSTMENT_INCREASE:
    case RC_ACTION_ADJUSTMENT_DECREASE:
        performActiveAdjustmentChange(controlRateConfig, action == RC_ACTION_ADJUSTMENT_INCREASE);
        break;

    case RC_ACTION_OSD_NEXT:
    case RC_ACTION_OSD_PREV:
        changeOSDProfileNext(action == RC_ACTION_OSD_NEXT);
        break;

    case RC_ACTION_OSD_TOGGLE:
        forceHideOSD = !forceHideOSD;
        break;

    case RC_ACTION_VTX_TOGGLE:
        // Handle VTX toggle
        break;

    case RC_ACTION_BEEPER_TOGGLE:
        // Handle beeper toggle
        break;

    case RC_ACTION_LED_STRIP_TOGGLE:
        // Handle LED strip toggle
        break;

    case RC_ACTION_PID_PROFILE_NEXT:
    case RC_ACTION_PID_PROFILE_PREV:
        // let's restore it before loading another since we will lose the backup with loading
        restoreCurrentPidProfileFromBackup();
        uint8_t newIndex = changePidProfileNext(action == RC_ACTION_PID_PROFILE_NEXT);
        tfp_sprintf(lastTriggeredRCActionResult, "%d", newIndex + 1); // show 1-based index in osd
        break;

    case RC_ACTION_PID_PROFILE_SAVE:
        // saves an in-ram backup, doesn't affect EEPROM
        backupCurrentPidProfile();
        break;

    case RC_ACTION_PID_PROFILE_RESET:
        restoreCurrentPidProfileFromBackup();
        pidInitConfig(currentPidProfile);
        break;

    case RC_ACTION_BB_TOGGLE:
        // Handle BlackBox toggle
        break;

    default:
        // Handle unknown action
        break;
    }
}

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
void getOSDActivatedActionMessageIntoBuffer(char *buffer)
{
    // if invalid index
    if (lastTriggeredRCAction <= RC_ACTION_NONE || lastTriggeredRCAction >= RC_ACTION_COUNT) {
        return;
    }
    // if too much time passed since trigger
    if (cmp32(millis(), lastTriggeredRCActionMs + VALUE_DISPLAY_LATENCY_MS) >= 0) {
        return;
    }
    const char *name = &rcActionLabels[lastTriggeredRCAction][0];
    int pos = tfp_sprintf(buffer, "%s", name);
    // note: lastTriggeredRCActionResult[0] is 0 when there is no result description.
    if (lastTriggeredRCActionResult[0]) {
        tfp_sprintf(buffer + pos, " %s", lastTriggeredRCActionResult);
    }
}
#endif

void processRCActionsAUXInput(controlRateConfig_t *controlRateConfig)
{
    const bool canUseRxData = isRxReceivingSignal();

    if (!canUseRxData) {
        // don't reset prevActiveRCAction here to prevent reconnects triggering `perform`
        return;
    }
    
    int16_t auxValue = lroundf(rcData[AUX8]);

    float resolved_action_idx = scaleRangef(
        rcData[AUX8], 
        PWM_RANGE_MIN, PWM_RANGE_MAX, 
        1, RC_ACTION_AUX_RESOLUTION
    );

    // protect from invalid values
    if (resolved_action_idx <= 0 || resolved_action_idx > RC_ACTION_AUX_RESOLUTION) {
        prevActiveRCAction = RC_ACTION_NONE;
        return;
    }

    int32_t aux_pos_idx = lroundf(resolved_action_idx);

    // drop center, offset right half by 1
    const int32_t middle_buffer_max = 1503;
    rcAction_e offcenterAction = aux_pos_idx;
    if (auxValue >= 1500 && auxValue <= middle_buffer_max) {
        offcenterAction = RC_ACTION_NONE;
    } else if (auxValue > middle_buffer_max) {
        // since we skipped one position around 1503, we need to shift all right half postions by 1 to the left
        offcenterAction = aux_pos_idx - 1;
    }

    /*
    In Hybrid mode, AUX8 / Chan12 is 4-bit / 16-position ... there's no "center position" (1500us) 
    in a 16-position switch, so using AUX8 with a 3-position switch means it will come out as 1533 at the flight
    controller.
    */
    if (lroundf(auxValue) == 1533) {
        offcenterAction = RC_ACTION_NONE;
    }

    rcAction_e activeAction = offcenterAction;

    if (activeAction >= RC_ACTION_COUNT) {
        activeAction = RC_ACTION_NONE;
    }

    if (activeAction != prevActiveRCAction && activeAction != RC_ACTION_NONE) {
        beeperConfirmationBeeps(1);
        performAction(activeAction, controlRateConfig);
        blackboxLogInflightActionEvent(activeAction, 0);
        lastTriggeredRCAction = activeAction;
        lastTriggeredRCActionMs = millis();
    }

    prevActiveRCAction = activeAction;

    // if (indexError < 0.25f)
    // int newValue = applyStepAdjustment(controlRateConfig, adjustmentFunction, delta);
    // setConfigDirty();
    // pidInitConfig(currentPidProfile);
    // adjustmentState->ready = false;
    // updateOsdAdjustmentData(newValue, adjustmentFunction);
}
