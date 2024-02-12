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
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/utils.h"
#include "common/time.h"

#include "fc/core.h"
#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "config/feature.h"
#include "config/config.h"

#include "flight_health.h"

// Becomes `true` when sum of absolute I terms for all axes exceeds the threshold.
// This indicates a high likelihood of issues with props or motors, signaling the need for mechanical inspection.
static bool thrustImbalanceDetected = false;

bool isThrustImbalanceDetected(void)
{
#ifdef USE_THRUST_IMBALANCE_DETECTION
    return thrustImbalanceDetected;
#else
    return false;
#endif    
}

#ifdef USE_THRUST_IMBALANCE_DETECTION
void thrustImbalanceDetectionProcess(timeUs_t currentTimeUs)
{
    static timeUs_t thrustImbalanceTriggerUs = 0;
    static timeUs_t thrustImbalanceUntriggerUs = 0;

    if (ARMING_FLAG(ARMED)
        && !isFixedWing()
        && currentPidProfile->thrust_imbalance_isum_threshold > 0
        && !isFlipOverAfterCrashActive()
        && calculateThrottleStatus() != THROTTLE_LOW) {

        const float threshold = currentPidProfile->thrust_imbalance_isum_threshold;
        const timeUs_t triggerDelayUs = currentPidProfile->thrust_imbalance_trigger_delay * 100000; // tenths to us
        const timeUs_t untriggerDelayUs = currentPidProfile->thrust_imbalance_untrigger_delay * 100000; // tenths to us

        // Sum of absolute I terms gives us a measure of total weight/thrust imbalance.
        const float iSum = 
            fabsf(pidData[FD_ROLL].I) +
            fabsf(pidData[FD_PITCH].I) +
            fabsf(pidData[FD_YAW].I);

        if (iSum >= threshold) {
            if (thrustImbalanceTriggerUs == 0) {
                thrustImbalanceTriggerUs = currentTimeUs + triggerDelayUs;
            }
            if (cmpTimeUs(currentTimeUs, thrustImbalanceTriggerUs) >= 0) {
                thrustImbalanceUntriggerUs = currentTimeUs + untriggerDelayUs;
                thrustImbalanceDetected = true;
            }
        } else {
            thrustImbalanceTriggerUs = 0;
            if (thrustImbalanceUntriggerUs != 0 && cmpTimeUs(currentTimeUs, thrustImbalanceUntriggerUs) >= 0) {
                thrustImbalanceUntriggerUs = 0;
                thrustImbalanceDetected = false;
            }
        }

        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 0, thrustImbalanceDetected ? 1 : 0);
        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 1, threshold);
        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 2, iSum);
    } else {
        thrustImbalanceTriggerUs = 0;
        thrustImbalanceUntriggerUs = 0;
        thrustImbalanceDetected = false;

        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 0, -1);
        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 1, 0);
        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 2, 0);
    }
}
#endif // USE_THRUST_IMBALANCE_DETECTION
