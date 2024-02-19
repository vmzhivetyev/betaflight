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

#include "sensors/gyro.h"

#include "config/feature.h"
#include "config/config.h"

#include "flight_health.h"

#define THRUST_IMBALANCE_GYRO_RATE_MAX 200
#define THRUST_IMBALANCE_THROTTLE_MIN 10
#define THRUST_IMBALANCE_TEMP_DISABLE_US 500000 // 0.5 sec

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
    static timeUs_t thrustImbalanceDisabledUntilUs = 0;

    if (ARMING_FLAG(ARMED)
        && !isFixedWing()
        && currentPidProfile->thrust_imbalance_threshold > 0
        && !isFlipOverAfterCrashActive()) {

        // Convert threshold to the working domain.
        const float threshold = currentPidProfile->thrust_imbalance_threshold * 1e5f;
        // Convert tenths to us.
        const timeUs_t triggerDelayUs = currentPidProfile->thrust_imbalance_trigger_delay * 100000;
        // Convert tenths to us.
        const timeUs_t untriggerDelayUs = currentPidProfile->thrust_imbalance_untrigger_delay * 100000;

        if (calculateThrottlePercentAbs() < THRUST_IMBALANCE_THROTTLE_MIN
            || gyroAbsRateDps(FD_ROLL) > THRUST_IMBALANCE_GYRO_RATE_MAX
            || gyroAbsRateDps(FD_PITCH) > THRUST_IMBALANCE_GYRO_RATE_MAX
            || gyroAbsRateDps(FD_YAW) > THRUST_IMBALANCE_GYRO_RATE_MAX) {
            thrustImbalanceDisabledUntilUs = currentTimeUs + THRUST_IMBALANCE_TEMP_DISABLE_US;
        }

        const bool detectionEnabled = cmpTimeUs(currentTimeUs, thrustImbalanceDisabledUntilUs) >= 0;
        if (detectionEnabled) {
            thrustImbalanceDisabledUntilUs = 0;
        }

        // Multiplication of I terms gives us a measure of total thrust imbalance in all of the axes.
        // A bad or loose prop will cause coupled I term buildup in roll, pitch, and yaw at the same time.
        const float iMult = fabsf(pidData[FD_ROLL].I * pidData[FD_PITCH].I * pidData[FD_YAW].I);

        if (detectionEnabled && iMult >= threshold) {
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
        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 1, currentPidProfile->thrust_imbalance_threshold);
        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 2, iMult / 1e5f);
        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 3, detectionEnabled ? 1 : 0);
    } else {
        thrustImbalanceTriggerUs = 0;
        thrustImbalanceUntriggerUs = 0;
        thrustImbalanceDetected = false;

        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 0, -1);
        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 1, 0);
        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 2, 0);
        DEBUG_SET(DEBUG_THRUST_IMBALANCE, 3, 0);
    }
}
#endif // USE_THRUST_IMBALANCE_DETECTION
