/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"
#include "alt_hold.h"

#ifdef USE_ALTHOLD_MODE

#include "drivers/time.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "flight/mixer.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "config/config.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "osd/osd.h"
#include "common/printf.h"
#include "common/maths.h"
#include "common/filter.h"
#include "math.h"
#include "build/debug.h"

#define ALTHOLD_DELTATIME 1.0f/(float)ALTHOLD_TASK_PERIOD

PG_REGISTER_WITH_RESET_TEMPLATE(altholdConfig_t, altholdConfig, PG_ALTHOLD_CONFIG, 3);

PG_RESET_TEMPLATE(altholdConfig_t, altholdConfig,
    .altPidP = 50,
    .altPidD = 1,
    // .altPidI = 0,

    .velPidP = 7,
    .velPidD = 2,
    .velPidI = 80,

    .minThrottle = 0,
    .maxThrottle = 35,

    .enterFadeTimeDecisec = 1, // 0.1s
    .exitFadeTimeDecisec = 1, // 0.1s
);


void simplePidInit(simplePid_s* simplePid, float min, float max, float kp, float kd, float ki)
{
    simplePid->max = max;
    simplePid->min = min;
    simplePid->kp = kp;
    simplePid->kd = kd;
    simplePid->ki = ki;
    simplePid->lastErr = 0;
    simplePid->integral = 0;
}

float simplePidCalculate(simplePid_s* simplePid, float dt, float targetValue, float measuredValue)
{
    float error = targetValue - measuredValue;

    float pOut = simplePid->kp * error;

    float iOut = simplePid->ki * simplePid->integral;

    simplePid->integral += error * dt;

    float derivative = (error - simplePid->lastErr) / dt;
    float dOut = simplePid->kd * derivative;

    float output = pOut + iOut + dOut;
    output = constrainf(output, simplePid->min, simplePid->max);

    simplePid->lastErr = error;
    return output;
}

// in meters
float getCurrentAltitude(void)
{
#ifdef USE_BARO
    if (sensors(SENSOR_BARO) && baroIsCalibrated()) {
        return 0.01f * baro.altitude;
    }
#endif
    return 0.01f * getEstimatedAltitudeCm(); // it is smoothed internally by PT2
}

// void altHoldUpdateSmoothedAltitude(altHoldState_s* altHoldState)
// {
//     float reportedAltitude = getCurrentAltitude();

//     if (altHoldState->smoothedAltitude == 0.0f) {
//         altHoldState->smoothedAltitude = reportedAltitude;
//     }
//     float smoothFactor = 0.98f;
//     altHoldState->smoothedAltitude = (1.0f - smoothFactor) * reportedAltitude + smoothFactor * altHoldState->smoothedAltitude;


// }

void altHoldReset(altHoldState_s* altHoldState)
{
    simplePidInit(&altHoldState->altPid, -5.0f, 5.0f,
                  0.01f * altholdConfig()->altPidP,
                  0.01f * altholdConfig()->altPidD,
                  0);

    simplePidInit(&altHoldState->velPid, 0.0f, 1.0f,
                  0.01f * altholdConfig()->velPidP,
                  0.01f * altholdConfig()->velPidD,
                  0.01f * altholdConfig()->velPidI);
    
    altHoldState->throttle = mixerGetThrottle();
    pt2FilterSetState(&altHoldState->throttleLpf, altHoldState->throttle);

    altHoldState->enterTime = millis();
    altHoldState->exitTime = 0;
    altHoldState->targetAltitude = getCurrentAltitude();
    altHoldState->smoothedAltitude = altHoldState->targetAltitude;

    pt2FilterInit(&altHoldState->throttleLpf, 
        pt2FilterGain(1, ALTHOLD_DELTATIME)
    );

    pt1FilterInit(&altHoldState->altitudeLpf, 
        pt1FilterGain(1, ALTHOLD_DELTATIME)
    );

    pt1FilterInit(&altHoldState->velocityLpf, 
        pt1FilterGain(1, ALTHOLD_DELTATIME)
    );
}

void altHoldInit(altHoldState_s* altHoldState)
{
    altHoldState->altHoldEnabled = false;
    altHoldState->throttleFactor = 0.0f;
    altHoldState->velocityEstimate = 0.0f;
    altHoldReset(altHoldState);
}

void altHoldProcessTransitions(altHoldState_s* altHoldState) {
    bool newAltHoldEnabled = FLIGHT_MODE(ALTHOLD_MODE);
    uint32_t enterFadeMs = decisecondsToMillis(altholdConfig()->enterFadeTimeDecisec);
    uint32_t exitFadeMs = decisecondsToMillis(altholdConfig()->exitFadeTimeDecisec);

    if (FLIGHT_MODE(GPS_RESCUE_MODE) || failsafeIsActive()) {
        newAltHoldEnabled = false;
    }

    if (newAltHoldEnabled && !altHoldState->altHoldEnabled)
    {
        altHoldReset(altHoldState);
    }
    if (!newAltHoldEnabled && altHoldState->altHoldEnabled) {
        altHoldState->exitTime = millis();
    }
    altHoldState->altHoldEnabled = newAltHoldEnabled;

    uint32_t currTime = millis();

    if (newAltHoldEnabled) {
        uint32_t timeSinceEnter = currTime - altHoldState->enterTime;
        if (timeSinceEnter < enterFadeMs) {
            float delta = (float)timeSinceEnter / (float)enterFadeMs;
            altHoldState->throttleFactor = MAX(delta, altHoldState->throttleFactor);
        } else {
            altHoldState->throttleFactor = 1.0f;
        }
        return;
    }

    if (altHoldState->exitTime == 0) {
        altHoldState->throttleFactor = 0.0f;
        return;
    }

    uint32_t timeSinceExit = currTime - altHoldState->exitTime;
    if (timeSinceExit < exitFadeMs) {
        float delta = (float)timeSinceExit / (float)exitFadeMs;
        altHoldState->throttleFactor = MIN(altHoldState->throttleFactor, 1.0f - delta);
        return;
    }

    altHoldState->throttleFactor = 0.0f;
    // Probably a good idea to add this. Tho currTime is millis and it overflows in 49 days.
    // altHoldState->exitTime = 0;
}

void altHoldUpdate(altHoldState_s* altHoldState)
{
    altHoldProcessTransitions(altHoldState);

    t_fp_vector accelerationVector = {{
        acc.accADC[X],
        acc.accADC[Y],
        acc.accADC[Z]
    }};

    imuTransformVectorBodyToEarth(&accelerationVector);

    float measuredAltitude = getCurrentAltitude();
    float measuredAccel = 9.8f * (accelerationVector.V.Z - acc.dev.acc_1G) / acc.dev.acc_1G;

    altHoldState->measuredAltitude = measuredAltitude;
    altHoldState->measuredAccel = measuredAccel;

    altHoldState->velocityEstimate += measuredAccel * ALTHOLD_DELTATIME;
    altHoldState->velocityEstimate *= 0.999f;

    altHoldState->smoothedAltitude = pt1FilterApply(&altHoldState->altitudeLpf, altHoldState->measuredAltitude);
    altHoldState->smoothedVelocity = pt1FilterApply(&altHoldState->velocityLpf, altHoldState->velocityEstimate);

    DEBUG_SET(DEBUG_ALTHOLD, 0, (int16_t)(100.0f * measuredAccel));

    DEBUG_SET(DEBUG_ALTHOLD, 1, (int16_t)(100.0f * altHoldState->velocityEstimate));
    DEBUG_SET(DEBUG_ALTHOLD, 2, (int16_t)(100.0f * altHoldState->smoothedVelocity));

    DEBUG_SET(DEBUG_ALTHOLD, 3, (int16_t)(100.0f * altHoldState->measuredAltitude));
    DEBUG_SET(DEBUG_ALTHOLD, 4, (int16_t)(100.0f * altHoldState->smoothedAltitude));
    DEBUG_SET(DEBUG_ALTHOLD, 5, (int16_t)(100.0f * altHoldState->targetAltitude));

    if (altHoldState->altHoldEnabled) {
        float velocityTarget = simplePidCalculate(&altHoldState->altPid, ALTHOLD_DELTATIME, altHoldState->targetAltitude, altHoldState->smoothedAltitude);

        DEBUG_SET(DEBUG_ALTHOLD, 6, (int16_t)(100.0f * velocityTarget));

        float accelerationTarget = simplePidCalculate(&altHoldState->velPid, ALTHOLD_DELTATIME, velocityTarget, altHoldState->smoothedVelocity);

        DEBUG_SET(DEBUG_ALTHOLD, 7, (int16_t)(100.0f * accelerationTarget));

        // means it will go 100% throttle when max velPid PID output is produced.
        float newThrottle = accelerationTarget; 

        newThrottle = scaleRangef(newThrottle, 0.0f, 1.0f, 0.01f * altholdConfig()->minThrottle, 0.01f * altholdConfig()->maxThrottle);

        newThrottle = pt2FilterApply(&altHoldState->throttleLpf, newThrottle);

        altHoldState->throttle = newThrottle;
    }
}

altHoldState_s altHoldState;

void initAltHoldState(void) {
    altHoldInit(&altHoldState);
}

void updateAltHoldState(timeUs_t currentTimeUs) {
    altHoldUpdate(&altHoldState);

    UNUSED(currentTimeUs);
}

float getAltHoldThrottle(void) {
    return altHoldState.throttle;
}

float getAltHoldThrottleFactor(float currentThrottle) {
    if (!altHoldState.altHoldEnabled
        && altHoldState.exitTime != 0
        && (ABS(currentThrottle - altHoldState.throttle) < 0.15f)) {

        altHoldState.exitTime = 0;
    }
    return altHoldState.throttleFactor;
}

#endif
