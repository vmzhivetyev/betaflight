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
    .throttlePidP = 50, // 50 == 5 %
    .throttlePidD = 50, // 50 == 5 %
    .throttlePidI = 50, // 50 == 5 %
    .throttlePidIMax = 40, // %

    .throttlePidDFiltCutoffFreq = 20, // 20 == 2 Hz
    .throttleFiltCutoffFreq = 50, // 50 == 5 Hz
    .altitudeFiltCutoffFreq = 50, // 50 == 5 Hz

    .minThrottle = 0, // %
    .maxThrottle = 30, // %
    .hoverThrottle = 20, // %

    .maxAltitude = 100, // meters

    .enterFadeTimeDecisec = 1, // 1 == 0.1s
    .exitFadeTimeDecisec = 1, // 1 == 0.1s
);


void nicePidInit(
    nicePid_s* simplePid, 
    float min, float max, 
    float kp, float kd, float ki, float iMax, 
    float dCutoff_f, float intervalSeconds
) {
    simplePid->max = max;
    simplePid->min = min;
    simplePid->kp = kp;
    simplePid->kd = kd;
    simplePid->ki = ki;
    simplePid->iMax = iMax;

    simplePid->lastErr = 0;
    simplePid->integral = 0;

    // simplePid->lastP = 0;
    // simplePid->lastI = 0;
    // simplePid->lastD = 0;

    float gain = pt2FilterGain(dCutoff_f, intervalSeconds);
    pt2FilterInit(&simplePid->dTermLpf, gain);
}

float nicePidCalculate(nicePid_s* simplePid, float dt, float targetValue, float currentValue)
{
    float error = targetValue - currentValue;

    // I term
    simplePid->integral += simplePid->ki * error * dt;
    simplePid->integral = constrainf(simplePid->integral, -simplePid->iMax, simplePid->iMax);

    // D term
    float derivative = (error - simplePid->lastErr) / dt;

    // output
    float pOut = simplePid->kp * error;
    float iOut = simplePid->integral;
    float dOut = simplePid->kd * pt2FilterApply(&simplePid->dTermLpf, derivative);

    float output = pOut + iOut + dOut;

    simplePid->lastErr = error;
    
    DEBUG_SET(DEBUG_ALTHOLD, 0, (int16_t)(1000.0f * targetValue));
    DEBUG_SET(DEBUG_ALTHOLD, 1, (int16_t)(1000.0f * currentValue));
    DEBUG_SET(DEBUG_ALTHOLD, 2, (int16_t)(1000.0f * dOut));
    DEBUG_SET(DEBUG_ALTHOLD, 3, (int16_t)(1000.0f * pOut));
    DEBUG_SET(DEBUG_ALTHOLD, 4, (int16_t)(1000.0f * iOut));
    DEBUG_SET(DEBUG_ALTHOLD, 5, (int16_t)(1000.0f * derivative));

    output = constrainf(output, simplePid->min, simplePid->max);

    // 0 - targetAlt
    // 1 - smoothedAlt
    // 2 - D term smoothed
    // 3 - P term
    // 4 - I term
    // 5 - pid sum (not clamped)
    // 6 - hover throttle + pid out (after throttleFilter)
    // 7 - final throttle value (after adding tilt and clamping)

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

void altHoldReset(altHoldState_s* altHoldState)
{
    nicePidInit(
        &altHoldState->throttlePid,
        -1.0f, 1.0f,
        0.001f * altholdConfig()->throttlePidP,
        0.001f * altholdConfig()->throttlePidD,
        0.001f * altholdConfig()->throttlePidI,
        0.01f * altholdConfig()->throttlePidIMax,
        0.1f * altholdConfig()->throttlePidDFiltCutoffFreq,
        ALTHOLD_DELTATIME    
    );

    altHoldState->throttle = mixerGetThrottle();

    altHoldState->enterTime = millis();
    altHoldState->exitTime = 0;
    altHoldState->targetAltitude = altHoldState->smoothedAltitude;

    pt2FilterInit(&altHoldState->throttleLpf, 
        pt2FilterGain(0.1f * altholdConfig()->throttleFiltCutoffFreq, ALTHOLD_DELTATIME)
    );

    pt2FilterInit(&altHoldState->altitudeLpf, 
        pt2FilterGain(0.1f * altholdConfig()->altitudeFiltCutoffFreq, ALTHOLD_DELTATIME)
    );

    // Make next filter outputs to be equal to current values.
    pt2FilterSetState(&altHoldState->altitudeLpf, altHoldState->smoothedAltitude);
    pt2FilterSetState(&altHoldState->throttleLpf, mixerGetThrottle());

    DEBUG_SET(DEBUG_ALTHOLD, 0, (int16_t)(11111));
    DEBUG_SET(DEBUG_ALTHOLD, 1, (int16_t)(777));
    DEBUG_SET(DEBUG_ALTHOLD, 2, (int16_t)(altholdConfig()->throttlePidD));
    DEBUG_SET(DEBUG_ALTHOLD, 3, (int16_t)(altholdConfig()->throttlePidP));
    DEBUG_SET(DEBUG_ALTHOLD, 4, (int16_t)(altholdConfig()->throttlePidI));
    DEBUG_SET(DEBUG_ALTHOLD, 5, (int16_t)(altholdConfig()->altitudeFiltCutoffFreq));
    DEBUG_SET(DEBUG_ALTHOLD, 6, (int16_t)(altholdConfig()->throttlePidDFiltCutoffFreq));
    DEBUG_SET(DEBUG_ALTHOLD, 7, (int16_t)(altholdConfig()->throttleFiltCutoffFreq));
}

void altHoldInit(altHoldState_s* altHoldState)
{
    altHoldState->altHoldEnabled = false;
    altHoldState->throttleFactor = 0.0f;
    altHoldReset(altHoldState);
}

void altHoldProcessTransitions(altHoldState_s* altHoldState) {
    bool newAltHoldEnabled = FLIGHT_MODE(ALTHOLD_MODE);
    uint32_t enterFadeMs = decisecondsToMillis(altholdConfig()->enterFadeTimeDecisec);
    uint32_t exitFadeMs = decisecondsToMillis(altholdConfig()->exitFadeTimeDecisec);

    if (FLIGHT_MODE(GPS_RESCUE_MODE) || failsafeIsActive() || !ARMING_FLAG(ARMED)) {
        newAltHoldEnabled = false;
    }

    // Toggled on
    if (newAltHoldEnabled && !altHoldState->altHoldEnabled)
    {
        altHoldReset(altHoldState);
    }

    // Toggled off
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

void processThrottleInput(altHoldState_s* altHoldState)
{
    if (!altHoldState->altHoldEnabled) {
        return;
    }

    float throttleInput = scaleRangef(rcCommand[THROTTLE], 1000.f, 2000.f, -1.0f, 1.0f); // 0.0f to 1.0f

    float deadZoneSize = 0.8f;
    float negativeDeadZoneEdge = -1.0f * deadZoneSize / 2.0f;
    float positiveDeadZoneEdge = deadZoneSize / 2.0f;
    float maxAltitudeChangeSpeed = 1.0f; // m/s
    float effect = 0; // from -1f to 1f

    if (throttleInput < negativeDeadZoneEdge) {
        effect = scaleRangef(throttleInput, -1.0f, negativeDeadZoneEdge, -1.0f, 0.0f);

    } else if (throttleInput > positiveDeadZoneEdge) {
        effect = scaleRangef(throttleInput, positiveDeadZoneEdge, 1.0f, 0.0f, 1.0f);
    } else {
        return;
    }

    float prevTargetAltitude = altHoldState->targetAltitude;
    altHoldState->targetAltitude += effect * maxAltitudeChangeSpeed * ALTHOLD_DELTATIME;

    if (altholdConfig()->maxAltitude != 0) {
        float altitudeLimit = (float)altholdConfig()->maxAltitude;

        // if new > limit and old <= limit then clamp.
        //      So we don't clamp in case when we enabled the ALTHOLD above limit.
        if (altHoldState->targetAltitude > altitudeLimit && prevTargetAltitude <= altitudeLimit) {
            altHoldState->targetAltitude = altitudeLimit;
        }
    }
}

void altHoldUpdate(altHoldState_s* altHoldState)
{
    float measuredAltitude = getCurrentAltitude();
    altHoldState->measuredAltitude = measuredAltitude;
    altHoldState->smoothedAltitude = pt2FilterApply(&altHoldState->altitudeLpf, altHoldState->measuredAltitude);

    // 0 - targetAlt
    // 1 - smoothedAlt
    // 2 - d smoothed
    // 3 - p
    // 4 - i
    // 5 - pid sum (not clamped)
    // 6 - hover throttle + pid out (after throttleFilter)
    // 7 - final throttle value (after adding tilt and clamping)

    if (altHoldState->altHoldEnabled) {
        float throttleMin = 0.01f * altholdConfig()->minThrottle;
        float throttleMax = 0.01f * altholdConfig()->maxThrottle;
        float hoverThrottle = 0.01f * altholdConfig()->hoverThrottle;

        float pidOutput = nicePidCalculate(
            &altHoldState->throttlePid,
            ALTHOLD_DELTATIME, 
            altHoldState->targetAltitude,
            altHoldState->smoothedAltitude
        );

        float newThrottle = hoverThrottle + pidOutput;

        // clamp before filter
        newThrottle = constrainf(newThrottle, throttleMin, throttleMax);
        newThrottle = constrainf(newThrottle, 0, 1);

        // filter throttle
        if (altholdConfig()->throttleFiltCutoffFreq != 255) {
            newThrottle = pt2FilterApply(&altHoldState->throttleLpf, newThrottle);
        }

        DEBUG_SET(DEBUG_ALTHOLD, 6, (int16_t)(1000.0f * newThrottle)); // hoverThrottle + pidOutput (after filter)

        // add tilt
        float tiltAdjustment = 1.0f - getCosTiltAngle(); // 0 = flat, gets to 0.2 correcting on a windy day
        tiltAdjustment *= hoverThrottle;
        newThrottle += tiltAdjustment;

        // clamp in the end once more
        newThrottle = constrainf(newThrottle, throttleMin, throttleMax);
        newThrottle = constrainf(newThrottle, 0, 1);

        altHoldState->throttle = newThrottle;

        DEBUG_SET(DEBUG_ALTHOLD, 7, (int16_t)(1000.0f * newThrottle)); // final throttle value (with tilt and clamped)
    }

    processThrottleInput(altHoldState);

    // Give blackbox a chance to write things set by altHoldReset.
    // Make sure we are writing data to the blackbox as the last entity in the task.
    altHoldProcessTransitions(altHoldState);
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

float getAltHoldTargetAltitude(void) {
    return altHoldState.targetAltitude;
}

float getAltHoldCurrentAltitude(void) {
    return altHoldState.smoothedAltitude;
}

bool getAltHoldActive(void) {
    return altHoldState.altHoldEnabled;
}

#endif
