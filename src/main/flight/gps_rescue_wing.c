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

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#ifdef USE_WING
#ifdef USE_GPS_RESCUE

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "drivers/time.h"

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "io/gps.h"
#include "rx/rx.h"
#include "pg/autopilot.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"

#include "gps_rescue.h"

typedef enum {
    RESCUE_IDLE, // 0
    RESCUE_INITIALIZE, // 1
    RESCUE_FLY_HOME, // 2
    RESCUE_DESCEND_TO_LOITER, // 3
    RESCUE_LOITER, // 4
    RESCUE_DESCEND_TO_LAND, // 5
    RESCUE_APPROACH_MANEUVRE, // 6
    RESCUE_WAIT_FOR_COURSE_AWAY, // 7
    RESCUE_PREPARE_TO_LAND, // 8
    RESCUE_WAIT_FOR_LANDING_DISTANCE, // 9
    RESCUE_LAND, // 10
    RESCUE_DISARM_ON_IMPACT,
    RESCUE_ABORT,
} rescuePhase_e;

static const char* rescuePhaseStrings[] = {
    [RESCUE_IDLE] = "RESCUE_IDLE",
    [RESCUE_INITIALIZE] = "RESCUE_INITIALIZE", 
    [RESCUE_FLY_HOME] = "RESCUE_FLY_HOME",
    [RESCUE_DESCEND_TO_LOITER] = "RESCUE_DESCEND_TO_LOITER",
    [RESCUE_LOITER] = "RESCUE_LOITER",
    [RESCUE_DESCEND_TO_LAND] = "RESCUE_DESCEND_TO_LAND",
    [RESCUE_APPROACH_MANEUVRE] = "RESCUE_APPROACH_MANEUVRE",
    [RESCUE_WAIT_FOR_COURSE_AWAY] = "RESCUE_WAIT_FOR_COURSE_AWAY",
    [RESCUE_PREPARE_TO_LAND] = "RESCUE_PREPARE_TO_LAND",
    [RESCUE_WAIT_FOR_LANDING_DISTANCE] = "RESCUE_WAIT_FOR_LANDING_DISTANCE",
    [RESCUE_LAND] = "RESCUE_LAND",
    [RESCUE_DISARM_ON_IMPACT] = "RESCUE_DISARM_ON_IMPACT",
    [RESCUE_ABORT] = "RESCUE_ABORT",
};

#define RESCUE_PHASE_STR(phase) \
    ((phase) < (sizeof(rescuePhaseStrings)/sizeof(rescuePhaseStrings[0])) && rescuePhaseStrings[phase] ? \
     rescuePhaseStrings[phase] : "UNKNOWN_RESCUE_PHASE")

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_FLYAWAY,
    RESCUE_GPSLOST,
    RESCUE_LOWSATS,
    RESCUE_CRASH_FLIP_DETECTED,
    RESCUE_STALLED,
    RESCUE_TOO_CLOSE,
    RESCUE_NO_HOME_POINT
} rescueFailureState_e;

typedef struct {
    float maxAltitudeCm;
    float returnAltitudeCm;
    float descentDistanceM;
    
    float targetAltitudeCm;
    float targetCourseDecidegrees;
    float targetVelocityCmS;

    float pitchAngleLimitDeg;
    float rollAngleLimitDeg;
    float disarmThreshold;

    int8_t secondsFailing;
    float velocityITermAccumulator;
    float velocityPidCutoff;
    float velocityPidCutoffModifier;
    float velocityItermAttenuator;
    float velocityItermRelax;
} rescueIntent_s;

typedef struct {
    float currentAltitudeCm;
    float distanceToHomeCm;
    float distanceToHomeM;
    uint16_t groundSpeedCmS;
    int16_t directionToHome;
    bool healthy;
    float errorAngle;
    float gpsDataIntervalSeconds;
    float altitudeDataIntervalSeconds;
    float gpsRescueTaskIntervalSeconds;
    float velocityToHomeCmS;
    float alitutudeStepCm;
    float maxPitchStep;
    float absErrorAngle;
    float imuYawCogGain;
} rescueSensorData_s;

typedef struct {
    rescuePhase_e phase;
    rescueFailureState_e failure;
    rescueSensorData_s sensor;
    rescueIntent_s intent;
    bool isAvailable;
    timeMs_t loiterActivatedTime;
    timeMs_t waitForCourseAwayActivatedTime;
    timeMs_t waitForCourseHomeActivatedTime;
    timeMs_t landActivatedTime;
} rescueState_s;

#define GPS_RESCUE_MAX_ANGULAR_ITERM     1500    // max iterm value for pitch in degrees * 100
#define GPS_RESCUE_ALLOWED_YAW_RANGE   30.0f  // yaw error must be less than this to enter fly home phase, and to pitch during descend()

float       gpsRescueAngle[RP_AXIS_COUNT] = { 0, 0 };
bool        magForceDisable = true;
// static bool newGPSData = false;
static pt2Filter_t throttleDLpf;
static pt1Filter_t velocityDLpf;
static pt3Filter_t velocityUpsampleLpf;
static float previousVelocityError = 0.0f;
static float velocityI = 0.0f;

rescueState_s rescueState;

#define desiredAltitudeCm rescueState.intent.targetAltitudeCm

// FORWARD DECLARATIONS

float g_gpsRescueGetVelocityPIDSum(bool newGpsData);

///////////////////////

void gpsRescueInit(void)
{
    rescueState.sensor.gpsRescueTaskIntervalSeconds = HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ);

    float cutoffHz, gain;
    cutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    gain = pt2FilterGain(cutoffHz, rescueState.sensor.gpsRescueTaskIntervalSeconds);
    pt2FilterInit(&throttleDLpf, gain);

    cutoffHz = gpsRescueConfig()->pitchCutoffHz / 100.0f;
    rescueState.intent.velocityPidCutoff = cutoffHz;
    rescueState.intent.velocityPidCutoffModifier = 1.0f;
    gain = pt1FilterGain(cutoffHz, 1.0f);
    pt1FilterInit(&velocityDLpf, gain);

    cutoffHz *= 4.0f;
    gain = pt3FilterGain(cutoffHz, rescueState.sensor.gpsRescueTaskIntervalSeconds);
    pt3FilterInit(&velocityUpsampleLpf, gain);
}

static void rescueStart(void)
{
    rescueState.phase = RESCUE_INITIALIZE;
}

static void rescueStop(void)
{
    rescueState.phase = RESCUE_IDLE;
}

static void g_updateMaxAltutude(void)
{
    // Hold maxAltitude at zero while disarmed, but if set_home_point_once is true, hold maxAlt until power cycled
    if (!ARMING_FLAG(ARMED) && !gpsConfig()->gps_set_home_point_once) {
        rescueState.intent.maxAltitudeCm = 0.0f;
    } else {
        rescueState.intent.maxAltitudeCm = fmaxf(rescueState.intent.maxAltitudeCm, rescueState.sensor.currentAltitudeCm);
    }
}

static void g_setDescentDistanceFromConfig(void)
{
    rescueState.intent.descentDistanceM = gpsRescueConfig()->descentDistanceM;
}

static void g_setReturnAltitude(void)
{
    const float initialClimbCm = gpsRescueConfig()->initialClimbM * 100.0f;

    switch (gpsRescueConfig()->altitudeMode) {
        case GPS_RESCUE_ALT_MODE_FIXED:
            rescueState.intent.returnAltitudeCm = gpsRescueConfig()->returnAltitudeM * 100.0f;
            break;
        case GPS_RESCUE_ALT_MODE_CURRENT:
            const float minSafeAltitudeCm = 10 * 100.0f; // 10M
            const float attemptedReturnAltitudeCm = rescueState.sensor.currentAltitudeCm + initialClimbCm;
            rescueState.intent.returnAltitudeCm = fmaxf(minSafeAltitudeCm, attemptedReturnAltitudeCm);
            break;
        case GPS_RESCUE_ALT_MODE_MAX:
        default:
            rescueState.intent.returnAltitudeCm = rescueState.intent.maxAltitudeCm + initialClimbCm;
            break;
    }
}

static void g_initializeIntent(void) {
    g_initialiseSimpleIntentValues();
    g_setDescentDistanceFromConfig();
    g_setReturnAltitude();
}

static void g_rescueControlRollAndPitch(bool newGpsData);

static void g_initializeGPSRescue() {
    // assert that we are in RESCUE_INITIALIZE phase

    g_rescueControlRollAndPitch(false); // <- Initialise func's internal variables

    if (!STATE(GPS_FIX_HOME)) { // we didn't get a home point on arming
        rescueState.failure = RESCUE_NO_HOME_POINT;
        rescueState.phase = RESCUE_ABORT;

    } else if (rescueState.sensor.distanceToHomeM < 5.0f && isBelowLandingAltitude()) {
        // attempted initiation within 5m of home, and 'on the ground' -> prevent rescue, for safety reasons
        rescueState.failure = RESCUE_TOO_CLOSE;
        rescueState.phase = RESCUE_ABORT;

    } else if (rescueState.sensor.distanceToHomeM < gpsRescueConfig()->minStartDistM) {
        rescueState.failure = RESCUE_TOO_CLOSE;
        rescueState.phase = RESCUE_ABORT;

    } else {
        rescueState.failure = RESCUE_HEALTHY;
        rescueState.phase = RESCUE_FLY_HOME;
        g_initializeIntent();
    }

    // sanity check
    if (rescueState.phase == RESCUE_INITIALIZE) {
        // something is wrong with the code above!
        rescueState.phase = RESCUE_ABORT;
        rescueState.failure = RESCUE_TOO_CLOSE;
    }
}

// TODO: refactor PID controllers to eliminate copy-paste code
// TODO: separate ROLL and PITCH control functions
static void g_rescueControlRollAndPitch(bool newGpsData)
{
    // runs at 100hz, but only updates RPYT settings when new GPS Data arrives and when not in idle phase.
    static float altI = 0.0f;
    static float courseI = 0.0f;
    static float previousAltitudeError = 0.0f;
    static float previousCourseError = 0.0f;
    static float calculatedRollDegrees = 0.0f;
    static float calculatedPitchDegrees = 0.0f;
    static float calculatedPitchDegreesPrevious = 0.0f;
    timeUs_t lastPitchJumpTime = 0;

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        // values to be returned when no rescue is active
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        return;
    case RESCUE_INITIALIZE:
        // Initialize internal variables each time GPS Rescue is started
        altI = 0.0f;
        previousAltitudeError = 0.0f;
        previousCourseError = 0.0f;
        previousVelocityError = 0.0f;
        velocityI = 0.0f;
        rescueState.intent.disarmThreshold = gpsRescueConfig()->disarmThreshold * 0.1f;
        rescueState.sensor.imuYawCogGain = 1.0f;
        return;
    case RESCUE_DISARM_ON_IMPACT:
        // 20s of slow descent for switch induced sanity failures to allow time to recover
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        return;
     default:
        break;
    }

    // currentAltitudeCm is updated at TASK_GPS_RESCUE_RATE_HZ
    const float altitudeError = (rescueState.sensor.currentAltitudeCm - desiredAltitudeCm) / 100.0f;
    // height above target in metres (negative means too low)
    // at the start, the target starts at current altitude plus one step.  Increases stepwise to intended value.

    // P component
    const float altP = 0.1f * gpsRescueConfig()->ap_wing_alt_p * altitudeError;

    // I component
    altI += 0.01f * gpsRescueConfig()->ap_wing_alt_i * altitudeError * rescueState.sensor.altitudeDataIntervalSeconds;
    altI = constrainf(altI, -1.0f * GPS_RESCUE_MAX_ANGULAR_ITERM, 1.0f * GPS_RESCUE_MAX_ANGULAR_ITERM);
    if (ABS(altitudeError) > 15.0f) { // turn off I-term if altitude is too far from the desired one
        altI = 0.0f;
    }
    // up to 20% increase in throttle from I alone

    // D component is error based, so includes positive boost when climbing and negative boost on descent
    float altD = ((previousAltitudeError - altitudeError) / rescueState.sensor.altitudeDataIntervalSeconds);
    previousAltitudeError = altitudeError;
    // apply user's throttle D gain
    altD *= 0.1f * gpsRescueConfig()->ap_wing_alt_d;
    // smooth
    altD = pt2FilterApply(&throttleDLpf, altD);

    calculatedPitchDegreesPrevious = calculatedPitchDegrees;
    calculatedPitchDegrees = altP + altI - altD;

    float pitchLimitUp = scaleRangef(previousVelocityError, 0.0f, 3.0f, -20.0f, 0.0f); // negative == up
    pitchLimitUp = constrainf(pitchLimitUp, -30.0f, -10.0f);
    calculatedPitchDegrees = constrainf(calculatedPitchDegrees, pitchLimitUp, 45.0f);

    //THROTTLED_PRINT_MS(500, ">>>> calculatedPitchDegrees: %f, pitchLimitUp: %f", (double)calculatedPitchDegrees, (double)pitchLimitUp);

    if (newGpsData) {
        // course PID
        const float courseErrorDegrees = normalizeCourseErrorDecidegrees(
            rescueState.intent.targetCourseDecidegrees,
            gpsSol.groundCourse
        ) / 10.0f;

        const float courseP = 0.1f * courseErrorDegrees * gpsRescueConfig()->ap_wing_cog_p;

        courseI += 0.001f * gpsRescueConfig()->ap_wing_cog_i * courseErrorDegrees * rescueState.sensor.gpsDataIntervalSeconds;
        courseI = constrainf(courseI, -1.0f * GPS_RESCUE_MAX_ANGULAR_ITERM / 100.0f, 1.0f * GPS_RESCUE_MAX_ANGULAR_ITERM / 100.0f);
        if (ABS(courseErrorDegrees) > 30.0f) {
            courseI = 0.0f;
        }

        float courseD = ((courseErrorDegrees - previousCourseError) / rescueState.sensor.gpsDataIntervalSeconds);
        courseD *= 0.01f * gpsRescueConfig()->ap_wing_cog_d;

        calculatedRollDegrees = courseP + courseI + courseD;
        calculatedRollDegrees = constrainf(calculatedRollDegrees, -gpsRescueConfig()->maxRescueAngle, gpsRescueConfig()->maxRescueAngle);

        previousCourseError = courseErrorDegrees;
    }

    const float pitchDegreesFromRoll = ABS((float)gpsRescueConfig()->ap_wing_roll_pitch_mix * calculatedRollDegrees);
    calculatedPitchDegrees -= pitchDegreesFromRoll; // pull up the nose to counteract a roll-induced pitch down
    calculatedPitchDegrees = constrainf(calculatedPitchDegrees, -gpsRescueConfig()->maxRescueAngle, gpsRescueConfig()->maxRescueAngle);

    gpsRescueAngle[AI_ROLL] = calculatedRollDegrees * 100.0f;
    gpsRescueAngle[AI_PITCH] = calculatedPitchDegrees * 100.0f;
}

static void performSanityChecks(void)
{
    static timeUs_t previousTimeUs = 0; // Last time Stalled/LowSat was checked
    static int8_t secondsLowSats = 0; // Minimum sat detection
    static int8_t secondsDoingNothing; // Limit on doing nothing
    const timeUs_t currentTimeUs = micros();

    if (rescueState.phase == RESCUE_IDLE) {
        rescueState.failure = RESCUE_HEALTHY;
        return;
    } else if (rescueState.phase == RESCUE_INITIALIZE) {
        // Initialize these variables each time a GPS Rescue is started
        previousTimeUs = currentTimeUs;
        secondsLowSats = 0;
        secondsDoingNothing = 0;
    }

    // Handle events that set a failure mode to other than healthy.
    // Disarm via Abort when sanity on, or for hard Rx loss in FS_ONLY mode
    // Otherwise allow 20s of semi-controlled descent with impact disarm detection
    const bool hardFailsafe = !isRxReceivingSignal();

    if (rescueState.failure != RESCUE_HEALTHY) {
        // Default to 20s semi-controlled descent with impact detection, then abort
        rescueState.phase = RESCUE_DISARM_ON_IMPACT;

        switch(gpsRescueConfig()->sanityChecks) {
        case RESCUE_SANITY_ON:
            rescueState.phase = RESCUE_ABORT;
            break;
        case RESCUE_SANITY_FS_ONLY:
            if (hardFailsafe) {
                rescueState.phase = RESCUE_ABORT;
            }
            break;
        default:
            // even with sanity checks off,
            // override when Allow Arming without Fix is enabled without GPS_FIX_HOME and no Control link available.
            if (gpsRescueConfig()->allowArmingWithoutFix && !STATE(GPS_FIX_HOME) && hardFailsafe) {
                rescueState.phase = RESCUE_ABORT;
            }
        }
    }

    // Crash detection is enabled in all rescues.  If triggered, immediately disarm.
    if (crashRecoveryModeActive()) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_CRASH_PROTECTION);
        rescueStop();
    }

    // Check if GPS comms are healthy
    // ToDo - check if we have an altitude reading; if we have Baro, we can use Landing mode for controlled descent without GPS
    if (!rescueState.sensor.healthy) {
        rescueState.failure = RESCUE_GPSLOST;
    }

    //  Things that should run at a low refresh rate (such as flyaway detection, etc) will be checked at 1Hz
    const timeDelta_t dTime = cmpTimeUs(currentTimeUs, previousTimeUs);
    if (dTime < 1000000) { //1hz
        return;
    }
    previousTimeUs = currentTimeUs;

    // checks that we are getting closer to home.
    // if the quad is stuck, or if GPS data packets stop, there will be no change in distance to home
    // we can't use rescueState.sensor.currentVelocity because it will be held at the last good value if GPS data updates stop
    if (rescueState.phase == RESCUE_FLY_HOME) {
        // TODO: verify we are getting closer
        // need to keep in mind that wing not nessessary walways getting closer. For example it can be circling around the home point
        // or it can do a slow turn after GPS rescue was triggered
    }

    secondsLowSats += (!STATE(GPS_FIX) || (gpsSol.numSat < GPS_MIN_SAT_COUNT)) ? 1 : -1;
    secondsLowSats = constrain(secondsLowSats, 0, 10);

    if (secondsLowSats == 10) {
        rescueState.failure = RESCUE_LOWSATS;
    }

    // These conditions ignore sanity mode settings, and apply in all rescues, to handle getting stuck in a climb or descend

    switch (rescueState.phase) {
    case RESCUE_DISARM_ON_IMPACT:
        secondsDoingNothing = MIN(secondsDoingNothing + 1, 20);
        if (secondsDoingNothing >= 20) {
            rescueState.phase = RESCUE_ABORT;
            // time-limited semi-controlled fall with impact detection
        }
        break;
    default:
        // do nothing
        break;
    }

    DEBUG_SET(DEBUG_RTH, 2, (rescueState.failure * 10 + rescueState.phase));
    DEBUG_SET(DEBUG_RTH, 3, (rescueState.intent.secondsFailing * 100 + secondsLowSats));
}

static void sensorUpdate(bool newGPSData)
{
    static float prevDistanceToHomeCm = 0.0f;
    const timeUs_t currentTimeUs = micros();

    static timeUs_t previousAltitudeDataTimeUs = 0;
    const timeDelta_t altitudeDataIntervalUs = cmpTimeUs(currentTimeUs, previousAltitudeDataTimeUs);
    rescueState.sensor.altitudeDataIntervalSeconds = altitudeDataIntervalUs * 0.000001f;
    previousAltitudeDataTimeUs = currentTimeUs;

    rescueState.sensor.currentAltitudeCm = getAltitudeCm();

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 2, lrintf(rescueState.sensor.currentAltitudeCm));
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 0, rescueState.sensor.groundSpeedCmS);  // groundspeed cm/s
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 1, gpsSol.groundCourse);                // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 2, attitude.values.yaw);                // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 3, rescueState.sensor.directionToHome); // computed from current GPS position in relation to home
    rescueState.sensor.healthy = gpsIsHealthy();

    rescueState.sensor.directionToHome = GPS_directionToHome; // extern value from gps.c using current position relative to home
    rescueState.sensor.errorAngle = (attitude.values.yaw - rescueState.sensor.directionToHome) / 10.0f;
    // both attitude and direction are in degrees * 10, errorAngle is degrees
    if (rescueState.sensor.errorAngle <= -180) {
        rescueState.sensor.errorAngle += 360;
    } else if (rescueState.sensor.errorAngle > 180) {
        rescueState.sensor.errorAngle -= 360;
    }
    rescueState.sensor.absErrorAngle = fabsf(rescueState.sensor.errorAngle);

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 4, lrintf(attitude.values.yaw));                 // estimated heading of the quad (direction nose is pointing in)
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 5, lrintf(rescueState.sensor.directionToHome));  // angle to home derived from GPS location and home position

    if (!newGPSData) {
        return;
        // GPS ground speed, velocity and distance to home will be held at last good values if no new packets
    }

    // THROTTLED_PRINT("GPS_distanceToHomeCm: %i", GPS_distanceToHomeCm);

    rescueState.sensor.distanceToHomeCm = GPS_distanceToHomeCm;
    rescueState.sensor.distanceToHomeM = rescueState.sensor.distanceToHomeCm / 100.0f;
    rescueState.sensor.groundSpeedCmS = gpsSol.groundSpeed; // cm/s

    rescueState.sensor.gpsDataIntervalSeconds = getGpsDataIntervalSeconds();
    // Range from 10ms (100hz) to 1000ms (1Hz). Intended to cover common GPS data rates and exclude unusual values.

    rescueState.sensor.velocityToHomeCmS = ((prevDistanceToHomeCm - rescueState.sensor.distanceToHomeCm) / rescueState.sensor.gpsDataIntervalSeconds);
    // positive = towards home.  First value is useless since prevDistanceToHomeCm was zero.
    prevDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;

    DEBUG_SET(DEBUG_ATTITUDE, 4, rescueState.sensor.velocityToHomeCmS); // velocity to home

    // when there is a flyaway due to IMU disorientation, increase IMU yaw CoG gain, and reduce max pitch angle
    if (gpsRescueConfig()->groundSpeedCmS) {
        // calculate a factor that can reduce pitch angle when flying away
        const float rescueGroundspeed = gpsRescueConfig()->imuYawGain * 100.0f; // in cm/s, imuYawGain is m/s groundspeed
        // rescueGroundspeed is effectively a normalising gain factor for the magnitude of the groundspeed error
        // a higher value reduces groundspeedErrorRatio, making the radius wider and reducing the circling behaviour

        const float groundspeedErrorRatio = fabsf(rescueState.sensor.groundSpeedCmS - rescueState.sensor.velocityToHomeCmS) / rescueGroundspeed;
        // 0 if groundspeed = velocity to home, or both are zero
        // 1 if forward velocity is zero but sideways speed is imuYawGain in m/s
        // 2 if moving backwards at imuYawGain m/s, 4 if moving backwards at 2* imuYawGain m/s, etc

        DEBUG_SET(DEBUG_ATTITUDE, 5, groundspeedErrorRatio * 100);

        rescueState.intent.velocityItermAttenuator = 4.0f / (groundspeedErrorRatio + 4.0f);
        // 1 if groundspeedErrorRatio = 0, falling to 2/3 if groundspeedErrorRatio = 2, 0.5 if groundspeedErrorRatio = 4, etc
        // limit (essentially prevent) velocity iTerm accumulation whenever there is a meaningful groundspeed error
        // this is a crude but simple way to prevent iTerm windup when recovering from an IMU error
        // the magnitude of the effect will be less at low GPS data rates, since there will be fewer multiplications per second
        // but for our purposes this should not matter much, our intent is to severely attenuate iTerm
        // if, for example, we had a 90 degree attitude error, groundspeedErrorRatio = 1, invGroundspeedError = 0.8,
        // after 10 iterations, iTerm is 0.1 of what it would have been
        // also is useful in blocking iTerm accumulation if we overshoot the landing point

        const float pitchForwardAngle = (gpsRescueAngle[AI_PITCH] > 0.0f) ? fminf(gpsRescueAngle[AI_PITCH] / 3000.0f, 2.0f) : 0.0f;
        // pitchForwardAngle is positive early in a rescue, and associates with a nose forward ground course
        // note: gpsRescueAngle[AI_PITCH] is in degrees * 100, and is halved when the IMU is 180 wrong
        // pitchForwardAngle is 0 when flat
        // pitchForwardAngle is 0.5 if pitch angle is 15 degrees (ie with rescue angle of 30 and 180deg IMU error)
        // pitchForwardAngle is 1.0 if pitch angle is 30 degrees (ie with rescue angle of 60 and 180deg IMU error)
        // pitchForwardAngle is 2.0 if pitch angle is 60 degrees and flying towards home (unlikely to be sustained at that angle)

        DEBUG_SET(DEBUG_ATTITUDE, 6, pitchForwardAngle * 100.0f);

        if (rescueState.phase != RESCUE_FLY_HOME) {
            // prevent IMU disorientation arising from drift during climb, rotate or do nothing phases, which have zero pitch angle
            // in descent, or too close, increase IMU yaw gain as pitch angle increases
            rescueState.sensor.imuYawCogGain = pitchForwardAngle;
        } else {
            rescueState.sensor.imuYawCogGain = pitchForwardAngle + fminf(groundspeedErrorRatio, 3.5f);
            // imuYawCogGain will be more positive at higher pitch angles and higher groundspeed errors
            // imuYawCogGain will be lowest (close to zero) at lower pitch angles and when flying straight towards home
        }
    }

    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 2, lrintf(rescueState.sensor.velocityToHomeCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 0, lrintf(rescueState.sensor.velocityToHomeCmS));
}

// This function flashes "RESCUE N/A" in the OSD if:
// 1. sensor healthy - GPS data is being received.
// 2. GPS has a 3D fix.
// 3. GPS number of satellites is greater than or equal to the minimum configured satellite count.
// Note 1: cannot arm without the required number of sats
// hence this flashing indicates that after having enough sats, we now have below the minimum and the rescue likely would fail
// Note 2: this function does not take into account the distance from home
// The sanity checks are independent, this just provides the OSD warning
static bool checkGPSRescueIsAvailable(void)
{
    static timeUs_t previousTimeUs = 0; // Last time LowSat was checked
    const timeUs_t currentTimeUs = micros();
    static int8_t secondsLowSats = 0; // Minimum sat detection
    static bool lowsats = false;
    static bool noGPSfix = false;
    bool result = true;

    if (!gpsIsHealthy() || !STATE(GPS_FIX_HOME) || !isFixedWing()) {
        return false;
    }

    //  Things that should run at a low refresh rate >> ~1hz
    const timeDelta_t dTime = cmpTimeUs(currentTimeUs, previousTimeUs);
    if (dTime < 1000000) { //1hz
        if (noGPSfix || lowsats) {
            result = false;
        }
        return result;
    }

    previousTimeUs = currentTimeUs;

    if (!STATE(GPS_FIX)) {
        result = false;
        noGPSfix = true;
    } else {
        noGPSfix = false;
    }

    secondsLowSats = constrain(secondsLowSats + ((gpsSol.numSat < GPS_MIN_SAT_COUNT) ? 1 : -1), 0, 2);
    if (secondsLowSats == 2) {
        lowsats = true;
        result = false;
    } else {
        lowsats = false;
    }

    return result;
}

void forceDisarm(flightLogDisarmReason_e reason)
{
    setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
    disarm(reason);
}

void disarmOnImpact(void)
{
    //DEBUG_SET(DEBUG_WING_RTH, 2, lrintf(acc.accMagnitude * 100.0f));
    //DEBUG_SET(DEBUG_WING_RTH, 3, lrintf(rescueState.intent.disarmThreshold * 100.0f));
    if (acc.accMagnitude > rescueState.intent.disarmThreshold) {
        forceDisarm(DISARM_REASON_CRASH_PROTECTION);
        rescueStop();
    }
}

void g_initialiseSimpleIntentValues (void)
{
    rescueState.intent.secondsFailing = 0; // reset the sanity check timer
    rescueState.intent.targetVelocityCmS = rescueState.sensor.velocityToHomeCmS; // avoid snap from D at the start
    rescueState.intent.rollAngleLimitDeg = 0.0f; // no roll until flying home
    rescueState.intent.velocityPidCutoffModifier = 1.0f; // normal velocity lowpass filter cutoff
    rescueState.intent.pitchAngleLimitDeg = 0.0f; // force pitch adjustment to zero - level mode will level out
    rescueState.intent.velocityItermAttenuator = 1.0f; // allow iTerm to accumulate normally unless constrained by IMU error or descent phase
    rescueState.intent.velocityItermRelax = 0.0f; // but don't accumulate any at the start, not until fly home
}

bool g_updateGPSData(void) {
    static uint16_t gpsStamp = 0;
    bool newGpsData = gpsHasNewData(&gpsStamp);
    return newGpsData;
}

static bool g_isLoiterNeeded(void)
{
    return gpsRescueConfig()->ap_wing_loiter_alt > 0 
        && gpsRescueConfig()->ap_wing_loiter_seconds > 0;
}

static void g_handleDescentDistanceReached(void)
{
    rescueState.intent.targetAltitudeCm = rescueState.sensor.currentAltitudeCm;
    if (g_isLoiterNeeded()) {
        rescueState.phase = RESCUE_DESCEND_TO_LOITER;
    } else {
        rescueState.phase = RESCUE_DESCEND_TO_LAND;
    }
    // TODO: something else from intent needs changing here?
}

static void g_performDescentWithRateCmS(float rate)
{
    desiredAltitudeCm -= rate * rescueState.sensor.gpsRescueTaskIntervalSeconds;
}

static bool g_isBelowLandingAltitude(void)
{
    return rescueState.sensor.currentAltitudeCm / 100.0f < gpsRescueConfig()->ap_wing_landing_alt;
}

static float normalizeCourseErrorDecidegrees(float targetDecidegrees, float currentDecidegrees) {
    float error = (targetDecidegrees - currentDecidegrees) / 10.0f;
    error = fmodf(error + 180.0f, 360.0f) - 180.0f;
    if (error < -180.0f) {
        error += 360.0f;
    }
    return error;
}

static void g_startRescueApproach(void)
{
    rescueState.phase = RESCUE_APPROACH_MANEUVRE;

    // turn away from home before starting to land; TODO: choose a course that is the same as the initial takeoff course
    rescueState.intent.targetCourseDecidegrees = 3600.0f - rescueState.sensor.directionToHome;
    rescueState.intent.targetAltitudeCm = gpsRescueConfig()->ap_wing_landing_alt * 100.0f;
}

void g_updateGPSRescue_processState(void)
{
    switch (rescueState.phase) {
    case RESCUE_IDLE:
        g_updateMaxAltutude();
        break;
        // sanity checks are bypassed in IDLE mode; instead, failure state is always initialised to HEALTHY
        // target altitude is always set to current altitude.

    case RESCUE_INITIALIZE:
        g_initializeGPSRescue();
        break;

    case RESCUE_FLY_HOME:
        if (rescueState.sensor.distanceToHomeM <= gpsRescueConfig()->descentDistanceM) {
            g_handleDescentDistanceReached();
        }
        break;

    case RESCUE_DESCEND_TO_LOITER:
        g_performDescentWithRateCmS(gpsRescueConfig()->descendRate);

        bool reachedLoiterAltitude = rescueState.sensor.currentAltitudeCm / 100.0f < gpsRescueConfig()->ap_wing_loiter_alt;
        if (reachedLoiterAltitude) {
            rescueState.loiterActivatedTime = millis();
            rescueState.phase = RESCUE_LOITER;
            desiredAltitudeCm = gpsRescueConfig()->ap_wing_loiter_alt * 100.0f;
        }
        break;

    case RESCUE_LOITER:
        bool loiterTimeLimitReached = (millis() - rescueState.loiterActivatedTime >= ((timeMs_t)gpsRescueConfig()->ap_wing_loiter_seconds) * 1000);
        if (loiterTimeLimitReached) {
            rescueState.phase = RESCUE_DESCEND_TO_LAND;
        }
        break;

    case RESCUE_DESCEND_TO_LAND:
        g_performDescentWithRateCmS(gpsRescueConfig()->descendRate);
        if (g_isBelowLandingAltitude()) {
            g_startRescueApproach();
        }
        break;

    case RESCUE_APPROACH_MANEUVRE:
        const float turnAroundBufferM = 25.0f; // 25m buffer for turning around
        const float sufficientDistanceToHomeM = gpsRescueConfig()->ap_wing_landing_approach_dist + turnAroundBufferM;
        bool isFarEnoughFromHome = rescueState.sensor.distanceToHomeM > gpsRescueConfig()->ap_wing_landing_approach_dist;
        if (isFarEnoughFromHome) {
            rescueState.waitForCourseAwayActivatedTime = millis();
            rescueState.phase = RESCUE_WAIT_FOR_COURSE_AWAY;
        }
        break;

    case RESCUE_WAIT_FOR_COURSE_AWAY:
        const float courseErrorDegrees = normalizeCourseErrorDecidegrees(
            rescueState.intent.targetCourseDecidegrees,
            gpsSol.groundCourse
        ) / 10.0f;
        
        const bool courseIsOK = ABS(courseErrorDegrees) < 20.0f;
        const bool waitedTooLong = millis() - rescueState.waitForCourseAwayActivatedTime >= 60 * 1000; // 1 minute

        if (courseIsOK || waitedTooLong) {
            rescueState.waitForCourseHomeActivatedTime = millis();
            rescueState.phase = RESCUE_WAIT_FOR_LANDING_DISTANCE;
        }
        break;

    case RESCUE_WAIT_FOR_LANDING_DISTANCE:
        // this needs to be kept up to date while turning around
        rescueState.intent.targetCourseDecidegrees = rescueState.sensor.directionToHome;

        const float courseErrorDegrees = normalizeCourseErrorDecidegrees(
            rescueState.intent.targetCourseDecidegrees,
            gpsSol.groundCourse
        ) / 10.0f;
        
        const bool courseIsOK = ABS(courseErrorDegrees) < 20.0f;
        const bool waitedTooLong = millis() - rescueState.waitForCourseHomeActivatedTime >= 120 * 1000;
        const bool isCloseToHome = rescueState.sensor.distanceToHomeM <= gpsRescueConfig()->ap_wing_landing_approach_dist;

        if (isCloseToHome) {
            if (courseIsOK) {
                rescueState.phase = RESCUE_LAND;
                rescueState.landActivatedTime = millis();
            } else {
                // retry approach manoeuvre again
                g_startRescueApproach();
            }
        } else if (waitedTooLong) {
            // we are unable to get close to landing distance in time, so we will land where we are
            // this is a bad situation because it means we just can't move towards home fast enough
            rescueState.phase = RESCUE_LAND;
            rescueState.landActivatedTime = millis();
        } else {
            // do nothing, wait for distance to home to be close enough
        }
        break;

    case RESCUE_LAND:
        const float currentAltitudeCm = rescueState.sensor.currentAltitudeCm;
        const float currentSpeedCmS = rescueState.sensor.groundSpeedCmS;
        const float distanceToHomeCm = rescueState.sensor.distanceToHomeM * 100.0f;
        const float timeToHomeSeconds = distanceToHomeCm / currentSpeedCmS;

        // aim to touch down exactly at the home point
        const float landingDescentRateCmS = currentAltitudeCm / timeToHomeSeconds;
        g_performDescentWithRateCmS(landingDescentRateCmS);

        // keep home direction up to date while landing
        rescueState.intent.targetCourseDecidegrees = rescueState.sensor.directionToHome;

        // make sure speed is lowered during landing
        const float touchDownSpeedCmS = 40.0f / 3.6f * 100.0f; // 40km/h in cm/s
        const float targetSpeedCmS = scaleRangef(
            distanceToHomeCm, 
            15.0f,              gpsRescueConfig()->ap_wing_landing_approach_dist, 
            touchDownSpeedCmS,  gpsRescueConfig()->groundSpeedCmS
        );
        rescueState.intent.targetVelocityCmS = constrainf(targetSpeedCmS, touchDownSpeedCmS, gpsRescueConfig()->groundSpeedCmS);
        
        // TODO: check if we are moving away from home

        const bool isLow = currentAltitudeCm < 3.0f;
        const bool isCloseToHome = distanceToHomeCm / 100.0f < 30.0f;
        const bool timeToHomeIsLow = timeToHomeSeconds < 15.0f;

        if (currentAltitudeCm < 5.0f 
            || distanceToHomeCm / 100.0f < 30.0f 
            || timeToHomeSeconds < 15.0f
        ) {
            disarmOnImpact();
        }
        
        if (currentAltitudeCm < 2.0f 
            || distanceToHomeCm / 100.0f < 15.0f 
            || timeToHomeSeconds < 5.0f
        ) {
            forceDisarm(DISARM_REASON_GPS_RESCUE);
            rescueStop();
        }
        break;

    case RESCUE_ABORT:
        // setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        // disarm(DISARM_REASON_FAILSAFE);
        // rescueState.intent.secondsFailing = 0; // reset sanity timers so we can re-arm
        rescueStop();
        break;

    case RESCUE_DISARM_ON_IMPACT:
        disarmOnImpact();
        break;

    default:
        break;
    }
}

void gpsRescueUpdate(void)
// runs at gpsRescueTaskIntervalSeconds, and runs whether or not rescue is active
{
    PRINT_ON_CHANGE(rescueState.phase, "rescue state changed to: %s", RESCUE_PHASE_STR(rescueState.phase));
    
    bool newGpsData = g_updateGPSData();

    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop(); // sets phase to RESCUE_IDLE; does nothing else.  RESCUE_IDLE tasks still run.
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE) {
        // executed only once
        PRINT("ENTERING GPS RESCUE MODE FROM RESCUE_IDLE");
        
        rescueStart(); // sets phase to rescue_initialise if we enter GPS Rescue mode while idle
        performSanityChecks(); // Initialises sanity check values when a Rescue starts
    }

    // Will now be in RESCUE_INITIALIZE mode, if just entered Rescue while IDLE, otherwise stays IDLE

    sensorUpdate(newGpsData); // always get latest GPS and Altitude data, update ascend and descend rates

    rescueState.isAvailable = checkGPSRescueIsAvailable();
    
    THROTTLED_PRINT("distance: %f, current: %f, return: %f, desired: %f",
                    (double)rescueState.sensor.distanceToHomeM,
                    (double)rescueState.sensor.currentAltitudeCm / 100.0,
                    (double)rescueState.intent.returnAltitudeCm / 100.0,
                    (double)desiredAltitudeCm / 100.0);

    g_updateGPSRescue_processState();
    g_rescueControlRollAndPitch(newGpsData);
    
    float velocityPIDSum = g_gpsRescueGetVelocityPIDSum(newGpsData);
    setAutopilotThrottle(gpsRescueGetThrottle(velocityPIDSum));
    
    performSanityChecks();
}

float gpsRescueGetYawRate(void)
{
    return - gpsRescueConfig()->ap_wing_roll_yaw_mix / 100.0f * gpsRescueAngle[AI_ROLL];
}

float gpsRescueGetImuYawCogGain(void)
{
    return rescueState.sensor.imuYawCogGain;
}

float g_gpsRescueGetVelocityPIDSum(bool newGpsData)
{
    static float velocityPIDSum = 0.0f;

    if (newGpsData) {
        const float sampleIntervalNormaliseFactor = rescueState.sensor.gpsDataIntervalSeconds * 10.0f;

        const float currentSpeed = rescueState.sensor.groundSpeedCmS;
        // TODO: check against "velocityTowardsTargetCourse" instead of absolute ground speed.

        const float velocityError = rescueState.intent.targetVelocityCmS - currentSpeed;
        // velocityError is in cm per second, positive means too slow.
        // NB positive pitch setpoint means nose down.
        // target velocity can be very negative leading to large error before the start, with overshoot

        // P component
        const float velocityP = velocityError * gpsRescueConfig()->velP;

        // I component
        velocityI += 0.01f * gpsRescueConfig()->velI * velocityError * sampleIntervalNormaliseFactor * rescueState.intent.velocityItermRelax;
        // velocityItermRelax is a time-based factor, 0->1 with time constant of 1s from when we start to fly home
        // avoids excess iTerm accumulation during the initial acceleration phase and during descent.

        velocityI *= rescueState.intent.velocityItermAttenuator;
        // used to minimise iTerm windup during IMU error states and iTerm overshoot in the descent phase
        // also, if we over-fly the home point, we need to re-accumulate iTerm from zero, not the previously accumulated value

        const float velocityILimit = 1.0f; // TODO: ???
        velocityI = constrainf(velocityI, 0, velocityILimit);

        // D component
        float velocityD = ((velocityError - previousVelocityError) / sampleIntervalNormaliseFactor);
        previousVelocityError = velocityError;
        velocityD *= gpsRescueConfig()->velD;
        DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 5, lrintf(velocityD)); // velocity D before lowpass smoothing
        // smooth the D steps
        const float cutoffHz = rescueState.intent.velocityPidCutoff * rescueState.intent.velocityPidCutoffModifier;
        // note that this cutoff is increased up to 2x as we get closer to landing point in descend()
        const float gain = pt1FilterGain(cutoffHz, rescueState.sensor.gpsDataIntervalSeconds);
        pt1FilterUpdateCutoff(&velocityDLpf, gain);
        velocityD = pt1FilterApply(&velocityDLpf, velocityD);

        velocityPIDSum = velocityP + velocityI + velocityD;
        // limit to maximum allowed angle
        velocityPIDSum = constrainf(velocityPIDSum, 0, 0.5f);
    }

    return velocityPIDSum;
}

// TODO: this function is too complicated and very strange.
float gpsRescueGetThrottle(float velocityPIDSum)
{
    float commandedThrottle = velocityPIDSum;
    THROTTLED_PRINT("velocityPIDSum: %f", (double)velocityPIDSum);
    //DEBUG_SET(DEBUG_WING_RTH, 1, lrintf(commandedThrottle * 100.0f));

    // less voltage - more throttle
    float batteryThrottleFactor = 1.0f;
    if (pidRuntime.tpaSpeed.maxVoltage > 0.0f) {
        batteryThrottleFactor = getBatteryVoltageLatest() / 100.0f / pidRuntime.tpaSpeed.maxVoltage;
        batteryThrottleFactor = constrainf(batteryThrottleFactor, 0.0f, 1.0f);
    }
    THROTTLED_PRINT_MS(30000, "batteryThrottleFactor: %f", (double)(1.0f/batteryThrottleFactor));
    commandedThrottle = commandedThrottle / batteryThrottleFactor;

    DEBUG_SET(DEBUG_WING_RTH, 2, lrintf(commandedThrottle * 100.0f));

    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_WING_RTH, 3, lrintf(commandedThrottle * 100.0f));

    // Nose down - less throttle, nose up - more throttle depending on pitch angle and thrust to weight ratio
    const float twr = pidRuntime.tpaSpeed.twr;
    DEBUG_SET(DEBUG_WING_RTH, 4, lrintf(twr * 100.0f));
    DEBUG_SET(DEBUG_WING_RTH, 5, lrintf(getSinPitchAngle() * 100.0f));

    THROTTLED_PRINT_MS(30000, "twr: %f", (double)twr);

    float underSqrt = (commandedThrottle * commandedThrottle * twr - getSinPitchAngle()) / twr;

    DEBUG_SET(DEBUG_WING_RTH, 6, lrintf(underSqrt * 100.0f));

    if (underSqrt < 0.0f) {
        underSqrt = 0.0f;
    }
    commandedThrottle = sqrtf(underSqrt);
    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);
    DEBUG_SET(DEBUG_WING_RTH, 7, lrintf(commandedThrottle * 100.0f));

    if (rescueState.phase == RESCUE_LAND || rescueState.phase == RESCUE_DISARM_ON_IMPACT || rescueState.phase == RESCUE_WAIT_FOR_LANDING_DISTANCE) {
        commandedThrottle = 0.0f;
    }

    DEBUG_SET(DEBUG_WING_RTH, 0, lrintf(commandedThrottle * 100.0f));

    THROTTLED_PRINT_MS(500, "throttle: %f", (double)commandedThrottle);
    return commandedThrottle;
}

bool gpsRescueIsConfigured(void)
{
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || isModeActivationConditionPresent(BOXGPSRESCUE);
}

bool gpsRescueIsAvailable(void)
{
    return rescueState.isAvailable;
}

bool gpsRescueIsDisabled(void)
// used for OSD warning
{
    return (!STATE(GPS_FIX_HOME));
}

#ifdef USE_MAG
bool gpsRescueDisableMag(void)
{
    // Enable mag on user request, but don't use it during fly home or if force disabled
    // Note that while flying home the course over ground from GPS provides a heading that is less affected by wind
    return !(gpsRescueConfig()->useMag && rescueState.phase != RESCUE_FLY_HOME && !magForceDisable);
}
#endif

#endif // USE_GPS_RESCUE

#endif // USE_WING
