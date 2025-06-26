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
    RESCUE_FLY_AWAY_BEFORE_APPROACH, // 6
    RESCUE_WAIT_FOR_COURSE_AWAY, // 7
    RESCUE_PREPARE_TO_LAND, // 8
    RESCUE_WAIT_FOR_LANDING_COURSE, // 9
    RESCUE_LAND, // 10
    RESCUE_DISARM_ON_IMPACT,
    RESCUE_EMERGENCY_LANDING,
    RESCUE_ABORT,
} rescuePhase_e;

#define ENUM_VALUE_STR(stringsMap, value) \
    ((value) < (sizeof(stringsMap)/sizeof(stringsMap[0])) && stringsMap[value] ? \
     stringsMap[value] : "UNKNOWN")

static const char* rescuePhaseStrings[] = {
    [RESCUE_IDLE] = "RESCUE_IDLE",
    [RESCUE_INITIALIZE] = "RESCUE_INITIALIZE", 
    [RESCUE_FLY_HOME] = "RESCUE_FLY_HOME",
    [RESCUE_DESCEND_TO_LOITER] = "RESCUE_DESCEND_TO_LOITER",
    [RESCUE_LOITER] = "RESCUE_LOITER",
    [RESCUE_DESCEND_TO_LAND] = "RESCUE_DESCEND_TO_LAND",
    [RESCUE_FLY_AWAY_BEFORE_APPROACH] = "RESCUE_FLY_AWAY_BEFORE_APPROACH",
    [RESCUE_WAIT_FOR_COURSE_AWAY] = "RESCUE_WAIT_FOR_COURSE_AWAY",
    [RESCUE_PREPARE_TO_LAND] = "RESCUE_PREPARE_TO_LAND",
    [RESCUE_WAIT_FOR_LANDING_COURSE] = "RESCUE_WAIT_FOR_LANDING_COURSE",
    [RESCUE_LAND] = "RESCUE_LAND",
    [RESCUE_DISARM_ON_IMPACT] = "RESCUE_DISARM_ON_IMPACT",
    [RESCUE_EMERGENCY_LANDING] = "RESCUE_EMERGENCY_LANDING",
    [RESCUE_ABORT] = "RESCUE_ABORT",
};

#define RESCUE_PHASE_STR(phase) ENUM_VALUE_STR(rescuePhaseStrings, phase)

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

static const char* rescueFailureStrings[] = {
    [RESCUE_HEALTHY] = "RESCUE_HEALTHY",
    [RESCUE_FLYAWAY] = "RESCUE_FLYAWAY",
    [RESCUE_GPSLOST] = "RESCUE_GPSLOST",
    [RESCUE_LOWSATS] = "RESCUE_LOWSATS",
    [RESCUE_CRASH_FLIP_DETECTED] = "RESCUE_CRASH_FLIP_DETECTED",
    [RESCUE_STALLED] = "RESCUE_STALLED",
    [RESCUE_TOO_CLOSE] = "RESCUE_TOO_CLOSE",
    [RESCUE_NO_HOME_POINT] = "RESCUE_NO_HOME_POINT",
};

#define RESCUE_FAILURE_STR(failure) ENUM_VALUE_STR(rescueFailureStrings, failure)

typedef struct {
    float maxAltitudeCm;
    float returnAltitudeCm;
    float descentDistanceM;
    float takeoffCourse;
    gpsLocation_t takeoffNeutralPoint;
    float homeCourseAtActivation;
    bool takeoffCourseValid;
    
    float smoothTargetAltitudeCm;
    float _targetAltitudeCm;
    float targetCourseDecidegrees;
    float targetVelocityCmS;

    float pitchAngleLimitDeg;
    float rollAngleLimitDeg;
    float disarmThreshold;

    int8_t secondsFailing;
    float velocityITermAccumulator;
    float velocityDLpfCutoff;
    // float velocityPidCutoffModifier;
    // float velocityItermAttenuator;
    // float velocityItermRelax;
} rescueIntent_s;

typedef struct {
    float currentAltitudeCm;
    uint16_t groundSpeedCmS;
    float velocityToHomeCmS;
    float errorAngle;
    float absErrorAngle;

    float distanceToHomeCm;
    float distanceToHomeM;
    float distanceToNeutralPointCm;
    int16_t directionToHome;
    int16_t directionToNeutralPoint;

    bool healthy;
    float gpsDataIntervalSeconds;
    float altitudeDataIntervalSeconds;
    float gpsRescueTaskIntervalSeconds;
    float alitutudeStepCm;
    float maxPitchStep;
    float imuYawCogGain;
} rescueSensorData_s;

typedef struct {
    uint16_t secondsLowSats;
    bool isGPSHealthy;
} rescueSanityState_s;

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
    float landingProgress;
} rescueState_s;

typedef struct {
    vector2_t velocity;
    float magnitude;
} windState_s;

#define GPS_RESCUE_MAX_ANGULAR_ITERM     1500    // max iterm value for pitch in degrees * 100
#define GPS_RESCUE_ALLOWED_YAW_RANGE   30.0f  // yaw error must be less than this to enter fly home phase, and to pitch during descend()

float       gpsRescueAngle[RP_AXIS_COUNT] = { 0, 0 };
bool        magForceDisable = true;
// static bool newGPSData = false;
static pt2Filter_t altitudeDLpf;
static pt1Filter_t velocityDLpf;
static pt3Filter_t velocityUpsampleLpf;
static pt3Filter_t targetAltitudeLpf;

static pt3Filter_t rollDegreesLpf;
static pt3Filter_t pitchDegreesLpf;

rescueState_s rescueState;
rescueSanityState_s sanityState;
windState_s windState;

#define desiredAltitudeCm rescueState.intent.targetAltitudeCm

// FORWARD DECLARATIONS

fileprivate float g_gpsRescueGetVelocityPIDSum(float dT);
fileprivate void g_rescueControlRollAndPitch(float dT);
fileprivate void g_initialiseSimpleIntentValues (void);

// UTILITY FUNCTIONS

fileprivate float normalizeCourseErrorDecidegrees(float targetDecidegrees, float currentDecidegrees) {
    float error = (targetDecidegrees - currentDecidegrees) / 10.0f;
    error = fmodf(error + 180.0f, 360.0f) - 180.0f;
    if (error < -180.0f) {
        error += 360.0f;
    }
    return error * 10.0f;
}

///////////////////////

void gpsRescueInit(void)
{
    rescueState.sensor.gpsRescueTaskIntervalSeconds = HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ);

    float cutoffHz, gain;
    cutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    gain = pt2FilterGain(cutoffHz, rescueState.sensor.gpsRescueTaskIntervalSeconds);
    pt2FilterInit(&altitudeDLpf, gain);

    rescueState.intent.velocityDLpfCutoff = gpsRescueConfig()->ap_wing_throttle_d_cutoff_decihz / 10.0f;
    // rescueState.intent.velocityPidCutoffModifier = 1.0f;
    gain = pt1FilterGain(cutoffHz, 1.0f);
    pt1FilterInit(&velocityDLpf, gain);

    cutoffHz *= 4.0f;
    gain = pt3FilterGain(cutoffHz, rescueState.sensor.gpsRescueTaskIntervalSeconds);
    pt3FilterInit(&velocityUpsampleLpf, gain);

    rescueState.intent.takeoffCourse = 0.0f;
    rescueState.intent.takeoffCourseValid = false;

    rescueState.sensor.imuYawCogGain = 1.0f; // idk what this is

    windState.velocity.x = 0.0f;
    windState.velocity.y = 0.0f;
    windState.magnitude = 0.0f;
}

fileprivate void rescueStart(void)
{
    rescueState.phase = RESCUE_INITIALIZE;
}

fileprivate void rescueStop(void)
{
    rescueState.phase = RESCUE_IDLE;
}

fileprivate void g_updateMaxAltutude(void)
{
    // Hold maxAltitude at zero while disarmed, but if set_home_point_once is true, hold maxAlt until power cycled
    if (!ARMING_FLAG(ARMED) && !gpsConfig()->gps_set_home_point_once) {
        rescueState.intent.maxAltitudeCm = 0.0f;
    } else {
        rescueState.intent.maxAltitudeCm = fmaxf(rescueState.intent.maxAltitudeCm, rescueState.sensor.currentAltitudeCm);
    }
}

fileprivate void g_setDescentDistanceFromConfig(void)
{
    rescueState.intent.descentDistanceM = gpsRescueConfig()->descentDistanceM;
}

fileprivate void g_setReturnAltitude(void)
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
            rescueState.intent.returnAltitudeCm = fmaxf(rescueState.intent.returnAltitudeCm, gpsRescueConfig()->returnAltitudeM * 100.0f);
            break;
    }
}

fileprivate void g_initializeIntent(void) {
    // INTENT
    g_initialiseSimpleIntentValues();
    g_setDescentDistanceFromConfig();
    g_setReturnAltitude();
    rescueState.intent._targetAltitudeCm = rescueState.intent.returnAltitudeCm;

    // FILTERS
    const float gainAlt = pt3FilterGain(0.1f, 1.0f/20.0f);
    pt3FilterInitValue(&targetAltitudeLpf, gainAlt, rescueState.sensor.currentAltitudeCm);

    const float gainRoll = pt3FilterGain(0.5f, 1.0f/20.0f);
    pt3FilterInitValue(&rollDegreesLpf, gainRoll, 0.0f);
    pt3FilterInitValue(&pitchDegreesLpf, gainRoll, 0.0f);

    rescueState.intent.smoothTargetAltitudeCm = rescueState.sensor.currentAltitudeCm;
    g_rescueControlRollAndPitch(1/20.0f);
}

fileprivate void g_initializeGPSRescue(void) {
    // assert that we are in RESCUE_INITIALIZE phase

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
        // actually start the gps rescue
        g_initializeIntent();
        g_rescueControlRollAndPitch(1/20.0f); // <- Initialise func's internal variables
        rescueState.failure = RESCUE_HEALTHY;
        rescueState.phase = RESCUE_FLY_HOME;
    }

    // assert
    if (rescueState.phase == RESCUE_INITIALIZE) {
        // something is wrong with the code above!
        rescueState.phase = RESCUE_ABORT;
        rescueState.failure = RESCUE_TOO_CLOSE;
    }
}

// ============== RESCUE RUNTIME ===============

void updateWind(float gpsCourseDegrees, float gpsSpeed, float estimatedForwardAirSpeed, float dT) {
    const float smoothing = 1.0f;
        
    // Convert GPS course from degrees to radians
    float gpsCourseRad = gpsCourseDegrees * M_PIf / 180.0f;
    
    // Calculate GPS velocity vector (ground speed)
    float groundVelocityX = gpsSpeed * cosf(gpsCourseRad);
    float groundVelocityY = gpsSpeed * sinf(gpsCourseRad);
    
    // Get aircraft attitude angles
    float sinPitch = getSinPitchAngle();
    float cosTilt = getCosTiltAngle();
    
    // Calculate horizontal component of airspeed considering aircraft attitude
    // cosTilt accounts for roll, sinPitch accounts for pitch
    float horizontalAirspeed = estimatedForwardAirSpeed * cosTilt * cosf(asinf(sinPitch));
    
    // Assume aircraft is pointing in GPS course direction for airspeed vector
    // (This is a simplification - in reality you'd need aircraft heading)
    float airVelocityX = horizontalAirspeed * cosf(gpsCourseRad);
    float airVelocityY = horizontalAirspeed * sinf(gpsCourseRad);
    
    // Wind vector = Ground velocity - Air velocity
    float instantWindEstimateX = groundVelocityX - airVelocityX;
    float instantWindEstimateY = groundVelocityY - airVelocityY;
    
    // Apply smoothing filter to reduce noise
    float alpha = dT / (smoothing + dT);
    windState.velocity.x = windState.velocity.x * (1.0f - alpha) + instantWindEstimateX * alpha;
    windState.velocity.y = windState.velocity.y * (1.0f - alpha) + instantWindEstimateY * alpha;
    
    // Calculate wind magnitude
    windState.magnitude = vector2Norm(&windState.velocity);

    LOG_UPDATE("wind", "%f m/s   ( %3.1f , %3.1f )", 
        (double)windState.magnitude, 
        (double)windState.velocity.x, 
        (double)windState.velocity.y
    );
}

fileprivate float g_calculateAltitudePID(float resolvedCurrentAltitudeCm, float dT)
{
    static float altI = 0.0f;
    static float previousAltitudeError = 0.0f;

    const float altitudeErrorM = (resolvedCurrentAltitudeCm - rescueState.intent.smoothTargetAltitudeCm) / 100.0f;

    // P
    const float altP = 0.1f * gpsRescueConfig()->ap_wing_alt_p * altitudeErrorM;

    // I
    altI += 0.1f * gpsRescueConfig()->ap_wing_alt_i * altitudeErrorM * dT;
    altI = constrainf(altI, -20.0f, 15.0f); // negative == nose up
    if (ABS(altitudeErrorM) > 15.0f) { // turn off I-term if altitude is too far from the desired one
        altI = 0.0f;
    }

    // D
    float altD = (previousAltitudeError - altitudeErrorM) / dT;
    altD *= 0.1f * gpsRescueConfig()->ap_wing_alt_d;
    altD = pt2FilterApply(&altitudeDLpf, altD);

    previousAltitudeError = altitudeErrorM;
    float pidSum = altP + altI - altD;

    LOG_UPDATE("d. altitude_pid", "p:%+6.3f i:%+6.3f d:%+6.3f sumRaw:%+6.3f, sumClamped:%+6.3f", 
                    (double)altP, (double)altI, (double)altD, (double)pidSum);

    LOG_UPDATE("d. altitude", "%+6.1f <- %6.1f <- %+6.1f err:%+6.1f", 
                    (double)rescueState.intent._targetAltitudeCm / 100.0,
                    (double)rescueState.intent.smoothTargetAltitudeCm / 100.0, 
                    (double)rescueState.sensor.currentAltitudeCm / 100.0,
                    (double)altitudeErrorM);

    if (rescueState.phase == RESCUE_INITIALIZE) {
        altitudeDLpf.state = 0.0f;
        altitudeDLpf.state1 = 0.0f;
        altI = 0.0f;
        return altP;
    }

    return pidSum;
}

fileprivate float g_calculateCoursePID(float resolvedCurrentCourseDecidegrees, float dT)
{
    static float courseI = 0.0f;
    static float previousCourseError = 0.0f;

    if (rescueState.phase == RESCUE_INITIALIZE) {
        courseI = 0.0f;
        previousCourseError = 0.0f;
    }

    const float courseErrorDegrees = normalizeCourseErrorDecidegrees(
        rescueState.intent.targetCourseDecidegrees,
        resolvedCurrentCourseDecidegrees
    ) / 10.0f;

    // P
    const float courseP = 0.1f * courseErrorDegrees * gpsRescueConfig()->ap_wing_cog_p;

    // I
    courseI += 0.001f * gpsRescueConfig()->ap_wing_cog_i * courseErrorDegrees * dT;
    courseI = constrainf(courseI, -15.0f, 15.0f);
    if (ABS(courseErrorDegrees) > 30.0f) {
        courseI = 0.0f;
    }

    // D
    float courseD = ((courseErrorDegrees - previousCourseError) / dT);
    courseD *= 0.01f * gpsRescueConfig()->ap_wing_cog_d;

    previousCourseError = courseErrorDegrees;
    float pidSum = courseP + courseI + courseD;

    LOG_UPDATE("d. course_pid", "p:%+6.3f i:%+6.3f d:%+6.3f sumRaw:%+6.3f", 
                    (double)courseP, (double)courseI, (double)courseD, (double)pidSum);

    LOG_UPDATE(
        "d. course", "%+6.1f <- %6.1f   err:%+6.1f", 
        (double)rescueState.intent.targetCourseDecidegrees / 100.0,
        (double)gpsSol.groundCourse / 10.0, 
        (double)courseErrorDegrees
    );

    return pidSum;
}

fileprivate bool g_setHardcodedRollPitchIfNeeded(void)
{
    switch (rescueState.phase) {
    case RESCUE_IDLE:
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        return true;
    
    case RESCUE_DISARM_ON_IMPACT:
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        return true;
        
    case RESCUE_EMERGENCY_LANDING:
        gpsRescueAngle[AI_PITCH] = -10.0f;
        gpsRescueAngle[AI_ROLL] = 45.0f;
        return true;

    default:
        return false;
    }
}

fileprivate void g_rescueControlRollAndPitch(float dT)
{
    // prepare data
    float currentCourse = gpsSol.groundCourse;
    float currentAltitude = rescueState.sensor.currentAltitudeCm;

    // roll
    const float rollDegrees = constrainf(
        g_calculateCoursePID(currentCourse, dT), 
        -gpsRescueConfig()->maxRescueAngle, 
        gpsRescueConfig()->maxRescueAngle
    );

    // pitch
    const float pitchDegreesRaw = g_calculateAltitudePID(currentAltitude, dT);
    const float pitchDegreesFromRoll = ABS((float)gpsRescueConfig()->ap_wing_roll_pitch_mix * rollDegrees);
    const float pitchDegrees = constrainf(
        pitchDegreesRaw - pitchDegreesFromRoll, 
        -gpsRescueConfig()->maxRescueAngle, 
        gpsRescueConfig()->maxRescueAngle
    );
    
    if (g_setHardcodedRollPitchIfNeeded()) {
        return;
    }

    float rollAngleSetpoint = pt3FilterApply(&rollDegreesLpf, rollDegrees);
    // float pitchAngleSetpoint = pt3FilterApply(&pitchDegreesLpf, pitchDegrees);

    gpsRescueAngle[AI_ROLL] = rollAngleSetpoint * 100.0f;
    gpsRescueAngle[AI_PITCH] = pitchDegrees * 100.0f;
}

fileprivate void g_sanity1hz_checkSatsCount(void)
{
    if (!STATE(GPS_FIX) || (gpsSol.numSat < GPS_MIN_SAT_COUNT)) {
        sanityState.secondsLowSats += 1;
        sanityState.secondsLowSats = MIN(sanityState.secondsLowSats, UINT16_MAX - 1);
    } else {
        sanityState.secondsLowSats = 0;
    }

    sanityState.isGPSHealthy = sanityState.secondsLowSats < 10;

    if (sanityState.secondsLowSats > 10) {
        rescueState.failure = RESCUE_LOWSATS;
    }
    
    // if (!rescueState.sensor.healthy) {
    //     rescueState.failure = RESCUE_GPSLOST;
    // }
}

fileprivate void g_performSanityChecks(void)
{    
    // 1Hz checks
    static timeUs_t previousTimeUs = 0;
    const timeUs_t currentTimeUs = micros();
    const timeDelta_t dTime = cmpTimeUs(currentTimeUs, previousTimeUs);
    if (dTime < 1000000) { // 1 second
        return;
    }
    previousTimeUs = currentTimeUs;

    g_sanity1hz_checkSatsCount();
}

fileprivate void sensorUpdate(bool newGPSData)
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
    rescueState.sensor.distanceToHomeCm = GPS_distanceToHomeCm;
    rescueState.sensor.distanceToHomeM = rescueState.sensor.distanceToHomeCm / 100.0f;

    LOG_UPDATE("home_distance", "%+6.1f", (double)rescueState.sensor.distanceToHomeM);

    LOG_UPDATE("velocity_to_home", "%+6.1f", (double)rescueState.sensor.velocityToHomeCmS / 100.0 * 3.6);

    if (rescueState.intent.takeoffCourseValid) {
        uint32_t distCm;
        int32_t dirDegrees;
        GPS_distance_cm_bearing(
            &gpsSol.llh, 
            &rescueState.intent.takeoffNeutralPoint, 
            false,
            &distCm,
            &dirDegrees
        );
        rescueState.sensor.distanceToNeutralPointCm = distCm;
        rescueState.sensor.directionToNeutralPoint = dirDegrees / 10; // deg to decidegrees
    } else {
        rescueState.sensor.distanceToNeutralPointCm = rescueState.sensor.distanceToHomeCm;
        rescueState.sensor.directionToNeutralPoint = rescueState.sensor.directionToHome;
    }

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

    updateWind(
        gpsSol.groundCourse, 
        gpsSol.groundSpeed / 100,
        pidRuntime.tpaSpeed.speed, // m/s
        rescueState.sensor.gpsDataIntervalSeconds
    );

    rescueState.sensor.groundSpeedCmS = gpsSol.groundSpeed; // cm/s

    rescueState.sensor.gpsDataIntervalSeconds = getGpsDataIntervalSeconds();
    // Range from 10ms (100hz) to 1000ms (1Hz). Intended to cover common GPS data rates and exclude unusual values.

    rescueState.sensor.velocityToHomeCmS = ((prevDistanceToHomeCm - rescueState.sensor.distanceToHomeCm) / rescueState.sensor.gpsDataIntervalSeconds);
    // positive = towards home.  First value is useless since prevDistanceToHomeCm was zero.
    prevDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;
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
        rescueState.phase = RESCUE_ABORT;
    }
}

void g_initialiseSimpleIntentValues (void)
{
    rescueState.intent.secondsFailing = 0; // reset the sanity check timer
    rescueState.intent.targetVelocityCmS = gpsRescueConfig()->groundSpeedCmS;
    rescueState.intent.rollAngleLimitDeg = 0.0f; // no roll until flying home
    // rescueState.intent.velocityPidCutoffModifier = 1.0f; // normal velocity lowpass filter cutoff
    rescueState.intent.pitchAngleLimitDeg = 0.0f; // force pitch adjustment to zero - level mode will level out
    rescueState.intent.disarmThreshold = gpsRescueConfig()->disarmThreshold * 0.1f;
    // rescueState.intent.velocityItermAttenuator = 1.0f; // allow iTerm to accumulate normally unless constrained by IMU error or descent phase
    // rescueState.intent.velocityItermRelax = 0.0f; // but don't accumulate any at the start, not until fly home
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

fileprivate void g_handleDescentDistanceReached(void)
{
    rescueState.intent._targetAltitudeCm = rescueState.sensor.currentAltitudeCm;
    if (g_isLoiterNeeded()) {
        rescueState.phase = RESCUE_DESCEND_TO_LOITER;
    } else {
        rescueState.phase = RESCUE_DESCEND_TO_LAND;
    }
    // TODO: something else from intent needs changing here?
}

fileprivate void g_performDescentWithRateCmS(float rate)
{
    rescueState.intent._targetAltitudeCm -= rate * rescueState.sensor.gpsRescueTaskIntervalSeconds;
}

fileprivate void g_startRescueApproach(void)
{
    rescueState.phase = RESCUE_FLY_AWAY_BEFORE_APPROACH;

    // turn away from home before starting to land;
    if (rescueState.intent.takeoffCourseValid) {
        rescueState.intent.targetCourseDecidegrees = rescueState.intent.takeoffCourse;
    } else {
        // assume takeoff course is just back from home course
        rescueState.intent.targetCourseDecidegrees = rescueState.intent.homeCourseAtActivation + 1800.0f;
    }
    rescueState.intent._targetAltitudeCm = gpsRescueConfig()->ap_wing_landing_alt * 100.0f;
}

static float g_loiterCourseEdgeDistanceM(void)
{
    return gpsRescueConfig()->descentDistanceM + 50.0f;
}

static bool g_calculateTargetCourseForLoiter(void)
{
    float offsetDegrees = scaleRangef(
        rescueState.sensor.distanceToHomeM, 
        gpsRescueConfig()->descentDistanceM, g_loiterCourseEdgeDistanceM(),
        90.0f, 0.0f
    );
    offsetDegrees = constrainf(offsetDegrees, 0.0f, 90.0f);
    rescueState.intent.targetCourseDecidegrees = rescueState.sensor.directionToNeutralPoint - offsetDegrees * 10.0f;

    // THROTTLED_PRINT("course offset: %f", (double)(offsetDegrees));
    return offsetDegrees > 1.0f;
}

static float g_calculateBestDescentRateCmS(float targetAltitudeCm, float idealTimeSeconds)
{
    const float descentDistanceM = ABS(rescueState.sensor.currentAltitudeCm / 100.0f - targetAltitudeCm);
    const float descentRateCmSIdeal = descentDistanceM / idealTimeSeconds * 100.0f; // cm/s
    const float descentRateCmSFinal = fmaxf(
        gpsRescueConfig()->descendRate, 
        descentRateCmSIdeal
    );
    return descentRateCmSFinal;
}

fileprivate void g_rememberTakeoffCourse(void)
{
    static bool previousWeHaveDistance = true; // prevent logging on first run
    
    const float idealDistanceM = 60.0f; // distance from home to trigger takeoff course recording
    const bool weHaveDistance = rescueState.sensor.distanceToHomeM > idealDistanceM;
    const bool weAreTooFar = rescueState.sensor.distanceToHomeM > idealDistanceM * 2.0f;

    LOG_UPDATE("weHaveDistance", weHaveDistance ? "true" : "false");
    LOG_UPDATE("weAreTooFar", weAreTooFar ? "true" : "false");
    if (weAreTooFar) {
        return;
    }
    
    if (!previousWeHaveDistance) {
        LOG_UPDATE("takeoff_zone_speed", "%f", (double)(rescueState.sensor.velocityToHomeCmS / 100.0f * 3.6f));
    }
        
    const bool didJustExitTakeoffZone = weHaveDistance && !previousWeHaveDistance;
    previousWeHaveDistance = weHaveDistance;

    // If we have a takeoff course, don't overwrite it
    if (rescueState.intent.takeoffCourseValid) {
        return;
    }

    if (!didJustExitTakeoffZone) {
        return; // we are still in the takeoff zone, no need to record the course
    }
    
    // one-off event -- exiting zone.

    const float minSpeedCmS = 20.0f * 100.0f / 3.6f; // 20km/h in cm/s
    const bool weHaveSpeed = -rescueState.sensor.velocityToHomeCmS > minSpeedCmS;

    if (weHaveSpeed) {
        rescueState.intent.takeoffCourse = gpsSol.groundCourse;
        rescueState.intent.takeoffNeutralPoint = gpsSol.llh;
        rescueState.intent.takeoffCourseValid = true;
        LOG_UPDATE("exit_data", "course:%+6.1f", (double)(rescueState.intent.takeoffCourse / 10.0f));
    } else {
        LOG_UPDATE("exit_data", "too slow %f", (double)(-rescueState.sensor.velocityToHomeCmS / 100.0f * 3.6f));
    }
}

void g_updateGPSRescue_processState(void)
{
    switch (rescueState.phase) {
    case RESCUE_IDLE: {
        g_updateMaxAltutude();
        g_rememberTakeoffCourse();
        break;
    }
        // sanity checks are bypassed in IDLE mode; instead, failure state is always initialised to HEALTHY
        // target altitude is always set to current altitude.

    case RESCUE_INITIALIZE: {
        g_initializeGPSRescue();
        break;
    }

    case RESCUE_FLY_HOME: {
        bool isClose = g_calculateTargetCourseForLoiter();
        if (isClose) {
            g_handleDescentDistanceReached();
        }

        // TODO: check if we are getting closer to home
        // if not, we should probably crash land
        break;
    }

    case RESCUE_DESCEND_TO_LOITER: {
        const float descentRateCmS = g_calculateBestDescentRateCmS(
            gpsRescueConfig()->ap_wing_loiter_alt * 100.0f,
            20.0f
        );
        g_performDescentWithRateCmS(descentRateCmS);
        rescueState.intent._targetAltitudeCm = fmaxf(gpsRescueConfig()->ap_wing_loiter_alt * 100.0f, rescueState.intent._targetAltitudeCm);
        
        g_calculateTargetCourseForLoiter();

        bool reachedLoiterAltitude = rescueState.sensor.currentAltitudeCm / 100.0f < gpsRescueConfig()->ap_wing_loiter_alt;
        if (reachedLoiterAltitude) {
            rescueState.loiterActivatedTime = millis();
            rescueState.phase = RESCUE_LOITER;
            rescueState.intent._targetAltitudeCm = fmin(gpsRescueConfig()->ap_wing_loiter_alt * 100.0f, rescueState.intent.returnAltitudeCm);
        }
        break;
    }

    case RESCUE_LOITER: {
        g_calculateTargetCourseForLoiter();

        bool loiterTimeLimitReached = (millis() - rescueState.loiterActivatedTime >= ((timeMs_t)gpsRescueConfig()->ap_wing_loiter_seconds) * 1000);
        if (loiterTimeLimitReached) {
            g_startRescueApproach();
        }
        break;
    }

    // NOT USED
    case RESCUE_DESCEND_TO_LAND: {
        g_calculateTargetCourseForLoiter();

        const float descentRateCmS = g_calculateBestDescentRateCmS(
            gpsRescueConfig()->ap_wing_landing_approach_dist * 100.0f,
            20.0f
        );
        g_performDescentWithRateCmS(descentRateCmS);
        rescueState.intent._targetAltitudeCm = fmaxf(gpsRescueConfig()->ap_wing_landing_approach_dist * 100.0f, rescueState.intent._targetAltitudeCm);

        const bool isBelowLandingAltitude = rescueState.sensor.currentAltitudeCm / 100.0f < gpsRescueConfig()->ap_wing_landing_alt;
        if (isBelowLandingAltitude) {
            PRINT("GPS RESCUE: Descend to land phase, below landing altitude, starting approach manoeuvre");
            g_startRescueApproach();
        }
        break;
    }

    case RESCUE_FLY_AWAY_BEFORE_APPROACH: {
        const float turnAroundBufferM = 20.0f;
        const float sufficientDistanceToHomeM = gpsRescueConfig()->ap_wing_landing_approach_dist + turnAroundBufferM;
        bool isFarEnoughFromHome = rescueState.sensor.distanceToHomeM > sufficientDistanceToHomeM;
        LOG_UPDATE("isFarEnoughFromHome", "%s   -   %f > %f", 
            isFarEnoughFromHome ? "true" : "false",
            (double)rescueState.sensor.distanceToHomeM, 
            (double)sufficientDistanceToHomeM
        );
        if (isFarEnoughFromHome) {
            rescueState.waitForCourseAwayActivatedTime = millis();
            rescueState.phase = RESCUE_WAIT_FOR_COURSE_AWAY;
        }
        break;
    }

    case RESCUE_WAIT_FOR_COURSE_AWAY: {
        const float courseErrorDegrees = normalizeCourseErrorDecidegrees(
            rescueState.intent.targetCourseDecidegrees,
            gpsSol.groundCourse
        ) / 10.0f;
        
        const bool courseIsOK = ABS(courseErrorDegrees) < 20.0f;
        const bool waitedTooLong = millis() - rescueState.waitForCourseAwayActivatedTime >= 60 * 1000; // 1 minute

        if (courseIsOK || waitedTooLong) {
            rescueState.waitForCourseHomeActivatedTime = millis();
            rescueState.phase = RESCUE_WAIT_FOR_LANDING_COURSE;
            // rescueState.intent.targetAltitudeCm = gpsRescueConfig()->ap_wing_landing_alt * 100.0f;
        }
        break;
    };

    case RESCUE_WAIT_FOR_LANDING_COURSE: {
        // this needs to be kept up to date while turning around
        rescueState.intent.targetCourseDecidegrees = rescueState.sensor.directionToHome;
        // rescueState.intent.targetAltitudeCm = gpsRescueConfig()->ap_wing_landing_alt * 100.0f;

        const float courseErrorDegrees = normalizeCourseErrorDecidegrees(
            rescueState.intent.targetCourseDecidegrees,
            gpsSol.groundCourse
        ) / 10.0f;
        
        const bool courseIsOK = ABS(courseErrorDegrees) < 30.0f;
        const bool waitedTooLong = millis() - rescueState.waitForCourseHomeActivatedTime >= 120 * 1000;
        const bool isCloseToHome = rescueState.sensor.distanceToHomeM <= gpsRescueConfig()->ap_wing_landing_approach_dist;

        if (courseIsOK) {
            // if (courseIsOK) {
                rescueState.phase = RESCUE_LAND;
                rescueState.landActivatedTime = millis();
            // } else {
            //     // retry approach manoeuvre again
            //     PRINT("GPS RESCUE: Wait for landing distance phase, course is not OK, reapproaching home");
            //     g_startRescueApproach();
            // }
        } else if (waitedTooLong) {
            // we are unable to get close to landing distance in time, so we will land where we are
            // this is a bad situation because it means we just can't move towards home fast enough
            rescueState.phase = RESCUE_LAND;
            rescueState.landActivatedTime = millis();
            // rescueState.intent.returnAltitudeCm = rescueState.sensor.currentAltitudeCm;
        } else if (isCloseToHome) {
            // retry approach manoeuvre again
            PRINT("GPS RESCUE: Wait for landing distance phase, course is not OK, reapproaching home");
            g_startRescueApproach();
        } else {
            // do nothing, wait for distance to home to be close enough
        }
        break;
    };

    case RESCUE_LAND: {
        const float currentAltitudeM = rescueState.sensor.currentAltitudeCm / 100.0f;
        // const float currentSpeedCmS = rescueState.sensor.groundSpeedCmS;
        // const float timeToHomeSeconds = rescueState.sensor.distanceToHomeCm / currentSpeedCmS;

        // keep home direction up to date while landing
        rescueState.intent.targetCourseDecidegrees = rescueState.sensor.directionToHome;

        // calculate descent progress
        const float descnedStartDistanceM = gpsRescueConfig()->ap_wing_landing_approach_dist;
        const float descendEndDistanceM = 15.0f;
        float landingProgress = scaleRangef(
            rescueState.sensor.distanceToHomeM, 
            descnedStartDistanceM, descendEndDistanceM, 
            0.0f, 1.0f
        );
        rescueState.landingProgress = landingProgress;
        landingProgress = constrainf(landingProgress, 0.0f, 1.0f);
        landingProgress = powf(landingProgress, 1.0f/4.0f);

        // calculate target speed
        const float touchDownSpeedCmS = gpsRescueConfig()->ap_wing_landing_speed / 3.6f * 100.0f; // km/h to cm/s
        const float targetSpeedCmS = scaleRangef(
            landingProgress,
            0.0f, 1.0f,
            gpsRescueConfig()->groundSpeedCmS, touchDownSpeedCmS
        );
        rescueState.intent.targetVelocityCmS = constrainf(targetSpeedCmS, touchDownSpeedCmS, gpsRescueConfig()->groundSpeedCmS);

        // calculate target altitude
        float targetAltCM = scaleRangef(
            landingProgress,
            0.0f, 1.0f,
            gpsRescueConfig()->ap_wing_landing_alt * 100.0f, 0.0f
        );
        float targetAlt = constrainf(targetAltCM, 0.0f, gpsRescueConfig()->ap_wing_landing_alt * 100.0f);
        rescueState.intent._targetAltitudeCm = targetAlt;
        if (descentProgress > 0.3f) {
            // bypass smoothing
            pt3FilterSetValue(&targetAltitudeLpf, targetAlt);
        }

        // print debug info
        LOG_UPDATE("landing_progress", "%f", (double)landingProgress);
        
        // TODO: check if we are moving away from home

        if (currentAltitudeM < 5.0f) {
            // THROTTLED_PRINT("GPS RESCUE: will disarm on impact...");
            disarmOnImpact();
        }
        
        if (currentAltitudeCm / 100.0f < 2.5f) {
            THROTTLED_PRINT("GPS RESCUE: forceDisarm!!!");
            forceDisarm(DISARM_REASON_GPS_RESCUE);
            rescueStop();
        }

        // if (currentAltitudeCm - rescueState.intent.targetAltitudeCm > 1000.0f) {
        //     THROTTLED_PRINT("GPS RESCUE: HUGE ALT ERROR, reapproaching home again...");
        //     g_startRescueApproach();
        // }
        break;
    };

    case RESCUE_ABORT: {
        // setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        // disarm(DISARM_REASON_FAILSAFE);
        // rescueState.intent.secondsFailing = 0; // reset sanity timers so we can re-arm
        rescueStop();
        break;
    }

    case RESCUE_DISARM_ON_IMPACT: {
        disarmOnImpact();
        break;
    }

    default:
        break;
    }
}

fileprivate void g_updateSmoothTargetAltitude(void)
{
    rescueState.intent.smoothTargetAltitudeCm = pt3FilterApply(&targetAltitudeLpf, rescueState.intent._targetAltitudeCm);
}

fileprivate void g_throttledUpdateLoop(bool newGpsData)
{
    static timeMs_t lastTime = 0;

    if (lastTime == 0) {
        lastTime = millis();
        return;
    }

    const float dT = (millis() - lastTime) / 1000.0f;

    // don't update too often
    if (dT >= 1.0f / 20.0f
        || newGpsData
        || rescueState.phase == RESCUE_INITIALIZE
    ) {
        lastTime = millis();
        LOG_UPDATE("gps loop dT", "%f", (double)dT);
        g_rescueControlRollAndPitch(dT);
        float velocityPIDSum = g_gpsRescueGetVelocityPIDSum(dT);
        setAutopilotThrottle(gpsRescueGetThrottle(velocityPIDSum));
        g_updateSmoothTargetAltitude();
    }
}

void gpsRescueUpdate(void)
// runs at gpsRescueTaskIntervalSeconds, and runs whether or not rescue is active
{
    LOG_UPDATE("rescue phase", RESCUE_PHASE_STR(rescueState.phase));
    LOG_UPDATE("rescue fialure", RESCUE_FAILURE_STR(rescueState.failure));
    LOG_DISPLAY_100MS();
    
    bool newGpsData = g_updateGPSData();

    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop(); // sets phase to RESCUE_IDLE; does nothing else.  RESCUE_IDLE tasks still run.
    } else if (rescueState.phase == RESCUE_IDLE && ARMING_FLAG(ARMED)) {
        // executed only once
        PRINT("STARTING GPS RESCUE MODE FROM RESCUE_IDLE");
        
        rescueStart(); // sets phase to rescue_initialise if we enter GPS Rescue mode while idle
        g_performSanityChecks(); // Initialises sanity check values when a Rescue starts
    }

    sensorUpdate(newGpsData);

    rescueState.isAvailable = checkGPSRescueIsAvailable();
    
    g_throttledUpdateLoop(newGpsData);
    g_updateGPSRescue_processState();
    g_performSanityChecks();
}

float gpsRescueGetYawRate(void)
{
    return - gpsRescueConfig()->ap_wing_roll_yaw_mix / 100.0f * gpsRescueAngle[AI_ROLL];
}

float gpsRescueGetImuYawCogGain(void)
{
    return rescueState.sensor.imuYawCogGain;
}

float g_calculateVelocityPID(float resolvedCurrentVelocityCmS, float dT)
{
    static float previousVelocityError = 0.0f;
    static float velocityI = 0.0f;

    if (rescueState.phase == RESCUE_INITIALIZE) {
        previousVelocityError = 0.0f;
        velocityI = 0.0f;
    }

    const float velocityError = rescueState.intent.targetVelocityCmS - resolvedCurrentVelocityCmS;
    // velocityError is in cm per second, positive means too slow.
    // NB positive pitch setpoint means nose down.
    // target velocity can be very negative leading to large error before the start, with overshoot

    // P component
    const float velocityP = velocityError * gpsRescueConfig()->velP * 0.001f;

    // I component
    velocityI += 0.001f * gpsRescueConfig()->velI * velocityError * dT; // * rescueState.intent.velocityItermRelax;

    const float velocityILimit = 0.25f; // TODO: ???
    velocityI = constrainf(velocityI, -velocityILimit, velocityILimit);

    // D component
    float velocityD = ((velocityError - previousVelocityError) / (dT * 10.0f));
    previousVelocityError = velocityError;
    velocityD *= gpsRescueConfig()->velD * 0.001f;
    pt1FilterUpdateCutoffWithDTSeconds(&velocityDLpf, rescueState.intent.velocityDLpfCutoff, dT);
    velocityD = pt1FilterApply(&velocityDLpf, velocityD);

    float velocityPIDSum = velocityP + velocityI + velocityD;
    // velocityPIDSum += g_antiStallThrottleBoost();
    velocityPIDSum = constrainf(velocityPIDSum, -1.0f, 1.0f);

    LOG_UPDATE("d. speed tracked", "%+6.1f -> %+6.1f   err: %+6.1f  km/h", 
                (double)(resolvedCurrentVelocityCmS / 100.0f) * 3.6,
                (double)(rescueState.intent.targetVelocityCmS / 100.0f) * 3.6, 
                (double)(velocityError / 100.0f) * 3.6);

    LOG_UPDATE("d. throttle_pid", "p:%+6.3f i:%+6.3f d:%+6.3f sum:%+6.3f", 
                    (double)velocityP, (double)velocityI, (double)velocityD, (double)velocityPIDSum);

    return velocityPIDSum;
}

// Returns throttle value offset.
float g_gpsRescueGetVelocityPIDSum(float dT)
{
    static float velocityPIDSum = 0.0f;

    const float airSpeedCmS = pidRuntime.tpaSpeed.speed * 100.0f;
    const float gpsSpeed = rescueState.sensor.groundSpeedCmS;
    const bool isGpsBad = !sanityState.isGPSHealthy;
    
    // during landing we ignore air speed, otherwise we will overspeed with wind.
    const bool isGpsPrioritised = !isGpsBad && rescueState.phase == RESCUE_LAND;

    float trackedSpeedCmS = isGpsPrioritised ? 
        gpsSpeed : 
        // Prevents stall during flight AND prevents very low ground speed when flying against wind.
        fminf(airSpeedCmS, gpsSpeed);

    // TODO: check against "velocityTowardsTargetCourse" instead of absolute ground speed.

    velocityPIDSum = g_calculateVelocityPID(trackedSpeedCmS, dT);
    
    LOG_UPDATE("d. speed", "gps: %+6.1f     air: %+6.1f     km/h", 
        isGpsBad ? -1 : (double)(gpsSpeed / 100.0f) * 3.6,
        (double)(airSpeedCmS / 100.0f) * 3.6
    );

    return velocityPIDSum;
}

// TODO: this function is too complicated and very strange.
float gpsRescueGetThrottle(float velocityPIDSum)
{
    float commandedThrottle = velocityPIDSum;
    // THROTTLED_PRINT("velocityPIDSum: %f", (double)velocityPIDSum);
    //DEBUG_SET(DEBUG_WING_RTH, 1, lrintf(commandedThrottle * 100.0f));

    // less voltage - more throttle
    float batteryThrottleFactor = 1.0f;
    if (pidRuntime.tpaSpeed.maxVoltage > 0.0f) {
        batteryThrottleFactor = getBatteryVoltageLatest() / 100.0f / pidRuntime.tpaSpeed.maxVoltage;
        batteryThrottleFactor = constrainf(batteryThrottleFactor, 0.0f, 1.0f);
    }
    THROTTLED_PRINT_MS(30000, "batteryThrottleFactor: %f", (double)(1.0f/batteryThrottleFactor));
    commandedThrottle = commandedThrottle / batteryThrottleFactor;

    commandedThrottle += 0.3f; // add a base throttle to make it fly before I term accumulation

    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);

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
