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
    RESCUE_ABORT,
} rescuePhase_e;

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
    float takeoffCourse;
    gpsLocation_t takeoffNeutralPoint;
    float homeCourseAtActivation;
    bool takeoffCourseValid;
    
    float targetAltitudeCm;
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
static pt2Filter_t altitudeDLpf;
static pt1Filter_t velocityDLpf;
static pt3Filter_t velocityUpsampleLpf;
static float previousVelocityError = 0.0f;
static float velocityI = 0.0f;

rescueState_s rescueState;

#define desiredAltitudeCm rescueState.intent.targetAltitudeCm

// FORWARD DECLARATIONS

float g_gpsRescueGetVelocityPIDSum(bool newGpsData);

// UTILITY FUNCTIONS

static float normalizeCourseErrorDecidegrees(float targetDecidegrees, float currentDecidegrees) {
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
            rescueState.intent.returnAltitudeCm = fmaxf(rescueState.intent.returnAltitudeCm, gpsRescueConfig()->returnAltitudeM * 100.0f);
            break;
    }
}

void g_initialiseSimpleIntentValues (void);
static void g_rescueControlRollAndPitch(bool newGpsData);

static void g_initializeIntent(void) {
    g_initialiseSimpleIntentValues();
    g_setDescentDistanceFromConfig();
    g_setReturnAltitude();
    
    rescueState.intent.targetAltitudeCm = rescueState.intent.returnAltitudeCm;
}

static void g_initializeGPSRescue(void) {
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

    bool doCleanExit = false;

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        // values to be returned when no rescue is active
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        doCleanExit = true;
        break;
    case RESCUE_INITIALIZE:
        // Initialize internal variables each time GPS Rescue is started
        altI = 0.0f;
        previousAltitudeError = 0.0f;
        previousCourseError = 0.0f;
        previousVelocityError = 0.0f;
        velocityI = 0.0f;
        rescueState.intent.disarmThreshold = gpsRescueConfig()->disarmThreshold * 0.1f;
        // crutch: make sure course calculaiton is strong because we are a wing and always fly nose forward.
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
    altI += 0.1f * gpsRescueConfig()->ap_wing_alt_i * altitudeError * rescueState.sensor.altitudeDataIntervalSeconds;
    altI = constrainf(altI, -20.0f, 15.0f); // negative == nose up
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
    altD = pt2FilterApply(&altitudeDLpf, altD);

    calculatedPitchDegrees = altP + altI - altD;

    // anti-stall for low powered huge crafts
    // float pitchLimitUp = scaleRangef(previousVelocityError, 0.0f, 3.0f, -20.0f, 0.0f); // negative == up
    // pitchLimitUp = constrainf(pitchLimitUp, -30.0f, -10.0f);
    // calculatedPitchDegrees = constrainf(calculatedPitchDegrees, pitchLimitUp, 45.0f);

    // THROTTLED_PRINT_MS(100, "pitch tar: %f cur: %f", (double)desiredAltitudeCm / 100.0, (double)rescueState.sensor.currentAltitudeCm / 100.0);

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
        
        // THROTTLED_PRINT_MS(1000, "dist: %f, ctarget: %f, course: %f, cerr: %f", 
        //     (double)rescueState.sensor.distanceToHomeM, 
        //     (double)rescueState.intent.targetCourseDecidegrees / 10.0,  
        //     (double)gpsSol.groundCourse / 10.0, 
        //     (double)courseErrorDegrees
        // );
    }

    const float calculatedPitchDegreesRaw = calculatedPitchDegrees;

    const float pitchDegreesFromRoll = ABS((float)gpsRescueConfig()->ap_wing_roll_pitch_mix * calculatedRollDegrees);
    calculatedPitchDegrees -= pitchDegreesFromRoll; // pull up the nose to counteract a roll-induced pitch down
    calculatedPitchDegrees = constrainf(calculatedPitchDegrees, -gpsRescueConfig()->maxRescueAngle, gpsRescueConfig()->maxRescueAngle);
    
    LOG_UPDATE_100MS("altitude_pid", "p:%+6.3f i:%+6.3f d:%+6.3f sumRaw:%+6.3f, sumClamped:%+6.3f", 
                    (double)altP, (double)altI, (double)altD, (double)calculatedPitchDegreesRaw, (double)calculatedPitchDegrees);

    LOG_UPDATE_100MS("altitude", "tar:%+6.1f cur:%+6.1f err:%+6.1f", 
                    (double)desiredAltitudeCm / 100.0, 
                    (double)rescueState.sensor.currentAltitudeCm / 100.0,
                    (double)altitudeError);

    if (doCleanExit) {
        return;
    }
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
    rescueState.sensor.distanceToHomeCm = GPS_distanceToHomeCm;
    rescueState.sensor.distanceToHomeM = rescueState.sensor.distanceToHomeCm / 100.0f;

    LOG_UPDATE_100MS("home_distance", "%+6.1f", (double)rescueState.sensor.distanceToHomeM);

    LOG_UPDATE_100MS("velocity_to_home", "%+6.1f", (double)rescueState.sensor.velocityToHomeCmS / 100.0 * 3.6);

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
    rescueState.intent.targetVelocityCmS = gpsRescueConfig()->groundSpeedCmS + 20.0f; // avoid snap from D at the start
    rescueState.intent.rollAngleLimitDeg = 0.0f; // no roll until flying home
    // rescueState.intent.velocityPidCutoffModifier = 1.0f; // normal velocity lowpass filter cutoff
    rescueState.intent.pitchAngleLimitDeg = 0.0f; // force pitch adjustment to zero - level mode will level out
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

static void g_startRescueApproach(void)
{
    rescueState.phase = RESCUE_FLY_AWAY_BEFORE_APPROACH;

    // turn away from home before starting to land;
    if (rescueState.intent.takeoffCourseValid) {
        rescueState.intent.targetCourseDecidegrees = rescueState.intent.takeoffCourse;
    } else {
        // assume takeoff course is just back from home course
        rescueState.intent.targetCourseDecidegrees = rescueState.intent.homeCourseAtActivation + 1800.0f;
    }
    rescueState.intent.targetAltitudeCm = gpsRescueConfig()->ap_wing_landing_alt * 100.0f;
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

static void g_rememberTakeoffCourse(void)
{
    static bool previousWeHaveDistance = true; // prevent logging on first run
    
    const float idealDistanceM = 60.0f; // distance from home to trigger takeoff course recording
    const bool weHaveDistance = rescueState.sensor.distanceToHomeM > idealDistanceM;
    const bool weAreTooFar = rescueState.sensor.distanceToHomeM > idealDistanceM * 2.0f;

    LOG_UPDATE_100MS("weHaveDistance", weHaveDistance ? "true" : "false");
    LOG_UPDATE_100MS("weAreTooFar", weAreTooFar ? "true" : "false");
    if (weAreTooFar) {
        return;
    }
    
    if (!previousWeHaveDistance) {
        LOG_UPDATE_100MS("takeoff_zone_speed", "%f", (double)(rescueState.sensor.velocityToHomeCmS / 100.0f * 3.6f));
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
        LOG_UPDATE_100MS("exit_data", "course:%+6.1f", (double)(rescueState.intent.takeoffCourse / 10.0f));
    } else {
        LOG_UPDATE_100MS("exit_data", "too slow %f", (double)(-rescueState.sensor.velocityToHomeCmS / 100.0f * 3.6f));
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
        rescueState.intent.targetAltitudeCm = fmaxf(gpsRescueConfig()->ap_wing_loiter_alt * 100.0f, rescueState.intent.targetAltitudeCm);
        
        g_calculateTargetCourseForLoiter();

        bool reachedLoiterAltitude = rescueState.sensor.currentAltitudeCm / 100.0f < gpsRescueConfig()->ap_wing_loiter_alt;
        if (reachedLoiterAltitude) {
            rescueState.loiterActivatedTime = millis();
            rescueState.phase = RESCUE_LOITER;
            desiredAltitudeCm = fmin(gpsRescueConfig()->ap_wing_loiter_alt * 100.0f, rescueState.intent.returnAltitudeCm);
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
        rescueState.intent.targetAltitudeCm = fmaxf(gpsRescueConfig()->ap_wing_landing_approach_dist * 100.0f, rescueState.intent.targetAltitudeCm);

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
        if (isFarEnoughFromHome) {
            PRINT("isFarEnoughFromHome, %f > %f", 
                (double)rescueState.sensor.distanceToHomeM, 
                (double)sufficientDistanceToHomeM
            );
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
        const float currentAltitudeCm = rescueState.sensor.currentAltitudeCm;
        const float currentSpeedCmS = rescueState.sensor.groundSpeedCmS;
        const float distanceToHomeCm = rescueState.sensor.distanceToHomeM * 100.0f;
        const float timeToHomeSeconds = distanceToHomeCm / currentSpeedCmS;

        // aim to touch down exactly at the home point
        const float landingDescentRateCmS = currentAltitudeCm / timeToHomeSeconds;
        g_performDescentWithRateCmS(landingDescentRateCmS);

        // keep home direction up to date while landing
        rescueState.intent.targetCourseDecidegrees = rescueState.sensor.directionToHome;

        // calculate descent progress
        const float descnedStartDistanceM = gpsRescueConfig()->ap_wing_landing_approach_dist;
        const float descendEndDistanceM = 15.0f;
        float descentProgress = scaleRangef(
            distanceToHomeCm / 100.0f, 
            descnedStartDistanceM, descendEndDistanceM, 
            0.0f, 1.0f
        );
        descentProgress = constrainf(descentProgress, 0.0f, 1.0f);
        descentProgress = powf(descentProgress, 1.0f/4.0f);

        // calculate target speed
        const float touchDownSpeedCmS = gpsRescueConfig()->ap_wing_landing_speed / 3.6f * 100.0f; // km/h to cm/s
        const float targetSpeedCmS = scaleRangef(
            descentProgress,
            0.0f, 1.0f,
            gpsRescueConfig()->groundSpeedCmS, touchDownSpeedCmS
        );
        rescueState.intent.targetVelocityCmS = constrainf(targetSpeedCmS, touchDownSpeedCmS, gpsRescueConfig()->groundSpeedCmS);

        // calculate target altitude
        float targetAltCM = scaleRangef(
            descentProgress,
            0.0f, 1.0f,
            gpsRescueConfig()->ap_wing_landing_alt * 100.0f, 50.0f
        );
        float targetAlt = constrainf(targetAltCM, 0.0f, gpsRescueConfig()->ap_wing_landing_alt * 100.0f);
        rescueState.intent.targetAltitudeCm = targetAlt;

        // print debug info
        LOG_UPDATE_100MS("landing_progress", "%f", (double)descentProgress);
        LOG_UPDATE_100MS("distanceToHomeM", "%f", (double)rescueState.sensor.distanceToHomeM);
        
        // TODO: check if we are moving away from home

        if (currentAltitudeCm / 100.0f < 5.0f) {
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

void gpsRescueUpdate(void)
// runs at gpsRescueTaskIntervalSeconds, and runs whether or not rescue is active
{
    LOG_DISPLAY_100MS();
    PRINT_ON_CHANGE(rescueState.phase, "rescue state changed to: %s", RESCUE_PHASE_STR(rescueState.phase));
    
    bool newGpsData = g_updateGPSData();

    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop(); // sets phase to RESCUE_IDLE; does nothing else.  RESCUE_IDLE tasks still run.
    } else if (rescueState.phase == RESCUE_IDLE && ARMING_FLAG(ARMED)) {
        // executed only once
        PRINT("STARTING GPS RESCUE MODE FROM RESCUE_IDLE");
        
        rescueStart(); // sets phase to rescue_initialise if we enter GPS Rescue mode while idle
        performSanityChecks(); // Initialises sanity check values when a Rescue starts
    }

    // Will now be in RESCUE_INITIALIZE mode, if just entered Rescue while IDLE, otherwise stays IDLE

    sensorUpdate(newGpsData); // always get latest GPS and Altitude data, update ascend and descend rates

    rescueState.isAvailable = checkGPSRescueIsAvailable();
    
    // THROTTLED_PRINT("distance: %f, current: %f, return: %f, desired: %f",
    //                 (double)rescueState.sensor.distanceToHomeM,
    //                 (double)rescueState.sensor.currentAltitudeCm / 100.0,
    //                 (double)rescueState.intent.returnAltitudeCm / 100.0,
    //                 (double)desiredAltitudeCm / 100.0);

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

// Returns throttle value offset.
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
        const float velocityP = velocityError * gpsRescueConfig()->velP * 0.001f;

        // I component
        velocityI += 0.0001f * gpsRescueConfig()->velI * velocityError * sampleIntervalNormaliseFactor; // * rescueState.intent.velocityItermRelax;
        // velocityItermRelax is a time-based factor, 0->1 with time constant of 1s from when we start to fly home
        // avoids excess iTerm accumulation during the initial acceleration phase and during descent.

        // velocityI *= rescueState.intent.velocityItermAttenuator;
        // used to minimise iTerm windup during IMU error states and iTerm overshoot in the descent phase
        // also, if we over-fly the home point, we need to re-accumulate iTerm from zero, not the previously accumulated value

        const float velocityILimit = 0.25f; // TODO: ???
        velocityI = constrainf(velocityI, -velocityILimit, velocityILimit);

        // D component
        float velocityD = ((velocityError - previousVelocityError) / sampleIntervalNormaliseFactor);
        previousVelocityError = velocityError;
        velocityD *= gpsRescueConfig()->velD * 0.001f;
        DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 5, lrintf(velocityD)); // velocity D before lowpass smoothing
        velocityD = pt1FilterApply(&velocityDLpf, velocityD);

        // note that this cutoff is increased up to 2x as we get closer to landing point in descend()
        const float cutoffHz = rescueState.intent.velocityDLpfCutoff; // * rescueState.intent.velocityPidCutoffModifier;
        pt1FilterUpdateCutoffWithDTSeconds(
            &velocityDLpf,
            cutoffHz,
            rescueState.sensor.gpsDataIntervalSeconds
        );

        // velocityD = pt1FilterApply(&velocityDLpf, velocityD);

        velocityPIDSum = velocityP + velocityI + velocityD;
        velocityPIDSum = constrainf(velocityPIDSum, -1.0f, 1.0f);

        LOG_UPDATE_DOUBLE_100MS("vel_pidsum", (double)velocityPIDSum);
        LOG_UPDATE_100MS("velocity", "tar:%+6.1f cur:%+6.1f err: %+6.1f", 
                 (double)(rescueState.intent.targetVelocityCmS / 100.0f) * 3.6, 
                 (double)(currentSpeed / 100.0f) * 3.6,
                 (double)(velocityError / 100.0f) * 3.6);
                 
        LOG_UPDATE_DOUBLE_100MS("cutoff_hz", (double)cutoffHz);

        LOG_UPDATE_100MS("throttle_pid", "p:%+6.3f i:%+6.3f d:%+6.3f sum:%+6.3f", 
                        (double)velocityP, (double)velocityI, (double)velocityD, (double)velocityPIDSum);
    }

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
