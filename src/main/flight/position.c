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
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "osd/osd.h"

static float displayAltitudeCm = 0.0f;
static bool altitudeAvailable = false;

static float zeroedAltitudeCm = 0.0f;
static float zeroedAltitudeDerivative = 0.0f;

static pt2Filter_t altitudeLpf;
static pt2Filter_t altitudeDerivativeLpf;

#ifdef USE_VARIO
static int16_t estimatedVario = 0; // in cm/s
#endif

void positionInit(void)
{
    const float sampleTimeS = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);

    const float altitudeCutoffHz = positionConfig()->altitude_lpf / 100.0f;
    const float altitudeGain = pt2FilterGain(altitudeCutoffHz, sampleTimeS);
    pt2FilterInit(&altitudeLpf, altitudeGain);

    const float altitudeDerivativeCutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    const float altitudeDerivativeGain = pt2FilterGain(altitudeDerivativeCutoffHz, sampleTimeS);
    pt2FilterInit(&altitudeDerivativeLpf, altitudeDerivativeGain);
}

typedef enum {
    DEFAULT = 0,
    BARO_ONLY,
    GPS_ONLY
} altitudeSource_e;

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 6);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altitude_source = DEFAULT,
    .altitude_prefer_baro = 100, // percentage 'trust' of baro data
    .altitude_lpf = 300,
    .altitude_d_lpf = 100,
);

#if defined(USE_BARO) || defined(USE_GPS)
void calculateEstimatedAltitude(void)
{
    static bool wasArmed = false;
    static bool gpsAltOffsetCmHasValue = false; // whether a zero for the GPS altitude value exists
    static float gpsAltOffsetCm = 0.0f;
    static float baroAltOffsetCm = 0.0f;
    static float pendingBaroAltOffsetCm = 0.0f;

    static float gpsAltCmRaw = 0.0f; // will hold last value on transient loss of 3D fix
    float baroAltCmRaw = 0.0f;
    float gpsTrust = 0.3f; // if no pDOP value, use 0.3, intended range 0-1;
    bool haveBaroAlt = false; // true if baro exists and has been calibrated on power up

    // *** Get sensor data
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        baroAltCmRaw = getBaroAltitude();
        haveBaroAlt = true; // false only if there is no sensor on the board, or it has failed
    }
#endif
#ifdef USE_GPS
    const bool haveGpsAlt = sensors(SENSOR_GPS) && STATE(GPS_FIX); // GPS_FIX means a 3D fix, which requires min 4 sats.
    if (haveGpsAlt) {
        // On loss of 3D fix, gpsAltCmRaw remains at the last value, haveGpsAlt becomes false, and gpsTrust goes to zero.
        gpsAltCmRaw = gpsSol.llh.altCm; // static, so hold last altitude value if 3D fix is lost to prevent fly to moon
        if (gpsSol.dop.pdop != 0) {
            // pDOP of 1.0 is good.  100 is very bad.  Our gpsSol.dop.pdop values are *100
            // When pDOP is a value less than 3.3, GPS trust will be stronger than default.
            gpsTrust = 100.0f / gpsSol.dop.pdop;
            // *** TO DO - investigate if we should use vDOP or vACC with UBlox units;
        }
        // always use at least 10% of other sources besides gps if available
        gpsTrust = MIN(gpsTrust, 0.9f);
    }
#else
    const bool haveGpsAlt = false; // true if GPS is connected and while it has a 3D fix, set each run to false
#endif

    const float baroAltCmZeroed = baroAltCmRaw - baroAltOffsetCm;
    const float gpsAltCmZeroed = gpsAltCmRaw - gpsAltOffsetCm;

    if (!ARMING_FLAG(ARMED)) {
        if (wasArmed) {
            // WE HAVE JUST DISARMED
            wasArmed = false;
        }
        pendingBaroAltOffsetCm = 0.2f * baroAltCmRaw + 0.8f * pendingBaroAltOffsetCm;

    } else {
        if (!wasArmed) {
            // WE HAVE JUST ARMED
            wasArmed = true;

            statistic_t *stats = osdGetStats();
            bool allowZeroing = cmpTimeUs(stats->armed_time / 1000000, 10) < 0; // if armed less than 10 seconds in total

            if (allowZeroing) {
                if (haveBaroAlt) {
                    baroAltOffsetCm = pendingBaroAltOffsetCm;
                }
                if (haveGpsAlt) {
                    gpsAltOffsetCm = gpsAltCmRaw;
                    gpsAltOffsetCmHasValue = true;
                }
            }
        }

        // armed without gps zero offset, we can use baro values to zero later
        if (!gpsAltOffsetCmHasValue && haveBaroAlt && haveGpsAlt) {
            gpsAltOffsetCm = gpsAltCmRaw - baroAltCmZeroed; // not very accurate
            gpsAltOffsetCmHasValue = true;
        }
    }

    // Note: Non-debug baro altitude is logged as `blackboxCurrent->baroAlt = baro.altitude;`
    DEBUG_SET(DEBUG_ALTITUDE, 0, baroAltCmZeroed / 10.0f); // Zeroed BARO altitude in 0.1m, max 3,276m
    DEBUG_SET(DEBUG_ALTITUDE, 1, gpsAltCmZeroed / 10.0f); // Zeroed GPS altitude in 0.1m, max 3,276m
    DEBUG_SET(DEBUG_ALTITUDE, 2, gpsTrust * 100.0f); // gps trust based on hdop only, from 0 to 100 in the log

    /// PROPER SENSOR FUSION ///

    altitudeSource_e altitude_source = positionConfig()->altitude_source;

    if (altitude_source == DEFAULT) {
        if (!haveGpsAlt || !gpsAltOffsetCmHasValue) {
            gpsTrust = 0;

        } else if (!haveBaroAlt) {
            gpsTrust = 1;

        } else {
            const float absDifferenceMeters = fabsf(gpsAltCmZeroed - baroAltCmZeroed) / 100.0f;
            const float differenceBoost = 1.0f + positionConfig()->altitude_prefer_baro / 100.0f; // from 1 to 2
            const float differenceSignificance = constrainf(
                scaleRangef(absDifferenceMeters * differenceBoost, 1.0f, 10.0f, 0.0f, 1.0f),
                0, 1
            );
            const float trustMult = 1.0f - differenceSignificance;
            gpsTrust *= trustMult;
            DEBUG_SET(DEBUG_ALTITUDE, 3, gpsTrust * 100.0f); // gps trust after comparing altitudes, from 0 to 100 in the log
        }
    }
    if (altitude_source == BARO_ONLY) {
        gpsTrust = 0;
    }
    if (altitude_source == GPS_ONLY) {
        gpsTrust = 1;
    }
    zeroedAltitudeCm = gpsAltCmZeroed * gpsTrust + baroAltCmZeroed * (1.0f - gpsTrust);
    displayAltitudeCm = zeroedAltitudeCm;

    DEBUG_SET(DEBUG_ALTITUDE, 4, zeroedAltitudeCm / 10.0f); // Zeroed BARO&GPS altitude in 0.1m, max 3,276m
    zeroedAltitudeCm = pt2FilterApply(&altitudeLpf, zeroedAltitudeCm);
    DEBUG_SET(DEBUG_ALTITUDE, 5, zeroedAltitudeCm / 10.0f); // Zeroed and smoothed BARO&GPS altitude in 0.1m, max 3,276m
    ///////////////////

    // *** calculate Vario signal

    static float previousZeroedAltitudeCm = 0.0f;
    zeroedAltitudeDerivative = (zeroedAltitudeCm - previousZeroedAltitudeCm) * TASK_ALTITUDE_RATE_HZ; // cm/s
    previousZeroedAltitudeCm = zeroedAltitudeCm;

    zeroedAltitudeDerivative = pt2FilterApply(&altitudeDerivativeLpf, zeroedAltitudeDerivative);

#ifdef USE_VARIO
    estimatedVario = lrintf(zeroedAltitudeDerivative);
    estimatedVario = applyDeadband(estimatedVario, 10); // ignore climb rates less than 0.1 m/s
    DEBUG_SET(DEBUG_ALTITUDE, 6, estimatedVario);
#endif

    DEBUG_SET(DEBUG_RTH, 1, lrintf(displayAltitudeCm / 10.0f));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 2, lrintf(zeroedAltitudeCm));

    altitudeAvailable = haveGpsAlt || haveBaroAlt;
}

#endif //defined(USE_BARO) || defined(USE_GPS)

float getAltitudeCm(void)
{
    return zeroedAltitudeCm;
}

float getAltitudeDerivative(void)
{
    return zeroedAltitudeDerivative; // cm/s
}

bool isAltitudeAvailable(void) {
    return altitudeAvailable;
}

int32_t getEstimatedAltitudeCm(void)
{
    return lrintf(displayAltitudeCm);
}

#ifdef USE_GPS
float getAltitudeAsl(void)
{
    return gpsSol.llh.altCm;
}
#endif

#ifdef USE_VARIO
int16_t getEstimatedVario(void)
{
    return estimatedVario;
}
#endif
