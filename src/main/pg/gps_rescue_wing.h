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

#pragma once

#ifdef USE_WING

#include <stdint.h>

#include "pg/pg.h"

typedef struct gpsRescue_s {
    uint16_t maxRescueAngle; // degrees
    uint16_t returnAltitudeM; // meters
    uint16_t descentDistanceM; // meters
    uint16_t groundSpeedCmS; // centimeters per second
    uint8_t  yawP;
    uint8_t  minSats;
    uint16_t minStartDistM; // meters
    uint8_t  sanityChecks;
    uint8_t  allowArmingWithoutFix;
    uint8_t  useMag;
    uint8_t  altitudeMode;
    uint16_t ascendRate;
    uint16_t descendRate;
    uint16_t initialClimbM; // meters
    uint8_t  disarmThreshold;
    uint8_t  imuYawGain;

    uint8_t  pitchCutoffHz;
    uint8_t  throttleP, throttleI, throttleD;
    uint16_t throttleMin;
    uint16_t throttleMax;
    uint8_t  velP, velI, velD;
    uint8_t  rollMix;

    uint8_t ap_wing_alt_p, ap_wing_alt_i, ap_wing_alt_d;
    uint8_t ap_wing_cog_p, ap_wing_cog_i, ap_wing_cog_d;
    uint8_t ap_wing_throttle_d_cutoff_decihz;

    uint8_t ap_wing_roll_pitch_mix;
    uint8_t ap_wing_roll_yaw_mix;

    uint8_t ap_wing_loiter_alt;
    uint16_t ap_wing_loiter_seconds;
    uint8_t ap_wing_landing_alt;
    uint8_t ap_wing_landing_speed; // km/h
    uint16_t ap_wing_landing_approach_dist;
} gpsRescueConfig_t;

PG_DECLARE(gpsRescueConfig_t, gpsRescueConfig);

#endif // USE_WING
