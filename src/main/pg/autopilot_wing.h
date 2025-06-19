/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef USE_WING

#include <stdint.h>

#include "pg/pg.h"

typedef struct autopilotConfig_s {
    uint16_t ap_throttle;
    uint16_t ap_throttle_min;
    uint16_t ap_throttle_max;

    uint8_t ap_altitude_P;
    uint8_t ap_altitude_I;
    uint8_t ap_altitude_D;

    uint8_t ap_altitude_d_lpf_hz;

    uint8_t ap_cog_P;
    uint8_t ap_cog_I;
    uint8_t ap_cog_D;

    uint8_t ap_max_roll;
    uint8_t ap_max_pitch;
} autopilotConfig_t;

PG_DECLARE(autopilotConfig_t, autopilotConfig);

#endif // USE_WING
