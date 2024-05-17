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

#include "platform.h"

#ifdef USE_ALTHOLD_MODE
#include "common/time.h"
#include "common/filter.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#define ALTHOLD_TASK_PERIOD 100         // hz

typedef struct {
    uint8_t throttlePidP; // %
    uint8_t throttlePidD; // %
    uint8_t throttlePidI; // %
    uint8_t throttlePidIMax; // %

    uint8_t throttlePidDFiltCutoffFreq; // 1 == 1 Hz
    uint8_t throttleFiltCutoffFreq; // 1 == 1 Hz
    uint8_t altitudeFiltCutoffFreq; // 1 == 1 Hz

    uint8_t minThrottle; // 1 == 1 %
    uint8_t maxThrottle; // 1 == 1 %
    uint8_t hoverThrottle; // 1 == 1 %

    uint16_t maxAltitude; // meters

    uint8_t enterFadeTimeDecisec; // 1 == 0.1s
    uint8_t exitFadeTimeDecisec; // 1 == 0.1s
} altholdConfig_t;

PG_DECLARE(altholdConfig_t, altholdConfig);

typedef struct {
    float max;
    float min;
    float kp;
    float kd;
    float ki;
    float iMax;

    float lastErr;
    // float lastP;
    // float lastI;
    // float lastD;
    float integral;
    pt2Filter_t dTermLpf;
} nicePid_s;

typedef struct {
    nicePid_s throttlePid;
    float throttle;
    float throttleFactor;
    float targetAltitude;
    float measuredAltitude;
    bool altHoldEnabled;
    uint32_t enterTime;
    uint32_t exitTime;
    float smoothedAltitude;
    pt2Filter_t throttleLpf;
    pt2Filter_t altitudeLpf;
} altHoldState_s;


void initAltHoldState(void);

void updateAltHoldState(timeUs_t currentTimeUs);

float getAltHoldThrottle(void);

float getAltHoldThrottleFactor(float currentThrottle);

 // In meters.
float getAltHoldTargetAltitude(void);

// In meters.
float getAltHoldCurrentAltitude(void);

bool getAltHoldActive(void);

#endif
