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
    uint8_t altPidP; // 1 - 100
    uint8_t altPidD; // 1 - 100
    // uint8_t altPidI;

    uint8_t velPidP; // 1 - 100
    uint8_t velPidD; // 1 - 100
    uint8_t velPidI; // 1 - 100
    uint8_t velPidIMax; // 1 - 100

    uint8_t minThrottle; // 1 - 100
    uint8_t maxThrottle; // 1 - 100

    uint8_t enterFadeTimeDecisec;
    uint8_t exitFadeTimeDecisec;
} altholdConfig_t;

PG_DECLARE(altholdConfig_t, altholdConfig);

typedef struct {
    float max;
    float min;
    float kp;
    float kd;
    float ki;
    float lastErr;
    float integral;
    float iMax;
} simplePid_s;

typedef struct {
    simplePid_s altPid;
    simplePid_s velPid;
    float throttle;
    float throttleFactor;
    float targetAltitude;
    float measuredAltitude;
    float measuredAccel;
    float velocityEstimate;
    bool altHoldEnabled;
    uint32_t enterTime;
    uint32_t exitTime;
    float smoothedAltitude;
    float smoothedVelocity;
    pt2Filter_t throttleLpf;
    pt1Filter_t altitudeLpf;
    pt1Filter_t velocityLpf;
} altHoldState_s;


void initAltHoldState(void);
void updateAltHoldState(timeUs_t currentTimeUs);
float getAltHoldThrottle(void);
float getAltHoldThrottleFactor(float currentThrottle);

#endif
