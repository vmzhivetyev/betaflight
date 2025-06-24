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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "../fc/rc_controls.h"

#ifdef USE_VIRTUAL_GPS

#include "io/gps_virtual.h"
#include <rx/rx.h>

static gpsSolutionData_t gpsVirtualData;

void setBadGPS(void)
{
    gpsVirtualData.numSat = 0;    // satellites_in_view
    gpsVirtualData.acc.hAcc = 9999999; // horizontal_pos_accuracy - convert cm to mm
    gpsVirtualData.acc.vAcc = 9999999; // vertical_pos_accuracy - convert cm to mm
    gpsVirtualData.acc.sAcc = 9999999; // horizontal_vel_accuracy - convert cm to mm
    gpsVirtualData.dop.pdop = 999; // hdop in 4.4 and earlier, pdop in 4.5 and above
    gpsVirtualData.llh.lon = 0;
    gpsVirtualData.llh.lat = 0;
    gpsVirtualData.llh.altCm = 0; // alt, cm
    gpsVirtualData.groundSpeed = 0;  // cm/sec
    gpsVirtualData.speed3d = 0;    // cm/sec
    gpsVirtualData.groundCourse = 0; // decidegrees
}

void setVirtualGPS(double latitude, double longitude, double altiutude, double speed, double speed3D, double course)
{
    LOG_UPDATE_100MS("AUX5", "%f", (double)rcData[AUX5]);

    if (rcData[AUX5] < 1010.0f) {
        setBadGPS();
        return;
    }
    gpsVirtualData.numSat = 12;    // satellites_in_view
    gpsVirtualData.acc.hAcc = 500; // horizontal_pos_accuracy - convert cm to mm
    gpsVirtualData.acc.vAcc = 500; // vertical_pos_accuracy - convert cm to mm
    gpsVirtualData.acc.sAcc = 400; // horizontal_vel_accuracy - convert cm to mm
    gpsVirtualData.dop.pdop = 10; // hdop in 4.4 and earlier, pdop in 4.5 and above
    gpsVirtualData.llh.lon = (int32_t)(longitude * GPS_DEGREES_DIVIDER);
    gpsVirtualData.llh.lat = (int32_t)(latitude * GPS_DEGREES_DIVIDER);
    gpsVirtualData.llh.altCm = (int32_t)(altiutude * 100.0); // alt, cm
    gpsVirtualData.groundSpeed = (uint16_t)(speed * 100.0);  // cm/sec
    gpsVirtualData.speed3d = (uint16_t)(speed3D * 100.0);    // cm/sec
    gpsVirtualData.groundCourse = (uint16_t)(course * 10.0); // decidegrees
}

void getVirtualGPS(gpsSolutionData_t *gpsSolData)
{
    *gpsSolData = gpsVirtualData;
}
#endif
