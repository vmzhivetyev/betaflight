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

#include "platform.h"


// #define GYRO_NOISE_INJECTION_ENABLED
#include "gyro_noise.h"

static FAST_CODE void GYRO_FILTER_FUNCTION_NAME(void)
{
    static bool noiseInitialized = false;
    if (!noiseInitialized) {
        gyroNoiseInit(12345); // Use fixed seed for reproducible results
        
        // Configure noise parameters
        gyroNoiseConfig_t config = GYRO_NOISE_CONFIG_DEFAULT();
        config.amplitude = rpmFilterConfig()->rpm_filter_q * 0.1f;
        config.enabledTypes = GYRO_NOISE_WHITE | GYRO_NOISE_PINK | GYRO_NOISE_BROWN;
        gyroNoiseSetConfig(&config);
        
        noiseInitialized = true;
    }
    
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // DEBUG_GYRO_RAW records the raw value read from the sensor (not zero offset, not scaled)
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_RAW, axis, gyro.rawSensorDev->gyroADCRaw[axis]);

        // DEBUG_GYRO_SAMPLE(0) Record the pre-downsample value for the selected debug axis (same as DEBUG_GYRO_SCALED)
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 0, lrintf(gyro.gyroADC[axis]));

        // downsample the individual gyro samples
        float gyroADCf = 0;
        if (gyro.downsampleFilterEnabled) {
            // using gyro lowpass 2 filter for downsampling
            gyroADCf = gyro.sampleSum[axis];
        } else {
            // using simple average for downsampling
            if (gyro.sampleCount) {
                gyroADCf = gyro.sampleSum[axis] / gyro.sampleCount;
            }
            gyro.sampleSum[axis] = 0;
        }

        // gyroADCf = GYRO_NOISE_INJECT(axis, gyroADCf);
        
        // DEBUG_GYRO_SAMPLE(1) Record the post-downsample value for the selected debug axis
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 1, lrintf(gyroADCf));

#ifdef USE_RPM_FILTER
        gyroADCf = rpmFilterApply(axis, gyroADCf);
#endif

        // DEBUG_GYRO_SAMPLE(2) Record the post-RPM Filter value for the selected debug axis
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 2, lrintf(gyroADCf));

        // apply static notch filters and software lowpass filters
        gyroADCf = gyro.notchFilter1ApplyFn((filter_t *)&gyro.notchFilter1[axis], gyroADCf);
        gyroADCf = gyro.notchFilter2ApplyFn((filter_t *)&gyro.notchFilter2[axis], gyroADCf);
        gyroADCf = gyro.lowpassFilterApplyFn((filter_t *)&gyro.lowpassFilter[axis], gyroADCf);

        // DEBUG_GYRO_SAMPLE(3) Record the post-static notch and lowpass filter value for the selected debug axis
        GYRO_FILTER_AXIS_DEBUG_SET(axis, DEBUG_GYRO_SAMPLE, 3, lrintf(gyroADCf));

#ifdef USE_DYN_NOTCH_FILTER
        if (isDynNotchActive()) {
            if (axis == gyro.gyroDebugAxis) {
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT, 0, lrintf(gyroADCf));
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT_FREQ, 0, lrintf(gyroADCf));
                GYRO_FILTER_DEBUG_SET(DEBUG_DYN_LPF, 0, lrintf(gyroADCf));
            }

            dynNotchPush(axis, gyroADCf);
            gyroADCf = dynNotchFilter(axis, gyroADCf);

            if (axis == gyro.gyroDebugAxis) {
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT, 1, lrintf(gyroADCf));
                GYRO_FILTER_DEBUG_SET(DEBUG_DYN_LPF, 3, lrintf(gyroADCf));
            }
        }
#endif

        // DEBUG_GYRO_FILTERED records the scaled, filtered, after all software filtering has been applied.
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_FILTERED, axis, lrintf(gyroADCf));

        gyro.gyroADCf[axis] = gyroADCf;
    }
    gyro.sampleCount = 0;
}
