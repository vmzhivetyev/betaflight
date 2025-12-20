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

#include <stdint.h>

#include "platform.h"

#ifdef USE_PINIO

#include "build/debug.h"

#include "pg/pinio.h"

#include "drivers/io.h"
#include "drivers/timer.h"
#include "drivers/pwm_output.h"

typedef struct pinioRuntime_s {
    IO_t io;
    bool inverted;
    bool state;
    bool isPWM;
    volatile timCCR_t *ccr;
} pinioRuntime_t;

static pinioRuntime_t pinioRuntime[PINIO_COUNT];

void pinioInit(const pinioConfig_t *pinioConfig)
{
    for (int i = 0; i < PINIO_COUNT; i++) {
        IO_t io = IOGetByTag(pinioConfig->ioTag[i]);

        if (!io) {
            continue;
        }

        IOInit(io, OWNER_PINIO, RESOURCE_INDEX(i));

        const uint8_t mode = pinioConfig->config[i] & PINIO_CONFIG_MODE_MASK;
        const bool inverted = pinioConfig->config[i] & PINIO_CONFIG_OUT_INVERTED;

        switch (mode) {
        case PINIO_CONFIG_MODE_OUT_PP:
            // Initial state after reset is input, pull-up.
            // Avoid momentary off by presetting the output to hi.
            if (inverted) {
                IOHi(io);
            }
            IOConfigGPIO(io, IOCFG_OUT_PP);
            pinioRuntime[i].isPWM = false;
            pinioRuntime[i].inverted = inverted;
            pinioRuntime[i].state = inverted;
            if (inverted) {
                IOHi(io);
            } else {
                IOLo(io);
            }
            break;

        case PINIO_CONFIG_MODE_PWM:
            {
                const timerHardware_t *timer = timerAllocate(pinioConfig->ioTag[i], OWNER_PINIO, RESOURCE_INDEX(i));
                if (timer == NULL) {
                    continue;
                }

                IOConfigGPIOAF(io, IOCFG_AF_PP, timer->alternateFunction);

                timerChannel_t channel;
                // Configure PWM at 50Hz (20ms period), 1MHz timer clock
                // Period = 1000000 / 50 = 20000 ticks
                pwmOutConfig(&channel, timer, PWM_TIMER_1MHZ, PWM_TIMER_1MHZ / 50, 1000, 0);

                pinioRuntime[i].isPWM = true;
                pinioRuntime[i].inverted = inverted;
                pinioRuntime[i].ccr = channel.ccr;
                pinioRuntime[i].state = false;
                // Set initial state to 1000us (off state)
                *pinioRuntime[i].ccr = inverted ? 2000 : 1000;
            }
            break;

        default:
            continue;
        }

        pinioRuntime[i].io = io;
    }
}

void pinioSet(int index, bool on)
{
    if (index < 0 || index >= PINIO_COUNT) {
        return;
    }

    if (pinioRuntime[index].isPWM) {
        // PWM mode: 1000us when off, 2000us when on
        // Respect inverted bit
        const uint16_t pwmValue = (on ^ pinioRuntime[index].inverted) ? 2000 : 1000;
        if (pinioRuntime[index].ccr) {
            *pinioRuntime[index].ccr = pwmValue;
        }
        pinioRuntime[index].state = on;
    } else {
        // GPIO mode
        const bool newState = on ^ pinioRuntime[index].inverted;
        if (newState != pinioRuntime[index].state) {
            IOWrite(pinioRuntime[index].io, newState);
            pinioRuntime[index].state = newState;
        }
    }
}
#endif
