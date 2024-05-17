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
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_ALTHOLD_MODE

#include "cli/settings.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_alt_hold.h"

#include "config/feature.h"

#include "config/config.h"

#include "flight/alt_hold.h"

static uint8_t altholdConfig_throttlePidP;
static uint8_t altholdConfig_throttlePidD;
static uint8_t altholdConfig_throttlePidI;
static uint8_t altholdConfig_throttlePidIMax;
static uint8_t altholdConfig_throttlePidDFiltCutoffFreq;

static uint8_t altholdConfig_minThrottle;
static uint8_t altholdConfig_maxThrottle;
static uint8_t altholdConfig_hoverThrottle;

static uint16_t altholdConfig_maxAltitude;

static uint8_t altholdConfig_enterFadeTimeDecisec;
static uint8_t altholdConfig_exitFadeTimeDecisec;

static const void *cmsx_menuAltitudeHoldOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    altholdConfig_throttlePidP = altholdConfig()->throttlePidP;
    altholdConfig_throttlePidD = altholdConfig()->throttlePidD;
    altholdConfig_throttlePidI = altholdConfig()->throttlePidI;
    altholdConfig_throttlePidIMax = altholdConfig()->throttlePidIMax;
    altholdConfig_throttlePidDFiltCutoffFreq = altholdConfig()->throttlePidDFiltCutoffFreq;

    altholdConfig_minThrottle = altholdConfig()->minThrottle;
    altholdConfig_maxThrottle = altholdConfig()->maxThrottle;
    altholdConfig_hoverThrottle = altholdConfig()->hoverThrottle;

    altholdConfig_maxAltitude = altholdConfig()->maxAltitude;

    altholdConfig_enterFadeTimeDecisec = altholdConfig()->enterFadeTimeDecisec;
    altholdConfig_exitFadeTimeDecisec = altholdConfig()->exitFadeTimeDecisec;

    return NULL;
}

static const void *cmsx_menuAltitudeHoldOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    altholdConfigMutable()->throttlePidP =                  altholdConfig_throttlePidP;
    altholdConfigMutable()->throttlePidD =                  altholdConfig_throttlePidD;
    altholdConfigMutable()->throttlePidI =                  altholdConfig_throttlePidI;
    altholdConfigMutable()->throttlePidIMax =               altholdConfig_throttlePidIMax;
    altholdConfigMutable()->throttlePidDFiltCutoffFreq =    altholdConfig_throttlePidDFiltCutoffFreq;

    altholdConfigMutable()->minThrottle =                   altholdConfig_minThrottle;
    altholdConfigMutable()->maxThrottle =                   altholdConfig_maxThrottle;
    altholdConfigMutable()->hoverThrottle =                 altholdConfig_hoverThrottle;

    altholdConfigMutable()->maxAltitude =                   altholdConfig_maxAltitude;

    altholdConfigMutable()->enterFadeTimeDecisec = altholdConfig_enterFadeTimeDecisec;
    altholdConfigMutable()->exitFadeTimeDecisec = altholdConfig_exitFadeTimeDecisec;

    return NULL;
}

const OSD_Entry cmsx_menuAltitudeHoldEntries[] =
{
    {"--- ALTITUDE HOLD ---", OME_Label, NULL, NULL},

    { "MAX ALTITUDE",      OME_UINT16, NULL, &(OSD_UINT16_t){ &altholdConfig_maxAltitude, 0, 4000, 1 } },

    { "MIN THRTL",         OME_UINT8, NULL, &(OSD_UINT8_t){ &altholdConfig_minThrottle, 0, 50, 1 } },
    { "MAX THRTL",         OME_UINT8, NULL, &(OSD_UINT8_t){ &altholdConfig_maxThrottle, 20, 100, 1 } },
    { "HOVER THRTL",       OME_UINT8, NULL, &(OSD_UINT8_t){ &altholdConfig_hoverThrottle, 0, 100, 1 } },

    { "PID P",        OME_UINT8, NULL, &(OSD_UINT8_t){ &altholdConfig_throttlePidP, 0, 255, 1 } },
    { "PID D",        OME_UINT8, NULL, &(OSD_UINT8_t){ &altholdConfig_throttlePidD, 0, 255, 1 } },
    { "PID I",        OME_UINT8, NULL, &(OSD_UINT8_t){ &altholdConfig_throttlePidI, 0, 255, 1 } },
    { "PID I MAX",    OME_UINT8, NULL, &(OSD_UINT8_t){ &altholdConfig_throttlePidIMax, 0, 50, 1 } },
    { "PID D CUTOFF", OME_UINT8, NULL, &(OSD_UINT8_t){ &altholdConfig_throttlePidDFiltCutoffFreq, 5, 255, 1 } },

    { "ENTER FADE T",      OME_UINT8, NULL, &(OSD_UINT8_t){ &altholdConfig_enterFadeTimeDecisec, 0, 10, 1 } },
    { "EXIT  FADE T",      OME_UINT8, NULL, &(OSD_UINT8_t){ &altholdConfig_exitFadeTimeDecisec, 0, 30, 1 } },

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuAltitudeHold = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUGPSRPID",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuAltitudeHoldOnEnter,
    .onExit = cmsx_menuAltitudeHoldOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuAltitudeHoldEntries,
};

#endif
