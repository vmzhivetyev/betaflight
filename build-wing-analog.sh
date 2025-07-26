#!/bin/bash

set -eu
source build-common.sh

TARGET="SPEEDYBEEF405MINI"

DEFINES=(
  "CLOUD_BUILD"
  "USE_VARIO"
  "USE_GPS"
  "USE_GPS_PLUS_CODES"
  "USE_LED_STRIP"
  "USE_OSD"
  "USE_OSD_SD"
  "USE_PINIO"
  "USE_PWM_OUTPUT"
  "USE_SERIALRX"
  "USE_SERIALRX_CRSF"
  "USE_SERVOS"
  "USE_TELEMETRY"
  "USE_TELEMETRY_CRSF"
  "USE_VTX"
  "USE_WING"
)

build_target "$TARGET" "${DEFINES[@]}"

copy_hex_to_downloads "$TARGET"