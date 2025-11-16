#!/bin/bash

set -eu
source build-common.sh

TARGET="TAKERG4AIO"

DEFINES=(
  "CLOUD_BUILD"
  "USE_DSHOT"
  "USE_GPS"
  "USE_GPS_PLUS_CODES"
  "USE_LED_STRIP"
  "USE_PINIO"
  "USE_ALTITUDE_HOLD"
  "USE_POSITION_HOLD"
  "USE_OSD"
  "USE_OSD_HD"
  "USE_VTX"
  "USE_WING"
  "USE_SERIALRX"
  "USE_SERIALRX_CRSF"
  "USE_TELEMETRY"
  "USE_TELEMETRY_CRSF"
)

build_target "$TARGET" "${DEFINES[@]}"

copy_hex_to_downloads "$TARGET"
