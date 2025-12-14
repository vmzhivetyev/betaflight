#!/bin/bash

set -eu
source build-common.sh

TARGET="MATEKF405TE"

DEFINES=(
  "CLOUD_BUILD"
  "USE_ALTITUDE_HOLD"
  "USE_DSHOT"
  "USE_GPS"
  "USE_GPS_PLUS_CODES"
  "USE_LED_STRIP"
  "USE_OSD_HD"
  "USE_PINIO"
  "USE_SERIALRX"
  "USE_SERIALRX_CRSF"
  "USE_TELEMETRY"
  "USE_TELEMETRY_CRSF"
  "USE_VTX"
  "USE_WING"
)

build_target "$TARGET" "${DEFINES[@]}"

copy_hex_to_downloads "$TARGET"
