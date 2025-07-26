#!/bin/bash

set -eu
source build-common.sh

TARGET="BETAFLIGHTF4"

DEFINES=(
  "CLOUD_BUILD"
  "USE_ALTITUDE_HOLD"
  "USE_DSHOT"
  "USE_LED_STRIP"
  "USE_OSD_HD"
  "USE_PINIO"
  "USE_SERIALRX"
  "USE_SERIALRX_CRSF"
  "USE_TELEMETRY"
  "USE_TELEMETRY_CRSF"
  "USE_VTX"
)

build_target "$TARGET" "${DEFINES[@]}"

copy_hex_to_downloads "$TARGET"
