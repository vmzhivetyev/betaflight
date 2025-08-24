#!/bin/bash

set -eu
source build-common.sh

TARGET="CRAZYBEEF4FR"

DEFINES=(
    "CLOUD_BUILD"
    "USE_DSHOT"
    "USE_OSD"
    "USE_OSD_SD"
    "USE_PINIO"
    "USE_SERIALRX"
    "USE_SERIALRX_CRSF"
    "USE_SERVOS"
    "USE_SOFTSERIAL"
    "USE_TELEMETRY"
    "USE_TELEMETRY_CRSF"
    "USE_VTX"
    "USE_WING"
    "USE_BLACKBOX"
    "DEFAULT_BLACKBOX_DEVICE=BLACKBOX_DEVICE_SERIAL"
)

build_target "$TARGET" "${DEFINES[@]}"

copy_hex_to_downloads "$TARGET"
