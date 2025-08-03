#!/bin/bash
# Enable strict error handling
set -eux

# Uncomment to clean build artifacts
# make clean

# Generate dynamic release name with git info and timestamp
RELEASE_NAME="$(git show --pretty=format:"%h" --no-patch) at $(git rev-parse --abbrev-ref HEAD) built $(date '+%Y-%m-%d %H:%M:%S')"
echo "Building release: $RELEASE_NAME"

make SPEEDYBEEF405MINI \
  EXTRA_FLAGS=" \
    -D'BUILD_KEY=$(git show --pretty=format:"%h" --no-patch)' \
    -D'RELEASE_NAME=$RELEASE_NAME' \
    -DCLOUD_BUILD \
    -DUSE_ALTITUDE_HOLD \
    -DUSE_DSHOT \
    -DUSE_GPS \
    -DUSE_GPS_PLUS_CODES \
    -DUSE_LED_STRIP \
    -DUSE_OSD_HD \
    -DUSE_PINIO \
    -DUSE_POSITION_HOLD \
    -DUSE_SERIALRX \
    -DUSE_SERIALRX_CRSF \
    -DUSE_SERVOS \
    -DUSE_SOFTSERIAL \
    -DUSE_TELEMETRY \
    -DUSE_TELEMETRY_CRSF \
    -DUSE_VTX \
    -DUSE_WING \
  "

cp ./obj/betaflight_4.6.0_STM32F405_SPEEDYBEEF405MINI.hex ~/Downloads/.
