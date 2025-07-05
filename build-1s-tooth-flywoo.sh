#!/bin/bash

# Enable strict error handling
set -eux

# Uncomment to clean build artifacts
# make clean

# Generate dynamic release name with git info and timestamp
RELEASE_NAME="$(git show --pretty=format:"%h" --no-patch) at $(git rev-parse --abbrev-ref HEAD) built $(date '+%Y-%m-%d %H:%M:%S')"
echo "Building release: $RELEASE_NAME"

make FLYWOOF405S_AIO \
  EXTRA_FLAGS=" \
    -D'BUILD_KEY=$(git show --pretty=format:"%h" --no-patch)' \
    -D'RELEASE_NAME=$RELEASE_NAME' \
    -DCLOUD_BUILD \
    -DUSE_DSHOT \
    -DUSE_LED_STRIP \
    -DUSE_PINIO \
    -DUSE_OSD \
    -DUSE_OSD_SD \
    -DUSE_SERIALRX \
    -DUSE_SERIALRX_CRSF \
    -DUSE_TELEMETRY \
    -DUSE_TELEMETRY_CRSF \
    -DUSE_VTX \
  "
