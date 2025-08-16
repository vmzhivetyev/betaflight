#!/bin/bash

# Enable strict error handling
set -eux

# Target configuration
TARGET="HDZERO_AIO15"

# Uncomment to clean build artifacts
# make clean

# Generate dynamic build info
BUILD_KEY="$(git show --pretty=format:"%h" --no-patch)"
RELEASE_NAME="$BUILD_KEY at $(git rev-parse --abbrev-ref HEAD) built $(date '+%Y-%m-%d %H:%M:%S')"

echo "Building target: $TARGET"
echo "Release: $RELEASE_NAME"

# Build the target
make "$TARGET" \
  EXTRA_FLAGS=" \
    -D'BUILD_KEY=$BUILD_KEY' \
    -D'RELEASE_NAME=$RELEASE_NAME' \
    -DCLOUD_BUILD \
    -DUSE_DSHOT \
    -DUSE_LED_STRIP \
    -DUSE_PINIO \
    -DUSE_OSD \
    -DUSE_OSD_HD \
    -DUSE_SERIALRX \
    -DUSE_SERIALRX_CRSF \
    -DUSE_TELEMETRY \
    -DUSE_TELEMETRY_CRSF \
    -DUSE_VTX \
    -DUSE_TELEMETRY_SMARTPORT \
    -DUSE_BLACKBOX \
    -D'DEFAULT_BLACKBOX_DEVICE=BLACKBOX_DEVICE_SERIAL' \
  "

# Find and copy the generated hex file
HEX_FILE=$(find ./obj -name "betaflight_*_${TARGET}.hex" -type f | head -1)

if [ -n "$HEX_FILE" ] && [ -f "$HEX_FILE" ]; then
    echo "Found hex file: $HEX_FILE"
    cp "$HEX_FILE" ~/Downloads/
    echo "Copied to: ~/Downloads/$(basename "$HEX_FILE")"
else
    echo "Error: Could not find hex file matching pattern betaflight_*_${TARGET}.hex in ./obj"
    exit 1
fi