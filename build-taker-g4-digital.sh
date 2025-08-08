set -eux
# make clean

RELEASE_NAME="$(git show --pretty=format:"%h" --no-patch) at $(git rev-parse --abbrev-ref HEAD) built $(date '+%Y-%m-%d %H:%M:%S')"
echo "$RELEASE_NAME"

make TAKERG4AIO \
    EXTRA_FLAGS=" \
        -D'BUILD_KEY=$(git show --pretty=format:"%h" --no-patch)' \
        -D'RELEASE_NAME=$RELEASE_NAME' \
        -DCLOUD_BUILD \
        -DUSE_ALTITUDE_HOLD \
        -DUSE_DSHOT \
        -DUSE_LED_STRIP \
        -DUSE_OSD_HD \
        -DUSE_PINIO \
        -DUSE_SERIALRX \
        -DUSE_SERIALRX_CRSF \
        -DUSE_TELEMETRY \
        -DUSE_TELEMETRY_CRSF \
        -DUSE_VTX \
    "
