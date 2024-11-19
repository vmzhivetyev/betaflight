set -eux
# make clean

RELEASE_NAME="$(git show --pretty=format:"%h" --no-patch) at $(git rev-parse --abbrev-ref HEAD) built $(date '+%Y-%m-%d %H:%M:%S')"
echo "$RELEASE_NAME"

make CRAZYBEEF4SX1280 EXTRA_FLAGS="-D'BUILD_KEY=$(git show --pretty=format:"%h" --no-patch)' -D'RELEASE_NAME=$RELEASE_NAME' -DCLOUD_BUILD -DUSE_DSHOT -DUSE_LED_STRIP -DUSE_OSD -DUSE_OSD_SD -DUSE_TELEMETRY -DUSE_TELEMETRY_CRSF -DUSE_VTX"
