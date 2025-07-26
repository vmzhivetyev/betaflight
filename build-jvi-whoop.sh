#!/bin/bash

set -eu
source build-common.sh

TARGET="BETAFPVG473"

DEFINES=(
  "CORE_BUILD"
)

build_target "$TARGET" "${DEFINES[@]}"

copy_hex_to_downloads "$TARGET"
