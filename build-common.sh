#!/bin/bash

# Generate dynamic build information
generate_build_info() {
    BUILD_KEY="$(git show --pretty=format:"%h" --no-patch)"
    local branch="$(git rev-parse --abbrev-ref HEAD)"
    local timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
    
    # Check for uncommitted changes
    local dirty_suffix=""
    if [[ -n "$(git status --porcelain)" ]]; then
        dirty_suffix=" (dirty)"
    fi
    
    RELEASE_NAME="$BUILD_KEY at $branch built $timestamp$dirty_suffix"
    
    echo "BUILD_KEY: $BUILD_KEY"
    echo "RELEASE_NAME: $RELEASE_NAME"
}

# Build target with defines
build_target() {
    local target="$1"
    shift
    local defines=("$@")

    echo "Building target: $target"

    # Record build start time for verification (export for copy_hex_to_downloads)
    export BUILD_START_TIME
    BUILD_START_TIME=$(date +%s)

    # Build compiler flags
    local flags=""

    # Add user-provided defines
    for define in "${defines[@]}"; do
        if [[ "$define" == *" "* ]] || [[ "$define" == *"$"* ]]; then
            flags+=" -D'$define'"
        else
            flags+=" -D$define"
        fi
    done

    # Add build information (always quoted)
    flags+=" -D'BUILD_KEY=$BUILD_KEY'"
    flags+=" -D'RELEASE_NAME=$RELEASE_NAME'"

    echo "Compiler flags:$flags"

    # Execute make command
    make "$target" EXTRA_FLAGS="$flags"

    echo "✓ Build completed successfully: $target"
    echo "  Release: $RELEASE_NAME"
}

# Find and copy generated hex file to Downloads
copy_hex_to_downloads() {
    local target="$1"

    # Extract version from version.h to construct hex filename pattern
    local fc_ver_major fc_ver_minor fc_ver_patch fc_ver
    fc_ver_major=$(grep -E '#define\s+FC_VERSION_MAJOR\s+[0-9]+' src/main/build/version.h | awk '{print $3}')
    fc_ver_minor=$(grep -E '#define\s+FC_VERSION_MINOR\s+[0-9]+' src/main/build/version.h | awk '{print $3}')
    fc_ver_patch=$(grep -E '#define\s+FC_VERSION_PATCH_LEVEL\s+[0-9]+' src/main/build/version.h | awk '{print $3}')
    fc_ver="${fc_ver_major}.${fc_ver_minor}.${fc_ver_patch}"

    # Construct specific hex filename pattern based on target
    # Pattern: betaflight_<VERSION>_<MCU>_<TARGET>.hex
    # We match on version and target, with MCU in between
    local hex_pattern="betaflight_${fc_ver}_*_${target}.hex"

    echo "Looking for hex file: $hex_pattern"

    # Find matching hex file (should be exactly one)
    local hex_files
    # shellcheck disable=SC2206
    hex_files=(./obj/${hex_pattern})

    # Check if glob matched anything
    if [[ ! -e "${hex_files[0]}" ]]; then
        echo "✗ Error: Could not find hex file matching pattern: $hex_pattern"
        echo "   Expected in: ./obj/"
        return 1
    fi

    # Ensure exactly one match
    if [[ ${#hex_files[@]} -gt 1 ]]; then
        echo "✗ Error: Found multiple hex files matching pattern:"
        printf '   %s\n' "${hex_files[@]}"
        return 1
    fi

    local hex_file="${hex_files[0]}"

    # Verify the hex file was modified after the build started
    local hex_mtime
    if [[ "$(uname)" == "Darwin" ]]; then
        # macOS: use stat -f %m for modification time
        hex_mtime=$(stat -f %m "$hex_file")
    else
        # Linux: use stat -c %Y for modification time
        hex_mtime=$(stat -c %Y "$hex_file")
    fi

    if [[ -n "$BUILD_START_TIME" ]] && [[ "$hex_mtime" -lt "$BUILD_START_TIME" ]]; then
        echo "✗ Error: Hex file exists but was not updated by this build"
        echo "   File: $hex_file"
        echo "   File modified: $(date -r "$hex_mtime" '+%Y-%m-%d %H:%M:%S')"
        echo "   Build started: $(date -r "$BUILD_START_TIME" '+%Y-%m-%d %H:%M:%S')"
        return 1
    fi

    local dest_file
    dest_file="$HOME/Downloads/$(basename "$hex_file")"
    cp "$hex_file" "$dest_file"
    echo "✓ Copied: $hex_file → $dest_file"
}

# Initialize build environment
init_build() {
    generate_build_info
    
    # Ensure we're in a git repository
    if ! git rev-parse --git-dir > /dev/null 2>&1; then
        echo "✗ Error: Not in a git repository"
        return 1
    fi
    
    # Check if obj directory exists (create if needed)
    [[ ! -d ./obj ]] && echo "Note: ./obj directory will be created by make"
    
    echo "✓ Build environment initialized"
}

# Run initialization when script is sourced
init_build