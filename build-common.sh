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
    local hex_pattern="betaflight_*_${target}.hex"
    local hex_file
    
    echo "Looking for hex file: $hex_pattern"
    
    hex_file=$(find ./obj -name "$hex_pattern" -type f | head -1)
    
    if [[ -n "$hex_file" && -f "$hex_file" ]]; then
        local dest_file="$HOME/Downloads/$(basename "$hex_file")"
        cp "$hex_file" "$dest_file"
        echo "✓ Copied: $hex_file → $dest_file"
    else
        echo "✗ Error: Could not find hex file matching $hex_pattern in ./obj"
        return 1
    fi
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