#!/bin/bash
# Build script for Corne TPS43 split keyboard
# Run from ~/personal/corne-zmk directory
# This matches the GitHub Actions build environment

WORKSPACE_DIR="$(pwd)"
CONFIG_DIR="${WORKSPACE_DIR}/config"

echo "Building Corne TPS43 keyboard firmware..."
echo "==========================================="
echo "Workspace: ${WORKSPACE_DIR}"
echo "Config: ${CONFIG_DIR}"

# Build left half (with USB logging snippet)
echo ""
echo "Building left half..."
west build -p -s zmk/app -d build/left -b nice_nano_v2 -S zmk-usb-logging -- -DZMK_CONFIG="${CONFIG_DIR}" -DSHIELD=corne_tps43_left

if [ $? -eq 0 ]; then
    echo "✓ Left half built successfully"
else
    echo "✗ Left half build failed"
    exit 1
fi

# Build right half (with USB logging snippet)
echo ""
echo "Building right half..."
west build -p -s zmk/app -d build/right -b nice_nano_v2 -S zmk-usb-logging -- -DZMK_CONFIG="${CONFIG_DIR}" -DSHIELD=corne_tps43_right

if [ $? -eq 0 ]; then
    echo "✓ Right half built successfully"
else
    echo "✗ Right half build failed"
    exit 1
fi

echo ""
echo "Build complete!"
echo "==========================================="
echo "Left firmware:  build/left/zephyr/zmk.uf2"
echo "Right firmware: build/right/zephyr/zmk.uf2"
echo ""
echo "To flash: Double-tap reset button on each nice!nano,"
echo "then copy the appropriate .uf2 file to the mounted drive."
