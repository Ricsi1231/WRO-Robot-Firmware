#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Check for ESP-IDF
if [ -z "$IDF_PATH" ]; then
    echo "Error: IDF_PATH not set. Please source ESP-IDF export.sh first."
    echo "  Example: source ~/esp/esp-idf/export.sh"
    exit 1
fi

BUILD_TYPE="${1:-}"

echo "=========================================="
echo "Building WRO Robot Software"
echo "=========================================="

cd "$PROJECT_ROOT"

# Build
if [ "$BUILD_TYPE" == "release" ]; then
    idf.py build -DCMAKE_BUILD_TYPE=Release
else
    idf.py build
fi

echo ""
echo "Build complete!"
echo "  Binary: $PROJECT_ROOT/build/WRO_Robot_Software.bin"
