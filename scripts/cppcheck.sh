#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Running cppcheck..."

# Source directories
SRCS="$PROJECT_ROOT/main $PROJECT_ROOT/components"

# Include directories
INCLUDES=""

# Add ESP-IDF includes if available
if [ -n "$IDF_PATH" ]; then
    INCLUDES="$INCLUDES -I$IDF_PATH/components/esp_common/include"
    INCLUDES="$INCLUDES -I$IDF_PATH/components/freertos/FreeRTOS-Kernel/include"
    INCLUDES="$INCLUDES -I$IDF_PATH/components/esp_hw_support/include"
    INCLUDES="$INCLUDES -I$IDF_PATH/components/log/include"
fi

cppcheck \
    --enable=all \
    --std=c++17 \
    --error-exitcode=1 \
    --suppress=missingInclude \
    --suppress=missingIncludeSystem \
    --suppress=unmatchedSuppression \
    --suppress=unusedFunction \
    --suppress=checkersReport \
    --inline-suppr \
    $INCLUDES \
    $SRCS

echo "cppcheck complete!"
