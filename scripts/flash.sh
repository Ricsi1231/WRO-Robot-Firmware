#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

PORT="${1:-/dev/ttyACM0}"

# Check for ESP-IDF
if [ -z "$IDF_PATH" ]; then
    echo "Error: IDF_PATH not set. Please source ESP-IDF export.sh first."
    echo "  Example: source ~/esp/esp-idf/export.sh"
    exit 1
fi

echo "=========================================="
echo "WRO Robot Software — Build, Flash & Monitor"
echo "  Port: $PORT"
echo "=========================================="

cd "$PROJECT_ROOT"

echo ""
echo ">>> Building..."
idf.py build

echo ""
echo ">>> Flashing..."
idf.py -p "$PORT" flash

echo ""
echo ">>> Starting monitor (Ctrl+] to exit)..."
idf.py -p "$PORT" monitor
