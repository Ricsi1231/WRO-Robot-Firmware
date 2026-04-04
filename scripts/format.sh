#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

CHECK_ONLY="${1:-}"

echo "Running clang-format..."

# Find source files in main and components
FILES=$(find "$PROJECT_ROOT/main" \
             "$PROJECT_ROOT/components" \
             "$PROJECT_ROOT/scripts" \
    -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
    2>/dev/null || true)

if [ -z "$FILES" ]; then
    echo "No source files found."
    exit 0
fi

if [ "$CHECK_ONLY" == "--check" ]; then
    echo "Checking format (dry-run)..."
    echo "$FILES" | xargs clang-format --dry-run --Werror
    echo "Format check passed!"
else
    echo "Formatting files..."
    echo "$FILES" | xargs clang-format -i
    echo "Format complete!"
fi
