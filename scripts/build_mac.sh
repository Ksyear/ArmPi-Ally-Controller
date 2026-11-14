#!/usr/bin/env bash
# Build the portable core, console tool and unit tests natively (macOS/Linux), then run the unit tests.
set -euo pipefail
cd "$(dirname "$0")/.."
cmake -S . -B build-mac -DCMAKE_BUILD_TYPE="${BUILD_TYPE:-Release}"
cmake --build build-mac -j"$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
ctest --test-dir build-mac --output-on-failure
