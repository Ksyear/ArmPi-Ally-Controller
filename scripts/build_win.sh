#!/usr/bin/env bash
# Cross-compile the Windows exes (armpi-ally.exe GUI, armpi-ally-cli.exe) with MinGW-w64.
# macOS: brew install mingw-w64   Ubuntu: sudo apt install mingw-w64
set -euo pipefail
cd "$(dirname "$0")/.."
cmake -S . -B build-win -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64-x86_64.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build-win -j"$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
ls -la build-win/*.exe
