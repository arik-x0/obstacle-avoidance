#!/usr/bin/env bash
# build.sh – Configure and build the obstacle_avoidance project.
#
# Usage:
#   ./scripts/build.sh                   # Release build
#   ./scripts/build.sh --debug           # Debug build with ASan/UBSan
#   ./scripts/build.sh --prefix /opt/mavsdk   # Custom MAVSDK install path
#   ./scripts/build.sh --clean           # Remove build dir first
#   ./scripts/build.sh --jobs 8          # Parallel jobs (default: nproc)

set -e

BUILD_TYPE="Release"
BUILD_DIR="build"
PREFIX_PATH=""
JOBS=$(nproc 2>/dev/null || echo 4)
CLEAN=0

for arg in "$@"; do
    case $arg in
        --debug)        BUILD_TYPE="Debug" ;;
        --clean)        CLEAN=1 ;;
        --prefix=*)     PREFIX_PATH="${arg#*=}" ;;
        --jobs=*)       JOBS="${arg#*=}" ;;
    esac
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "======================================================"
echo "  obstacle_avoidance build"
echo "  Build type : $BUILD_TYPE"
echo "  Build dir  : $BUILD_DIR"
echo "  Jobs       : $JOBS"
if [ -n "$PREFIX_PATH" ]; then
    echo "  Prefix     : $PREFIX_PATH"
fi
echo "======================================================"

cd "$REPO_ROOT"

if [ $CLEAN -eq 1 ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

CMAKE_ARGS="-DCMAKE_BUILD_TYPE=$BUILD_TYPE"
if [ -n "$PREFIX_PATH" ]; then
    CMAKE_ARGS+=" -DCMAKE_PREFIX_PATH=$PREFIX_PATH"
fi

echo "Configuring..."
cmake .. $CMAKE_ARGS

echo "Building with $JOBS parallel jobs..."
cmake --build . --parallel "$JOBS"

echo ""
echo "======================================================"
echo "  Build complete!"
echo "  Binary: $REPO_ROOT/$BUILD_DIR/obstacle_avoidance"
echo ""
echo "  To run:"
echo "    ./$BUILD_DIR/obstacle_avoidance config/params.json"
echo "======================================================"
