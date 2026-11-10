#!/usr/bin/env bash
set -Ee -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$SCRIPT_DIR"

source /opt/bxi/bxi_ros2_pkg/setup.bash
BUILD_ROOT="$HOME/.cache/bxi-build/bxi_rl_controller_ros2_example"
mkdir -p "$BUILD_ROOT"

colcon --log-base "$BUILD_ROOT/log" build \
    --merge-install \
    --build-base "$BUILD_ROOT/build" \
    --install-base "$SCRIPT_DIR/install" \
    --cmake-force-configure \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On \
    -Wall -Wextra -Wpedantic
