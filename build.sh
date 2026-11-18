#!/usr/bin/env bash
set -Ee -o pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$SCRIPT_DIR"
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/local_setup.bash
# XMZ uses the original ONNX on CPU; no RKNN conversion is required.
colcon --log-base "$HOME/.cache/bxi-build/arm05-example/log" build \
    --build-base "$HOME/.cache/bxi-build/arm05-example/build" \
    --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
