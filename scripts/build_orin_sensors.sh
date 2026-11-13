#!/usr/bin/env bash
set -eo pipefail
ORIN_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ $(uname -m) == aarch64 ]] || { printf '此入口仅供 Orin ARM64。\n' >&2; exit 1; }
ORIN_SDK_PREFIX="$ORIN_PROJECT/install/livox_sdk"
cmake -S "$ORIN_PROJECT/.tools/livox-sdk2" -B "$ORIN_PROJECT/build/livox-sdk2" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$ORIN_SDK_PREFIX"
cmake --build "$ORIN_PROJECT/build/livox-sdk2" --parallel "${ORIN_BUILD_JOBS:-4}"
cmake --install "$ORIN_PROJECT/build/livox-sdk2"
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$ORIN_SDK_PREFIX:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$ORIN_SDK_PREFIX/lib:${LD_LIBRARY_PATH:-}"
cd "$ORIN_PROJECT"
colcon --log-base "$ORIN_PROJECT/logs/sensors_build" build \
  --base-paths "$ORIN_PROJECT/.tools/livox_ros_driver2" \
  --build-base "$ORIN_PROJECT/build/sensors" --install-base "$ORIN_PROJECT/install/sensors" \
  --executor sequential --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_INSTALL_RPATH="$ORIN_SDK_PREFIX/lib"
printf 'MID360 ROS 2 驱动已编译；尚未连接或启动传感器。\n'
