#!/usr/bin/env bash
set -eo pipefail
SEMANTIC_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
cd "$SEMANTIC_PROJECT"
SEMANTIC_PATHS=("$SEMANTIC_PROJECT/src/rtabmap_semantic_msgs" "$SEMANTIC_PROJECT/src/rtabmap_semantic")
SEMANTIC_PACKAGES=(rtabmap_semantic_msgs rtabmap_semantic)
SEMANTIC_OPTIONS=()
SEMANTIC_CMAKE_OPTIONS=()
SEMANTIC_OPENCV="$SEMANTIC_PROJECT/.tools/opencv-ros/root/usr/lib/aarch64-linux-gnu/cmake/opencv4"
if [[ -f "$SEMANTIC_OPENCV/OpenCVConfig.cmake" ]]; then
  SEMANTIC_CMAKE_OPTIONS+=("-DOpenCV_DIR=$SEMANTIC_OPENCV")
fi
if [[ ${1:-} == --headless ]]; then
  shift
else
  SEMANTIC_PATHS+=("$SEMANTIC_PROJECT/src/rtabmap_ros/rtabmap_viz")
  SEMANTIC_PACKAGES+=(rtabmap_viz)
  SEMANTIC_OPTIONS+=(--allow-overriding rtabmap_viz)
fi
if (( $# )); then
  printf '用法：bash scripts/build_semantic.sh [--headless]\n' >&2
  exit 2
fi
colcon --log-base "$SEMANTIC_PROJECT/logs/semantic_build" build \
  --base-paths "${SEMANTIC_PATHS[@]}" \
  --build-base "$SEMANTIC_PROJECT/build/semantic" \
  --install-base "$SEMANTIC_PROJECT/install/semantic" \
  --packages-select "${SEMANTIC_PACKAGES[@]}" \
  "${SEMANTIC_OPTIONS[@]}" \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DRTABMAP_SEMANTIC_GUI_CHECK=ON \
               -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
               "${SEMANTIC_CMAKE_OPTIONS[@]}"
