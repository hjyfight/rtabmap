#!/usr/bin/env bash
set -eo pipefail
SEMANTIC_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
cd "$SEMANTIC_PROJECT"
colcon --log-base "$SEMANTIC_PROJECT/logs/semantic_build" build \
  --base-paths "$SEMANTIC_PROJECT/src/rtabmap_semantic_msgs" \
               "$SEMANTIC_PROJECT/src/rtabmap_semantic" \
               "$SEMANTIC_PROJECT/src/rtabmap_ros/rtabmap_viz" \
  --build-base "$SEMANTIC_PROJECT/build/semantic" \
  --install-base "$SEMANTIC_PROJECT/install/semantic" \
  --packages-select rtabmap_semantic_msgs rtabmap_semantic rtabmap_viz \
  --allow-overriding rtabmap_viz \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DRTABMAP_SEMANTIC_GUI_CHECK=ON \
               -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
