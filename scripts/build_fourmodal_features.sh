#!/usr/bin/env bash
set -eo pipefail
FEATURE_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FEATURE_PROJECT"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="$FEATURE_PROJECT/logs/ros"
colcon --log-base logs/fourmodal-build build --base-paths src/rtabmap_features_msgs src/rtabmap_features \
  --build-base build/features --install-base install/features \
  --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
