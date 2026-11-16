#!/usr/bin/env bash
set -eo pipefail
SEG_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$SEG_PROJECT"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="$SEG_PROJECT/logs/ros"
colcon --log-base logs/segmentation_build build --base-paths src/rtabmap_semantic_msgs \
  --build-base build/segmentation --install-base install/segmentation --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
