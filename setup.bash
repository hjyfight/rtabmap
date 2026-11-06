#!/usr/bin/env bash
# Source this file from bash before using the project.
source /opt/ros/humble/setup.bash || return 1
if [[ -f /home/jyfight/ros2_ws/install/local_setup.bash ]]; then
    source /home/jyfight/ros2_ws/install/local_setup.bash || return 1
fi
export RTABMAP_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -d "$RTABMAP_PROJECT_ROOT/.tools/depth_registration/root/opt/ros/humble" ]]; then
    export AMENT_PREFIX_PATH="$RTABMAP_PROJECT_ROOT/.tools/depth_registration/root/opt/ros/humble${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$RTABMAP_PROJECT_ROOT/.tools/depth_registration/root/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export ROS_LOG_DIR="${RTABMAP_PROJECT_ROOT}/logs/ros"
mkdir -p "$ROS_LOG_DIR" "${RTABMAP_PROJECT_ROOT}/maps"
