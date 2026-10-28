#!/usr/bin/env bash
# Start LiDAR-only RTAB-Map with data published by the Go2.
# Detects /utlidar/cloud (head L1) or /livox/lidar (Mid-360) automatically.
#
# Usage:
#   bash scripts/start_go2_slam.sh [extra launch args]
# Override detection:
#   LIDAR_TOPIC=/livox/lidar LIDAR_FRAME=livox_frame bash scripts/start_go2_slam.sh
set -euo pipefail

project_root="/home/jyfight/Project/rtabmap_slam"
unitree_setup="${HOME}/unitree_ros2/setup.sh"

if [[ ! -f "$unitree_setup" ]]; then
    echo "Missing ${unitree_setup}; install unitree_ros2 first." >&2
    exit 1
fi

# shellcheck disable=SC1090
set +u
source "$unitree_setup"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

rmem_max="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
if [[ "$rmem_max" -lt 52428800 ]]; then
    echo "warning: net.core.rmem_max=${rmem_max}; run: sudo sysctl -w net.core.rmem_max=52428800" >&2
fi

topic="${LIDAR_TOPIC:-}"
frame="${LIDAR_FRAME:-}"

if [[ -z "$topic" ]]; then
    topics="$(timeout 20 ros2 topic list 2>/dev/null || true)"
    if grep -qx '/utlidar/cloud' <<<"$topics"; then
        topic=/utlidar/cloud
        frame="${frame:-utlidar_lidar}"
    elif grep -qx '/livox/lidar' <<<"$topics"; then
        topic=/livox/lidar
        frame="${frame:-livox_frame}"
    else
        echo "No /utlidar/cloud or /livox/lidar visible on DDS." >&2
        echo "Run: bash ${project_root}/scripts/go2_connect_check.sh" >&2
        exit 1
    fi
fi
frame="${frame:-utlidar_lidar}"

echo "Go2 LiDAR: topic=${topic} frame_id=${frame} ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
cd "$project_root"
exec bash scripts/start_livox_slam.sh "lidar_topic:=${topic}" "frame_id:=${frame}" "$@"
