#!/usr/bin/env bash
# Start fusion mapping only after observing a consistent, aligned RGB-D stream.
set -e

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_ROOT/setup.bash"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enxec1ac312a316"/></Interfaces></General></Domain></CycloneDDS>'

FUSION_CHECK_FILE=$(mktemp /tmp/fusion_input_check_XXXXXX.json)
python3 "$PROJECT_ROOT/scripts/check_rgbd_input.py" \
  --seconds 5 --output "$FUSION_CHECK_FILE"
python3 "$PROJECT_ROOT/scripts/verify_fusion_input.py" "$FUSION_CHECK_FILE"

# Defaults match the earlier example; use measured values through CAM_* variables.
FUSION_CAM_X=${CAM_X:-0.0}
FUSION_CAM_Y=${CAM_Y:--0.12}
FUSION_CAM_Z=${CAM_Z:--0.03}
FUSION_CAM_ROLL=${CAM_ROLL:-0.0}
FUSION_CAM_PITCH=${CAM_PITCH:-0.0}
FUSION_CAM_YAW=${CAM_YAW:--1.5708}
FUSION_DB_PATH="$PROJECT_ROOT/maps/livox_realsense_$(date +%Y%m%d_%H%M%S_%N).db"
printf '%s\n' "$FUSION_DB_PATH" > "$PROJECT_ROOT/maps/current_fusion_db.txt"
printf '本轮数据库：%s\n' "$FUSION_DB_PATH"
printf '当前 livox_frame -> camera_link 外参：%s %s %s / %s %s %s（米 / 弧度）\n' \
  "$FUSION_CAM_X" "$FUSION_CAM_Y" "$FUSION_CAM_Z" \
  "$FUSION_CAM_ROLL" "$FUSION_CAM_PITCH" "$FUSION_CAM_YAW"

ros2 launch "$PROJECT_ROOT/launch/livox_realsense_3dslam.launch.py" \
  use_camera:=true use_imu:=true filter_imu:=true \
  deskewing:=true assemble:="${ASSEMBLE:-false}" \
  min_loop_closure_overlap:="${MIN_LOOP_CLOSURE_OVERLAP:-0.2}" \
  fusion_stm_size:="${FUSION_STM_SIZE:-10}" \
  proximity_by_space:="${PROXIMITY_BY_SPACE:-true}" \
  cam_to_lidar_x:="$FUSION_CAM_X" \
  cam_to_lidar_y:="$FUSION_CAM_Y" \
  cam_to_lidar_z:="$FUSION_CAM_Z" \
  cam_to_lidar_roll:="$FUSION_CAM_ROLL" \
  cam_to_lidar_pitch:="$FUSION_CAM_PITCH" \
  cam_to_lidar_yaw:="$FUSION_CAM_YAW" \
  qos:=2 qos_imu:=2 \
  database_path:="$FUSION_DB_PATH"
