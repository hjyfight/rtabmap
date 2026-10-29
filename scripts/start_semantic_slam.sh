#!/usr/bin/env bash
set -eo pipefail
SEMANTIC_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$SEMANTIC_PROJECT/setup.bash"
if [[ ! -f "$SEMANTIC_PROJECT/install/semantic/setup.bash" ]]; then
  printf '先运行 bash scripts/build_semantic.sh\n' >&2
  exit 1
fi
source "$SEMANTIC_PROJECT/install/semantic/setup.bash"
if [[ ! -x "$SEMANTIC_PROJECT/.venv-semantic/bin/python3" || ! -s "$SEMANTIC_PROJECT/models/yolov8n-seg.pt" ]]; then
  printf '先运行 bash scripts/install_semantic.sh\n' >&2
  exit 1
fi
export YOLO_CONFIG_DIR="$SEMANTIC_PROJECT/models/.ultralytics"
export MPLCONFIGDIR="$SEMANTIC_PROJECT/models/.matplotlib"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
exec 9>"$SEMANTIC_PROJECT/logs/semantic_slam_domain_${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then
  printf '本 ROS 域已有语义 SLAM 启动进程，请先在原终端 Ctrl+C 并等待退出。\n' >&2
  exit 1
fi
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-'<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enxec1ac312a316"/></Interfaces></General></Domain></CycloneDDS>'}
SEMANTIC_CHECK=$(mktemp /tmp/semantic_rgbd_check_XXXXXX.json)
python3 "$SEMANTIC_PROJECT/scripts/check_rgbd_input.py" --seconds 5 --output "$SEMANTIC_CHECK"
python3 "$SEMANTIC_PROJECT/scripts/verify_fusion_input.py" "$SEMANTIC_CHECK" --guarded
for SEMANTIC_CAM_VAR in CAM_X CAM_Y CAM_Z CAM_ROLL CAM_PITCH CAM_YAW; do
  if [[ -z "${!SEMANTIC_CAM_VAR}" ]]; then
    printf '外参提示：%s 未赋值，沿用融合启动脚本示例值，请核验标定。\n' "$SEMANTIC_CAM_VAR" >&2
  fi
done
exec ros2 launch "$SEMANTIC_PROJECT/launch/semantic_slam.launch.py" \
  qos:=2 qos_imu:=2 \
  cam_to_lidar_x:="${CAM_X:-0.0}" cam_to_lidar_y:="${CAM_Y:--0.12}" \
  cam_to_lidar_z:="${CAM_Z:--0.03}" cam_to_lidar_roll:="${CAM_ROLL:-0.0}" \
  cam_to_lidar_pitch:="${CAM_PITCH:-0.0}" cam_to_lidar_yaw:="${CAM_YAW:--1.5708}" \
  "$@"
