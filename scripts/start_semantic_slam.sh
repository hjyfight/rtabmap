#!/usr/bin/env bash
set -eo pipefail
SEMANTIC_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SEMANTIC_CHECK_ONLY=false
if [[ ${1:-} == --check-input-only ]]; then
  SEMANTIC_CHECK_ONLY=true
  shift
fi
source "$SEMANTIC_PROJECT/setup.bash"
if [[ ! -f "$SEMANTIC_PROJECT/install/semantic/setup.bash" ]]; then
  printf '先运行 bash scripts/build_semantic.sh\n' >&2
  exit 1
fi
source "$SEMANTIC_PROJECT/install/semantic/local_setup.bash"
SEMANTIC_RUNTIME_PYTHON=${SEMANTIC_RUNTIME_PYTHON:-"$SEMANTIC_PROJECT/.venv-semantic/bin/python3"}
SEMANTIC_MODEL_PATH=${SEMANTIC_MODEL_PATH:-"$SEMANTIC_PROJECT/models/yolov8n-seg.pt"}
SEMANTIC_BACKEND=${SEMANTIC_BACKEND:-yolov8}
if [[ ! -x "$SEMANTIC_RUNTIME_PYTHON" || ! -s "$SEMANTIC_MODEL_PATH" ]]; then
  if [[ $SEMANTIC_BACKEND == yoloe_pf ]]; then
    printf '先运行 bash scripts/install_yoloe.sh\n' >&2
  else
    printf '先运行 bash scripts/install_semantic.sh\n' >&2
  fi
  exit 1
fi
export YOLO_CONFIG_DIR=${YOLO_CONFIG_DIR:-"$SEMANTIC_PROJECT/models/.ultralytics"}
export MPLCONFIGDIR=${MPLCONFIGDIR:-"$SEMANTIC_PROJECT/models/.matplotlib"}
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
SEMANTIC_REGISTRATION_PID=''
cleanup_registration() {
  if [[ -n "$SEMANTIC_REGISTRATION_PID" ]]; then
    kill -INT "$SEMANTIC_REGISTRATION_PID" 2>/dev/null || true
    wait "$SEMANTIC_REGISTRATION_PID" 2>/dev/null || true
  fi
  rm -f -- "$SEMANTIC_CHECK"
}
trap cleanup_registration EXIT
SEMANTIC_DEPTH_MODE=$(python3 "$SEMANTIC_PROJECT/scripts/check_registered_depth.py")
if [[ $SEMANTIC_DEPTH_MODE == start ]]; then
  python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp("ros2", ["ros2", "launch"] + sys.argv[1:])' \
    "$SEMANTIC_PROJECT/launch/realsense_depth_registration.launch.py" 9>&- &
  SEMANTIC_REGISTRATION_PID=$!
elif [[ $SEMANTIC_DEPTH_MODE == reuse ]]; then
  printf '复用已有配准深度发布端；不重复启动 depth_registration。\n'
else
  printf '无法确认配准深度发布端状态。\n' >&2
  exit 1
fi
python3 "$SEMANTIC_PROJECT/scripts/check_rgbd_input.py" --seconds "${SEMANTIC_INPUT_CHECK_SECONDS:-5}" --output "$SEMANTIC_CHECK"
python3 "$SEMANTIC_PROJECT/scripts/verify_fusion_input.py" "$SEMANTIC_CHECK" --guarded
if $SEMANTIC_CHECK_ONLY; then
  printf '建图输入预检查通过；模型为 %s，后端为 %s。\n' "$SEMANTIC_MODEL_PATH" "$SEMANTIC_BACKEND"
  exit 0
fi
for SEMANTIC_CAM_VAR in CAM_X CAM_Y CAM_Z CAM_ROLL CAM_PITCH CAM_YAW; do
  if [[ -z "${!SEMANTIC_CAM_VAR}" ]]; then
    printf '外参提示：%s 未赋值，沿用融合启动脚本示例值，请核验标定。\n' "$SEMANTIC_CAM_VAR" >&2
  fi
done
ros2 launch "$SEMANTIC_PROJECT/launch/semantic_slam.launch.py" \
  model_path:="$SEMANTIC_MODEL_PATH" semantic_python:="$SEMANTIC_RUNTIME_PYTHON" \
  semantic_backend:="$SEMANTIC_BACKEND" \
  qos:=2 qos_imu:=2 register_depth:=false \
  cam_to_lidar_x:="${CAM_X:-0.0}" cam_to_lidar_y:="${CAM_Y:--0.12}" \
  cam_to_lidar_z:="${CAM_Z:--0.03}" cam_to_lidar_roll:="${CAM_ROLL:-0.0}" \
  cam_to_lidar_pitch:="${CAM_PITCH:-0.0}" cam_to_lidar_yaw:="${CAM_YAW:--1.5708}" \
  "$@"
