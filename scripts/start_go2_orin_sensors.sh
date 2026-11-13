#!/usr/bin/env bash
# Separate Go2 -> Orin transport; original wired launch commands stay usable.
set -eo pipefail
source "$HOME/yao_space/ws_livox/install/setup.bash"
source "$HOME/ros2_hotspot_sensor_env.sh"
mkdir -p "$HOME/.cache"
exec 9>"$HOME/.cache/hotspot_sensors.lock"
flock -n 9 || { printf '热点传感器已启动。\n' >&2; exit 1; }
if /usr/bin/python3 - <<'PY'
from pathlib import Path
import sys
for item in Path('/proc').glob('[0-9]*/cmdline'):
    try:
        executable = item.read_bytes().split(b'\0')[0]
        if executable.endswith((b'/livox_ros_driver2_node', b'/realsense2_camera_node')):
            sys.exit(0)
    except OSError:
        pass
sys.exit(1)
PY
then
  printf '已有雷达或相机驱动，请先在其启动终端结束原实例。\n' >&2
  exit 1
fi
GO2_ORIN_SENSOR_IP=$(ip -4 -o addr show dev wlan0 | awk '{split($4,a,"/"); print a[1]}')
[[ -n $GO2_ORIN_SENSOR_IP ]] || { printf 'wlan0 尚未取得 IPv4 地址。\n' >&2; exit 1; }
GO2_ORIN_CHILDREN=()
GO2_ORIN_TRANSPORT_OPTIONS=(--jpeg-quality "${GO2_ORIN_JPEG_QUALITY:-0}")
if [[ ${GO2_ORIN_LIDAR_ONLY:-0} == 1 && ${GO2_ORIN_LIDAR_IMU_ONLY:-0} == 1 ]]; then
  printf 'GO2_ORIN_LIDAR_ONLY 和 GO2_ORIN_LIDAR_IMU_ONLY 只能启用一个。\n' >&2
  exit 1
fi
if [[ ${GO2_ORIN_LIDAR_IMU_ONLY:-0} == 1 ]]; then
  GO2_ORIN_TRANSPORT_OPTIONS+=(--lidar-imu-only --socket-timeout 60)
  printf '发送点云、Livox IMU 和静态 TF，用于原有雷达运动建图方案。\n'
elif [[ ${GO2_ORIN_LIDAR_ONLY:-0} == 1 ]]; then
  GO2_ORIN_TRANSPORT_OPTIONS+=(--lidar-only --socket-timeout 60)
fi
launch_go2_orin_driver() {
  exec /usr/bin/python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execv(sys.executable, [sys.executable] + sys.argv[1:])' \
    /opt/ros/foxy/bin/ros2 launch "$@"
}
cleanup_go2_orin_sensors() {
  trap - EXIT INT TERM
  for GO2_ORIN_PID in "${GO2_ORIN_CHILDREN[@]}"; do kill -INT "$GO2_ORIN_PID" 2>/dev/null || true; done
  for GO2_ORIN_PID in "${GO2_ORIN_CHILDREN[@]}"; do wait "$GO2_ORIN_PID" 2>/dev/null || true; done
}
trap cleanup_go2_orin_sensors EXIT
trap 'exit 0' INT TERM
printf 'Go2 -> Orin：%s:7447，本地域 42，RGB JPEG=%s（0 为无损）。\n' \
  "$GO2_ORIN_SENSOR_IP" "${GO2_ORIN_JPEG_QUALITY:-0}"
launch_go2_orin_driver livox_ros_driver2 msg_MID360_launch.py 9>&- &
GO2_ORIN_CHILDREN+=("$!")
if [[ ${GO2_ORIN_LIDAR_IMU_ONLY:-0} != 1 ]]; then
  launch_go2_orin_driver realsense2_camera rs_launch.py \
    config_file:="'$HOME/realsense_slam_sensor_qos.yaml'" \
    rgb_camera.profile:=424x240x6 depth_module.profile:=640x480x6 \
    align_depth.enable:=false enable_color:=true enable_depth:=true \
    enable_infra1:=true enable_infra2:=true enable_sync:=false 9>&- &
  GO2_ORIN_CHILDREN+=("$!")
fi
/usr/bin/python3 -u "$HOME/go2_orin_sensor_transport.py" send --host "$GO2_ORIN_SENSOR_IP" \
  "${GO2_ORIN_TRANSPORT_OPTIONS[@]}" 9>&- &
GO2_ORIN_CHILDREN+=("$!")
wait -n "${GO2_ORIN_CHILDREN[@]}"
