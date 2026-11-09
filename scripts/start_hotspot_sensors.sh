#!/usr/bin/env bash
# Installed in the onboard user's home. Camera profiles match the raw workflow.
set -eo pipefail
source "$HOME/yao_space/ws_livox/install/setup.bash"
source "$HOME/ros2_hotspot_sensor_env.sh"
mkdir -p "$HOME/.cache"
exec 9>"$HOME/.cache/hotspot_sensors.lock"
flock -n 9 || { printf '热点传感器已启动。\n'; exit 1; }
if /usr/bin/python3 - <<'PY'
from pathlib import Path
import sys
for item in Path('/proc').glob('[0-9]*/cmdline'):
    try:
        exe = item.read_bytes().split(b'\0')[0]
        if exe.endswith((b'/livox_ros_driver2_node', b'/realsense2_camera_node')):
            sys.exit(0)
    except OSError:
        pass
sys.exit(1)
PY
then
  printf '已有雷达或相机驱动，请在原启动终端 Ctrl+C 后再启动热点压缩模式。\n' >&2
  exit 1
fi
HOTSPOT_HOST=$(ip -4 -o addr show dev wlan0 | awk '{split($4,a,"/"); print a[1]}')
[[ -n $HOTSPOT_HOST ]] || { printf 'wlan0 未连接热点。\n' >&2; exit 1; }
HOTSPOT_CHILDREN=()
launch_hotspot_node() {
  # Bash background jobs inherit ignored SIGINT; reset it before ROS launch.
  exec /usr/bin/python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execv(sys.executable, [sys.executable] + sys.argv[1:])' \
    /opt/ros/foxy/bin/ros2 launch "$@"
}
cleanup_hotspot() {
  trap - EXIT INT TERM
  for HOTSPOT_PID in "${HOTSPOT_CHILDREN[@]}"; do kill -INT "$HOTSPOT_PID" 2>/dev/null || true; done
  for HOTSPOT_PID in "${HOTSPOT_CHILDREN[@]}"; do wait "$HOTSPOT_PID" 2>/dev/null || true; done
}
trap cleanup_hotspot EXIT
trap 'exit 0' INT TERM
printf '2.4 GHz 压缩传输：%s:7447，传感器本地域 42，RGB JPEG=%s（0 为无损）。\n' "$HOTSPOT_HOST" "${HOTSPOT_JPEG_QUALITY:-85}"
launch_hotspot_node livox_ros_driver2 msg_MID360_launch.py 9>&- &
HOTSPOT_CHILDREN+=("$!")
launch_hotspot_node realsense2_camera rs_launch.py \
  config_file:="'$HOME/realsense_slam_sensor_qos.yaml'" \
  rgb_camera.profile:=424x240x6 depth_module.profile:=640x480x6 \
  align_depth.enable:=false enable_color:=true enable_depth:=true enable_sync:=false 9>&- &
HOTSPOT_CHILDREN+=("$!")
/usr/bin/python3 -u "$HOME/hotspot_sensor_transport.py" send --host "$HOTSPOT_HOST" \
  --jpeg-quality "${HOTSPOT_JPEG_QUALITY:-85}" 9>&- &
HOTSPOT_CHILDREN+=("$!")
wait -n "${HOTSPOT_CHILDREN[@]}"
