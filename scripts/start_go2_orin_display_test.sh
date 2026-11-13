#!/usr/bin/env bash
# Independent display smoke test; the original wired commands keep their domain.
set -eo pipefail
GO2_ORIN_DISPLAY_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# -lt 1 || $# -gt 4 || ( $1 != receive && $1 != slam && $1 != slam_imu ) ]]; then
  printf '用法：bash scripts/start_go2_orin_display_test.sh receive|slam|slam_imu [Go2无线IP [显示主机IP [Orin网卡]]]\n' >&2
  exit 1
fi
GO2_ORIN_SENSOR_HOST=${2:-${GO2_ORIN_SENSOR_HOST:-192.168.1.104}}
GO2_ORIN_DISPLAY_HOST=${3:-${GO2_ORIN_DISPLAY_HOST:-192.168.1.100}}
GO2_ORIN_COMPUTE_NIC=${4:-${GO2_ORIN_COMPUTE_NIC:-wlP1p1s0}}
source "$GO2_ORIN_DISPLAY_ROOT/setup.bash"
source "$GO2_ORIN_DISPLAY_ROOT/scripts/ros2_wifi_direct_env.sh" "$GO2_ORIN_COMPUTE_NIC" "$GO2_ORIN_DISPLAY_HOST"
export ROS_DOMAIN_ID=74
mkdir -p "$GO2_ORIN_DISPLAY_ROOT/logs/go2_orin"
printf '独立建图显示测试：ROS domain 74，Go2 %s，显示主机 %s。\n' \
  "$GO2_ORIN_SENSOR_HOST" "$GO2_ORIN_DISPLAY_HOST"
if [[ $1 == receive ]]; then
  exec 9>"$GO2_ORIN_DISPLAY_ROOT/logs/hotspot_receive.lock"
  flock -n 9 || { printf '已有接收实例，请先在原接收终端 Ctrl+C。\n' >&2; exit 1; }
  exec /usr/bin/python3 -u "$GO2_ORIN_DISPLAY_ROOT/scripts/hotspot_sensor_transport.py" \
    receive --host "$GO2_ORIN_SENSOR_HOST" --socket-timeout 60
else
  exec 9>"$GO2_ORIN_DISPLAY_ROOT/logs/go2_orin/display_slam.lock"
  flock -n 9 || { printf '此显示测试的建图进程已启动。\n' >&2; exit 1; }
  printf '等待 /livox/lidar 有效点云（最多 15 秒）……\n'
  # Humble applies --field before --filter. Hide arrays without narrowing the message.
  if timeout --signal=INT --kill-after=2s 15s ros2 topic echo \
      /livox/lidar sensor_msgs/msg/PointCloud2 --no-daemon --once \
      --qos-reliability best_effort --qos-durability volatile --qos-depth 1 \
      --filter 'm.width * m.height > 0 and len(m.data) > 0 and bool(m.header.frame_id) and (m.header.stamp.sec != 0 or m.header.stamp.nanosec != 0)' \
      --no-arr >"$GO2_ORIN_DISPLAY_ROOT/logs/go2_orin/lidar_preflight.log" 2>&1; then
    printf '已收到有效点云。\n'
  else
    GO2_ORIN_PREFLIGHT_STATUS=$?
    cat "$GO2_ORIN_DISPLAY_ROOT/logs/go2_orin/lidar_preflight.log" >&2
    if [[ $GO2_ORIN_PREFLIGHT_STATUS != 124 ]]; then
      printf '点云检查程序执行失败（退出码 %s），建图尚未启动。请检查上方错误；此错误不能判定为雷达无数据。\n' \
        "$GO2_ORIN_PREFLIGHT_STATUS" >&2
      exit "$GO2_ORIN_PREFLIGHT_STATUS"
    fi
    printf '未收到有效雷达点云，建图尚未启动。请先在 Go2 启动传感器发送端，再在 Orin 另开终端执行：\n' >&2
    printf 'bash scripts/start_go2_orin_display_test.sh receive %q %q %q\n' \
      "$GO2_ORIN_SENSOR_HOST" "$GO2_ORIN_DISPLAY_HOST" "$GO2_ORIN_COMPUTE_NIC" >&2
    exit 1
  fi
  if [[ $1 == slam_imu ]]; then
    printf '等待 /livox/imu（最多 15 秒）……\n'
    if timeout --signal=INT --kill-after=2s 15s ros2 topic echo \
        /livox/imu sensor_msgs/msg/Imu --no-daemon --once \
        --qos-reliability best_effort --qos-durability volatile --qos-depth 1 \
        --filter 'bool(m.header.frame_id) and (m.header.stamp.sec != 0 or m.header.stamp.nanosec != 0)' \
        --no-arr >"$GO2_ORIN_DISPLAY_ROOT/logs/go2_orin/imu_preflight.log" 2>&1; then
      printf '已收到 IMU，使用原有 Livox IMU + 去畸变建图方案。\n'
    else
      GO2_ORIN_PREFLIGHT_STATUS=$?
      cat "$GO2_ORIN_DISPLAY_ROOT/logs/go2_orin/imu_preflight.log" >&2
      if [[ $GO2_ORIN_PREFLIGHT_STATUS != 124 ]]; then
        printf 'IMU 检查程序执行失败（退出码 %s），建图尚未启动。\n' "$GO2_ORIN_PREFLIGHT_STATUS" >&2
        exit "$GO2_ORIN_PREFLIGHT_STATUS"
      fi
      printf '未收到 Livox IMU。请在 Go2 使用 GO2_ORIN_LIDAR_IMU_ONLY=1 启动发送端，并保持 Orin receive 运行。\n' >&2
      exit 1
    fi
    GO2_ORIN_IMU_DB="$GO2_ORIN_DISPLAY_ROOT/maps/livox_imu_$(date +%Y%m%d_%H%M%S_%N).db"
    exec ros2 launch "$GO2_ORIN_DISPLAY_ROOT/launch/livox_realsense_3dslam.launch.py" \
      use_camera:=false use_imu:=true filter_imu:=true deskewing:=true assemble:=false \
      qos:=2 qos_imu:=2 rtabmap_viz:=false loop_status_window:=false \
      database_path:="$GO2_ORIN_IMU_DB"
  fi
  printf '启动基础 ICP 显示测试；带 IMU 的运动建图请使用 slam_imu。\n'
  exec ros2 launch "$GO2_ORIN_DISPLAY_ROOT/launch/orin_lidar_display_test.launch.py"
fi
