#!/usr/bin/env bash
# 自动订阅 Livox 点云，再打开 RViz（Fixed Frame = livox_frame）。
#
# 用法：
#   bash scripts/livox_rviz.sh
#
# 可选环境变量：
#   LIDAR_TOPIC=/livox/lidar
#   LIDAR_FRAME=livox_frame
#   RVIZ_CONFIG=/path/to/xxx.rviz
#   START_DRIVER=1    # 话题不存在时尝试在本机启动 MID-360 驱动
#
# 说明：脚本先常驻一个订阅者（ros2 topic echo --no-arr），
# 等到第一帧后再启动 RViz；退出 RViz 时自动结束后台订阅。
set -euo pipefail

TOPIC="${LIDAR_TOPIC:-/livox/lidar}"
FRAME="${LIDAR_FRAME:-livox_frame}"
ROS_WS_SETUP="${ROS_WS_SETUP:-/home/jyfight/ros2_ws/install/setup.bash}"
UNITREE_SETUP="${UNITREE_SETUP:-$HOME/unitree_ros2/setup.sh}"
RVIZ_CONFIG="${RVIZ_CONFIG:-/home/jyfight/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/display_point_cloud_ROS2.rviz}"
SRC_RVIZ_CONFIG="/home/jyfight/ros2_ws/src/livox_ros_driver2/config/display_point_cloud_ROS2.rviz"
START_DRIVER="${START_DRIVER:-0}"

if [[ ! -f "$RVIZ_CONFIG" && -f "$SRC_RVIZ_CONFIG" ]]; then
    RVIZ_CONFIG="$SRC_RVIZ_CONFIG"
fi

# 与机器人使用同一套 DDS：CycloneDDS + eno1（由 unitree setup.sh 设置）。
# ROS 的 setup.bash 在 set -u 下会引用未定义变量，source 时临时关闭 -u。
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$ROS_WS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$ROS_WS_SETUP"
fi
if [[ -f "$UNITREE_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$UNITREE_SETUP"
fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "topic=${TOPIC}  frame=${FRAME}"

rmem_max="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
if [[ "$rmem_max" -lt 52428800 ]]; then
    echo "提示: net.core.rmem_max=${rmem_max}，大点云建议先执行：" >&2
    echo "  sudo sysctl -w net.core.rmem_max=52428800" >&2
fi

pids=()
tmp_config=""
cleanup() {
    for pid in "${pids[@]}"; do
        kill "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    if [[ -n "$tmp_config" ]]; then
        rm -f "$tmp_config"
    fi
    return 0
}
trap cleanup EXIT INT TERM

start_subscriber() {
    ros2 topic echo "$TOPIC" --no-arr --qos-reliability best_effort \
        >/dev/null 2>&1 &
    pids+=("$!")
}

have_data() {
    timeout 3 ros2 topic echo "$TOPIC" --no-arr --once \
        --qos-reliability best_effort >/dev/null 2>&1
}

echo "== 订阅 ${TOPIC}（后台保活）=="
start_subscriber

if ! timeout 15 ros2 topic echo "$TOPIC" --no-arr --once \
        --qos-reliability best_effort >/dev/null 2>&1; then
    if [[ "$START_DRIVER" == "1" ]]; then
        echo "未发现 ${TOPIC}，尝试启动本机 Livox 驱动..." >&2
        echo "注意：本机 MID360_config.json 的 host_net_info 必须填本机 IP，" >&2
        echo "      lidar_configs[0].ip 必须是雷达 IP。" >&2
        ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
            -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 \
            -p publish_freq:=10.0 -p output_data_type:=0 \
            -p frame_id:="$FRAME" \
            -p user_config_path:="/home/jyfight/ros2_ws/src/livox_ros_driver2/config/MID360_config.json" \
            >/dev/null 2>&1 &
        pids+=("$!")
        for _ in $(seq 1 15); do
            have_data && break
            sleep 1
        done
    fi
fi

if ! have_data; then
    echo "没有收到 ${TOPIC} 的数据。" >&2
    echo "请依次检查：" >&2
    echo "  1) 已切到机器人网络: sudo nmcli connection up go2" >&2
    echo "  2) ping 192.168.123.18 是否通" >&2
    echo "  3) 另一终端: source ~/unitree_ros2/setup.sh; export ROS_DOMAIN_ID=0; ros2 topic list" >&2
    echo "  4) 雷达驱动是否在机器人/本机上运行（ros2 node list 应有 livox_lidar_publisher）" >&2
    exit 2
fi

# 如果话题/坐标系不是默认值，临时生成一份 RViz 配置。
if [[ "$TOPIC" != "/livox/lidar" || "$FRAME" != "livox_frame" ]]; then
    tmp_config="$(mktemp /tmp/livox_rviz_XXXXXX.rviz)"
    sed -e "s|Topic: /livox/lidar|Topic: ${TOPIC}|" \
        -e "s|Fixed Frame: livox_frame|Fixed Frame: ${FRAME}|" \
        "$RVIZ_CONFIG" > "$tmp_config"
    RVIZ_CONFIG="$tmp_config"
fi

echo "== 数据正常，启动 RViz =="
echo "配置文件: $RVIZ_CONFIG"
rviz2 -d "$RVIZ_CONFIG" "$@"
