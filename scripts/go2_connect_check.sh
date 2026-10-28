#!/usr/bin/env bash
# Check the Go2 Ethernet link, DDS visibility, and LiDAR topics from the PC.
# Usage: bash scripts/go2_connect_check.sh [interface]   (default: eno1)
set -u

nic="${1:-${NIC:-eno1}}"
unitree_setup="${HOME}/unitree_ros2/setup.sh"

echo "== interface: ${nic} =="
ip -br -4 addr show "$nic" 2>/dev/null || echo "interface ${nic} not found (check: ip -br link)"
printf 'link state: '
cat "/sys/class/net/${nic}/operstate" 2>/dev/null || echo unknown

echo
echo "== ping Go2 hosts =="
for ip in 192.168.123.18 192.168.123.20 192.168.123.161; do
    if ping -c1 -W1 "$ip" >/dev/null 2>&1; then
        echo "  ${ip}  OK"
    else
        echo "  ${ip}  no reply"
    fi
done

if [[ ! -f "$unitree_setup" ]]; then
    echo
    echo "Missing ${unitree_setup}; install unitree_ros2 first." >&2
    exit 1
fi

# shellcheck disable=SC1090
set +u
source "$unitree_setup"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo
echo "== DDS environment =="
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"

rmem_max="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
if [[ "$rmem_max" -lt 52428800 ]]; then
    echo "  warning: net.core.rmem_max=${rmem_max}; run: sudo sysctl -w net.core.rmem_max=52428800"
fi

ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo
echo "== LiDAR / robot topics =="
topics="$(timeout 20 ros2 topic list 2>/dev/null || true)"
if [[ -z "$topics" ]]; then
    echo "  ros2 topic list returned nothing (DDS discovery failed)"
    exit 2
fi
printf '%s\n' "$topics" | grep -E '^/(utlidar|livox|camera|sport|odom|tf|lowstate)' || echo "  no matching topics"

for topic in /utlidar/cloud /livox/lidar /utlidar/imu; do
    if grep -qx "$topic" <<<"$topics"; then
        echo
        echo "== ${topic} (2 s sample) =="
        timeout 2 ros2 topic hz "$topic" 2>&1 | tail -n 3 || true
        timeout 5 ros2 topic echo --no-arr --once "$topic" 2>/dev/null \
            | grep -E 'frame_id|width:|height:' | head -n 3 || true
    fi
done
