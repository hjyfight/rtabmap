#!/usr/bin/env bash
# Source in each workstation terminal used for wired sensors, SLAM, or MITI.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf '请使用 source scripts/ros2_wired_env.sh，让有线环境作用于当前终端。\n' >&2
  exit 1
fi

GO2_WIRED_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -d /sys/class/net/enxec1ac312a316 ]]; then
  printf '未找到工作站有线网卡 enxec1ac312a316；请检查 USB 网卡及 config/wired_cyclonedds.xml。\n' >&2
  return 1
fi
source "$GO2_WIRED_PROJECT/setup.bash" || return 1
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$GO2_WIRED_PROJECT/config/wired_cyclonedds.xml"
printf '已加载工作站有线环境：enxec1ac312a316 -> 192.168.123.18，ROS domain 0。\n'
