#!/usr/bin/env bash
# Separate wireless display; keep the original domain-0/port-3000 service.
set -eo pipefail
GO2_ORIN_DISPLAY_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# -gt 2 ]]; then
  printf '用法：bash scripts/start_go2_orin_display_miti.sh [Orin无线IP [工作站网卡]]\n' >&2
  exit 1
fi
GO2_ORIN_COMPUTE_HOST=${1:-${GO2_ORIN_COMPUTE_HOST:-192.168.1.103}}
GO2_ORIN_DISPLAY_NIC=${2:-${GO2_ORIN_DISPLAY_NIC:-wlx6c1ff712636d}}
source "$GO2_ORIN_DISPLAY_ROOT/setup.bash"
source "$GO2_ORIN_DISPLAY_ROOT/scripts/ros2_wifi_direct_env.sh" "$GO2_ORIN_DISPLAY_NIC" "$GO2_ORIN_COMPUTE_HOST"
export ROS_DOMAIN_ID=74
export BRIDGE_PORT=8766
export MITI_STATE_DIR="$GO2_ORIN_DISPLAY_ROOT/.tools/miti/state/go2-orin-display-74"
export MITI_GEOMETRY=0
export MITI_GRAPH_FEATURES=0
export MITI_NAVIGATION=0
exec bash "$GO2_ORIN_DISPLAY_ROOT/scripts/start_miti.sh" --port 3001
