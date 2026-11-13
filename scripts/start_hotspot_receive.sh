#!/usr/bin/env bash
set -eo pipefail
HOTSPOT_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# != 0 && $# != 1 && $# != 3 ]]; then
  printf '用法：bash scripts/start_hotspot_receive.sh [Go2无线IP [本机网卡 显示主机IP]]\n' >&2
  exit 1
fi
source "$HOTSPOT_PROJECT/setup.bash"
if [[ $# == 3 ]]; then
  source "$HOTSPOT_PROJECT/scripts/ros2_wifi_direct_env.sh" "$2" "$3"
else
  source "$HOME/ros2_wifi_env.sh"
fi
exec 9>"$HOTSPOT_PROJECT/logs/hotspot_receive.lock"
flock -n 9 || { printf '热点数据接收已启动。\n'; exit 1; }
exec /usr/bin/python3 -u "$HOTSPOT_PROJECT/scripts/hotspot_sensor_transport.py" \
  receive --host "${1:-172.29.38.125}"
