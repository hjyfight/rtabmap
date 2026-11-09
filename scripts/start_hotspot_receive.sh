#!/usr/bin/env bash
set -eo pipefail
HOTSPOT_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$HOTSPOT_PROJECT/setup.bash"
source "$HOME/ros2_wifi_env.sh"
exec 9>"$HOTSPOT_PROJECT/logs/hotspot_receive.lock"
flock -n 9 || { printf '热点数据接收已启动。\n'; exit 1; }
exec /usr/bin/python3 -u "$HOTSPOT_PROJECT/scripts/hotspot_sensor_transport.py" \
  receive --host "${1:-172.29.38.125}"
