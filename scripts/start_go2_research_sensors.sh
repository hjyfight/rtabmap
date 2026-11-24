#!/usr/bin/env bash
set -eo pipefail
GO2_RESEARCH_ROOT=${GO2_RESEARCH_ROOT:-$HOME/go2_orin_network}
source /opt/ros/foxy/setup.bash
source "$HOME/yao_space/ws_livox/install/setup.bash"
source "$GO2_RESEARCH_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/hotspot_sensor_cyclonedds.xml"
GO2_RESEARCH_IP=$(ip -4 -o addr show dev wlan0 | awk '{split($4,a,"/"); print a[1]}')
[[ -n $GO2_RESEARCH_IP ]] || { printf 'Go2 wlan0 has no IPv4 address.\n' >&2; exit 1; }
exec /usr/bin/python3 "$GO2_RESEARCH_ROOT/scripts/go2_research_sensor_supervisor.py" \
  --host "$GO2_RESEARCH_IP" --root "$GO2_RESEARCH_ROOT"
