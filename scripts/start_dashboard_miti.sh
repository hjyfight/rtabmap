#!/usr/bin/env bash
# Unified domain-0 console. Preserve an explicit DDS configuration, or use live wired sensors.
set -eo pipefail
DASHBOARD_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
if [[ -z ${CYCLONEDDS_URI:-} ]]; then
  if [[ -d /sys/class/net/enxec1ac312a316 &&
        $(cat /sys/class/net/enxec1ac312a316/operstate) == up &&
        -n $(ip -4 -o addr show dev enxec1ac312a316) ]]; then
    export CYCLONEDDS_URI="file://$DASHBOARD_PROJECT/config/wired_cyclonedds.xml"
  else
    export CYCLONEDDS_URI="file://$DASHBOARD_PROJECT/config/hotspot_sensor_cyclonedds.xml"
  fi
fi
export BRIDGE_PORT=8765
export MITI_STATE_DIR="$DASHBOARD_PROJECT/.tools/miti/state/domain-0"
unset MITI_INTERFACE
exec bash "$DASHBOARD_PROJECT/scripts/start_miti.sh" --port 3000 "$@"
