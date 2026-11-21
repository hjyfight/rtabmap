#!/usr/bin/env bash
# Independent 3001 web / frozen inference / read-only ROS bridge. No SLAM restart.
set -eo pipefail
FUSION_RESEARCH_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FUSION_RESEARCH_ROOT"
source scripts/ros2_wired_env.sh
source scripts/setup_miti.bash
source install/features/local_setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOG_DIR="$FUSION_RESEARCH_ROOT/logs/research/ros"
mkdir -p "$ROS_LOG_DIR"
# A disconnected sensor cable must leave the web/status available. This fallback
# applies only to the new research processes, never to the 3000 environment.
if ! ip -4 -o addr show dev enxec1ac312a316 | rg -q 'inet '; then
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
  printf '传感器有线连接尚未就绪：页面先启动，恢复网络后重新启动本3001服务。\n'
fi
exec /usr/bin/python3 scripts/supervise_fusion_research.py "$@"
