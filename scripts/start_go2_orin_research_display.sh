#!/usr/bin/env bash
set -eo pipefail
RESEARCH_DISPLAY_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$RESEARCH_DISPLAY_ROOT"
RESEARCH_DISPLAY_HOST=${1:-192.168.1.103}
RESEARCH_DISPLAY_INTERFACE=${2:-wlx6c1ff712636d}
source scripts/setup_miti.bash
source install/features/local_setup.bash
export ROS_DOMAIN_ID=74
export ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
RESEARCH_DISPLAY_CLOCK=$(python3 -c 'import json; print(json.load(open("config/go2_orin_research_network.json"))["clock"]["host_minus_orin_s"])')
exec /usr/bin/python3 scripts/supervise_fusion_research.py --audio reuse \
  --remote-research-url "http://$RESEARCH_DISPLAY_HOST:7448" --remote-clock-offset "$RESEARCH_DISPLAY_CLOCK" \
  --remote-display-host "$RESEARCH_DISPLAY_HOST"
