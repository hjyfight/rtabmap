#!/usr/bin/env bash
# Dedicated patrol sound dashboard; independent state and bridge port.
set -eo pipefail
YAMNET_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$YAMNET_PROJECT"
if [[ ${YAMNET_MITI_PORT:-3000} == 3000 ]]; then
  exec bash scripts/start_dashboard_miti.sh "$@"
fi
source scripts/setup_miti.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export MITI_HOST=127.0.0.1
export BRIDGE_PORT=${YAMNET_BRIDGE_PORT:-8768}
export MITI_STATE_DIR="$YAMNET_PROJECT/.tools/miti/state/yamnet-domain-$ROS_DOMAIN_ID"
export BRIDGE_TOPIC_WHITELIST="['^/perception/audio/(events|status)$','^/(semantic|miti)/.*$','^/perception/segmentation/(instances|status|fusion_preview|fusion_status)$','^/camera/.*(image_raw|image_rect|image_rect_raw|camera_info)(/compressed)?$','^/(tf|tf_static|cloud_map|mapGraph|icp_odom|robot_pose|info)$']"
export MITI_BRIDGE_HINT="bash $YAMNET_PROJECT/scripts/start_yamnet_miti.sh"
mkdir -p "$MITI_STATE_DIR" logs/audio/yamnet
exec 9>"logs/audio/yamnet/miti_domain_$ROS_DOMAIN_ID.lock"
flock -n 9 || { printf '本 ROS 域的声音识别网页已经在运行。\n' >&2; exit 1; }
python3 scripts/install_miti_loop_panel.py
cp config/miti_live.html "$MITI_APP/public/rtabmap.html"
cp config/miti_live_layout.json "$MITI_APP/public/rtabmap-layout.json"
cp config/miti_fusion_layout.json "$MITI_APP/public/rtabmap-fusion-layout.json"
cp config/miti_object_fusion_layout.json "$MITI_APP/public/rtabmap-object-fusion-layout.json"
cp config/miti_audio_layout.json "$MITI_APP/public/rtabmap-audio-layout.json"
printf '哨兵巡逻声音识别：http://localhost:%s/rtabmap.html?view=audio\n' "${YAMNET_MITI_PORT:-3004}"
exec "$MITI_APP/node/bin/node" "$MITI_APP/scripts/start-miti.mjs" --bridge --port "${YAMNET_MITI_PORT:-3004}" "$@"
