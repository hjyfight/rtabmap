#!/usr/bin/env bash
# Dedicated target-result display with its own bridge and frontend state.
set -eo pipefail
OBJECT_MITI_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$OBJECT_MITI_ROOT"
source scripts/setup_miti.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export MITI_HOST=127.0.0.1
export BRIDGE_PORT=${OBJECT_MITI_BRIDGE_PORT:-8767}
export MITI_STATE_DIR="$OBJECT_MITI_ROOT/.tools/miti/state/object-multimodal-domain-$ROS_DOMAIN_ID"
export BRIDGE_TOPIC_WHITELIST="['^/perception/fusion/(targets|status|timing)$','^/perception/audio/(events|status)$','^/miti/.*$','^/camera/.*(image_raw|image_rect|image_rect_raw|camera_info)(/compressed)?$','^/livox/lidar$','^/(tf|tf_static)$']"
export MITI_BRIDGE_HINT="bash $OBJECT_MITI_ROOT/scripts/start_object_multimodal_miti.sh"
mkdir -p "$MITI_STATE_DIR" logs/miti
exec 9>"logs/miti/object-multimodal-domain-$ROS_DOMAIN_ID.lock"
flock -n 9 || { printf '本 ROS 域的目标融合展示已在运行。\n' >&2; exit 1; }
if [[ -f "$MITI_STATE_DIR/bridge.json" ]]; then
  python3 - "$MITI_STATE_DIR/bridge.json" "$ROS_DOMAIN_ID" <<'PY'
import json, sys
if json.load(open(sys.argv[1])).get('domainId') != int(sys.argv[2]):
    sys.exit('目标展示保存的 ROS 域与当前环境不一致。')
PY
fi
python3 scripts/install_miti_loop_panel.py
cp config/miti_live.html "$MITI_APP/public/rtabmap.html"
cp config/miti_live_layout.json "$MITI_APP/public/rtabmap-layout.json"
cp config/miti_fusion_layout.json "$MITI_APP/public/rtabmap-fusion-layout.json"
cp config/miti_object_fusion_layout.json "$MITI_APP/public/rtabmap-object-fusion-layout.json"
printf '主机目标融合：http://localhost:%s/rtabmap.html?view=target\n' "${OBJECT_MITI_PORT:-3003}"
exec "$MITI_APP/node/bin/node" "$MITI_APP/scripts/start-miti.mjs" --bridge --port "${OBJECT_MITI_PORT:-3003}" "$@"
