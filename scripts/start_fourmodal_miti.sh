#!/usr/bin/env bash
# A fresh frontend on a separate origin; reuse the existing bridge and preserve SLAM/layouts.
set -eo pipefail
FOURMODAL_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FOURMODAL_PROJECT"
source scripts/ros2_wired_env.sh
source scripts/setup_miti.bash
python3 scripts/install_miti_loop_panel.py
cp config/miti_live.html "$MITI_APP/public/rtabmap.html"
cp config/miti_live_layout.json "$MITI_APP/public/rtabmap-layout.json"
cp config/miti_fusion_layout.json "$MITI_APP/public/rtabmap-fusion-layout.json"
cp config/miti_object_fusion_layout.json "$MITI_APP/public/rtabmap-object-fusion-layout.json"
export MITI_STATE_DIR="$FOURMODAL_PROJECT/.tools/miti/state/fourmodal-frontend"
export MITI_HOST=127.0.0.1 BRIDGE_PORT=${BRIDGE_PORT:-8765}
printf '融合专用展示：http://localhost:3002/rtabmap.html?view=fusion，复用现有 ROS 桥接。\n'
exec "$MITI_APP/node/bin/node" "$MITI_APP/scripts/start-miti.mjs" --port 3002 "$@"
