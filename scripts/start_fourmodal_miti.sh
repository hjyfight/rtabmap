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
export MITI_HOST=127.0.0.1 BRIDGE_PORT=${BRIDGE_PORT:-8768}
export BRIDGE_TOPIC_WHITELIST="['^/perception/audio/(events|status)$','^/perception/fusion/(targets|status|timing)$','^/perception/segmentation/.*$','^/(semantic|miti)/.*$','^/camera/.*(image_raw|image_rect|image_rect_raw|camera_info)(/compressed)?$','^/livox/(lidar|imu)$','^/(tf|tf_static|cloud_map|map|mapGraph|icp_odom|robot_pose|info)$']"
printf '融合专用展示：http://localhost:3002/，包含地图与 YAMNet 声音；使用桥接端口 %s。\n' "$BRIDGE_PORT"
exec "$MITI_APP/node/bin/node" "$MITI_APP/scripts/start-miti.mjs" --bridge --port 3002 "$@"
