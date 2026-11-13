#!/usr/bin/env bash
set -eo pipefail
MITI_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --help ]]; then
  printf '用法：bash scripts/start_miti.sh [--port 3000]\n'
  printf '变量：ROS_DOMAIN_ID、MITI_INTERFACE、CYCLONEDDS_URI、BRIDGE_PORT、MITI_ADAPTER=0、MITI_GEOMETRY=0、MITI_GRID=0、MITI_GRAPH_FEATURES=0、MITI_NAVIGATION=0\n'
  printf '只启动展示服务；传感器和 SLAM 使用项目原启动入口。Ctrl+C 关闭展示。\n'
  exit 0
fi
source "$MITI_PROJECT/scripts/setup_miti.bash"
if [[ ! -x "$MITI_APP/node/bin/node" || ! -x "$MITI_ROS_PREFIX/lib/foxglove_bridge/foxglove_bridge" ]]; then
  printf '请先运行 bash scripts/install_miti.sh\n' >&2
  exit 1
fi
python3 "$MITI_PROJECT/scripts/install_miti_loop_panel.py"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
# Advertise this project's display inputs. Robot control/custom telemetry packages are
# not installed in this workspace and otherwise flood the bridge log with schema errors.
export BRIDGE_TOPIC_WHITELIST="${BRIDGE_TOPIC_WHITELIST:-['^/(tf|tf_static|clock|rosout|cloud_map|map|grid_map|odom|icp_odom|robot_pose|mapGraph|mapData|info)$','^/livox/(lidar|imu)$','^/camera/.*(image_raw|image_rect|image_rect_raw|camera_info)(/compressed)?$','^/arc_camera/(image_raw|camera_info)(/compressed)?$','^/(semantic|miti)/.*$']}"
# Match the project's sensor NIC when it is connected. Respect an explicit DDS configuration.
if [[ -z ${CYCLONEDDS_URI:-} && $RMW_IMPLEMENTATION == rmw_cyclonedds_cpp ]]; then
  MITI_INTERFACE=${MITI_INTERFACE:-}
  if [[ -z $MITI_INTERFACE && $(cat /sys/class/net/enxec1ac312a316/operstate 2>/dev/null || true) == up ]]; then
    MITI_INTERFACE=enxec1ac312a316
  fi
  if [[ $ROS_LOCALHOST_ONLY == 1 ]]; then
    # Humble adds lo itself in localhost-only mode; selecting lo again is an error.
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
    MITI_INTERFACE=lo
  elif [[ -n $MITI_INTERFACE ]]; then
    if [[ ! $MITI_INTERFACE =~ ^[a-zA-Z0-9_.:-]+$ || ! -d /sys/class/net/$MITI_INTERFACE ]]; then
      printf '无效网卡：%s\n' "$MITI_INTERFACE" >&2
      exit 1
    fi
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$MITI_INTERFACE\"/></Interfaces></General></Domain></CycloneDDS>"
  else
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine="true"/></Interfaces></General></Domain></CycloneDDS>'
  fi
fi
export MITI_STATE_DIR=${MITI_STATE_DIR:-"$MITI_PROJECT/.tools/miti/state/domain-$ROS_DOMAIN_ID"}
export MITI_BRIDGE_HINT="bash $MITI_PROJECT/scripts/start_miti.sh"
mkdir -p "$MITI_STATE_DIR" "$MITI_PROJECT/logs/miti"
exec 9>"$MITI_PROJECT/logs/miti/domain-$ROS_DOMAIN_ID.lock"
flock -n 9 || { printf '本 ROS 域已有项目展示服务。\n' >&2; exit 1; }
cp "$MITI_PROJECT/config/miti_live.html" "$MITI_APP/public/rtabmap.html"
cp "$MITI_PROJECT/config/miti_live_layout.json" "$MITI_APP/public/rtabmap-layout.json"
printf 'ROS_DOMAIN_ID=%s，DDS=%s，网卡=%s\n' "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION" "${MITI_INTERFACE:-自动/现有 DDS 配置}"
printf '在下方 MITI 地址后加 /rtabmap.html，进入项目展示布局。\n'
# The adapter and bridge must stay in the same domain. Change ROS settings in the shell,
# then restart this script; do not migrate only the bridge through MITI's network settings.
if [[ -f "$MITI_STATE_DIR/bridge.json" ]]; then
  python3 - "$MITI_STATE_DIR/bridge.json" "$ROS_DOMAIN_ID" <<'PY'
import json, sys
saved = json.load(open(sys.argv[1]))
if saved.get('domainId') != int(sys.argv[2]):
    sys.exit('MITI 保存的 ROS domain 与启动环境不一致；请设置 MITI_STATE_DIR 为新目录，或修正 bridge.json 后重启。')
PY
fi
MITI_CHILDREN=()
miti_cleanup() {
  trap - EXIT INT TERM
  for MITI_PID in "${MITI_CHILDREN[@]}"; do kill -TERM "$MITI_PID" 2>/dev/null || true; done
  for MITI_PID in "${MITI_CHILDREN[@]}"; do wait "$MITI_PID" 2>/dev/null || true; done
}
trap miti_cleanup EXIT
trap 'exit 0' INT TERM
if [[ ${MITI_ADAPTER:-1} != 0 ]]; then
  python3 -u "$MITI_PROJECT/scripts/miti_scene_adapter.py" &
  MITI_CHILDREN+=("$!")
fi
if [[ ! -f "$MITI_PROJECT/build/dynamic_cloud/libdynamic_cloud.so" ||
      "$MITI_PROJECT/scripts/dynamic_cloud_filter.cpp" -nt "$MITI_PROJECT/build/dynamic_cloud/libdynamic_cloud.so" ]]; then
  bash "$MITI_PROJECT/scripts/build_dynamic_cloud.sh"
fi
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -u "$MITI_PROJECT/scripts/dynamic_cloud_node.py" \
  --ros-args --params-file "$MITI_PROJECT/config/dynamic_cloud.yaml" &
MITI_CHILDREN+=("$!")
if [[ ${MITI_GEOMETRY:-1} != 0 ]]; then
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -u "$MITI_PROJECT/scripts/map_geometry_node.py" \
    --ros-args --params-file "$MITI_PROJECT/config/map_geometry.yaml" &
  MITI_CHILDREN+=("$!")
fi
if [[ ${MITI_GRID:-1} != 0 ]]; then
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -u "$MITI_PROJECT/scripts/grid_projection_node.py" \
    --ros-args --params-file "$MITI_PROJECT/config/grid_projection.yaml" &
  MITI_CHILDREN+=("$!")
fi
if [[ ${MITI_GRAPH_FEATURES:-1} != 0 ]]; then
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -u "$MITI_PROJECT/scripts/graph_feature_map_node.py" \
    --ros-args --params-file "$MITI_PROJECT/config/graph_feature_map.yaml" &
  MITI_CHILDREN+=("$!")
fi
if [[ ${MITI_NAVIGATION:-1} != 0 ]]; then
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -u "$MITI_PROJECT/scripts/navigation_topology_node.py" \
    --ros-args --params-file "$MITI_PROJECT/config/navigation_topology.yaml" &
  MITI_CHILDREN+=("$!")
fi
"$MITI_APP/node/bin/node" "$MITI_APP/scripts/start-miti.mjs" --bridge "$@" &
MITI_CHILDREN+=("$!")
wait -n "${MITI_CHILDREN[@]}"
