#!/usr/bin/env bash
set -eo pipefail
FOURMODAL_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FOURMODAL_PROJECT"
source scripts/setup_miti.bash
source install/features/local_setup.bash
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
export BRIDGE_PORT=18766 MITI_HOST=127.0.0.1 BRIDGE_ADDRESS=127.0.0.1
export MITI_ADAPTER=0 MITI_GEOMETRY=0 MITI_GRID=0 MITI_GRAPH_FEATURES=0 MITI_NAVIGATION=0
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 PYTHONNOUSERSITE=1
python3 - <<'PY'
import socket
for port in (13001, 18766):
    with socket.socket() as stream:
        stream.bind(('127.0.0.1', port))
PY
bash scripts/start_miti.sh --port 13001 > logs/miti/fourmodal_test_server.log 2>&1 &
FOURMODAL_MITI_PID=$!
fourmodal_cleanup() {
  trap - EXIT INT TERM
  kill -TERM "$FOURMODAL_MITI_PID" 2>/dev/null || true
  wait "$FOURMODAL_MITI_PID" 2>/dev/null || true
}
trap fourmodal_cleanup EXIT
trap 'exit 1' INT TERM
for FOURMODAL_ATTEMPT in $(seq 1 40); do
  kill -0 "$FOURMODAL_MITI_PID" || { cat logs/miti/fourmodal_test_server.log; exit 1; }
  if curl --noproxy '*' -fs http://127.0.0.1:13001/api/bridge | \
      python3 -c 'import sys,json; sys.exit(json.load(sys.stdin).get("state") != "running")' 2>/dev/null; then
    break
  fi
  sleep 0.5
done
.venv-fourmodal/bin/python3 scripts/test_fourmodal_ros.py --bridge-port 18766 --http-port 13001 \
  --output logs/fourmodal_validation/miti_real_model.json "$@"
