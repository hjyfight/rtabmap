#!/usr/bin/env bash
set -eo pipefail
MITI_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$MITI_PROJECT/scripts/setup_miti.bash"
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
export BRIDGE_PORT=18765 MITI_HOST=127.0.0.1 BRIDGE_ADDRESS=127.0.0.1
mkdir -p "$MITI_PROJECT/logs/miti"
# Tests use exact ports; do not accidentally verify a service belonging to another run.
python3 - <<'PY'
import socket
for port in (13000, 18765):
    with socket.socket() as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('127.0.0.1', port))
PY
python3 -m unittest discover -s "$MITI_PROJECT/tests" -p test_miti_scene_adapter.py -v
"$MITI_APP/node/bin/node" "$MITI_PROJECT/tests/test_miti_loop_panel.js"
"$MITI_APP/node/bin/node" "$MITI_PROJECT/tests/test_miti_height_clip.js"
bash "$MITI_PROJECT/scripts/start_miti.sh" --port 13000 >"$MITI_PROJECT/logs/miti/test_server.log" 2>&1 &
MITI_TEST_PID=$!
miti_test_cleanup() {
  trap - EXIT INT TERM
  kill -TERM "$MITI_TEST_PID" 2>/dev/null || true
  wait "$MITI_TEST_PID" 2>/dev/null || true
}
trap miti_test_cleanup EXIT
trap 'exit 1' INT TERM
for MITI_ATTEMPT in $(seq 1 30); do
  kill -0 "$MITI_TEST_PID" || { cat "$MITI_PROJECT/logs/miti/test_server.log"; exit 1; }
  if curl --noproxy '*' -fs http://127.0.0.1:13000/api/bridge | python3 -c 'import sys,json; sys.exit(json.load(sys.stdin).get("state") != "running")' 2>/dev/null; then
    break
  fi
  sleep 0.5
done
python3 "$MITI_PROJECT/tests/check_miti_connection.py"
