#!/usr/bin/env bash
set -eo pipefail
SEMANTIC_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$SEMANTIC_PROJECT/install/semantic/setup.bash"
export ROS_LOG_DIR="$SEMANTIC_PROJECT/logs/semantic_tests"
mkdir -p "$ROS_LOG_DIR"
export ROS_DOMAIN_ID=${SEMANTIC_TEST_DOMAIN_ID:-93}
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
export PYTHONPATH="$SEMANTIC_PROJECT/src/rtabmap_semantic:${PYTHONPATH:-}"
export YOLO_CONFIG_DIR="$SEMANTIC_PROJECT/models/.ultralytics"
export MPLCONFIGDIR="$SEMANTIC_PROJECT/models/.matplotlib"
python3 -m unittest discover -s "$SEMANTIC_PROJECT/tests" -p 'test_semantic_launch.py' -v
python3 -m unittest discover -s "$SEMANTIC_PROJECT/tests" -p 'test_rgbd_input_guard.py' -v
python3 -m unittest discover -s "$SEMANTIC_PROJECT/src/rtabmap_semantic/tests" -p 'test_*.py' -v
python3 "$SEMANTIC_PROJECT/src/rtabmap_semantic/tests/ros_pipeline_check.py"
if [[ "${1:-}" == --real-yolo || "${SEMANTIC_TEST_REAL_YOLO:-false}" == true ]]; then
  "$SEMANTIC_PROJECT/.venv-semantic/bin/python3" \
    "$SEMANTIC_PROJECT/src/rtabmap_semantic/tests/ros_pipeline_check.py" --real-yolo
fi
