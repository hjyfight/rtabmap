#!/usr/bin/env bash
set -eo pipefail
YOLOE_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$YOLOE_PROJECT/install/semantic/setup.bash"
export ROS_LOG_DIR="$YOLOE_PROJECT/logs/yoloe_tests"
mkdir -p "$ROS_LOG_DIR"
export ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
export PYTHONPATH="$YOLOE_PROJECT/src/rtabmap_semantic:${PYTHONPATH:-}"
export PYTHONNOUSERSITE=1
export YOLO_CONFIG_DIR="$YOLOE_PROJECT/models/.ultralytics-yoloe"
export MPLCONFIGDIR="$YOLOE_PROJECT/models/.matplotlib-yoloe"
"$YOLOE_PROJECT/.venv-yoloe/bin/python3" -m unittest discover \
  -s "$YOLOE_PROJECT/src/rtabmap_semantic/tests" -p 'test_*.py' -v
"$YOLOE_PROJECT/.venv-yoloe/bin/python3" \
  "$YOLOE_PROJECT/src/rtabmap_semantic/tests/ros_pipeline_check.py" --real-yoloe
