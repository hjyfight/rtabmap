#!/usr/bin/env bash
set -eo pipefail
EMSANET_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$EMSANET_PROJECT/install/semantic/setup.bash"
export PYTHONPATH="$EMSANET_PROJECT/src/rtabmap_semantic:${PYTHONPATH:-}"
export PYTHONNOUSERSITE=1
export ROS_LOG_DIR="$EMSANET_PROJECT/logs/emsanet/ros_tests"
export ROS_DOMAIN_ID=95 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
export MPLCONFIGDIR="$EMSANET_PROJECT/models/.matplotlib-emsanet"
mkdir -p "$ROS_LOG_DIR"
python3 -m unittest discover -s "$EMSANET_PROJECT/src/rtabmap_semantic/tests" -p 'test_emsanet.py' -v
# Concat (cat + 1x1 conv) encoder fusion needs torch from the EMSANet runtime.
"$EMSANET_PROJECT/.venv-emsanet/bin/python3" -m unittest discover \
  -s "$EMSANET_PROJECT/src/rtabmap_semantic/tests" -p 'test_emsanet_fusion.py' -v
python3 "$EMSANET_PROJECT/src/rtabmap_semantic/tests/ros_pipeline_check.py" --emsanet-fixture
if [[ "${1:-}" == --fixture-only ]]; then
  exit 0
fi
"$EMSANET_PROJECT/.venv-emsanet/bin/python3" "$EMSANET_PROJECT/scripts/predict_emsanet.py" "$@"
"$EMSANET_PROJECT/.venv-emsanet/bin/python3" \
  "$EMSANET_PROJECT/src/rtabmap_semantic/tests/ros_pipeline_check.py" --real-emsanet
