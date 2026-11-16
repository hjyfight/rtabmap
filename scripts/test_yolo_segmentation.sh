#!/usr/bin/env bash
set -eo pipefail
SEG_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$SEG_PROJECT"
source /opt/ros/humble/setup.bash
source install/segmentation/local_setup.bash
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
export ROS_LOG_DIR="$SEG_PROJECT/logs/segmentation_validation/ros"
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
.venv-semantic/bin/python3 -m unittest discover -s tests -p test_yolo_segmentation.py -v
.venv-semantic/bin/python3 -m unittest discover -s tests -p test_segmentation_fusion.py -v
.tools/miti/root/usr/lib/miti/node/bin/node --test tests/test_miti_segmentation.js
.venv-semantic/bin/python3 tests/segmentation_ros_check.py
.venv-semantic/bin/python3 tests/segmentation_fusion_ros_check.py
