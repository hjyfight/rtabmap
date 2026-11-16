#!/usr/bin/env bash
set -eo pipefail
SEG_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$SEG_PROJECT"
source /opt/ros/humble/setup.bash
[[ -f install/segmentation/local_setup.bash ]] || {
  printf 'Run bash scripts/build_yolo_segmentation.sh first\n' >&2; exit 1;
}
source install/segmentation/local_setup.bash
mkdir -p logs/segmentation models/.ultralytics models/.matplotlib
exec 9>"logs/segmentation/domain-${ROS_DOMAIN_ID:-0}.lock"
flock -n 9 || { printf 'Segmentation is already running in this ROS domain\n' >&2; exit 1; }
export ROS_LOG_DIR="$SEG_PROJECT/logs/segmentation/ros"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
export YOLO_CONFIG_DIR="$SEG_PROJECT/models/.ultralytics" MPLCONFIGDIR="$SEG_PROJECT/models/.matplotlib"
exec "${SEGMENTATION_PYTHON:-.venv-semantic/bin/python3}" scripts/run_yolo_segmentation.py \
  --ros-args --params-file config/yolo_segmentation.yaml "$@"
