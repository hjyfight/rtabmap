#!/usr/bin/env bash
set -eo pipefail
YOLOE_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export SEMANTIC_RUNTIME_PYTHON="$YOLOE_PROJECT/.venv-yoloe/bin/python3"
export SEMANTIC_MODEL_PATH=${YOLOE_MODEL_PATH:-"$YOLOE_PROJECT/models/yoloe-11s-seg-pf.pt"}
export SEMANTIC_BACKEND=yoloe_pf
export PYTHONNOUSERSITE=1
export YOLO_CONFIG_DIR="$YOLOE_PROJECT/models/.ultralytics-yoloe"
export MPLCONFIGDIR="$YOLOE_PROJECT/models/.matplotlib-yoloe"
exec bash "$YOLOE_PROJECT/scripts/start_semantic_slam.sh" \
  semantic_config:="$YOLOE_PROJECT/src/rtabmap_semantic/config/yoloe.yaml" \
  semantic_label_details:=false "$@"
