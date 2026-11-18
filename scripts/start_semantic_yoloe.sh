#!/usr/bin/env bash
set -eo pipefail
YOLOE_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
YOLOE_GPU=false
YOLOE_CHECK_ARGS=()
YOLOE_DEVICE_ARGS=()
while [[ ${1:-} == --gpu || ${1:-} == --check-input-only ]]; do
  case "$1" in
    --gpu) YOLOE_GPU=true ;;
    --check-input-only) YOLOE_CHECK_ARGS=(--check-input-only) ;;
  esac
  shift
done
export SEMANTIC_RUNTIME_PYTHON="$YOLOE_PROJECT/.venv-yoloe/bin/python3"
if $YOLOE_GPU; then
  export SEMANTIC_RUNTIME_PYTHON="$YOLOE_PROJECT/.venv-segmentation-gpu/bin/python3"
  [[ -x "$SEMANTIC_RUNTIME_PYTHON" ]] || { printf '先运行 bash scripts/install_segmentation_gpu.sh\n' >&2; exit 1; }
  "$SEMANTIC_RUNTIME_PYTHON" -c 'import torch; assert torch.cuda.is_available(), "CUDA is unavailable"'
  YOLOE_DEVICE_ARGS=(semantic_device:=cuda:0)
fi
export SEMANTIC_MODEL_PATH=${YOLOE_MODEL_PATH:-"$YOLOE_PROJECT/models/yoloe-11s-seg-pf.pt"}
export SEMANTIC_BACKEND=yoloe_pf
export PYTHONNOUSERSITE=1
export YOLO_CONFIG_DIR="$YOLOE_PROJECT/models/.ultralytics-yoloe"
export MPLCONFIGDIR="$YOLOE_PROJECT/models/.matplotlib-yoloe"
exec bash "$YOLOE_PROJECT/scripts/start_semantic_slam.sh" \
  "${YOLOE_CHECK_ARGS[@]}" \
  semantic_config:="$YOLOE_PROJECT/src/rtabmap_semantic/config/yoloe.yaml" \
  semantic_label_details:=false "${YOLOE_DEVICE_ARGS[@]}" "$@"
