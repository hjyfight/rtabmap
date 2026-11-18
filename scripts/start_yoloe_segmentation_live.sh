#!/usr/bin/env bash
set -eo pipefail
SEG_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$SEG_PROJECT"
export SEGMENTATION_PYTHON="$SEG_PROJECT/.venv-yoloe/bin/python3"
SEG_RUNTIME_ARGS=()
if [[ ${1:-} == --gpu ]]; then
  shift
  export SEGMENTATION_PYTHON="$SEG_PROJECT/.venv-segmentation-gpu/bin/python3"
  [[ -x "$SEGMENTATION_PYTHON" ]] || { printf 'Run bash scripts/install_segmentation_gpu.sh first\n' >&2; exit 1; }
  "$SEGMENTATION_PYTHON" -c 'import torch; assert torch.cuda.is_available(), "CUDA is unavailable"'
  SEG_RUNTIME_ARGS=(--params-file config/yoloe_segmentation_gpu.yaml)
fi
exec bash scripts/start_yolo_segmentation_live.sh "$@" --params-file config/yoloe_segmentation.yaml "${SEG_RUNTIME_ARGS[@]}"
