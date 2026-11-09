#!/usr/bin/env bash
# RGB-D EMSANet entry point. Existing YOLO launch commands retain their defaults.
set -eo pipefail
EMSANET_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export SEMANTIC_RUNTIME_PYTHON="$EMSANET_PROJECT/.venv-emsanet/bin/python3"
export SEMANTIC_MODEL_PATH=${EMSANET_MODEL_PATH:-"$EMSANET_PROJECT/models/emsanet/nyuv2_r34_NBt1D_pre.pth"}
export SEMANTIC_BACKEND=emsanet
export PYTHONNOUSERSITE=1
export MPLCONFIGDIR="$EMSANET_PROJECT/models/.matplotlib-emsanet"
if [[ ! -x "$SEMANTIC_RUNTIME_PYTHON" || ! -s "$SEMANTIC_MODEL_PATH" ]]; then
  printf '先运行 bash scripts/install_emsanet.sh\n' >&2
  exit 1
fi
if ! "$SEMANTIC_RUNTIME_PYTHON" -c 'import torch, emsanet, nicr_mt_scene_analysis'; then
  printf 'EMSANet 环境不完整，请运行 bash scripts/install_emsanet.sh\n' >&2
  exit 1
fi
exec bash "$EMSANET_PROJECT/scripts/start_semantic_slam.sh" \
  semantic_config:="$EMSANET_PROJECT/src/rtabmap_semantic/config/emsanet.yaml" \
  semantic_label_details:=false "$@"
