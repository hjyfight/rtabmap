#!/usr/bin/env bash
# Source a network environment first when showing the map on another machine.
set -eo pipefail
ORIN_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-4}
export OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS:-2}
if [[ -z ${CYCLONEDDS_URI:-} ]]; then
  printf '请先加载 scripts/ros2_wifi_direct_env.sh 本机网卡 主机IP。\n' >&2
  exit 1
fi
PYTHONNOUSERSITE=1 "$ORIN_PROJECT/.venv-emsanet/bin/python3" -c \
  'import torch; assert torch.cuda.is_available(), "CUDA unavailable"; print("GPU:", torch.cuda.get_device_name(0))'
exec bash "$ORIN_PROJECT/scripts/start_semantic_emsanet.sh" \
  rtabmap_viz:=false loop_status_window:=false semantic_device:=cuda:0 "$@"
