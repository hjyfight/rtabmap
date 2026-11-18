#!/usr/bin/env bash
set -eo pipefail
OBJECT_FUSION_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$OBJECT_FUSION_ROOT"
source setup.bash
source install/features/local_setup.bash
source install/segmentation/local_setup.bash
source install/semantic/local_setup.bash
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
mkdir -p logs/object_multimodal
exec 9>logs/object_multimodal/worker.lock
flock -n 9 || { printf '目标级四模态融合已在运行。\n' >&2; exit 1; }
exec .venv-segmentation-gpu/bin/python3 scripts/run_object_multimodal_fusion.py \
  --ros-args --params-file config/object_multimodal_fusion.yaml "$@"
