#!/usr/bin/env bash
set -eo pipefail
FUSION_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FUSION_PROJECT"
FUSION_PYTHON="$FUSION_PROJECT/.venv-fourmodal/bin/python3"
FUSION_NIR_PYTHON="$FUSION_PROJECT/.venv-yoloe/bin/python3"
FUSION_RUNTIME_ARGS=()
if [[ ${1:-} == --gpu ]]; then
  shift
  FUSION_PYTHON="$FUSION_PROJECT/.venv-segmentation-gpu/bin/python3"
  FUSION_NIR_PYTHON="$FUSION_PYTHON"
  [[ -x "$FUSION_PYTHON" ]] || { printf 'Run bash scripts/install_segmentation_gpu.sh first\n' >&2; exit 1; }
  "$FUSION_PYTHON" -c 'import torch; assert torch.cuda.is_available(), "CUDA is unavailable"'
  FUSION_RUNTIME_ARGS=(--params-file config/segmentation_fusion_gpu.yaml)
fi
source scripts/ros2_wired_env.sh
source install/segmentation/local_setup.bash
mkdir -p logs/segmentation_fusion
exec 9>logs/segmentation_fusion/live.lock
flock -n 9 || { printf 'Segmentation fusion is already running\n' >&2; exit 1; }
FUSION_RUN="logs/segmentation_fusion/capture_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$FUSION_RUN"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
export ROS_LOG_DIR="$FUSION_PROJECT/logs/segmentation_fusion/ros"
export YOLO_CONFIG_DIR="$FUSION_PROJECT/models/.ultralytics" MPLCONFIGDIR="$FUSION_PROJECT/models/.matplotlib"
"$FUSION_NIR_PYTHON" scripts/run_nir_segmentation.py --ros-args --params-file config/segmentation_fusion.yaml "${FUSION_RUNTIME_ARGS[@]}" >"$FUSION_RUN/nir.log" 2>&1 &
FUSION_NIR_PID=$!
"$FUSION_PYTHON" scripts/run_segmentation_fusion.py --ros-args --params-file config/segmentation_fusion.yaml "${FUSION_RUNTIME_ARGS[@]}" \
  -p "capture_directory:=$FUSION_RUN" "$@" >"$FUSION_RUN/fusion.log" 2>&1 &
FUSION_WORKER_PID=$!
python3 - "$FUSION_RUN" "$FUSION_NIR_PID" "$FUSION_WORKER_PID" <<'PY'
import json
from pathlib import Path
import sys,time
record=dict(directory=str(Path(sys.argv[1]).resolve()),nir_pid=int(sys.argv[2]),fusion_pid=int(sys.argv[3]),started_at=time.time())
Path('logs/segmentation_fusion/live_current.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record))
PY
cleanup() {
  kill -TERM "$FUSION_NIR_PID" "$FUSION_WORKER_PID" 2>/dev/null || true
  wait "$FUSION_NIR_PID" "$FUSION_WORKER_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
wait -n "$FUSION_NIR_PID" "$FUSION_WORKER_PID"
