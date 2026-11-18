#!/usr/bin/env bash
# Reuse the existing audio publisher; this process owns only sound classification.
set -eo pipefail
YAMNET_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$YAMNET_PROJECT"
if [[ ! -x .venv-yamnet/bin/python3 || ! -s models/yamnet/yamnet.tflite ]]; then
  printf '先运行 bash scripts/install_yamnet.sh\n' >&2
  exit 1
fi
source setup.bash
if [[ ! -f install/features/local_setup.bash ]]; then
  printf '先运行 bash scripts/build_fourmodal_features.sh\n' >&2
  exit 1
fi
source install/features/local_setup.bash
export RTABMAP_PROJECT_ROOT="$YAMNET_PROJECT"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
mkdir -p logs/audio/yamnet
exec 9>"logs/audio/yamnet/domain_${ROS_DOMAIN_ID:-0}.lock"
flock -n 9 || { printf '本 ROS 域的 YAMNet 声音识别已经在运行。\n' >&2; exit 1; }
printf 'YAMNet：复用 /multimodal/audio，输出 /perception/audio/events；Ctrl+C 停止声音识别。\n'
exec .venv-yamnet/bin/python3 scripts/run_yamnet_audio.py \
  --ros-args --params-file config/yamnet_audio.yaml "$@"
