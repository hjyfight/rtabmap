#!/usr/bin/env bash
set -eo pipefail
FOURMODAL_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FOURMODAL_PROJECT"
source scripts/ros2_wired_env.sh
source install/features/local_setup.bash
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
FOURMODAL_CAPTURE_PATH=${1:-"logs/fourmodal/hardware_$(date +%Y%m%d_%H%M%S)"}
mkdir -p logs/fourmodal
bash scripts/start_fourmodal_audio.sh > logs/fourmodal/hardware_audio.log 2>&1 &
FOURMODAL_AUDIO_PID=$!
fourmodal_cleanup() {
  trap - EXIT INT TERM
  kill -TERM "$FOURMODAL_AUDIO_PID" 2>/dev/null || true
  wait "$FOURMODAL_AUDIO_PID" 2>/dev/null || true
}
trap fourmodal_cleanup EXIT
trap 'exit 1' INT TERM
.venv-fourmodal/bin/python3 scripts/capture_fourmodal_hardware.py --seconds 15 --output "$FOURMODAL_CAPTURE_PATH"
