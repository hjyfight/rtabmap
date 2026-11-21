#!/usr/bin/env bash
set -eo pipefail
FEATURE_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FEATURE_PROJECT"
source setup.bash
[[ -f install/features/local_setup.bash ]] || {
  printf 'Run bash scripts/build_fourmodal_features.sh first\n' >&2; exit 1;
}
source install/features/local_setup.bash
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
exec "${FOURMODAL_RUNTIME_PYTHON:-$FEATURE_PROJECT/.venv-fourmodal/bin/python3}" scripts/run_fourmodal_features.py \
  --ros-args --params-file config/fourmodal_features.yaml "$@"
