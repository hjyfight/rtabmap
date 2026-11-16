#!/usr/bin/env bash
set -euo pipefail
FEATURE_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FEATURE_PROJECT"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
if [[ ! -f logs/fourmodal/smoke_inputs/manifest.json ]]; then
  .venv-fourmodal/bin/python3 scripts/make_fourmodal_smoke_bundle.py --output logs/fourmodal/smoke_inputs
fi
FEATURE_OUTPUT="logs/fourmodal/demo_$(date +%Y%m%d_%H%M%S)_$$"
exec .venv-fourmodal/bin/python3 scripts/predict_fourmodal_features.py \
  --manifest logs/fourmodal/smoke_inputs/manifest.json --allow-unaligned \
  --device cpu --output "$FEATURE_OUTPUT" "$@"
