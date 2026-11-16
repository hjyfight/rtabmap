#!/usr/bin/env bash
set -euo pipefail
FEATURE_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FEATURE_PROJECT"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
.venv-fourmodal/bin/python3 -m unittest discover -s tests -p 'test_fourmodal*.py' -v
