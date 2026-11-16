#!/usr/bin/env bash
set -eo pipefail
FEATURE_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FEATURE_PROJECT"
source setup.bash
source install/features/local_setup.bash
export PYTHONNOUSERSITE=1
exec .venv-fourmodal/bin/python3 scripts/publish_fourmodal_audio.py "$@"
