#!/usr/bin/env bash
set -eo pipefail
SEG_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$SEG_PROJECT"
export SEGMENTATION_PYTHON="$SEG_PROJECT/.venv-yoloe/bin/python3"
exec bash scripts/start_yolo_segmentation_live.sh "$@" --params-file config/yoloe_segmentation.yaml
