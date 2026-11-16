#!/usr/bin/env bash
# Run only a policy that passed human-labeled, independent-scene evaluation.
set -eo pipefail
GUARDED_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$GUARDED_PROJECT"
[[ $# == 1 && -f "$1" ]] || { printf 'Usage: bash scripts/start_segmentation_fusion_guarded.sh PASSED_REPORT.json\n' >&2; exit 2; }
GUARDED_REPORT=$(.venv-semantic/bin/python3 - "$1" <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, 'src/rtabmap_semantic')
from rtabmap_semantic.fusion_evaluation import load_passed_report
path=Path(sys.argv[1]).resolve()
report=load_passed_report(path)
if not report['fusion_enabled']:
    raise ValueError('Evaluation selected the RGB baseline; keep shadow mode')
print(path)
PY
)
export SEGMENTATION_PYTHON="$GUARDED_PROJECT/.venv-yoloe/bin/python3"
bash scripts/start_yolo_segmentation_live.sh --params-file config/yoloe_segmentation.yaml \
  -p fusion_mode:=guarded -p "fusion_evaluation_report:=$GUARDED_REPORT" \
  -p 'approved_fusion_providers:=[unibind_rgb_nir_v1]' -p fusion_timeout_s:=30.0 \
  -p max_input_age_s:=30.0 -p cache_frames:=4 &
GUARDED_RGB_PID=$!
bash scripts/start_segmentation_fusion.sh -p "evaluation_report:=$GUARDED_REPORT" &
GUARDED_FUSION_PID=$!
cleanup() {
  kill -TERM "$GUARDED_RGB_PID" "$GUARDED_FUSION_PID" 2>/dev/null || true
  wait "$GUARDED_RGB_PID" "$GUARDED_FUSION_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
wait -n "$GUARDED_RGB_PID" "$GUARDED_FUSION_PID"
