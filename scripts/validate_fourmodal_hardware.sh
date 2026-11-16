#!/usr/bin/env bash
# One bounded physical-sensor run. Existing sensor/SLAM/MITI processes are preserved.
set -eo pipefail
FOURMODAL_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FOURMODAL_PROJECT"
source scripts/ros2_wired_env.sh
source install/features/local_setup.bash
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
FOURMODAL_RUN="logs/fourmodal/hardware_live_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$FOURMODAL_RUN"
python3 scripts/measure_fourmodal_clock.py --output "$FOURMODAL_RUN/clock.json"
python3 - "$FOURMODAL_RUN" <<'PY'
import json, sys
from pathlib import Path
run = Path(sys.argv[1])
clock = json.loads((run/'clock.json').read_text())
offset = clock['workstation_minus_robot_s']
config = {'fourmodal_features': {'ros__parameters': {
    'clock_offsets_s': [offset, offset, offset, 0.0],
    'calibration_version': 'ssh_clock_estimate_geometry_unverified',
    'save_directory': str(run/'features')}}}
(run/'runtime.yaml').write_text(json.dumps(config, indent=2)+'\n')
PY
FOURMODAL_CHILDREN=()
fourmodal_cleanup() {
  trap - EXIT INT TERM
  for pid in "${FOURMODAL_CHILDREN[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${FOURMODAL_CHILDREN[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap fourmodal_cleanup EXIT
trap 'exit 1' INT TERM
bash scripts/start_fourmodal_audio.sh > "$FOURMODAL_RUN/audio.log" 2>&1 &
FOURMODAL_CHILDREN+=("$!")
bash scripts/start_fourmodal_features.sh --ros-args --params-file "$FOURMODAL_RUN/runtime.yaml" \
  > "$FOURMODAL_RUN/features.log" 2>&1 &
FOURMODAL_CHILDREN+=("$!")
.venv-fourmodal/bin/python3 scripts/check_fourmodal_features_live.py --seconds 45 \
  --output "$FOURMODAL_RUN/validation.json" "$@"
mkdir -p logs/fourmodal_validation
cp "$FOURMODAL_RUN/validation.json" logs/fourmodal_validation/hardware_live.json
.venv-fourmodal/bin/python3 scripts/export_fourmodal_miti.py --result-root "$FOURMODAL_RUN/features"
printf '实机验证结果：%s/validation.json\n' "$FOURMODAL_RUN"
