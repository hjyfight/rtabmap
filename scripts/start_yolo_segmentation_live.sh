#!/usr/bin/env bash
# Workstation entrypoint: estimate robot wall-clock offset without changing either clock.
set -eo pipefail
SEG_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$SEG_PROJECT"
source scripts/ros2_wired_env.sh
SEG_RUN="logs/segmentation/live_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$SEG_RUN"
if [[ ${1:-} == --clock-report ]]; then
  [[ -f ${2:-} ]] || { printf 'Missing clock report\n' >&2; exit 2; }
  cp "$2" "$SEG_RUN/clock.json"
  shift 2
else
  python3 scripts/measure_fourmodal_clock.py --sensor-fallback --output "$SEG_RUN/clock.json"
fi
python3 - "$SEG_RUN" <<'PY'
import json
from pathlib import Path
import sys
import time
run = Path(sys.argv[1])
clock = json.loads((run/'clock.json').read_text())
if not 0 <= time.time()-clock['measured_at'] < 86400:
    raise ValueError('Clock report must be less than one day old; measure the robot clock again')
config = {'yolo_segmentation': {'ros__parameters': {
    'rgb_clock_offset_s': clock['workstation_minus_robot_s'],
    'clock_offset_uncertainty_s': clock.get('round_trip_uncertainty_s') or min(5.0,clock.get('offset_observed_spread_s',0.0)),
    'clock_offset_method': clock['method']+'@'+str(clock['measured_at'])}}}
(run/'runtime.yaml').write_text(json.dumps(config,indent=2)+'\n')
PY
exec bash scripts/start_yolo_segmentation.sh --params-file "$SEG_RUN/runtime.yaml" "$@"
