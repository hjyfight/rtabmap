#!/usr/bin/env bash
# Wired robot inputs; read wall time without changing clocks or mounting calibration.
set -eo pipefail
OBJECT_LIVE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$OBJECT_LIVE_ROOT"
source scripts/ros2_wired_env.sh
mkdir -p logs/object_multimodal
OBJECT_LIVE_RUN=$(mktemp -d "$OBJECT_LIVE_ROOT/logs/object_multimodal/live_XXXXXXXX")
if [[ ${1:-} == --clock-report ]]; then
  [[ -f ${2:-} ]] || { printf 'Missing clock report\n' >&2; exit 2; }
  cp "$2" "$OBJECT_LIVE_RUN/clock.json"
  shift 2
else
  python3 scripts/measure_fourmodal_clock.py --output "$OBJECT_LIVE_RUN/clock.json"
fi
python3 scripts/prepare_object_multimodal_live.py \
  --clock-report "$OBJECT_LIVE_RUN/clock.json" \
  --output "$OBJECT_LIVE_RUN/calibration.json"
printf '使用机器狗墙钟估计补偿帧龄；物理标定与同步仍未验证。\n'
exec bash scripts/start_object_multimodal_fusion.sh \
  -p "calibration_path:=$OBJECT_LIVE_RUN/calibration.json" "$@"
