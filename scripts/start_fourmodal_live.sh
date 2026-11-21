#!/usr/bin/env bash
# Continuous physical-sensor features, with explicit clock estimation and owned-process cleanup.
set -eo pipefail
FOURMODAL_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FOURMODAL_PROJECT"
FOURMODAL_REUSE_AUDIO=false
FOURMODAL_DEVICE=cpu
while [[ $# -gt 0 ]]; do
  case "$1" in
    --reuse-audio) FOURMODAL_REUSE_AUDIO=true ;;
    --gpu)
      FOURMODAL_DEVICE=cuda:0
      export FOURMODAL_RUNTIME_PYTHON="$FOURMODAL_PROJECT/.venv-segmentation-gpu/bin/python3"
      ;;
    *) printf '用法：bash scripts/start_fourmodal_live.sh [--reuse-audio] [--gpu]\n' >&2; exit 2 ;;
  esac
  shift
done
source scripts/ros2_wired_env.sh
source install/features/local_setup.bash
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
mkdir -p logs/fourmodal
exec 9>logs/fourmodal/live.lock
flock -n 9 || { printf '四模态实时采集已经在运行。\n' >&2; exit 1; }

# Avoid competing with a manually started producer or the bounded hardware test.
.venv-fourmodal/bin/python3 - "$FOURMODAL_REUSE_AUDIO" <<'PY'
import sys
import time
import rclpy
rclpy.init()
node = rclpy.create_node('fourmodal_live_start_check')
try:
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    reuse_audio = sys.argv[1] == 'true'
    existing = {'fourmodal_features'} & set(node.get_node_names())
    audio = node.get_publishers_info_by_topic('/multimodal/audio')
    if reuse_audio and len(audio) != 1:
        raise SystemExit('复用音频要求当前 ROS 网络中恰好有一个 /multimodal/audio 发布端')
    if not reuse_audio and audio:
        existing.add('fourmodal_audio_capture')
    if existing:
        raise SystemExit('已有四模态节点，请复用现有输入：' + ', '.join(sorted(existing)))
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

FOURMODAL_RUN="logs/fourmodal/live_$(date +%Y%m%d_%H%M%S_%N)"
mkdir -p "$FOURMODAL_RUN"
python3 scripts/measure_fourmodal_clock.py --sensor-fallback --output "$FOURMODAL_RUN/clock.json" > "$FOURMODAL_RUN/clock_measurement.log"
python3 - "$FOURMODAL_RUN" "$FOURMODAL_DEVICE" <<'PY'
import json, sys
from pathlib import Path
run = Path(sys.argv[1])
clock = json.loads((run/'clock.json').read_text())
offset = clock['workstation_minus_robot_s']
offsets = clock.get('modality_offsets_s',dict(rgb=offset, infrared=offset, lidar=offset, audio=0.0))
config = {'fourmodal_features': {'ros__parameters': {
    'clock_offsets_s': [offsets[name] for name in ('rgb','infrared','lidar','audio')],
    'device': sys.argv[2],
    'calibration_version': clock['method']+'_geometry_unverified',
    'save_directory': ''}}}
(run/'runtime.yaml').write_text(json.dumps(config, indent=2)+'\n')
print(f'实时匹配时钟偏移估计：{offset:.6f} 秒；原始采集时间保留。')
PY

FOURMODAL_CHILDREN=()
fourmodal_cleanup() {
  trap - EXIT INT TERM
  for pid in "${FOURMODAL_CHILDREN[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${FOURMODAL_CHILDREN[@]}"; do wait "$pid" 2>/dev/null || true; done
  printf '本次实时麦克风与特征节点已停止。日志：%s\n' "$FOURMODAL_RUN"
}
trap fourmodal_cleanup EXIT
trap 'exit 0' INT TERM
FOURMODAL_AUDIO_PID=''
if ! $FOURMODAL_REUSE_AUDIO; then
  bash scripts/start_fourmodal_audio.sh > "$FOURMODAL_RUN/audio.log" 2>&1 9>&- &
  FOURMODAL_AUDIO_PID=$!
  FOURMODAL_CHILDREN+=("$FOURMODAL_AUDIO_PID")
fi
bash scripts/start_fourmodal_features.sh --ros-args --params-file "$FOURMODAL_RUN/runtime.yaml" \
  > "$FOURMODAL_RUN/features.log" 2>&1 9>&- &
FOURMODAL_CHILDREN+=("$!")
FOURMODAL_FEATURES_PID=$!
python3 - "$FOURMODAL_RUN" "$$" "$FOURMODAL_AUDIO_PID" "$FOURMODAL_FEATURES_PID" <<'PY'
import json, sys, time
from pathlib import Path
run = Path(sys.argv[1]).resolve()
state = dict(started_at=time.time(), run_directory=str(run), launcher_pid=int(sys.argv[2]),
             audio_pid=int(sys.argv[3]) if sys.argv[3] else None, features_pid=int(sys.argv[4]),
             audio_reused=not bool(sys.argv[3]), web_url='http://localhost:3000', ros_domain_id=0)
temporary = Path('logs/fourmodal/live_current.tmp.json')
temporary.write_text(json.dumps(state, indent=2)+'\n')
temporary.replace('logs/fourmodal/live_current.json')
PY
printf '持续实时采集已启动，模型正在加载。MITI 中选择“实时输入”。\n'
printf '日志：%s；停止本次采集：kill -TERM %s\n' "$FOURMODAL_RUN" "$$"
wait -n "${FOURMODAL_CHILDREN[@]}"
