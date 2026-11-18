#!/usr/bin/env bash
set -euo pipefail
FOURMODAL_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$FOURMODAL_PROJECT"
python3 scripts/install_miti_loop_panel.py
cp config/miti_live.html .tools/miti/root/usr/lib/miti/public/rtabmap.html
cp config/miti_live_layout.json .tools/miti/root/usr/lib/miti/public/rtabmap-layout.json
cp config/miti_fusion_layout.json .tools/miti/root/usr/lib/miti/public/rtabmap-fusion-layout.json
cp config/miti_object_fusion_layout.json .tools/miti/root/usr/lib/miti/public/rtabmap-object-fusion-layout.json
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
  .venv-fourmodal/bin/python3 scripts/export_fourmodal_miti.py "$@"
printf '运行 bash scripts/start_fourmodal_miti.sh，打开 http://localhost:3002/rtabmap.html?view=fusion 查看融合流程。\n'
printf '四模态面板可冻结当前帧或查看离线结果；原有布局可通过 /rtabmap.html 更新显示。\n'
