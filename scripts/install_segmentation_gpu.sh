#!/usr/bin/env bash
# Isolated CUDA runtime; reuse project dependencies without changing CPU environments.
set -euo pipefail
SEG_GPU_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SEG_GPU_ENV="$SEG_GPU_PROJECT/.venv-segmentation-gpu"
export PYTHONNOUSERSITE=1 PIP_CACHE_DIR="$SEG_GPU_PROJECT/.cache/segmentation-gpu-pip"
mkdir -p "$PIP_CACHE_DIR"
if [[ ! -x "$SEG_GPU_ENV/bin/python3" ]]; then
  /usr/bin/python3 -m venv --system-site-packages --without-pip "$SEG_GPU_ENV"
fi
python3 - "$SEG_GPU_PROJECT" "$SEG_GPU_ENV" <<'PY'
from pathlib import Path
import sys
root,target=map(Path,sys.argv[1:])
dependencies=[root/'.venv-fourmodal/lib/python3.10/site-packages',
              root/'.venv-yoloe/lib/python3.10/site-packages']
if not all(p.is_dir() for p in dependencies):raise RuntimeError('Install CPU feature and YOLOE environments first')
(target/'lib/python3.10/site-packages/project_segmentation_dependencies.pth').write_text(
    '\n'.join(str(p) for p in dependencies+[root/'src/rtabmap_features',root/'src/rtabmap_semantic'])+'\n')
PY
"$SEG_GPU_ENV/bin/python3" -m pip install --disable-pip-version-check \
  --index-url https://download.pytorch.org/whl/cu118 \
  'torch==2.5.1+cu118' 'torchvision==0.20.1+cu118' 'torchaudio==2.5.1+cu118'
"$SEG_GPU_ENV/bin/python3" - <<'PY'
import torch
if not torch.cuda.is_available():raise RuntimeError('Installed CUDA runtime cannot access a GPU')
x=torch.ones((16,16),device='cuda')
assert torch.allclose(x@x,torch.full_like(x,16))
print('CUDA segmentation runtime:',torch.__version__,torch.cuda.get_device_name(0))
PY
