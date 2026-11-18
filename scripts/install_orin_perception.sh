#!/usr/bin/env bash
# JetPack 6.2: reuse NVIDIA torch and the already compiled CUDA torchvision.
set -euo pipefail
ORIN_PERCEPTION_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ORIN_PERCEPTION_BASE="$ORIN_PERCEPTION_ROOT/.venv-emsanet/bin/python3"
ORIN_PERCEPTION_ENV="$ORIN_PERCEPTION_ROOT/.venv-segmentation-gpu"
ORIN_AUDIO_SOURCE="$ORIN_PERCEPTION_ROOT/.tools/torchaudio"
ORIN_AUDIO_ARCHIVE="$ORIN_PERCEPTION_ROOT/.cache/orin-perception/torchaudio-v2.5.0.tar.gz"
[[ $(uname -m) == aarch64 ]] || { printf '此安装入口仅供 Orin ARM64。\n' >&2; exit 1; }
[[ -x $ORIN_PERCEPTION_BASE ]] || { printf '先配置已有 EMSANet JetPack 环境。\n' >&2; exit 1; }
export PYTHONNOUSERSITE=1
export PIP_CACHE_DIR="$ORIN_PERCEPTION_ROOT/.cache/orin-perception/pip"
export MAX_JOBS=${MAX_JOBS:-4}
export CMAKE_BUILD_PARALLEL_LEVEL=$MAX_JOBS
mkdir -p "$PIP_CACHE_DIR" "$ORIN_PERCEPTION_ROOT/logs/orin_perception"
"$ORIN_PERCEPTION_BASE" - "$ORIN_PERCEPTION_ROOT" <<'PY'
from pathlib import Path
import subprocess
import sys
import sysconfig
import torch
import torchvision
import numpy
import cv2

root = Path(sys.argv[1])
assert sys.version_info[:2] == (3, 10), 'Requires ROS Humble Python 3.10'
assert torch.cuda.is_available() and torch.__version__.startswith('2.5.'), 'Requires verified JetPack CUDA torch 2.5'
assert torchvision.extension._has_ops(), 'Requires compiled JetPack torchvision ops'
torchvision.ops.nms(torch.tensor([[0., 0., 1., 1.]], device='cuda'), torch.ones(1, device='cuda'), .5)
paths = [sysconfig.get_path('purelib'), str(Path(torch.__file__).resolve().parent.parent),
         str(root / 'src/rtabmap_features'), str(root / 'src/rtabmap_semantic')]
central = root / '.venv-segmentation-gpu/lib/python3.10/site-packages'
for name in ('.venv-segmentation-gpu', '.venv-fourmodal', '.venv-yoloe'):
    env = root / name
    if not (env / 'bin/python3').exists():
        subprocess.run(['/usr/bin/python3', '-m', 'venv', '--system-site-packages', '--without-pip', str(env)], check=True)
    site = env / 'lib/python3.10/site-packages'
    inherited = paths if name == '.venv-segmentation-gpu' else [str(central)] + paths
    (site / 'orin_jetpack_perception.pth').write_text('\n'.join(dict.fromkeys(inherited)) + '\n')
print('JetPack runtime:', torch.__version__, torch.version.cuda, torchvision.__version__, numpy.__version__, cv2.__version__)
PY
ORIN_PIP_OPTIONS=(--disable-pip-version-check --no-deps --no-build-isolation --timeout 20 --retries 1)
if [[ -n ${ORIN_PERCEPTION_WHEELHOUSE:-} ]]; then
  ORIN_PIP_OPTIONS+=(--no-index --find-links "$ORIN_PERCEPTION_WHEELHOUSE")
fi
"$ORIN_PERCEPTION_ENV/bin/python3" -m pip install "${ORIN_PIP_OPTIONS[@]}" \
  -r "$ORIN_PERCEPTION_ROOT/scripts/requirements_orin_perception.txt"
if ! "$ORIN_PERCEPTION_ENV/bin/python3" -c 'import torchaudio; assert torchaudio.__version__.startswith("2.5.")'; then
  [[ -s $ORIN_AUDIO_ARCHIVE ]] || { printf '缺少 %s；请准备官方 torchaudio v2.5.0 源码包。\n' "$ORIN_AUDIO_ARCHIVE" >&2; exit 1; }
  mkdir -p "$ORIN_AUDIO_SOURCE"
  tar -xzf "$ORIN_AUDIO_ARCHIVE" -C "$ORIN_AUDIO_SOURCE" --strip-components=1
  python3 - "$ORIN_AUDIO_SOURCE/tools/setup_helpers/extension.py" <<'PY'
from pathlib import Path
import sys
path = Path(sys.argv[1])
source = path.read_text()
for old, new in [("f\"-DCMAKE_CUDA_COMPILER='{CUDA_HOME}/bin/nvcc'\"", "f\"-DCMAKE_CUDA_COMPILER={CUDA_HOME}/bin/nvcc\""),
                 ("f\"-DCUDA_TOOLKIT_ROOT_DIR='{CUDA_HOME}'\"", "f\"-DCUDA_TOOLKIT_ROOT_DIR={CUDA_HOME}\"")]:
    if source.count(old) != 1:
        raise RuntimeError('Unexpected TorchAudio v2.5.0 CMake source')
    source = source.replace(old, new)
path.write_text(source)
PY
  # Audio preprocessing runs on CPU; the pretrained audio encoder uses CUDA.
  # Disable optional downloads/decoders and compile against NVIDIA's torch ABI.
  BUILD_VERSION=2.5.0 USE_CUDA=0 USE_FFMPEG=0 BUILD_SOX=0 BUILD_RIR=0 \
    BUILD_RNNT=0 BUILD_ALIGN=0 TORCH_CUDA_ARCH_LIST=8.7 \
    "$ORIN_PERCEPTION_ENV/bin/python3" -m pip install --disable-pip-version-check \
      --no-deps --no-build-isolation "$ORIN_AUDIO_SOURCE"
fi
"$ORIN_PERCEPTION_ENV/bin/python3" - "$ORIN_PERCEPTION_ROOT" <<'PY'
from pathlib import Path
import json
import sys
import torch
import torchvision
import torchaudio
import ultralytics
import soundfile
import timm
import einops
import iopath
import ftfy
import cv2
import numpy
assert torch.cuda.is_available()
x = torch.ones((16, 16), device='cuda')
assert torch.allclose(x @ x, torch.full_like(x, 16))
torchaudio.compliance.kaldi.fbank(torch.zeros(1, 32000), sample_frequency=16000, num_mel_bins=128)
report = dict(architecture='aarch64', torch=torch.__version__, cuda=torch.version.cuda,
              torchvision=torchvision.__version__, torchaudio=torchaudio.__version__,
              ultralytics=ultralytics.__version__, numpy=numpy.__version__, opencv=cv2.__version__,
              cuda_tensor_test=True, audio_fbank_test=True, torch_replaced=False)
path = Path(sys.argv[1]) / 'logs/orin_perception/environment.json'
path.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
PY
printf 'Orin 四模态和 YOLOE CUDA 环境已配置；再构建 features、segmentation 并验证模型。\n'
