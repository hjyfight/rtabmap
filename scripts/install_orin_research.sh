#!/usr/bin/env bash
# ARM64 research inference; reuse JetPack's existing CUDA-enabled torch.
set -euo pipefail
ORIN_RESEARCH_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ORIN_RESEARCH_BASE="$ORIN_RESEARCH_ROOT/.venv-segmentation-gpu/bin/python3"
ORIN_RESEARCH_ENV="$ORIN_RESEARCH_ROOT/.venv-indoor-research"
ORIN_RESEARCH_CACHE="$ORIN_RESEARCH_ROOT/.cache/orin-research"
[[ $(uname -m) == aarch64 ]] || { printf 'This installer requires Orin ARM64.\n' >&2; exit 1; }
[[ -x $ORIN_RESEARCH_BASE ]] || { printf 'Configure the JetPack perception environment first.\n' >&2; exit 1; }
export CUDA_HOME=/usr/local/cuda
export PATH="$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST=8.7
export CUMM_CUDA_ARCH_LIST=8.7
export MAX_JOBS=${MAX_JOBS:-4}
export PIP_CACHE_DIR="$ORIN_RESEARCH_CACHE/pip"
export PYTHONNOUSERSITE=1
mkdir -p "$ORIN_RESEARCH_CACHE" "$ORIN_RESEARCH_ROOT/logs/orin_research_deploy"
"$ORIN_RESEARCH_BASE" - "$ORIN_RESEARCH_ROOT" <<'PY'
from pathlib import Path
import json, subprocess, sys, sysconfig
import torch, torchvision
root = Path(sys.argv[1])
assert torch.cuda.is_available() and torch.__version__.startswith('2.5.'), 'Requires JetPack CUDA PyTorch 2.5'
assert torch.cuda.get_device_capability(0) == (8, 7), 'Requires Orin SM87'
assert torchvision.extension._has_ops(), 'Requires existing CUDA torchvision ops'
paths = [sysconfig.get_path('purelib')]
for name in ('torch', 'torchvision', 'numpy', 'cv2', 'torchaudio', 'ultralytics', 'timm'):
    module = __import__(name)
    paths.append(str(Path(module.__file__).resolve().parent.parent))
paths.append(str(root / '.tools/odin-research-deps'))
for name in ('.venv-indoor-research', '.venv-fusion-research'):
    env = root / name
    if not (env / 'bin/python3').exists():
        subprocess.run(['/usr/bin/python3', '-m', 'venv', '--system-site-packages', '--without-pip', str(env)], check=True)
    site = env / 'lib/python3.10/site-packages'
    inherited = paths if name == '.venv-indoor-research' else [str(root / '.venv-indoor-research/lib/python3.10/site-packages')] + paths
    (site / 'orin_research_runtime.pth').write_text('import site; [__import__("site").addsitedir(p) for p in ' + repr(list(dict.fromkeys(inherited))) + ']\n')
print('Reusing JetPack:', torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0), flush=True)
PY
ORIN_RESEARCH_PY="$ORIN_RESEARCH_ENV/bin/python3"
ORIN_RESEARCH_PIP=(--disable-pip-version-check --no-deps --no-build-isolation --timeout 30 --retries 2)
"$ORIN_RESEARCH_PY" -m pip install "${ORIN_RESEARCH_PIP[@]}" \
  setuptools==69.5.1 wheel==0.43.0 ninja==1.11.1.1 \
  addict==2.4.0 pccm==0.4.16 ccimport==0.4.4 pybind11==2.13.6 \
  fire==0.7.0 lark==1.2.2 timm==0.6.13 \
  resampy==0.4.3 numba==0.60.0 llvmlite==0.43.0
if ! "$ORIN_RESEARCH_PY" -c 'import torch_scatter; import torch; x=torch.tensor([1.,3.],device="cuda"); assert torch_scatter.scatter_mean(x,torch.tensor([0,0],device="cuda")).item()==2'; then
  FORCE_CUDA=1 "$ORIN_RESEARCH_PY" -m pip install "${ORIN_RESEARCH_PIP[@]}" \
    --no-binary=torch-scatter torch-scatter==2.1.2
fi
for ORIN_RESEARCH_PACKAGE in cumm spconv; do
  if [[ $ORIN_RESEARCH_PACKAGE == cumm ]]; then
    ORIN_RESEARCH_VERSION=0.7.11
    ORIN_RESEARCH_REPOSITORY=FindDefinition/cumm
  else
    ORIN_RESEARCH_VERSION=2.3.8
    ORIN_RESEARCH_REPOSITORY=traveller59/spconv
  fi
  ORIN_RESEARCH_SOURCE="$ORIN_RESEARCH_ROOT/.tools/$ORIN_RESEARCH_PACKAGE-$ORIN_RESEARCH_VERSION"
  ORIN_RESEARCH_ARCHIVE="$ORIN_RESEARCH_CACHE/$ORIN_RESEARCH_PACKAGE-v$ORIN_RESEARCH_VERSION.tar.gz"
  if [[ ! -f $ORIN_RESEARCH_SOURCE/setup.py ]]; then
    curl -fL --retry 2 --connect-timeout 20 --max-time 180 \
      "https://codeload.github.com/$ORIN_RESEARCH_REPOSITORY/tar.gz/refs/tags/v$ORIN_RESEARCH_VERSION" \
      -o "$ORIN_RESEARCH_ARCHIVE.part"
    mv "$ORIN_RESEARCH_ARCHIVE.part" "$ORIN_RESEARCH_ARCHIVE"
    mkdir -p "$ORIN_RESEARCH_SOURCE"
    tar -xzf "$ORIN_RESEARCH_ARCHIVE" -C "$ORIN_RESEARCH_SOURCE" --strip-components=1
  fi
  "$ORIN_RESEARCH_PY" -m pip install "${ORIN_RESEARCH_PIP[@]}" -e "$ORIN_RESEARCH_SOURCE"
done
"$ORIN_RESEARCH_PY" - "$ORIN_RESEARCH_ROOT" <<'PY'
from pathlib import Path
import importlib.metadata as metadata
import json, sys
import torch, torch_scatter
import spconv.pytorch as spconv
torch.set_num_threads(2)
coords = torch.cartesian_prod(torch.arange(4), torch.arange(4), torch.arange(4)).int().cuda()
indices = torch.cat([torch.zeros((len(coords), 1), dtype=torch.int32, device='cuda'), coords], dim=1)
value = spconv.SparseConvTensor(torch.ones((len(coords), 4), device='cuda'), indices, [4, 4, 4], 1)
module = spconv.SubMConv3d(4, 8, 3, padding=1, bias=False).cuda().eval()
with torch.inference_mode():
    output = module(value)
assert output.features.shape == (64, 8) and torch.isfinite(output.features).all()
assert torch_scatter.scatter_mean(torch.tensor([1., 3.], device='cuda'), torch.tensor([0, 0], device='cuda')).item() == 2
torch.cuda.synchronize()
report = dict(architecture='aarch64', torch=torch.__version__, cuda=torch.version.cuda,
              gpu=torch.cuda.get_device_name(0), capability=list(torch.cuda.get_device_capability(0)),
              dependencies={name: metadata.version(name) for name in ['cumm', 'spconv', 'torch-scatter', 'timm']},
              sparse_cuda_test=True, scatter_cuda_test=True)
path = Path(sys.argv[1]) / 'logs/orin_research_deploy/environment.json'
path.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
PY
printf 'Orin research CUDA environment configured.\n'
