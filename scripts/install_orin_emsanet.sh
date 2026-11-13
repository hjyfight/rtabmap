#!/usr/bin/env bash
# JetPack 6.2 deployment: retain NVIDIA's installed CUDA-enabled PyTorch 2.5.
set -euo pipefail
ORIN_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ORIN_ENV="$ORIN_PROJECT/.venv-emsanet"
ORIN_SOURCE="$ORIN_PROJECT/.tools/emsanet"
ORIN_VISION="$ORIN_PROJECT/.tools/torchvision"
[[ $(uname -m) == aarch64 ]] || { printf '此入口仅供 Orin ARM64。\n' >&2; exit 1; }
# Resolve before disabling user-site packages: JetPack PyTorch may live in ~/.local.
ORIN_TORCH_SITE=$(/usr/bin/python3 -c 'import pathlib, torch; assert torch.__version__.startswith("2.5.") and torch.cuda.is_available(), "需要可用的 NVIDIA CUDA PyTorch 2.5"; print(pathlib.Path(torch.__file__).resolve().parent.parent)')
ORIN_TORCH_VERSION=$(/usr/bin/python3 -c 'import importlib.metadata; print(importlib.metadata.version("torch"))')
[[ -f $ORIN_SOURCE/lib/nicr-multitask-scene-analysis/setup.py ]] || { printf '请先同步 EMSANet 固定版本源码。\n' >&2; exit 1; }
[[ -f $ORIN_PROJECT/logs/orin_deploy/torchvision-v0.20.0.tar.gz ]] || { printf '缺少 torchvision v0.20.0 源码包。\n' >&2; exit 1; }
export PYTHONNOUSERSITE=1
export PIP_CACHE_DIR="$ORIN_PROJECT/.cache/emsanet-pip"
export MPLCONFIGDIR="$ORIN_PROJECT/models/.matplotlib-emsanet"
export CUDA_HOME=/usr/local/cuda
export PATH="$CUDA_HOME/bin:$PATH"
export MAX_JOBS=${MAX_JOBS:-4}
/usr/bin/python3 -m venv --system-site-packages --without-pip "$ORIN_ENV"
# Local venv packages take precedence; the existing GPU runtime is read in place.
"$ORIN_ENV/bin/python3" -c 'import pathlib, sys, sysconfig; pathlib.Path(sysconfig.get_path("purelib"), "jetpack_pytorch.pth").write_text(sys.argv[1]+"\n")' "$ORIN_TORCH_SITE"
mkdir -p "$ORIN_PROJECT/logs/orin_deploy"
printf 'torch===%s\n' "$ORIN_TORCH_VERSION" > "$ORIN_PROJECT/logs/orin_deploy/torch-constraint.txt"
"$ORIN_ENV/bin/python3" -m pip install --disable-pip-version-check \
  -c "$ORIN_PROJECT/logs/orin_deploy/torch-constraint.txt" \
  'setuptools==69.5.1' 'wheel==0.43.0' ninja \
  -r "$ORIN_PROJECT/scripts/requirements_emsanet.txt"
if ! "$ORIN_ENV/bin/python3" -c 'import torch, torchvision; assert torchvision.extension._has_ops(); assert torchvision.__version__.startswith("0.20."); torchvision.ops.nms(torch.tensor([[0.,0.,1.,1.]], device="cuda"), torch.ones(1, device="cuda"), 0.5)'; then
  mkdir -p "$ORIN_VISION"
  tar -xzf "$ORIN_PROJECT/logs/orin_deploy/torchvision-v0.20.0.tar.gz" -C "$ORIN_VISION" --strip-components=1
  FORCE_CUDA=1 TORCH_CUDA_ARCH_LIST=8.7 TORCHVISION_USE_FFMPEG=0 TORCHVISION_USE_VIDEO_CODEC=0 \
    "$ORIN_ENV/bin/python3" -m pip install --no-deps --no-build-isolation "$ORIN_VISION"
fi
"$ORIN_ENV/bin/python3" -m pip install --no-deps --no-build-isolation \
  "$ORIN_SOURCE/lib/nicr-scene-analysis-datasets" \
  "$ORIN_SOURCE/lib/nicr-multitask-scene-analysis"
"$ORIN_ENV/bin/python3" -c 'import pathlib, sys, sysconfig; pathlib.Path(sysconfig.get_path("purelib"), "emsanet_source.pth").write_text(sys.argv[1]+"\n")' "$ORIN_SOURCE"
printf '%s  %s\n' 1814ef0b3173e6e9d16b11a66027ec41477b5dc100e9f8b100a91e2786bc951a \
  "$ORIN_PROJECT/models/emsanet/nyuv2_r34_NBt1D_pre.pth" | sha256sum --check
"$ORIN_ENV/bin/python3" -c 'import torch, torchvision, emsanet, nicr_mt_scene_analysis; assert torch.cuda.is_available(); print(torch.__version__, torchvision.__version__, torch.cuda.get_device_name(0))'
printf 'Orin EMSANet GPU 环境已配置。\n'
