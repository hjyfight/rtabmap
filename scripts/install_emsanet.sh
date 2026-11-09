#!/usr/bin/env bash
# Install a pinned RGB-D inference runtime without changing either YOLO environment.
set -euo pipefail
if [[ "${1:-}" == --proxy ]]; then
  EMSANET_DOWNLOAD_PROXY=${2:?Usage: install_emsanet.sh --proxy http://127.0.0.1:7897}
  shift 2
fi
if [[ $# != 0 ]]; then
  printf 'Usage: bash scripts/install_emsanet.sh [--proxy URL]\n' >&2
  exit 2
fi
if [[ -n "${EMSANET_DOWNLOAD_PROXY:-}" ]]; then
  export http_proxy="$EMSANET_DOWNLOAD_PROXY" https_proxy="$EMSANET_DOWNLOAD_PROXY"
  export HTTP_PROXY="$EMSANET_DOWNLOAD_PROXY" HTTPS_PROXY="$EMSANET_DOWNLOAD_PROXY"
  export PIP_PROXY="$EMSANET_DOWNLOAD_PROXY"
fi
EMSANET_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
EMSANET_ENV="$EMSANET_PROJECT/.venv-emsanet"
EMSANET_SOURCE="$EMSANET_PROJECT/.tools/emsanet"
EMSANET_MODELS="$EMSANET_PROJECT/models/emsanet"
export PYTHONNOUSERSITE=1
export PIP_CACHE_DIR="$EMSANET_PROJECT/.cache/emsanet-pip"
mkdir -p "$EMSANET_MODELS" "$PIP_CACHE_DIR"

fetch_source() {
  local repository=$1 revision=$2 destination=$3
  if [[ -f "$destination/.source-revision" ]]; then
    [[ $(cat "$destination/.source-revision") == "$revision" ]] || {
      printf 'Source revision mismatch: %s\n' "$destination" >&2
      exit 1
    }
    return
  fi
  local archive="$PIP_CACHE_DIR/$repository-$revision.tar.gz"
  curl -fL --retry 3 --connect-timeout 20 --max-time 180 \
    "https://codeload.github.com/TUI-NICR/$repository/tar.gz/$revision" -o "$archive.part"
  mv "$archive.part" "$archive"
  mkdir -p "$destination"
  tar -xzf "$archive" -C "$destination" --strip-components=1
  printf '%s\n' "$revision" > "$destination/.source-revision"
}

# Official 2023 release and its exact submodule revisions, compatible with Python 3.10.
fetch_source EMSANet 9d2ade475ba44c6bd82f6fac6d44ac82086bcd53 "$EMSANET_SOURCE"
fetch_source nicr-scene-analysis-datasets 90344ecea1209ab9a5cf7ae8cf4d936e5835a8f0 \
  "$EMSANET_SOURCE/lib/nicr-scene-analysis-datasets"
fetch_source nicr-multitask-scene-analysis 834bd58c3e79f30153525950057a45a33f446f0f \
  "$EMSANET_SOURCE/lib/nicr-multitask-scene-analysis"

/usr/bin/python3 -m venv --system-site-packages --without-pip "$EMSANET_ENV"
if ! "$EMSANET_ENV/bin/python3" -m pip --version >/dev/null 2>&1; then
  curl -fL --retry 3 --connect-timeout 20 --max-time 120 \
    https://bootstrap.pypa.io/get-pip.py -o "$EMSANET_ENV/get-pip.py"
  "$EMSANET_ENV/bin/python3" "$EMSANET_ENV/get-pip.py"
fi
"$EMSANET_ENV/bin/python3" -m pip install --disable-pip-version-check \
  'setuptools==69.5.1' 'wheel==0.43.0'
"$EMSANET_ENV/bin/python3" -m pip install --disable-pip-version-check \
  --index-url https://download.pytorch.org/whl/cpu 'torch==2.5.1' 'torchvision==0.20.1'
"$EMSANET_ENV/bin/python3" -m pip install --disable-pip-version-check \
  -r "$EMSANET_PROJECT/scripts/requirements_emsanet.txt"
"$EMSANET_ENV/bin/python3" -m pip install --no-deps --no-build-isolation \
  "$EMSANET_SOURCE/lib/nicr-scene-analysis-datasets" \
  "$EMSANET_SOURCE/lib/nicr-multitask-scene-analysis"
"$EMSANET_ENV/bin/python3" -c \
  'import pathlib,sys,sysconfig; pathlib.Path(sysconfig.get_path("purelib"),"emsanet_source.pth").write_text(sys.argv[1]+"\n")' \
  "$EMSANET_SOURCE"

if [[ ! -s "$EMSANET_MODELS/nyuv2_r34_NBt1D_pre.pth" ]]; then
  # Official NYUv2 checkpoint, initialized by Hypersim pretraining.
  EMSANET_ARCHIVE="$EMSANET_MODELS/nyuv2_r34_NBt1D_pre.tar.gz"
  for EMSANET_ATTEMPT in 1 2 3 4 5; do
    if [[ -s "$EMSANET_ARCHIVE" ]] && gzip -t "$EMSANET_ARCHIVE" 2>/dev/null; then
      break
    fi
    # Some proxies close a response early without gdown reporting an error.
    # A .part name lets gdown actually resume it instead of skipping it.
    if [[ -s "$EMSANET_ARCHIVE" ]]; then
      mv "$EMSANET_ARCHIVE" "$EMSANET_ARCHIVE.resume.part"
    fi
    "$EMSANET_ENV/bin/python3" -m gdown --continue \
      1QbOJXVrOzsVM8ltX7AxqFSVLsY6vzvNX -O "$EMSANET_ARCHIVE" || true
  done
  if ! gzip -t "$EMSANET_ARCHIVE"; then
    printf '权重下载不完整，已保留断点；请重新运行本安装脚本。\n' >&2
    exit 1
  fi
  tar -xOf "$EMSANET_ARCHIVE" nyuv2/r34_NBt1D_pre.pth \
    > "$EMSANET_MODELS/nyuv2_r34_NBt1D_pre.pth.part"
  mv "$EMSANET_MODELS/nyuv2_r34_NBt1D_pre.pth.part" "$EMSANET_MODELS/nyuv2_r34_NBt1D_pre.pth"
fi
printf '%s  %s\n' \
  1814ef0b3173e6e9d16b11a66027ec41477b5dc100e9f8b100a91e2786bc951a \
  "$EMSANET_MODELS/nyuv2_r34_NBt1D_pre.pth" | sha256sum --check
printf 'EMSANet runtime ready: %s\nWeights: %s\n' \
  "$EMSANET_ENV" "$EMSANET_MODELS/nyuv2_r34_NBt1D_pre.pth"
