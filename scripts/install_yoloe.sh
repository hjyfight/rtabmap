#!/usr/bin/env bash
# Independent CPU runtime; never install packages into .venv-semantic.
set -eo pipefail
YOLOE_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
YOLOE_ENV="$YOLOE_PROJECT/.venv-yoloe"
YOLOE_MODEL="$YOLOE_PROJECT/models/yoloe-11s-seg-pf.pt"
YOLOE_SHA256=292bdf157a9ec7315f34b567cb93467c5043cd1889a1cc18abbfdeb88d7a948d
export PIP_CACHE_DIR="$YOLOE_PROJECT/.cache/yoloe-pip"
export PYTHONNOUSERSITE=1
mkdir -p "$YOLOE_PROJECT/models" "$PIP_CACHE_DIR"
python3 -m venv --system-site-packages --without-pip "$YOLOE_ENV"
if ! "$YOLOE_ENV/bin/python3" -m pip --version >/dev/null 2>&1; then
  curl --fail --location https://bootstrap.pypa.io/get-pip.py --output "$YOLOE_ENV/get-pip.py"
  "$YOLOE_ENV/bin/python3" "$YOLOE_ENV/get-pip.py"
fi
"$YOLOE_ENV/bin/python3" -m pip install --disable-pip-version-check \
  --index-url https://download.pytorch.org/whl/cpu torch==2.5.1+cpu torchvision==0.20.1+cpu
# NumPy 1.x and this OpenCV version retain ROS Humble/cv_bridge compatibility.
"$YOLOE_ENV/bin/python3" -m pip install --disable-pip-version-check \
  --index-url https://pypi.org/simple \
  numpy==1.24.4 opencv-python==4.10.0.84 ultralytics==8.4.198
if [[ ! -s "$YOLOE_MODEL" ]]; then
  YOLOE_CURL_OPTIONS=(--retry 3 --retry-all-errors)
  if [[ -n ${SEMANTIC_DOWNLOAD_PROXY:-} ]]; then
    YOLOE_CURL_OPTIONS+=(--proxy "$SEMANTIC_DOWNLOAD_PROXY")
  fi
  curl --fail --location "${YOLOE_CURL_OPTIONS[@]}" \
    https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-11s-seg-pf.pt \
    --output "$YOLOE_MODEL.part"
  printf '%s  %s\n' "$YOLOE_SHA256" "$YOLOE_MODEL.part" | sha256sum --check --status
  mv "$YOLOE_MODEL.part" "$YOLOE_MODEL"
fi
printf '%s  %s\n' "$YOLOE_SHA256" "$YOLOE_MODEL" | sha256sum --check
printf 'YOLOE 无提示环境已安装：%s\n权重：%s\n' "$YOLOE_ENV" "$YOLOE_MODEL"
