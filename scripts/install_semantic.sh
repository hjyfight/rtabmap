#!/usr/bin/env bash
set -eo pipefail
SEMANTIC_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SEMANTIC_ENV="$SEMANTIC_PROJECT/.venv-semantic"
mkdir -p "$SEMANTIC_PROJECT/models"
python3 -m venv --system-site-packages --without-pip "$SEMANTIC_ENV"
if ! "$SEMANTIC_ENV/bin/python3" -m pip --version >/dev/null 2>&1; then
  curl --fail --location https://bootstrap.pypa.io/get-pip.py --output "$SEMANTIC_ENV/get-pip.py"
  "$SEMANTIC_ENV/bin/python3" "$SEMANTIC_ENV/get-pip.py"
fi
"$SEMANTIC_ENV/bin/python3" -m pip install --disable-pip-version-check \
  --index-url https://download.pytorch.org/whl/cpu torch==2.5.1 torchvision==0.20.1
"$SEMANTIC_ENV/bin/python3" -m pip install --disable-pip-version-check \
  'numpy==1.24.4' 'opencv-python==4.10.0.84' 'ultralytics==8.3.40'
if [[ ! -s "$SEMANTIC_PROJECT/models/yolov8n-seg.pt" ]]; then
  SEMANTIC_CURL_OPTIONS=(--retry 3 --retry-all-errors)
  if [[ -n "${SEMANTIC_DOWNLOAD_PROXY:-}" ]]; then
    SEMANTIC_CURL_OPTIONS+=(--proxy "$SEMANTIC_DOWNLOAD_PROXY")
  fi
  curl --fail --location "${SEMANTIC_CURL_OPTIONS[@]}" \
    https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n-seg.pt \
    --output "$SEMANTIC_PROJECT/models/yolov8n-seg.pt.part"
  mv "$SEMANTIC_PROJECT/models/yolov8n-seg.pt.part" "$SEMANTIC_PROJECT/models/yolov8n-seg.pt"
fi
printf 'YOLO environment ready: %s\n' "$SEMANTIC_ENV"
