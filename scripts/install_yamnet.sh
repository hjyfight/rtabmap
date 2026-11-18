#!/usr/bin/env bash
# Install the pinned official 521-class TFLite model in an isolated CPU environment.
set -euo pipefail
YAMNET_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$YAMNET_PROJECT"
YAMNET_ENV="$YAMNET_PROJECT/.venv-yamnet"
YAMNET_MODEL="$YAMNET_PROJECT/models/yamnet/yamnet.tflite"
YAMNET_SHA=4d8b4a53282dc83ef04e3e7dbc4fbc98082e34e44ed798e16c3a0cdd4c584faf
YAMNET_INSTALL_DEPS=true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-deps) YAMNET_INSTALL_DEPS=false; shift ;;
    --proxy) export https_proxy="${2:?missing proxy}" http_proxy="$2"; shift 2 ;;
    *) printf 'Usage: %s [--no-deps] [--proxy URL]\n' "$0" >&2; exit 2 ;;
  esac
done
export PYTHONNOUSERSITE=1 PIP_CACHE_DIR="$YAMNET_PROJECT/.cache/yamnet/pip"
mkdir -p models/yamnet "$PIP_CACHE_DIR"
if [[ ! -x "$YAMNET_ENV/bin/python3" ]]; then
  /usr/bin/python3 -m venv --system-site-packages --without-pip "$YAMNET_ENV"
fi
if $YAMNET_INSTALL_DEPS; then
  YAMNET_PIP_PYTHON=${YAMNET_PIP_PYTHON:-"$YAMNET_PROJECT/.venv-emsanet/bin/python3"}
  [[ -x "$YAMNET_PIP_PYTHON" ]] || { printf '设置 YAMNET_PIP_PYTHON 为已有 pip 的 Python 解释器。\n' >&2; exit 1; }
  "$YAMNET_PIP_PYTHON" -m pip --python "$YAMNET_ENV/bin/python3" install \
    --disable-pip-version-check -r scripts/requirements_yamnet.txt
fi
if [[ ! -s "$YAMNET_MODEL" ]] || [[ $(sha256sum "$YAMNET_MODEL" | cut -d ' ' -f 1) != "$YAMNET_SHA" ]]; then
  curl -fL --retry 3 --connect-timeout 20 --max-time 120 \
    https://storage.googleapis.com/mediapipe-models/audio_classifier/yamnet/float32/1/yamnet.tflite \
    -o "$YAMNET_MODEL.part"
  [[ $(sha256sum "$YAMNET_MODEL.part" | cut -d ' ' -f 1) == "$YAMNET_SHA" ]] || {
    printf 'YAMNet 模型校验失败，下载文件保留为 .part。\n' >&2; exit 1;
  }
  mv "$YAMNET_MODEL.part" "$YAMNET_MODEL"
fi
"$YAMNET_ENV/bin/python3" scripts/predict_yamnet_audio.py --check
printf '声音识别环境已就绪。启动：bash scripts/start_yamnet_audio.sh\n'
