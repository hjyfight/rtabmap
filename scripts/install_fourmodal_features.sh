#!/usr/bin/env bash
# Install the UniBind zero-shot backbone and CPU inference dependencies locally.
set -euo pipefail
FEATURE_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
FEATURE_REVISION=7b22a7790af38d25ffa5b12d49fbe786dc302e6e
FEATURE_ENV="$FEATURE_PROJECT/.venv-fourmodal"
FEATURE_SOURCE="$FEATURE_PROJECT/.tools/unibind"
FEATURE_CACHE="$FEATURE_PROJECT/.cache/fourmodal"
FEATURE_MODEL="$FEATURE_PROJECT/models/unibind/pretrained_weights.pt"
FEATURE_DOWNLOAD_WEIGHTS=true
FEATURE_INSTALL_DEPS=true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --proxy) export https_proxy="${2:?missing proxy}" http_proxy="$2"; shift 2 ;;
    --no-weights) FEATURE_DOWNLOAD_WEIGHTS=false; shift ;;
    --no-deps) FEATURE_INSTALL_DEPS=false; shift ;;
    *) printf 'Usage: %s [--proxy URL] [--no-weights] [--no-deps]\n' "$0" >&2; exit 2 ;;
  esac
done
export PYTHONNOUSERSITE=1 PIP_CACHE_DIR="$FEATURE_CACHE/pip"
mkdir -p "$FEATURE_CACHE" "$(dirname "$FEATURE_MODEL")"
if [[ ! -f "$FEATURE_SOURCE/.source-revision" ]]; then
  curl -fL --retry 3 --connect-timeout 20 --max-time 180 \
    "https://codeload.github.com/QC-LY/UniBind/tar.gz/$FEATURE_REVISION" \
    -o "$FEATURE_CACHE/source.tar.gz.part"
  mkdir -p "$FEATURE_SOURCE"
  tar -xzf "$FEATURE_CACHE/source.tar.gz.part" -C "$FEATURE_SOURCE" --strip-components=1
  printf '%s\n' "$FEATURE_REVISION" > "$FEATURE_SOURCE/.source-revision"
fi
[[ $(cat "$FEATURE_SOURCE/.source-revision") == "$FEATURE_REVISION" ]] || {
  printf 'UniBind source revision mismatch\n' >&2; exit 1;
}
if [[ ! -x "$FEATURE_ENV/bin/python3" ]]; then
  /usr/bin/python3 -m venv --system-site-packages --without-pip "$FEATURE_ENV"
fi
# Reuse the project's verified CPU torch; install additions into a separate venv.
FEATURE_BASE="$FEATURE_PROJECT/.venv-emsanet/bin/python3"
[[ -x "$FEATURE_BASE" ]] || { printf 'The project CPU torch environment is required\n' >&2; exit 1; }
"$FEATURE_BASE" - "$FEATURE_ENV" "$FEATURE_PROJECT" <<'PY'
import pathlib, sys, sysconfig
target = pathlib.Path(sys.argv[1]) / 'lib/python3.10/site-packages'
target.mkdir(parents=True, exist_ok=True)
(target / 'project_feature_runtime.pth').write_text(
    sysconfig.get_path('purelib') + '\n' + sys.argv[2] + '/src/rtabmap_features\n')
PY
if $FEATURE_INSTALL_DEPS; then
  "$FEATURE_BASE" -m pip install --disable-pip-version-check --no-deps \
    --target "$FEATURE_ENV/lib/python3.10/site-packages" \
    --index-url https://download.pytorch.org/whl/cpu 'torchaudio==2.5.1+cpu'
  "$FEATURE_BASE" -m pip install --disable-pip-version-check --no-deps \
    --target "$FEATURE_ENV/lib/python3.10/site-packages" \
    -r "$FEATURE_PROJECT/scripts/requirements_fourmodal.txt"
fi
if $FEATURE_DOWNLOAD_WEIGHTS; then
  FEATURE_VERIFIED=false
  for FEATURE_ATTEMPT in {1..20}; do
    if [[ -s "$FEATURE_MODEL" ]] && "$FEATURE_ENV/bin/python3" \
      "$FEATURE_PROJECT/scripts/verify_fourmodal_checkpoint.py" \
      --checkpoint "$FEATURE_MODEL" --revision "$FEATURE_REVISION"; then
      FEATURE_VERIFIED=true
      break
    fi
    # A proxy may end a response early while gdown reports success. Retain and
    # resume the incomplete bytes; only a validated checkpoint becomes usable.
    if [[ -s "$FEATURE_MODEL" ]]; then
      mv "$FEATURE_MODEL" "$FEATURE_MODEL.resume.part"
    fi
    printf 'Downloading/resuming official checkpoint, attempt %s/20\n' "$FEATURE_ATTEMPT"
    "$FEATURE_ENV/bin/python3" -m gdown --quiet --continue \
      1Dgmj7ajdoT8hYHobJQfgIuB4CbVRoKUn -O "$FEATURE_MODEL" || true
  done
  if ! $FEATURE_VERIFIED && [[ -s "$FEATURE_MODEL" ]]; then
    "$FEATURE_ENV/bin/python3" "$FEATURE_PROJECT/scripts/verify_fourmodal_checkpoint.py" \
      --checkpoint "$FEATURE_MODEL" --revision "$FEATURE_REVISION" && FEATURE_VERIFIED=true
  fi
  $FEATURE_VERIFIED || { printf 'Incomplete checkpoint retained; rerun this installer to resume\n' >&2; exit 1; }
fi
printf 'Runtime: %s\nSource: %s\nCheckpoint: %s\n' "$FEATURE_ENV" "$FEATURE_SOURCE" "$FEATURE_MODEL"
