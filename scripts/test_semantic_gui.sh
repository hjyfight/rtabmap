#!/usr/bin/env bash
set -eo pipefail
SEMANTIC_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$SEMANTIC_PROJECT/install/semantic/setup.bash"
SEMANTIC_XVFB=${1:-${SEMANTIC_XVFB:-$(command -v Xvfb || true)}}
if [[ -z "$SEMANTIC_XVFB" ]]; then
  printf '需要 Xvfb；可通过 SEMANTIC_XVFB 指定可执行文件。\n' >&2
  exit 1
fi
mkdir -p "$SEMANTIC_PROJECT/logs/semantic_validation"
SEMANTIC_DISPLAY_FILE=$(mktemp /tmp/semantic_display_XXXXXX)
"$SEMANTIC_XVFB" -displayfd 3 -screen 0 1280x900x24 -nolisten tcp -ac \
  3>"$SEMANTIC_DISPLAY_FILE" >"$SEMANTIC_PROJECT/logs/semantic_validation/xvfb.log" 2>&1 &
SEMANTIC_XVFB_PID=$!
trap 'kill "$SEMANTIC_XVFB_PID" 2>/dev/null || true' EXIT
for SEMANTIC_WAIT in {1..50}; do
  [[ -s "$SEMANTIC_DISPLAY_FILE" ]] && break
  sleep 0.1
done
if [[ ! -s "$SEMANTIC_DISPLAY_FILE" ]]; then
  cat "$SEMANTIC_PROJECT/logs/semantic_validation/xvfb.log" >&2
  exit 1
fi
export DISPLAY=":$(cat "$SEMANTIC_DISPLAY_FILE")"
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1
timeout 45 "$SEMANTIC_PROJECT/install/semantic/rtabmap_viz/lib/rtabmap_viz/semantic_gui_check" \
  "$SEMANTIC_PROJECT/logs/semantic_validation/native_gui.png"
