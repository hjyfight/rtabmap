#!/usr/bin/env bash
# Use Ubuntu's ROS-matching OpenCV headers without replacing JetPack's OpenCV.
set -euo pipefail
ORIN_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ORIN_ARCH=$(dpkg-architecture -qDEB_HOST_MULTIARCH)
[[ $ORIN_ARCH == aarch64-linux-gnu ]] || { printf '此入口仅供 Orin ARM64。\n' >&2; exit 1; }
ORIN_CV_VERSION=$(dpkg-query -W -f='${Version}' libopencv-core4.5d)
ORIN_CV_ROOT="$ORIN_PROJECT/.tools/opencv-ros/root"
ORIN_CV_CACHE="$ORIN_PROJECT/.tools/opencv-ros/packages"
mkdir -p "$ORIN_CV_ROOT" "$ORIN_CV_CACHE"
mapfile -t ORIN_CV_PACKAGES < <(/usr/bin/python3 - "$ORIN_CV_VERSION" <<'PY'
import re, subprocess, sys
info = subprocess.check_output(['apt-cache', 'show', 'libopencv-dev=' + sys.argv[1]], text=True)
deps = next(line for line in info.splitlines() if line.startswith('Depends:'))
for name in sorted({'libopencv-dev', *re.findall(r'\blibopencv-[a-z0-9]+-dev\b', deps)}):
    print(name + '=' + sys.argv[1])
PY
)
(( ${#ORIN_CV_PACKAGES[@]} > 10 )) || { printf '未解析到完整的 OpenCV 开发包。\n' >&2; exit 1; }
ORIN_APT_OPTIONS=(-o Acquire::Retries=2 -o Acquire::http::Timeout=30)
if [[ -n ${ORIN_DOWNLOAD_PROXY:-} ]]; then
  ORIN_APT_OPTIONS+=(-o "Acquire::http::Proxy=$ORIN_DOWNLOAD_PROXY" -o "Acquire::https::Proxy=$ORIN_DOWNLOAD_PROXY")
fi
cd "$ORIN_CV_CACHE"
apt-get "${ORIN_APT_OPTIONS[@]}" download "${ORIN_CV_PACKAGES[@]}"
for ORIN_CV_DEB in ./*.deb; do
  dpkg-deb -x "$ORIN_CV_DEB" "$ORIN_CV_ROOT"
done
# Development symlinks expect their matching runtime in the same directory.
# ROS already installed that runtime in /usr/lib; reference it instead of copying.
/usr/bin/python3 - "$ORIN_CV_ROOT/usr/lib/$ORIN_ARCH" "/usr/lib/$ORIN_ARCH" <<'PY'
from pathlib import Path
import os, sys
local, system = map(Path, sys.argv[1:])
for runtime in system.glob('libopencv_*.so.4.5*'):
    if not (local / runtime.name).exists():
        (local / runtime.name).symlink_to(runtime)
for link in local.glob('libopencv_*.so'):
    if not link.is_symlink():
        continue
    target = os.readlink(link)
    if not (local / target).exists():
        runtime = system / target
        if not runtime.is_file():
            raise SystemExit(f'Missing ROS OpenCV runtime: {runtime}')
        (local / target).symlink_to(runtime)
PY
printf 'OpenCV %s 开发文件已部署到 %s；系统 OpenCV 保持原状。\n' "$ORIN_CV_VERSION" "$ORIN_CV_ROOT"
