#!/usr/bin/env bash
# Project-local installation of official prebuilt packages; no sudo or npm needed.
set -eo pipefail
MITI_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
MITI_BASE="$MITI_PROJECT/.tools/miti"
MITI_VERSION=3.3.6
MITI_SHA256=69b996bff7d9fc2567d16433d7e025f7d4488663d4466c85470d67fa8ebc68d4
if [[ $(dpkg --print-architecture) != amd64 || ! -f /opt/ros/humble/setup.bash ]]; then
  printf '此安装入口适用于 Ubuntu 22.04 amd64 + ROS 2 Humble。\n' >&2
  exit 1
fi
mkdir -p "$MITI_BASE/downloads" "$MITI_BASE/root"
cd "$MITI_BASE/downloads"
MITI_DEB="miti_${MITI_VERSION}_amd64.deb"
if [[ ! -f "$MITI_DEB" ]]; then
  curl -fL --retry 2 --connect-timeout 20 \
    "https://hiran-t.github.io/MITI/pool/main/m/miti/$MITI_DEB" -o "$MITI_DEB.part"
  printf '%s  %s\n' "$MITI_SHA256" "$MITI_DEB.part" | sha256sum --check
  mv "$MITI_DEB.part" "$MITI_DEB"
fi
printf '%s  %s\n' "$MITI_SHA256" "$MITI_DEB" | sha256sum --check
# apt-get download verifies packages against the local apt index. Cached packages permit
# offline reinstallation on this machine; the installed ROS desktop supplies other libraries.
for MITI_PKG in ros-humble-foxglove-bridge ros-humble-rosx-introspection rapidjson-dev; do
  if ! compgen -G "${MITI_PKG}_*.deb" >/dev/null; then
    apt-get download "$MITI_PKG"
  fi
done
for MITI_PACKAGE in "$MITI_DEB" ros-humble-*.deb rapidjson-dev_*.deb; do
  dpkg-deb --extract "$MITI_PACKAGE" "$MITI_BASE/root"
done
sha256sum ./*.deb > "$MITI_BASE/package-sha256.txt"
cp "$MITI_PROJECT/config/miti_live.html" "$MITI_BASE/root/usr/lib/miti/public/rtabmap.html"
cp "$MITI_PROJECT/config/miti_live_layout.json" "$MITI_BASE/root/usr/lib/miti/public/rtabmap-layout.json"
source "$MITI_PROJECT/scripts/setup_miti.bash"
python3 "$MITI_PROJECT/scripts/install_miti_loop_panel.py"
MITI_MISSING=$(ldd "$MITI_ROS_PREFIX/lib/foxglove_bridge/foxglove_bridge" | sed -n '/not found/p')
if [[ -n "$MITI_MISSING" ]]; then
  printf '缺少运行库：\n%s\n' "$MITI_MISSING" >&2
  exit 1
fi
"$MITI_APP/node/bin/node" -p "'MITI ' + require('$MITI_APP/package.json').version"
ros2 pkg prefix foxglove_bridge
printf '安装完成。启动：bash %s/scripts/start_miti.sh\n' "$MITI_PROJECT"
