#!/usr/bin/env bash
# Install the official ROS package into this project's optional dependency prefix.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
dependency_root="$project_root/.tools/depth_registration"
mkdir -p "$dependency_root/packages" "$dependency_root/root"
download_dir="$(mktemp -d "$dependency_root/packages/download.XXXXXX")"
trap 'rm -rf -- "$download_dir"' EXIT
cd "$download_dir"
apt-get download ros-humble-depth-image-proc
for archive in ./*.deb; do
    dpkg-deb -x "$archive" "$dependency_root/root"
    mv -- "$archive" "$dependency_root/packages/"
done
printf '深度配准已安装。请重新 source %s/setup.bash\n' "$project_root"
