#!/usr/bin/env bash
# System dependencies for Ubuntu 22.04 Jetson; leaves JetPack/CUDA/PyTorch intact.
set -euo pipefail
ORIN_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ORIN_ROS_KEY=${1:-"$ORIN_PROJECT/logs/orin_deploy/ros.key"}
source /etc/os-release
if [[ ${ID:-} != ubuntu || ${VERSION_ID:-} != 22.04 || $(dpkg --print-architecture) != arm64 ]]; then
  printf '需要 Ubuntu 22.04 arm64（JetPack 6）。\n' >&2
  exit 1
fi
if (( EUID != 0 )); then
  printf '请执行 sudo bash %q %q\n' "${BASH_SOURCE[0]}" "$ORIN_ROS_KEY" >&2
  exit 1
fi
[[ -s $ORIN_ROS_KEY ]] || { printf '缺少 ROS 官方签名公钥：%s\n' "$ORIN_ROS_KEY" >&2; exit 1; }
install -m 0644 "$ORIN_ROS_KEY" /usr/share/keyrings/rtabmap-orin-ros.gpg
ORIN_APT_SOURCE='deb [arch=arm64 signed-by=/usr/share/keyrings/rtabmap-orin-ros.gpg] http://packages.ros.org/ros2/ubuntu jammy main'
if [[ -f /etc/apt/sources.list.d/rtabmap-orin.list ]] &&
   [[ $(cat /etc/apt/sources.list.d/rtabmap-orin.list) != "$ORIN_APT_SOURCE" ]]; then
  printf 'rtabmap-orin.list 已有不同配置，请先核对。\n' >&2
  exit 1
fi
printf '%s\n' "$ORIN_APT_SOURCE" > /etc/apt/sources.list.d/rtabmap-orin.list
ORIN_APT_OPTIONS=(-o Acquire::Retries=3 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30)
apt-get "${ORIN_APT_OPTIONS[@]}" update
# Install project dependencies only; no dist-upgrade or JetPack replacement.
DEBIAN_FRONTEND=noninteractive apt-get "${ORIN_APT_OPTIONS[@]}" install -y --no-install-recommends \
  ros-humble-ros-base ros-humble-rtabmap-ros ros-humble-rmw-cyclonedds-cpp \
  ros-humble-depth-image-proc ros-humble-image-transport-plugins \
  ros-humble-imu-filter-madgwick ros-humble-foxglove-bridge \
  ros-humble-realsense2-camera ros-humble-realsense2-description \
  python3-colcon-common-extensions python3-rosdep python3-venv python3-pip \
  python3-numpy python3-scipy python3-opencv python3-yaml python3-websocket \
  build-essential cmake ninja-build pkg-config libjpeg-dev libpng-dev \
  libopenblas-dev curl rsync
printf 'Orin 系统依赖安装完成；接下来以普通用户编译项目和配置 GPU 推理环境。\n'
