#!/usr/bin/env bash
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/setup.bash"
set -euo pipefail
exec ros2 launch "$root/launch/depth_icp_slam.launch.py" "$@"
