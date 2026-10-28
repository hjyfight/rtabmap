#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/../setup.bash"
exec ros2 launch "$RTABMAP_PROJECT_ROOT/launch/livox_slam.launch.py" "$@"
