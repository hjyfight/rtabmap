#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/../setup.bash"
# Keep test traffic on this computer and away from the default robot domain.
export ROS_DOMAIN_ID="${RTABMAP_TEST_DOMAIN_ID:-90}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OMP_NUM_THREADS=2
ros2 launch "$RTABMAP_PROJECT_ROOT/launch/go2_slam.launch.py" --show-args >/dev/null
exec /usr/bin/python3 "$script_dir/test_go2_fusion.py"
