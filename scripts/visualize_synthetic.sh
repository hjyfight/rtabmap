#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/../setup.bash"
export ROS_DOMAIN_ID="${RTABMAP_VIS_DOMAIN_ID:-94}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OMP_NUM_THREADS=2
exec /usr/bin/python3 "$script_dir/visualize_synthetic.py"
