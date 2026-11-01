#!/usr/bin/env bash
# Source from bash. Keep the portable packages ahead of /opt/ros without changing it.
MITI_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$MITI_PROJECT/setup.bash" || return 1
if [[ -f "$MITI_PROJECT/install/semantic/setup.bash" ]]; then
  source "$MITI_PROJECT/install/semantic/setup.bash" || return 1
fi
export MITI_APP="$MITI_PROJECT/.tools/miti/root/usr/lib/miti"
export MITI_ROS_PREFIX="$MITI_PROJECT/.tools/miti/root/opt/ros/humble"
export AMENT_PREFIX_PATH="$MITI_ROS_PREFIX:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$MITI_ROS_PREFIX/lib:${LD_LIBRARY_PATH:-}"
export PATH="$MITI_APP/node/bin:$PATH"
