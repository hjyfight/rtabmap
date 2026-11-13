#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/../setup.bash"
# Keep test traffic on this computer and away from the default robot domain.
export ROS_DOMAIN_ID="${RTABMAP_TEST_DOMAIN_ID:-87}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=${RTABMAP_TEST_RMW:-rmw_fastrtps_cpp}
if [[ $RMW_IMPLEMENTATION == rmw_cyclonedds_cpp ]]; then
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
fi
export OMP_NUM_THREADS=2
for package in rtabmap_slam rtabmap_odom rtabmap_launch rtabmap_viz rtabmap_msgs; do
    ros2 pkg prefix "$package"
done
ros2 launch "$RTABMAP_PROJECT_ROOT/launch/livox_slam.launch.py" --show-args
exec /usr/bin/python3 "$script_dir/synthetic_slam_test.py"
