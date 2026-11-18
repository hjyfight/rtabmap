#!/usr/bin/env bash
# Model, UI and isolated ROS checks; never publish test audio in the live domain.
set -eo pipefail
YAMNET_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$YAMNET_PROJECT"
source setup.bash
source install/features/local_setup.bash
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
.venv-yamnet/bin/python3 -m unittest discover -s tests -p test_yamnet.py -v
.venv-yamnet/bin/python3 -m unittest discover -s tests -p test_miti_frontend_install.py -v
.tools/miti/root/usr/lib/miti/node/bin/node tests/test_miti_audio_events.js
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
.venv-yamnet/bin/python3 tests/yamnet_ros_check.py "$@"
