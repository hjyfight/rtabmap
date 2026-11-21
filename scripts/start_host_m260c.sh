#!/usr/bin/env bash
# The M260C is plugged into the workstation; publish into the unified 3000 UI domain.
set -eo pipefail
M260C_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$M260C_PROJECT"
source setup.bash
source install/features/local_setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_LOCALHOST_ONLY=0
if [[ -z ${CYCLONEDDS_URI:-} ]]; then
  if [[ -d /sys/class/net/enxec1ac312a316 &&
        $(cat /sys/class/net/enxec1ac312a316/operstate) == up &&
        -n $(ip -4 -o addr show dev enxec1ac312a316) ]]; then
    export CYCLONEDDS_URI="file://$M260C_PROJECT/config/wired_cyclonedds.xml"
  else
    export CYCLONEDDS_URI="file://$M260C_PROJECT/config/hotspot_sensor_cyclonedds.xml"
  fi
fi
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
exec .venv-yamnet/bin/python3 scripts/orin_audio.py --host --localization "$@"
