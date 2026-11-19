#!/usr/bin/env bash
# WHEELTEC capture and YAMNet share the Orin perception profile and ROS domain.
set -eo pipefail
ORIN_AUDIO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ORIN_AUDIO_ROOT"
source setup.bash
if [[ ! -f install/features/local_setup.bash ]]; then
  printf '先运行 bash scripts/build_fourmodal_features.sh\n' >&2
  exit 1
fi
source install/features/local_setup.bash
exec /usr/bin/python3 scripts/orin_audio.py "$@"
