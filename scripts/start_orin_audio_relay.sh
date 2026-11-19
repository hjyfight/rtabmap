#!/usr/bin/env bash
# Run on the workstation to forward Orin audio into the existing domain-0 UI.
set -eo pipefail
ORIN_AUDIO_RELAY_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ORIN_AUDIO_RELAY_ROOT"
source setup.bash
source install/features/local_setup.bash
exec /usr/bin/python3 scripts/relay_orin_audio.py "$@"
