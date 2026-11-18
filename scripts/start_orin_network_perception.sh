#!/usr/bin/env bash
# Receiver/perception launcher only; no robot SSH, driver or control commands.
set -eo pipefail
ORIN_NETWORK_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ORIN_NETWORK_ROOT"
source setup.bash
for ORIN_NETWORK_OVERLAY in features segmentation semantic; do
  if [[ -f install/$ORIN_NETWORK_OVERLAY/local_setup.bash ]]; then
    source "install/$ORIN_NETWORK_OVERLAY/local_setup.bash"
  fi
done
exec /usr/bin/python3 scripts/orin_network_perception.py "$@"
