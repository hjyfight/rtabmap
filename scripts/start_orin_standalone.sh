#!/usr/bin/env bash
set -eo pipefail
ORIN_STANDALONE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ORIN_STANDALONE_ROOT"
source setup.bash
source install/features/local_setup.bash
source install/semantic/local_setup.bash
exec /usr/bin/python3 scripts/orin_standalone.py "$@"
