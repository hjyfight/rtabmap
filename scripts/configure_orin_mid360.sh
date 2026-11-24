#!/usr/bin/env bash
# Run on Orin after plugging its Ethernet adapter into the MID360 network.
set -eo pipefail
ORIN_MID360_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ORIN_MID360_ROOT"
ORIN_MID360_INTERFACE=${1:-}
if [[ -z $ORIN_MID360_INTERFACE ]]; then
  mapfile -t ORIN_MID360_CANDIDATES < <(python3 - <<'PY'
from pathlib import Path
for node in Path('/sys/class/net').iterdir():
 if (node.name.startswith(('eth','en')) and not (node/'wireless').exists()):print(node.name)
PY
  )
  [[ ${#ORIN_MID360_CANDIDATES[@]} == 1 ]] || {
    printf '请指定雷达以太网接口：bash scripts/configure_orin_mid360.sh 接口名\n'; ip -br -4 addr; exit 1;
  }
  ORIN_MID360_INTERFACE=${ORIN_MID360_CANDIDATES[0]}
fi
[[ $ORIN_MID360_INTERFACE =~ ^[A-Za-z0-9_.:-]+$ && -d /sys/class/net/$ORIN_MID360_INTERFACE && ! -d /sys/class/net/$ORIN_MID360_INTERFACE/wireless ]] || {
  printf '需要实际连接 MID360 的以太网接口。\n' >&2; exit 1;
}
ORIN_MID360_PROFILE="orin-mid360-$ORIN_MID360_INTERFACE"
if ! nmcli connection show "$ORIN_MID360_PROFILE" >/dev/null 2>&1; then
  sudo nmcli connection add type ethernet ifname "$ORIN_MID360_INTERFACE" con-name "$ORIN_MID360_PROFILE" \
    ipv4.method manual ipv4.addresses 192.168.123.30/24 ipv4.never-default yes ipv6.method disabled
else
  sudo nmcli connection modify "$ORIN_MID360_PROFILE" connection.interface-name "$ORIN_MID360_INTERFACE" \
    ipv4.method manual ipv4.addresses 192.168.123.30/24 ipv4.never-default yes
fi
sudo nmcli connection up "$ORIN_MID360_PROFILE"
python3 - "$ORIN_MID360_INTERFACE" <<'PY'
import json,sys
from pathlib import Path
path=Path('config/orin_direct_research.json');data=json.loads(path.read_text())
data['local_sensors']['lidar_interface']=sys.argv[1]
path.write_text(json.dumps(data,indent=2)+'\n')
print('MID360 interface configured: '+sys.argv[1]+' 192.168.123.30/24; LiDAR 192.168.123.20')
PY
