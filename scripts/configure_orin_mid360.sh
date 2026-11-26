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
mapfile -t ORIN_MID360_ADDRESSES < <(python3 - <<'PY'
import ipaddress,json
from pathlib import Path
profile=json.loads(Path('config/orin_direct_research.json').read_text())['local_sensors']
host=ipaddress.IPv4Address(profile['lidar_host_ip'])
lidar=ipaddress.IPv4Address(profile['lidar_ip'])
network=ipaddress.IPv4Network(str(host)+'/24',strict=False)
if host == lidar or any(address not in network or address in (network.network_address,network.broadcast_address)
                        for address in (host,lidar)):
    raise ValueError('MID360 and its host must use distinct usable addresses in the same /24 subnet')
config=json.loads(Path(profile['lidar_config']).read_text())
destinations=config['MID360']['host_net_info']
if not all(destinations[key] == str(host) for key in ('cmd_data_ip','push_msg_ip','point_data_ip','imu_data_ip')):
    raise ValueError('MID360 host destinations do not match the direct sensor profile')
if not any(row['ip'] == str(lidar) for row in config['lidar_configs']):
    raise ValueError('MID360 target IP does not match the direct sensor profile')
print(str(host)+'/24')
print(lidar)
PY
)
[[ ${#ORIN_MID360_ADDRESSES[@]} == 2 ]] || { printf 'MID360 地址配置无效。\n' >&2; exit 1; }
ORIN_MID360_HOST_CIDR=${ORIN_MID360_ADDRESSES[0]}
ORIN_MID360_PROFILE="orin-mid360-$ORIN_MID360_INTERFACE"
if ! nmcli connection show "$ORIN_MID360_PROFILE" >/dev/null 2>&1; then
  sudo nmcli connection add type ethernet ifname "$ORIN_MID360_INTERFACE" con-name "$ORIN_MID360_PROFILE" \
    ipv4.method manual ipv4.addresses "$ORIN_MID360_HOST_CIDR" ipv4.never-default yes ipv6.method disabled
else
  sudo nmcli connection modify "$ORIN_MID360_PROFILE" connection.interface-name "$ORIN_MID360_INTERFACE" \
    ipv4.method manual ipv4.addresses "$ORIN_MID360_HOST_CIDR" ipv4.gateway "" ipv4.never-default yes ipv6.method disabled
fi
sudo nmcli connection up "$ORIN_MID360_PROFILE"
python3 - "$ORIN_MID360_INTERFACE" <<'PY'
import json,sys
from pathlib import Path
path=Path('config/orin_direct_research.json');data=json.loads(path.read_text())
data['local_sensors']['lidar_interface']=sys.argv[1]
path.write_text(json.dumps(data,indent=2)+'\n')
print('MID360 interface configured: '+sys.argv[1]+' '+data['local_sensors']['lidar_host_ip']+'/24; LiDAR '+data['local_sensors']['lidar_ip'])
PY
