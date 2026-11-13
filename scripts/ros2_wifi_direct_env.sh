#!/usr/bin/env bash
# Source after the ROS setup on either machine: local interface, peer Wi-Fi IPv4.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf '请使用 source scripts/ros2_wifi_direct_env.sh 本机无线网卡 对端无线IPv4\n' >&2
  exit 1
fi

_go2_wifi_direct_env() {
  if [[ $# != 2 ]]; then
    printf '用法：source ros2_wifi_direct_env.sh 本机无线网卡 对端无线IPv4\n' >&2
    return 1
  fi
  local nic=$1 peer=$2 octet addresses
  local -a octets
  if [[ ! $nic =~ ^[a-zA-Z0-9_.:-]+$ || ! -d /sys/class/net/$nic ]]; then
    printf '网卡不存在或名称无效：%s；请先执行 ip -br -4 addr。\n' "$nic" >&2
    return 1
  fi
  if [[ ! $peer =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]]; then
    printf '请填写对端实际的无线 IPv4 地址：%s\n' "$peer" >&2
    return 1
  fi
  IFS=. read -r -a octets <<< "$peer"
  for octet in "${octets[@]}"; do
    if (( 10#$octet > 255 )) || [[ ${#octet} -gt 1 && $octet == 0* ]]; then
      printf '无效 IPv4 地址：%s\n' "$peer" >&2
      return 1
    fi
  done
  addresses=$(ip -4 -o addr show dev "$nic") || return 1
  if [[ -z $addresses ]]; then
    printf '网卡 %s 还没有 IPv4 地址，请先连接 Wi-Fi。\n' "$nic" >&2
    return 1
  fi

  export ROS_DOMAIN_ID=0
  export ROS_LOCALHOST_ONLY=0
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  # Explicit peers avoid relying on the access point forwarding multicast.
  # Loopback also lets drivers, SLAM and visualization discover local nodes.
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$nic\"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer Address=\"$peer\"/><Peer Address=\"127.0.0.1\"/></Peers></Discovery></Domain></CycloneDDS>"
  printf '已加载无线 DDS：%s -> %s，ROS domain 0。\n' "$nic" "$peer"
}

if _go2_wifi_direct_env "$@"; then
  unset -f _go2_wifi_direct_env
else
  unset -f _go2_wifi_direct_env
  return 1
fi
