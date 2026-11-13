# 主机与机器狗通过同一路由器无线建图

两端都连接同一个无线局域网，就可以移除工作站到机器狗的网线。推荐使用同一台
5 GHz 路由器的普通局域网，关闭 AP/客户端隔离，不使用隔离的访客网络。
不要求路由器能上互联网。同一子网内互通不需要原先的跨网段转发和静态路由。

```text
工作站（SLAM、语义模型、MITI）
        │ Wi-Fi
      无线路由器
        │ Wi-Fi
机器狗机载电脑（Livox / RealSense 驱动）
        ├── 网线 ── MID360
        └── USB ─── RealSense
```

MID360 到机载电脑的网线及 RealSense 的 USB 仍需保留。无线连接的是机载电脑与工作站。
这里配置的是建图传感器链路，不需要修改机器狗运动控制网络。

## 当前主机状态（2026-11-11）

| 项目 | 实际读到的状态 |
| --- | --- |
| 工作站无线接口 | `wlx6c1ff712636d` |
| 当前 Wi-Fi | `TP-LINK_6E07_2.4G`，2.4 GHz |
| 工作站无线 IPv4 | `192.168.1.100/24` |
| 无线路由器 | `192.168.1.1` |
| 原 USB 有线接口 | `enxec1ac312a316`，已断开 |
| 工作站另一个有线接口 | `eno1`，`172.18.2.79/24`，当前默认上网路由 |
| 机载电脑无线 IP | 尚未确认，需要在机载电脑或路由器客户端列表查看 |

这些是本次读取值，DHCP 地址可能变化。建议在路由器中为两端设置 DHCP 地址保留。
`eno1` 可以继续上网，下面的 DDS 配置明确绑定无线接口，传感器通信无需经过它。
若主机连上网也要完全无线，先确认 Wi-Fi 能满足上网需求，再断开 `eno1` 的网线。

旧 `~/ros2_wifi_cyclonedds.xml` 仍绑定 `enxec1ac312a316`，对端仍是旧热点地址
`172.29.38.125`，不适用于当前连接。本流程使用仓库内独立的
`scripts/ros2_wifi_direct_env.sh`，参数依次为**本机网卡、对端无线 IP**。
它只设置当前终端的 ROS/DDS 环境，不修改 NetworkManager、路由或家目录配置。

## 1. 让机载电脑接入同一 Wi-Fi

在机载电脑的本地终端（显示器/键盘）或仍可用的远程终端执行：

```bash
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
ip -br -4 addr
```

以前机载电脑的无线接口叫 `wlan0`；若新网卡名称不同，替换下文的 `wlan0`。
如果尚未接入主机所在的 Wi-Fi，执行以下命令，由终端提示输入密码：

```bash
nmcli --ask device wifi connect 'TP-LINK_6E07_2.4G' ifname wlan0
ip -br -4 addr show wlan0
ping -c 3 192.168.1.100
```

若切换的是当前 SSH 使用的无线连接，旧会话会断开，需要用新 IP 重新连接。
两端改接 5 GHz 时使用实际 SSID，并重新检查两端地址。
取得的是机载电脑的无线 IP，不是 MID360 的 `192.168.123.20`，也不是之前的有线地址。

在工作站执行：

```bash
cd /home/jyfight/Project/rtabmap_slam
read -rp '机载电脑当前的无线 IPv4：' ROBOT_IP
ip route get "$ROBOT_IP"
ping -c 3 "$ROBOT_IP"
scp scripts/ros2_wifi_direct_env.sh "unitree@${ROBOT_IP}:ros2_wifi_direct_env.sh"
ssh "unitree@$ROBOT_IP"
```

路由查询应显示 `dev wlx6c1ff712636d src 192.168.1.100`（以实时地址为准）。
双向 ping 和 SSH 正常后再启动传感器。

## 2. 机载电脑启动传感器

以下命令在机载电脑执行，`192.168.1.100` 是**工作站无线 IP**。
每个终端先加载 ROS/驱动环境，最后加载无线 DDS 环境。
若有旧驱动，先在其启动终端 Ctrl+C，再启动一次，避免重复占用设备。

雷达终端：

```bash
source ~/yao_space/ws_livox/install/setup.bash
source ~/ros2_wifi_direct_env.sh wlan0 192.168.1.100
ping -c 3 192.168.123.20
/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

机载雷达网口保持原配置。历史配置为 `eth1 = 192.168.123.18/24`；若未恢复且该连接仍存在，
可执行 `sudo nmcli connection up livox-eth1`。驱动应输出 `sensor_msgs/msg/PointCloud2`。
无线替换不需要把 Livox 驱动的雷达地址改成 Wi-Fi 地址。

相机终端（另开 SSH）：

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_wifi_direct_env.sh wlan0 192.168.1.100
/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py \
  config_file:="'/home/unitree/realsense_slam_sensor_qos.yaml'" \
  rgb_camera.profile:=424x240x6 \
  depth_module.profile:=640x480x6 \
  align_depth.enable:=false \
  enable_color:=true enable_depth:=true \
  enable_infra1:=false enable_infra2:=false \
  enable_sync:=false
```

先使用 RGB + 深度接通链路；需要近红外时再开启相应流并重新测量带宽。
项目在工作站完成深度配准，保留 `config_file` 的两层引号。

## 3. 工作站检查真实数据

在工作站本地终端执行：

```bash
cd /home/jyfight/Project/rtabmap_slam
read -rp '机载电脑当前的无线 IPv4：' ROBOT_IP
source setup.bash
source scripts/ros2_wifi_direct_env.sh wlx6c1ff712636d "$ROBOT_IP"
ros2 daemon stop
ros2 daemon start
ros2 topic list
ros2 topic type /livox/lidar
timeout 10s ros2 topic hz /livox/lidar
timeout 10s ros2 topic hz /livox/imu
timeout 10s ros2 topic hz /camera/color/image_raw
timeout 10s ros2 topic hz /camera/depth/image_rect_raw
```

`timeout` 到时退出码为 124 是正常结束采样。雷达类型应为 `sensor_msgs/msg/PointCloud2`；
四路都应持续输出频率，而不只是能列出话题。相机设定是 6 Hz，雷达按驱动实际设置核对。
CLI 测频也会增加订阅负载；频率明显偏低或频繁中断时，先解决链路再建图。

## 4. 工作站启动建图与显示

建图终端：

```bash
cd /home/jyfight/Project/rtabmap_slam
read -rp '机载电脑当前的无线 IPv4：' ROBOT_IP
source setup.bash
source scripts/ros2_wifi_direct_env.sh wlx6c1ff712636d "$ROBOT_IP"
bash scripts/start_semantic_emsanet.sh
```

继续沿用实际标定的 `CAM_*` 外参。也可选择 `start_semantic_slam.sh`（YOLOv8）或
`start_semantic_yoloe.sh`，一次启动一种。这里的 EMSANet 默认方案使用 RGB-D。
当前 `start_verified_fusion.sh` 会覆盖 DDS 为旧有线配置，不用于本无线流程。

MITI 终端：

```bash
cd /home/jyfight/Project/rtabmap_slam
read -rp '机载电脑当前的无线 IPv4：' ROBOT_IP
source setup.bash
source scripts/ros2_wifi_direct_env.sh wlx6c1ff712636d "$ROBOT_IP"
bash scripts/start_miti.sh
```

打开 <http://localhost:3000/rtabmap.html>。
切换网络后，旧传感器、SLAM、RViz、MITI 进程需要在对应的新环境中重启。
只执行 `source` 或重启 `ros2 daemon` 不会更新已经运行的节点。
本流程加载无线环境后，不要再加载旧 `~/go2env.sh`、`~/ros2_wifi_env.sh` 或
`scripts/ros2_wired_env.sh`，这些文件会覆盖 DDS 网卡配置。

## 带宽与排查

424×240 RGB8、640×480 16 位深度都以 6 Hz 发送时，原始图像约占
`(424×240×3 + 640×480×2)×6×8 ≈ 44.1 Mbps`，还未计入雷达、协议开销和重传。
两端都通过同一 Wi-Fi AP 传输还要共享无线空口。路由器显示的连接速率不等于实测有效吞吐。
优先选择两端都支持的 5 GHz 网络，并通过实际采样确认效果。

仓库已有 `scripts/hotspot_sensor_transport.py` 压缩传输工具，可以在 2.4 GHz 链路不足时使用：
机载传感器置于本机 ROS domain 42，通过 TCP 7447 单路传输，再在工作站 domain 0 发布。
点云与深度使用无损压缩，RGB 可选 JPEG；JPEG 会改变彩色像素。
原有 `start_hotspot_sensors.sh`、`start_hotspot_receive.sh` 仍依赖家目录旧热点环境，
不要直接与本流程混用；需另行适配其环境与 IP 后再切换。

| 现象 | 优先检查 |
| --- | --- |
| ping / SSH 不通 | 两端无线 IP、同一子网、AP 隔离、`ip route get 对端IP` 的出口 |
| SSH 正常，ROS 话题不可见 | 两端是否加载新脚本、参数是否填了对端 IP、是否都在 domain 0、UDP 是否被防火墙阻断 |
| 话题可见但没有持续数据 | 是否仍有旧 DDS 进程、QoS 是否匹配、链路丢包和实际带宽 |
| `eth0` 或 `enxec1ac312a316` 不存在/不可用 | 某个终端或启动入口仍加载了旧有线 DDS 配置 |
| 时间延迟巨大、TF 外推错误 | 两端系统时间与传感器时间基准；需校时则先停建图和传感器，校时后重启 |

新环境脚本固定 CycloneDDS、domain 0、允许跨主机，并设置对端与本机的单播发现。
单播配置能绕开组播发现限制，但不能绕开 AP 客户端隔离或防火墙。
本次已通过 Bash 语法及无效参数检查，并在主机实际无线接口上成功初始化 CycloneDDS
（隔离 domain 98，对端填写主机自身地址）。这验证了本机配置可加载。
当前尚未确认机器狗无线 IP，也未完成两机无线传感器实测；以上为待接入流程。
