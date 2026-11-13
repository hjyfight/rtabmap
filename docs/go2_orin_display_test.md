# Go2 → Orin → 工作站建图显示测试

2026-11-11 已使用 Go2 的真实 MID360 点云验证 ICP、RTAB-Map 和 MITI 显示链路。
工作站收到 10,057 个地图点、2 个图节点与 ICP 里程计；用户已通过页面及截图
确认三维点云和二维栅格显示。

## 当前链路

| 设备 | 作用 | 网络 |
| --- | --- | --- |
| Go2 | Livox / RealSense 驱动，本次只发送雷达点云和静态 TF | `wlan0`，`192.168.1.104` |
| Orin | TCP 接收、ICP、RTAB-Map | `wlP1p1s0`，`192.168.1.103` |
| 工作站 | MITI 三维点云及二维栅格显示 | `wlx6c1ff712636d`，`192.168.1.100` |

以上地址于 2026-11-12 核对，已从昨天的 `192.168.43.*` 热点切换到 `192.168.1.*`。
2026-11-11 的显示验证使用 `92865` 的 2.4 GHz 热点。当前 Go2 驱动的可用信道列表只提供
2.4 GHz；手机支持 5 GHz 不代表这套 Go2 网卡/驱动能够接入。

新测试使用 ROS domain **74**，显示地址为 <http://localhost:3001/rtabmap.html>，
WebSocket 端口为 **8766**。Go2 传感器在本机 domain 42 中采集，经 TCP 7447
发送到 Orin。原有线启动命令、配置文件和工作站 domain 0 / 3000 显示服务保留。

## 启动

### 运动建图：沿用原有点云 + Livox IMU 方案

2026-11-12 已按 SHA-256 核对：Orin 与工作站的
`launch/livox_realsense_3dslam.launch.py`、`scripts/livox_pointcloud_time.py` 完全一致。
此前 `slam` 运行的是另一个简化入口，未接 IMU/去畸变，并使用默认
`Icp/MaxTranslation=0.2`；现场日志显示配准修正超过 0.2 m 被拒绝，定位丢失。
移动建图改用以下 `slam_imu`，直接调用原有主 launch，不复制其算法参数。

先在原终端正常结束旧建图和 Go2 发送端。Go2 使用新传输模式：

```bash
GO2_ORIN_LIDAR_ONLY=0 GO2_ORIN_LIDAR_IMU_ONLY=1 bash ~/start_go2_orin_sensors.sh
```

该模式启动 Livox 驱动，发送 `/livox/lidar`、`/livox/imu` 和静态 TF，不启动相机。
原 `GO2_ORIN_LIDAR_ONLY=1` 只传点云，不能作为启用 IMU 建图的输入。
Orin 接收命令仍为 `bash scripts/start_go2_orin_display_test.sh receive`；
如果该接收进程一直运行，会自动重连 Go2。确认统计中两路消息均增加。

Orin 另一个终端：

```bash
cd /home/qiyuan/Project/rtabmap_slam
bash scripts/start_go2_orin_display_test.sh slam_imu
```

启动前同时检查点云和 IMU；实际调用主 launch 时设置
`use_camera=false use_imu=true filter_imu=true deskewing=true assemble=false qos=2 qos_imu=2`，
关闭 Orin 本地 GUI，仍使用 domain 74 和本地 MITI 3001。每次创建独立
`maps/livox_imu_时间戳.db`。旧的 `slam` 保留用于基础显示测试。
不要把工作站旧命令中的有线网卡名直接复制到 Orin。

此外，本次 SSH 往返采样显示 Go2 系统时间比工作站慢约 59 秒，且
`timedatectl` 报告 `System clock synchronized: no`。日志约 60 秒的年龄不能全部
当作网络延迟；重新采集前应在停止传感器和建图后完成校时。本次未修改系统时钟。
主文件一致也不表示工作站 x86 与 Orin ARM 的 RTAB-Map 二进制构建完全相同。

### 基础点云显示测试（不含 IMU）

2026-11-12 排查发现 Go2 驱动/发送端及 Orin 接收端均未运行，只有建图节点等待输入。
必须依次启动发送、接收和建图；`slam` 不会代为启动前两者。不要重复启动同一设备的驱动、
接收或建图实例；已有前台实例先在其终端 Ctrl+C。

Go2 终端：

```bash
GO2_ORIN_LIDAR_ONLY=1 bash ~/start_go2_orin_sensors.sh
```

该入口使用新增的 `~/go2_orin_sensor_transport.py`，只发送雷达与静态 TF，并将大帧
的发送超时延长至 60 秒。原 `~/start_hotspot_sensors.sh` 和
`~/hotspot_sensor_transport.py` 保留。相机仍按原 424×240×6 RGB、640×480×6
深度及两路红外设置采集，本次测试不传输图像。

Orin 接收终端：

```bash
cd /home/qiyuan/Project/rtabmap_slam
bash scripts/start_go2_orin_display_test.sh receive
```

该命令连接当前 Go2 的 `192.168.1.104:7447`，在 Orin 的 ROS domain 74 发布数据。
部署时须一并同步 `scripts/hotspot_sensor_transport.py`，接收入口使用其
`--socket-timeout 60` 参数；旧版本缺少此参数会直接退出。更新后在 Orin 运行
`/usr/bin/python3 -m unittest discover -s tests -p test_hotspot_sensor_transport.py -v`
可检查启动参数解析与传输数据兼容性（需先 `source setup.bash`）。
接收终端应显示 `Compressed sensor link connected`，且统计中的
`received_/livox/lidar` 持续增加；仅显示连接成功不能证明收到点云。

Orin 建图终端：

```bash
cd /home/qiyuan/Project/rtabmap_slam
bash scripts/start_go2_orin_display_test.sh slam
```

建图前最多等待 15 秒，检查是否真正收到带非零时间戳、坐标系和数据的点云；
没有输入时退出并提示启动接收端。修复脚本不会更新已运行节点的环境，旧建图终端
需先 Ctrl+C 再执行上述命令。每个终端保持运行。

后续更换网络时，可以显式传入地址，两条 Orin 命令使用同一组参数：

```bash
bash scripts/start_go2_orin_display_test.sh receive 192.168.1.104 192.168.1.100 wlP1p1s0
bash scripts/start_go2_orin_display_test.sh slam 192.168.1.104 192.168.1.100 wlP1p1s0
```

工作站显示终端：

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/start_go2_orin_display_miti.sh
```

工作站显示端默认连接 `192.168.1.103`；也可使用
`bash scripts/start_go2_orin_display_miti.sh Orin无线IP 工作站网卡`。
切网前已启动的显示端需要在原终端 Ctrl+C 后重新启动。

打开 <http://localhost:3001/rtabmap.html>。新建图入口为
`launch/orin_lidar_display_test.launch.py`，发布 `/cloud_map`、`/mapGraph`、`/icp_odom`
和 `/tf`，使用 `map → icp_odom → livox_frame`。每次启动默认创建独立的
`maps/orin_display_test_时间戳.db`，不删除或覆盖历史地图；正常结束建图可完成写盘。

## 验证与范围

2026-11-12 显示排查：切换无线网络后，旧 MITI 进程仍绑定昨天的地址，需要完整
重启本地 domain 74 显示服务。另发现纯雷达 ICP 的 `Odom/GuessMotion=false`
会使 F2M 在初始化两帧后缺少配准初值，持续发布丢失状态；此入口已改为 `true`，
并开启底层警告转发到 ROS 日志。运行中的 Orin 建图需 Ctrl+C 后重新启动使其生效。
既有短时显示验证只证明首张地图能显示，未覆盖这项持续跟踪问题。

修正后的同一 launch 在工作站隔离 domain 88 中通过 40 帧模拟验证：40 帧有效
里程计、0 帧丢失、7 个图节点、32,426 个地图点，0.75 m 轨迹估计为 0.75088 m。
检查脚本为 `tests/run_orin_display_slam_check.py`。这是模拟持续跟踪验证，
仍需在 Orin 重启后验收实机跟踪。本次 MITI WebSocket 已收到既有地图快照的
8,756 个点和 185×256 栅格；记录为 `logs/go2_orin/display_recovery_20261112.json`，
不代表浏览器画面已被直接检查或传感器链路仍在运行。

- 传输程序的 7 项既有测试通过，覆盖点云时间字段、无损图像、标定、IMU、TF 和坏帧。
- Orin 新建图入口的 launch 参数解析通过。
- 工作站显示接口返回 HTTP 200，桥接状态 `running`，domain 74。
- WebSocket 采到 `/cloud_map` 和 `/miti/clean_map`，均为 10,057 点、`map` 坐标系；
  `/mapGraph` 含节点 1、2，`/icp_odom` 的父坐标系为 `icp_odom`、子坐标系为 `livox_frame`。
- 用户截图确认三维点云和二维栅格已显示。采样记录在 `logs/go2_orin/display_probe.json`。

本次验证的是低速链路上的真实雷达建图与显示流程。RGB-D 融合、图像面板、语义推理、
专用回环诊断与移动建图质量尚未验收；相应面板等待数据符合当前测试范围。
