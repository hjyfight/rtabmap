# Orin 建图与语义识别环境

部署日期：2026-11-11。工作站项目为 `/home/jyfight/Project/rtabmap_slam`；
Orin 项目为 `/home/qiyuan/Project/rtabmap_slam`，SSH 为 `qiyuan@192.168.43.15`。
环境已通过无硬件测试；随后通过 Go2 采集真实雷达数据，在 Orin 上运行 ICP 与
RTAB-Map，并在工作站确认三维点云和二维栅格显示。当前低速显示测试的独立启动
入口和验证范围见 [Go2 → Orin 显示测试](go2_orin_display_test.md)。

## 设备与分工

| 项目 | 配置 |
| --- | --- |
| Orin 系统 | Ubuntu 22.04.5，ARM64，JetPack 6.2 / L4T 36.4.3 |
| CUDA | 12.6；保留原有 NVIDIA 驱动和 PyTorch |
| PyTorch / torchvision | NVIDIA `2.5.0a0+872d972e41.nv24.08` / 本机编译 `0.20.0a0` |
| ROS | ROS 2 Humble，CycloneDDS |
| RTAB-Map | 官方 ARM64 0.23.13 软件包 |
| RealSense ROS 驱动 | 4.58.4，已安装，未接硬件验证 |
| MID360 | Livox SDK2 与 ROS 2 驱动已在 Orin 编译 |
| Orin 无线接口 | `wlP1p1s0`，当前 `192.168.43.15` |
| 工作站无线接口 | `wlx6c1ff712636d`，当前 `192.168.43.14` |

DHCP 地址可能变化。工作站的 `eno1` 保留外网连接；DDS 使用明确指定的局域网接口。
Orin 负责 SLAM 与 GPU 语义推理，工作站运行已有 MITI 显示入口。
桌面阶段 Orin 使用配套适配器；上狗前另行确认载板供电要求、固定方式和散热。

传感器以后直接接 Orin 时，MID360 的以太网与 RealSense 的 USB 接到 Orin，
机器狗自身运行 ROS 1 不影响这套 ROS 2 传感器和建图链路。
MID360 自带 IMU，由同一个驱动发布 `/livox/lidar` 与 `/livox/imu`。
当前程序使用 `map → odom → livox_frame → camera_link → 相机光学坐标系`；
雷达到相机的静态外参需要实测。若整套传感器保持刚性安装一起移动，外参通常可沿用。
需要狗身 `base_link` 位姿时再增加正确的机身安装变换；若使用狗的 ROS 1 里程计、
关节或控制数据，需要单独适配通信。本次没有部署 ROS 1 桥。

## 安装与编译入口

系统依赖由 `scripts/install_orin_system.sh` 安装，包含 ROS、RTAB-Map、RealSense、
深度配准、IMU 滤波与编译工具；不执行系统升级或替换 JetPack。
ROS 官方公钥保存在 `logs/orin_deploy/ros.key`。

以下命令均在 **Orin** 项目目录运行；已有环境不必重复安装。

```bash
cd /home/qiyuan/Project/rtabmap_slam
# 首次系统安装需要 sudo：
sudo bash scripts/install_orin_system.sh

# 其余步骤使用 qiyuan 普通用户：
bash scripts/install_orin_opencv_dev.sh
bash scripts/build_semantic.sh --headless
bash scripts/build_orin_sensors.sh
bash scripts/install_orin_emsanet.sh
```

重要的 Jetson 适配：

- NVIDIA 自带 OpenCV 4.8 开发包与 ROS 二进制所需 OpenCV 4.5 不一致。
  `install_orin_opencv_dev.sh` 将匹配的 Ubuntu 开发文件提取到 `.tools/opencv-ros`，
  运行库引用已安装的 ROS 依赖；不替换系统 OpenCV。
- `build_semantic.sh --headless` 编译语义消息和关键帧桥接，省去自定义桌面 GUI 的编译。
- `install_orin_emsanet.sh` 在 `.venv-emsanet` 配置依赖，复用已验证的 NVIDIA CUDA
  PyTorch，并由固定的 torchvision 0.20.0 源码编译扩展，不安装工作站的 CPU torch。
  虚拟环境中的 `jetpack_pytorch.pth` 引用当前用户原有 Python 包位置，因此该 GPU
  环境依赖原有 NVIDIA torch 保持可用。
- `.tools/emsanet` 与模型权重从现有项目同步；模型 SHA-256 会再次校验。
  torchvision 源码包位于 `logs/orin_deploy/torchvision-v0.20.0.tar.gz`。
- Livox SDK2 固定提交为 `c0796f04c143143899c87a773d9f6b7136453c0b`；
  驱动源码从工作站现有 `~/ros2_ws/src/livox_ros_driver2` 同步。
  SDK 和驱动安装在项目 `install/livox_sdk`、`install/sensors`，由 `setup.bash` 加载。
- 没有把工作站的 x86 构建产物、虚拟环境或 MITI 安装包复制为 Orin 可执行环境。

源码与权重同步不包含历史地图；新地图保存在 Orin 项目的 `maps/` 中。

## 无硬件验证

在 Orin 本地隔离 ROS 域测试建图：

```bash
cd /home/qiyuan/Project/rtabmap_slam
PYTHONNOUSERSITE=1 RTABMAP_TEST_RMW=rmw_cyclonedds_cpp \
  bash scripts/test_environment.sh
```

结果见 `logs/latest_test.json`。只有 `passed: true` 才表示模拟建图、输出点云和数据库
检查通过，不代表实际传感器标定或实地建图已经验收。

GPU 样例推理：

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
  .venv-emsanet/bin/python3 scripts/predict_emsanet.py \
  --device cuda:0 --output logs/orin_deploy/gpu_sample
```

样例结果保存在 `logs/orin_deploy/gpu_sample/report.json` 和 `instances.png`。
当前细分阶段计时器仅支持 CPU，CUDA 报告中的 `model_timing.state=unsupported_device`
表示不提供那组阶段计时；应结合实际输出、模型设备和总推理时间判断推理是否成功。
单次样例耗时不能当作稳定运行时的帧率。

完整的 GPU 语义 ROS 验证（包括关键帧解压、投影、回环重投影、保存和重载）：

```bash
SEMANTIC_TEST_DEVICE=cuda:0 bash scripts/test_emsanet.sh --device cuda:0
```

报告为 `logs/semantic_validation/ros_real_emsanet.json`。

## 后续接传感器与启动

接硬件前确认 MID360 的 IP、Orin 专用雷达网口及静态地址，准备对应 SDK JSON 配置。
本次没有修改网口、热点或雷达 IP。独立启动入口显式输出 `PointCloud2`，避免使用
默认输出厂家自定义消息的 `msg_MID360_launch.py`：

```bash
# Orin；先填写实际的 JSON 路径：
cd /home/qiyuan/Project/rtabmap_slam
source setup.bash
source scripts/ros2_wifi_direct_env.sh wlP1p1s0 192.168.43.14
ros2 launch "$PWD/launch/orin_mid360.launch.py" config_path:=/实际路径/MID360_config.json
```

RealSense 使用 Orin 上的 Humble 驱动单独配置；旧机载电脑 Foxy 驱动的参数名和命名空间
不能直接假定相同。接入时检查输出是否符合项目所需的 `/camera/color/image_raw`、
`/camera/depth/image_rect_raw`、相机内参和内部 TF，再检查尺寸、时间戳与实际频率。

填写实测 `CAM_X`、`CAM_Y`、`CAM_Z`、`CAM_ROLL`、`CAM_PITCH`、`CAM_YAW` 后，
在 Orin 另开终端：

```bash
cd /home/qiyuan/Project/rtabmap_slam
source setup.bash
source scripts/ros2_wifi_direct_env.sh wlP1p1s0 192.168.43.14
bash scripts/start_orin_slam.sh
```

此入口使用 EMSANet `cuda:0`，关闭 Orin 上的 RTAB-Map 桌面窗口与 Qt 回环提示窗。
启动前会检查 RGB-D 输入；未连接传感器时检查失败是预期行为。
现有启动入口对未设置的外参会采用示例值并报警，实际使用前必须设置实测值。

在 **工作站** 的新终端启动显示：

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
source scripts/ros2_wifi_direct_env.sh wlx6c1ff712636d 192.168.43.15
bash scripts/start_miti.sh
```

浏览器打开 <http://localhost:3000/rtabmap.html>。切换网络后，在新环境中重启相关节点；
已经运行的节点不会因为另一个终端 `source` 了脚本而更新 DDS 配置。
Orin 以后改用 LAN 网线接路由器时，把环境脚本的第一个参数换成实际 LAN 接口即可；
虽然脚本名包含 `wifi`，它接受本机存在且有 IPv4 的网络接口。

本次验证的最终结果见文末部署记录。

## 部署记录

| 验证项 | 结果 |
| --- | --- |
| MID360 SDK 与驱动、无 GUI 语义模块编译 | 通过 |
| 语义 C++ 桥接运行库 | 已加载 ROS 环境后全部可解析，使用 OpenCV 4.5 |
| 模拟 ICP / SLAM / SQLite | `passed: true`；9 个图节点，最大 47,389 点，位移 1.2004 m；保存 8 个节点，数据库完整性 `ok` |
| EMSANet 样例 GPU 推理 | `cuda:0` 成功输出 8 个对象实例 |
| EMSANet 单元与融合检查 | 28 项通过 |
| ROS 语义测试输入与真实 GPU 推理链路 | 均 `passed: true`；真实模型处理 3 个关键帧，覆盖压缩输入、三维投影、回环重投影、重载和地图重置 |
| 工作站启动参数回归 | 9 项通过 |
| 当前热点的双向 ROS 通信 | 隔离 domain 186，两端运行 45 秒；工作站收到 223 条，Orin 收到 224 条，均通过 |
| MID360、RealSense、无界面语义 launch 参数解析 | 通过；没有启动硬件节点 |

网络验证只覆盖发现和小消息传输，未测量图像与点云的持续带宽，也未验收 MITI 实图渲染。
实机采集、相机与雷达外参、时间同步、实际输入频率仍须在接设备后验证。
测试进程已退出，未启动长期建图服务或设置开机自启。

Orin 上的原始记录：

- `logs/latest_test.json`：模拟建图。
- `logs/semantic_validation/ros_real_emsanet.json`：真实 GPU 语义 ROS 链路。
- `logs/orin_deploy/`：编译日志、GPU 样例、网络测试和启动参数检查。

工作站将这次 Orin 的验证记录备份到 `logs/orin_deploy/remote/`；工作站侧的网络报告为
`logs/orin_deploy/host_link.json`。

## 传感器保留在 Go2，通过热点送到 Orin

2026-11-11 已确认三台设备加入热点 `92865`（2.4 GHz、信道 11、WPA2）：

| 设备 | 无线接口 | IPv4 |
| --- | --- | --- |
| Go2 机载电脑 | `wlan0` | `192.168.43.16/24` |
| Orin | `wlP1p1s0` | `192.168.43.15/24` |
| 工作站 | `wlx6c1ff712636d` | `192.168.43.14/24` |

Go2 的有线管理地址仍为 `192.168.123.18`。MID360 的网线和 RealSense 的 USB
保留在 Go2，Go2 运行传感器驱动，Orin 接收数据并建图，工作站显示结果。
DHCP 地址变化后须按实时值更新启动参数。

Go2 到 Orin 的 3 次 ping、工作站到 Go2 的 5 次 ping、Orin 到 Go2 的 5 次 ping
均无丢包。工作站到 Go2 延迟为 22.7–86.0 ms，Orin 到 Go2 为 34.1–184.9 ms；
这些短时测试只确认网络互通，未验证持续传感器吞吐或时间同步。

Orin 已确认可加载接收程序所需的 ROS 消息、`rclpy` 和 `numpy`。
新增接收参数已同步到 Orin，并通过 Bash 语法检查；使用实际无线接口和隔离
domain 197 初始化 CycloneDDS 也已通过，验证进程已退出。
`start_hotspot_receive.sh` 的三参数模式使用仓库内无线 DDS 环境，接收 Go2 的 TCP
压缩数据，并在 Orin 的 domain 0 发布给本机 SLAM 和工作站：

```bash
# Orin；须先确认 Go2 的压缩发送端已经正确配置。
cd /home/qiyuan/Project/rtabmap_slam
bash scripts/start_hotspot_receive.sh 192.168.43.16 wlP1p1s0 192.168.43.14
```

接收端连接 Go2 的 TCP 7447 端口。点云与深度使用现有无损压缩；彩色图像是否有损
由发送端的 `HOTSPOT_JPEG_QUALITY` 决定，设为 `0` 可保持无损。
旧的零参数或一参数模式仍加载家目录的历史热点环境，当前 Orin 流程使用三参数模式。

Go2 的 SSH 公钥安装已完成，工作站可通过 `192.168.123.18` 免密登录。
原有线启动命令、雷达配置、相机 YAML、`start_hotspot_sensors.sh` 和原传输程序
保留。新增并部署了独立的 `~/start_go2_orin_sensors.sh` 与
`~/go2_orin_sensor_transport.py`，Go2 当前使用新增入口运行传感器和发送端。
雷达输出 PointCloud2，D435i 保持 RGB 424×240×6、深度及两路红外 640×480×6。

首次短时接收确认点云、IMU、RGB、深度、右红外和相机内参进入 Orin，但约 50 秒
只收到 1 帧点云和 3 帧 RGB，实际吞吐与掉帧尚不满足建图条件。两端无线省电均开启；
需关闭无线省电后重新采样，不能将这次接通判为持续传输验收通过。

应用户要求，Orin 后续由用户手动操作；先前由助手启动的后台接收进程已停止。
用户在 Orin 前台执行：

```bash
sudo iw dev wlP1p1s0 set power_save off
cd /home/qiyuan/Project/rtabmap_slam
bash scripts/start_hotspot_receive.sh 192.168.43.16 wlP1p1s0 192.168.43.14
```

Go2 无线省电由用户在 Go2 终端临时关闭：`sudo iw dev wlan0 set power_save off`。
这两条无线命令不修改原有线启动配置。

后续按用户要求先验证低速建图显示：独立 domain 74 / 网页端口 3001 已运行真实
雷达 ICP 和 RTAB-Map，主机收到 10,057 个地图点与 2 个图节点。用户页面与截图
确认三维点云和二维栅格已显示。原 domain 0 / 端口 3000 有线显示服务保留；
完整命令见 [Go2 → Orin 显示测试](go2_orin_display_test.md)。
