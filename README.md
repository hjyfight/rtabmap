# RTAB-Map + ROS 2 Humble 建图项目

目标目录：`/home/jyfight/Project/rtabmap_slam`。

2026-11-06：USB 转发接入后，MITI 已支持近红外、RGB 和深度同时显示，
相机朝向、三维地图和二维栅格显示已恢复。已修复 RealSense 对齐尺寸跳变引起的 SLAM 断流。
现在使用原始深度、
相机内参和内部 TF 在工作站完成配准，RGB 尺寸可独立选择。启动与验证见
[深度配准修复](docs/depth_registration_fix.md)。

MITI 三维地图新增墙面开关与机器人位姿读数，标准位姿输出为 `/robot_pose`。
运行 `bash scripts/start_miti.sh` 自动启动；参考点、参数及验证说明见
[墙面与位姿](docs/walls_and_robot_pose.md)。

## MITI 网页实时展示

项目内已部署 MITI 3.3.6 与 foxglove_bridge 3.6.0。传感器和 SLAM 启动后，
在新终端运行 `bash scripts/start_miti.sh`，浏览器打开
<http://localhost:3000/rtabmap.html> 并应用项目布局，可查看三维地图、优化轨迹、
语义标签、并排的二维栅格地图、彩色/深度图和日志。已有布局可在入口选择
“更新显示并保留当前布局”。栅格数据及更新方式见[双地图显示](docs/grid_map_display.md)。安装、网卡设置及验证范围见
[MITI 实时展示说明](docs/miti_live_demo.md)。

## YOLO 语义地图标签

新增 YOLOv8n-seg 关键帧识别、RGB-D 三维定位、跨帧对象融合和语义地图保存。
标签直接显示在 RTAB-Map 原生三维地图窗口，并随回环优化更新。

```bash
bash scripts/install_semantic.sh
bash scripts/build_semantic.sh
# 按深度配准说明启动 Livox / RealSense 原始数据并设置实际 CAM_* 外参后：
bash scripts/start_semantic_slam.sh
```

本机推理环境与编译输出保存在项目内。配置、恢复地图和测试命令见
[语义地图说明](docs/semantic_mapping.md)。

当前机器已确认是 Ubuntu 22.04，已安装 ROS 2 Humble Desktop、Livox ROS 2 驱动和 RealSense ROS 2 驱动。**仅安装驱动不代表传感器已连接。**

## 已验证输入的融合建图入口

```bash
bash scripts/start_verified_fusion.sh
```

这是从 `rgbd_input_check_20261027/start_verified_fusion.sh` 整合的启动入口。
请先按[深度配准说明](docs/depth_registration_fix.md)启动 Livox 和 RealSense 原始数据。
脚本启动工作站配准后，使用项目内的
`check_rgbd_input.py` 与 `verify_fusion_input.py` 采样 5 秒，确认 RGB、对齐深度和内参的
尺寸和坐标系一致、时间差在 0.05 秒内，且没有旧建图节点后，启动融合建图。

默认沿用本机网络接口 `enxec1ac312a316`、ROS domain 0 与 CycloneDDS 配置；
换机器时需调整网络接口，并准备 ROS 2 Humble 及传感器驱动。
`setup.bash` 会加载本机 `/home/jyfight/ros2_ws/install/local_setup.bash`（存在时）。
相机外参可通过 `CAM_X`、`CAM_Y`、`CAM_Z`、`CAM_ROLL`、`CAM_PITCH`、`CAM_YAW` 设置。
每次创建独立的 `maps/livox_realsense_*.db`，当前数据库路径记录到 `maps/current_fusion_db.txt`。

融合模式现默认开启空间邻近回环；RGB-D 同步上限为 0.05 s，视觉关键点最大深度为 4 m。
装配与入库扫描按 5 cm 体素降采样。启动脚本可通过 `ASSEMBLE=true` 开启后端装配；
`MIN_LOOP_CLOSURE_OVERLAP`、`FUSION_STM_SIZE`、`PROXIMITY_BY_SPACE` 可作对照实验。
外参保持原值。修改依据、限制与命令见 [融合回环诊断](docs/fusion_loop_diagnostics.md#2026-10-28回环配置核查与调整)。

地图窗口默认隐藏底部网格，保留雷达地图与实时扫描显示（1 像素点）。
融合启动时同时打开可移动的置顶回环状态窗：后端接受回环后绿色提示保持 8 秒，
显示成功次数、最近匹配节点和时间；候选或拒绝不会提示成功。
`LOOP_STATUS_WINDOW=false bash scripts/start_verified_fusion.sh` 可关闭提示窗。
移动时的几何拖影仍需实测；当前 IMU 去畸变只补偿旋转，详见
[网格、回环提示与运动拖影说明](docs/fusion_loop_diagnostics.md#2026-10-28网格回环提示与运动拖影)。

仓库保存代码、配置和上游源码快照，版本见 [源码版本](docs/source_versions.md)。
地图、日志、构建结果和 Python 缓存保留在本机，不提交到 GitHub。

## 当前状态（2026-10-20）

- `ros-humble-rtabmap-ros` **0.23.13** 已安装（15 个二进制包，位于 `/opt/ros/humble`）。
- 源码已克隆到 `src/rtabmap`、`src/rtabmap_ros`，标签 `0.23.13-humble`，提交记录在 `logs/*_source_commit.txt`。
- 合成建图测试已通过：`passed=true`，ICP 位移 1.2004 m，10 个图节点，41806 点地图云，SQLite 完整性检查通过。
- 安装时清华 ROS 镜像的索引与包文件不一致（大量 404），已将 `/etc/apt/sources.list.d/ros2.list` 切换为官方 `packages.ros.org`；原文件备份为 `/etc/apt/sources.list.d/ros2.list.tsinghua.bak`。`install.sh` 现在会在安装失败时自动回退到官方源。

## 安装

在本机终端运行准备目录中的脚本：

```bash
bash /home/jyfight/ros2_ws/rtabmap_setup/install.sh
```

脚本先将项目复制到 Project，再更新 apt 索引，安装官方 `ros-humble-rtabmap-ros` 及依赖，下载 RTAB-Map 核心与 ROS 2 封装源码，最后运行模拟点云建图测试。默认通过 Clash `http://127.0.0.1:7897` 下载，仅对本次命令生效。sudo 密码在终端提示时输入，不存入脚本。

若安装中途失败，重新执行目标目录下的脚本：

```bash
bash /home/jyfight/Project/rtabmap_slam/install.sh
```

不用代理时：`RTABMAP_INSTALL_PROXY= bash /home/jyfight/Project/rtabmap_slam/install.sh`。

程序和系统依赖通过 apt 安装在 `/opt/ros/humble` 等标准目录；Project 保存源码、配置、地图与测试结果。本次运行使用官方二进制包，不需要 colcon 编译。源码优先下载与二进制版本对应的标签，无匹配标签时使用官方 `rtabmap/master` 和 `rtabmap_ros/ros2` 分支；实际源码提交记录在 `logs/*_source_commit.txt`，系统包版本在 `logs/packages.tsv`。源码未编译时不会影响运行中的软件版本。

## 环境测试

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/test_environment.sh
```

测试使用独立 ROS domain 87、仅本机通信，向真实的 `icp_odometry` 和 `rtabmap` 节点发送模拟三维点云，检查 DDS 通信、ICP 里程计、地图节点、输出点云和 SQLite 地图保存。测试轨迹约 1.2 米，不依赖真实传感器。测试结束后会关闭其启动的节点。合成房间按 0.08 m 间距采样，以匹配 RTAB-Map 默认的 0.1 m 聚类半径；点云过稀会被聚类过滤，导致 `cloud_map` 为空。

只有命令成功退出，且 `logs/latest_test.json` 中的 `passed` 为 `true`，才能认为模拟建图测试通过。详细日志和数据库在 `logs/synthetic-*/`。这不等于真实 Livox、IMU、标定或实地闭环测试通过。

如果 domain 87 正在使用，可运行：`RTABMAP_TEST_DOMAIN_ID=88 bash scripts/test_environment.sh`。

## 使用 Livox 点云建图

2026-10-27：`launch/livox_realsense_3dslam.launch.py` 默认使用 Livox 内置 IMU，
启用点时间单位兼容、去畸变、旋转初值和重力约束，默认逐帧建图；纯雷达关闭回环检测。
`use_camera=true` 的融合模式使用视觉候选与 ICP 验证，自动打印检测状态、候选结果和成功日志。
运行命令与日志说明见 [融合回环诊断](docs/fusion_loop_diagnostics.md)。
这与下面的基础 `start_livox_slam.sh` 是两个入口；修正版命令和验证范围见
[Livox IMU 与运动补偿修复](docs/livox_imu_motion_fix.md)。

本项目读取 `/livox/lidar` 的 `sensor_msgs/msg/PointCloud2`，默认 `frame_id=livox_frame`，由 ICP 估计里程计，不要求已有 `/odom`。

当前 Livox 驱动的 `rviz_MID360_launch.py` 使用 `xfer_format=0`，会输出所需的 PointCloud2；`msg_MID360_launch.py` 使用 `xfer_format=1`，输出自定义消息，不能直接作为本项目输入。其他型号应选择对应的 PointCloud2 启动配置。

连接真实雷达后，先确认型号、雷达 IP、本机网卡 IP 和驱动 JSON 配置。以 MID360 为例，在第一个终端执行：

```bash
source /home/jyfight/Project/rtabmap_slam/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

第二个终端检查数据并启动 SLAM：

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
ros2 topic type /livox/lidar
ros2 topic hz /livox/lidar
# 确认有持续的 PointCloud2 数据后，Ctrl+C 结束频率检查。
bash scripts/start_livox_slam.sh
```

无图形界面时添加 `rviz:=false`。不同话题/frame 示例：

```bash
bash scripts/start_livox_slam.sh lidar_topic:=/your/points frame_id:=your_lidar_frame
```

默认 TF 为 `map -> odom -> livox_frame`，地图文件为 `maps/livox.db`；Ctrl+C 正常退出会保存数据库。默认保留已有地图。开始一个独立新地图时提供新的文件名：

```bash
bash scripts/start_livox_slam.sh database_path:=/home/jyfight/Project/rtabmap_slam/maps/session_02.db
```

如需以机器人 `base_link` 建图，应提供真实标定的 `base_link -> livox_frame` TF 并设置 `frame_id:=base_link`。本配置是基础激光 ICP 建图，不包含 IMU 融合和运动去畸变；快速运动的 Livox 实机应用需进一步配置和验证。

## 可视化合成数据

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/visualize_synthetic.sh
```

脚本运行一次合成建图并生成 `logs/visualization/synthetic_visualization.png`（合成输入点云、RTAB-Map 组装地图、俯视轨迹、轨迹误差）和 `synthetic_result.npz`。已有 npz 时只重绘图片：

```bash
/usr/bin/python3 scripts/plot_synthetic_result.py
```

## Go2：RealSense + Livox 融合

`launch/go2_slam.launch.py` 使用 Livox 做 ICP 里程计，RealSense 的彩色图 + 对齐深度图参与视觉闭环，默认由 Livox 点云生成 3D 地图。

先启动两个传感器（两个终端）：

```bash
# 终端 1：Livox（MID360）
source /home/jyfight/Project/rtabmap_slam/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# 终端 2：RealSense，启用深度对齐到彩色图
source /opt/ros/humble/setup.bash
```

确认数据：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

再启动 SLAM，**必须按实际安装位置填写外参**（单位米/弧度，顺序为 roll/pitch/yaw）：

```bash
source /home/jyfight/Project/rtabmap_slam/setup.bash
ros2 launch /home/jyfight/Project/rtabmap_slam/launch/go2_slam.launch.py \
  lidar_x:=0.0  lidar_y:=0.0  lidar_z:=0.0  lidar_roll:=0.0  lidar_pitch:=0.0  lidar_yaw:=0.0 \
  camera_x:=0.0 camera_y:=0.0 camera_z:=0.0 camera_roll:=0.0 camera_pitch:=0.0 camera_yaw:=0.0
```

TF 约定与选项：

- 启动文件默认发布 `base_link -> livox_frame` 和 `base_link -> camera_link` 静态 TF。若 Go2 的 URDF/其他节点已经发布这两组 TF，加 `publish_static_tf:=false`，避免重复 TF。
- RealSense 驱动默认会发布 `camera_link -> camera_color_frame -> camera_color_optical_frame`；若话题/坐标系不同，覆盖 `rgb_topic`、`depth_topic`、`camera_info_topic`、`lidar_topic`、`lidar_frame`。
- 使用 Go2 自带里程计时：`use_icp_odom:=false odom_topic:=/your/odom`，并确保存在 `odom -> base_link` TF。
- `reg_strategy:=1`（ICP 配准），`grid_sensor:=0`（Livox 建栅格）；改成 `2` 可同时用雷达和深度图建栅格。
- 地图默认保存为 `maps/go2.db`，可用 `database_path:=...` 指定新文件。

注意：以上融合配置尚未用真实传感器验证；外参必须实际测量或从 Go2 的 URDF 获取，否则地图会错位。

没有真实传感器时，可用合成 RGB + 深度 + 点云数据验证 launch 的订阅、TF 和建图链路：

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/test_go2_environment.sh
```

测试使用 ROS domain 90，结果写入 `logs/latest_go2_test.json`；当前本机结果为 `passed=true`（11 个图节点、约 3.5 万点地图云、SQLite 完整）。

## 官方仓库

<https://github.com/introlab/rtabmap>、<https://github.com/introlab/rtabmap_ros/tree/ros2>。

## 纯 LiDAR 与相机辅助建图对比

`launch/slam_comparison.launch.py` 使用当前主 launch，支持 `lidar`（雷达＋IMU，无回环）和
`rgbd`（雷达＋IMU，RGB-D 辅助回环）两组独立实验。结束并保存后，
`scripts/compare_slam_databases.py` 从两份数据库的只读快照导出地图，生成统一视角的
三维、俯视、水平切片及优化前后轨迹对比图，不依赖 `/cloud_map`。
已有数据库也可直接比较；采集命令与结果解释见 [对比实验说明](docs/slam_comparison.md)。








## 雷达命令
ssh unitree@192.168.123.18
source ~/yao_space/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py


## 相机命令

当前采用工作站深度配准。机载电脑运行下面命令；先退出已有相机驱动，再启动一次：

```bash
ssh unitree@192.168.123.18
source /opt/ros/foxy/setup.bash
/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py \
  config_file:="'/home/unitree/realsense_slam_sensor_qos.yaml'" \
  rgb_camera.profile:=424x240x6 depth_module.profile:=640x480x6 \
  align_depth.enable:=false enable_sync:=false \
  enable_color:=true enable_depth:=true
```

工作站的融合/语义 launch 自动生成 `/camera/depth_registered/image_rect`。
修改 RGB 尺寸时，输出深度会按 RGB CameraInfo 自动调整；详见
[深度配准修复](docs/depth_registration_fix.md)。

## 本地cmd

source ~/go2env.sh
source /home/jyfight/Project/rtabmap_slam/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enxec1ac312a316"/></Interfaces></General></Domain></CycloneDDS>'

  


ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic hz /camera/color/image_raw

source /opt/ros/humble/setup.bash
ros2 topic list



## 启动建图


## 纯雷达建图
source /home/jyfight/Project/rtabmap_slam/setup.bash

DB=/home/jyfight/Project/rtabmap_slam/maps/livox_imu_$(date +%Y%m%d_%H%M%S).db

ros2 launch /home/jyfight/Project/rtabmap_slam/launch/livox_realsense_3dslam.launch.py \
  use_camera:=false \
  use_imu:=true \
  filter_imu:=true \
  deskewing:=true \
  assemble:=false \
  qos:=2 \
  qos_imu:=2 \
  database_path:="$DB"



## 融合建图
source ~/go2env.sh
source /home/jyfight/Project/rtabmap_slam/setup.bash

# 以下数值必须替换为实际测量的 livox_frame -> camera_link 外参
CAM_X=0.0
CAM_Y=-0.12
CAM_Z=-0.03
CAM_ROLL=0.0
CAM_PITCH=0.0
CAM_YAW=-1.5708

DB=/home/jyfight/Project/rtabmap_slam/maps/livox_realsense_$(date +%Y%m%d_%H%M%S).db



ros2 launch /home/jyfight/Project/rtabmap_slam/launch/livox_realsense_3dslam.launch.py \
  use_camera:=true \
  use_imu:=true \
  filter_imu:=true \
  deskewing:=true \
  assemble:=false \
  cam_to_lidar_x:="$CAM_X" \
  cam_to_lidar_y:="$CAM_Y" \
  cam_to_lidar_z:="$CAM_Z" \
  cam_to_lidar_roll:="$CAM_ROLL" \
  cam_to_lidar_pitch:="$CAM_PITCH" \
  cam_to_lidar_yaw:="$CAM_YAW" \
  qos:=2 \
  qos_imu:=2 \
  database_path:="$DB"




## 最新建图命令
export CAM_X CAM_Y CAM_Z CAM_ROLL CAM_PITCH CAM_YAW
bash /home/jyfight/ros2_ws/rgbd_input_check_20261027/start_verified_fusion.sh


# 查看回环
bash /home/jyfight/ros2_ws/fusion_loop_search_fix_20261027/apply_live_loop_fix.sh

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run rviz2 rviz2




## 纯雷达建图
cd /home/jyfight/Project/rtabmap_slam
source setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enxec1ac312a316"/></Interfaces></General></Domain></CycloneDDS>'

LIDAR_DB="/home/jyfight/Project/rtabmap_slam/maps/livox_imu_$(date +%Y%m%d_%H%M%S_%N).db"

ros2 launch /home/jyfight/Project/rtabmap_slam/launch/livox_realsense_3dslam.launch.py \
  use_camera:=false \
  use_imu:=true \
  filter_imu:=true \
  deskewing:=true \
  assemble:=false \
  qos:=2 \
  qos_imu:=2 \
  database_path:="$LIDAR_DB"


  ## 融合建图
  bash /home/jyfight/Project/rtabmap_slam/scripts/start_verified_fusion.sh



## 语义建图
cd /home/jyfight/Project/rtabmap_slam
bash scripts/start_semantic_slam.sh





## 有线连接建图 ####################################################

## 雷达命令
ssh unitree@192.168.123.18
source ~/yao_space/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

## 相机命令

ssh unitree@192.168.123.18

source /opt/ros/foxy/setup.bash

/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py \
  config_file:="'/home/unitree/realsense_slam_sensor_qos.yaml'" \
  rgb_camera.profile:=424x240x6 \
  depth_module.profile:=640x480x6 \
  align_depth.enable:=false \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=false


## 查看本地是否正常接收到传感器数据

source ~/go2env.sh


cd /home/jyfight/Project/rtabmap_slam
source scripts/ros2_wired_env.sh
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic hz /camera/color/image_raw

## 最新建图命令  yolov8 分割模型
bash /home/jyfight/Project/rtabmap_slam/scripts/start_semantic_slam.sh

## yoloe 分割模型
bash /home/jyfight/Project/rtabmap_slam/scripts/start_semantic_yoloe.sh


## 红外+深度 语义分割
cd /home/jyfight/Project/rtabmap_slam
bash scripts/start_semantic_emsanet.sh

## miti 展示

启动建图后，新开终端执行：
source ~/go2env.sh

cd /home/jyfight/Project/rtabmap_slam
source scripts/ros2_wired_env.sh
bash scripts/start_miti.sh

打开浏览器 点击 应用项目布局并进入展示
http://localhost:3000/rtabmap.html

## 通过热点连接与传输

本节记录本次已配置的热点连接方式：机载电脑连接 Wi-Fi 热点 `yxm`，通过无线链路和路由转发向工作站发送 ROS 2 数据。MID360 通过有线连接机载电脑，RealSense 通过 USB 连接机载电脑；工作站通过 USB 有线网卡接入现有转发链路。

| 设备 / 接口 | 当前地址 | 用途 |
| --- | --- | --- |
| MID360 | `192.168.123.20` | 雷达原始数据源 |
| 机载电脑 `eth1` | `192.168.123.18/24` | 有线接收雷达数据 |
| 机载电脑 `wlan0` | `172.29.38.125/24` | 热点连接、SSH 和 ROS 2 数据传输 |
| 工作站 `enxec1ac312a316` | `192.168.123.220/24` | 接入转发链路，接收 ROS 2 数据 |

当前工作站到 `172.29.38.125/32` 的下一跳为 `192.168.123.221`；机载电脑到 `192.168.123.220/32` 的下一跳为 `172.29.38.168`。两条路由已保存到各自的 NetworkManager 连接配置。机载电脑的 `livox-eth1` 连接已配置自动连接及地址 `192.168.123.18/24`。

两端处于不同网段，使用 CycloneDDS 单播 `Peers` 发现对方，统一设置 `ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。两端各自的 `~/ros2_wifi_env.sh` 会加载 `~/ros2_wifi_cyclonedds.xml`：机载电脑绑定 `wlan0`，工作站绑定 `enxec1ac312a316`。这些文件已部署在本次两台机器的用户家目录中，不属于仓库文件；工作站的 `~/go2env.sh` 也已指向这份热点 DDS 配置。

### 1. 通过热点 SSH 连接机载电脑

在工作站终端执行：

```bash
ping -c 3 172.29.38.125
ssh unitree@172.29.38.125
```

登录后，若出现 `ros:foxy(1) noetic(2) ?`，输入 `1`。机载电脑上可检查两个接口和雷达连接：

```bash
ip -br addr
ping -c 3 192.168.123.20
```

应看到 `wlan0` 的热点地址和 `eth1` 的 `192.168.123.18/24`。若 `eth1` 地址未恢复，可在机载电脑执行 `sudo nmcli connection up livox-eth1`。

### 2. 机载电脑启动 MID360（不启动 RViz）

在第一个 SSH 终端运行；已有雷达驱动时先停止已有实例，再启动一次：

```bash
bash ~/start_livox_wifi.sh
```

这个已部署的脚本等效于下面三条命令，两种方式任选一种：

```bash
source ~/yao_space/ws_livox/install/setup.bash
source ~/ros2_wifi_env.sh
/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

本次已核验机载电脑 `~/yao_space/ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` 的 `xfer_format=0`，输出 `/livox/lidar`（`sensor_msgs/msg/PointCloud2`）和 `/livox/imu`。该文件不启动 RViz，适用于 SSH 会话。此处格式说明针对本次机载安装；其他版本的同名 launch 可能默认输出自定义消息。

### 3. 机载电脑启动 RealSense

在工作站另开一个终端，执行 `ssh unitree@172.29.38.125`，登录后选择 Foxy。已有相机驱动时先停止已有实例，再在这个 SSH 终端运行：

```bash
source ~/ros2_wifi_env.sh
/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py \
  config_file:="'/home/unitree/realsense_slam_sensor_qos.yaml'" \
  rgb_camera.profile:=424x240x6 \
  depth_module.profile:=640x480x6 \
  align_depth.enable:=false \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=false
```

这里沿用项目的原始深度方案，在工作站完成深度配准。先检查 `/camera/color/image_raw` 和 `/camera/depth/image_rect_raw`；配准输出及尺寸要求见[深度配准修复](docs/depth_registration_fix.md)。`config_file` 参数需保留示例中的两层引号。

### 4. 工作站加载热点环境并检查接收

在工作站本地终端执行。先加载项目 setup，最后加载热点环境，确保 DDS 单播配置生效：

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
source ~/ros2_wifi_env.sh

# 切换 DDS 配置后重建 CLI daemon，清除旧环境影响。
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

应能看到 `/livox/lidar`、`/livox/imu`、`/camera/color/image_raw` 和 `/camera/depth/image_rect_raw`。下面每项采样 10 秒后自动停止：

```bash
ros2 topic type /livox/lidar
timeout 10s ros2 topic hz /livox/lidar
timeout 10s ros2 topic hz /livox/imu
timeout 10s ros2 topic hz /camera/color/image_raw
timeout 10s ros2 topic hz /camera/depth/image_rect_raw
```

需要 RViz 时，在工作站的图形桌面终端加载上述环境后执行 `ros2 run rviz2 rviz2`。机载 SSH 会话使用第 2 步的无 RViz 启动命令，避免 `could not connect to display` / `xcb` 错误。

### 5. 工作站通过热点数据启动建图与 MITI

完成传感器接收检查、项目安装及实际相机外参设置后，可在工作站启动现有 YOLOv8 语义建图入口：

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
source ~/ros2_wifi_env.sh
bash scripts/start_semantic_slam.sh
```

YOLOE 入口为 `bash scripts/start_semantic_yoloe.sh`，需先加载相同环境；两种建图入口任选一种。以上语义建图入口会保留已设置的 `CYCLONEDDS_URI`。原有 `scripts/start_verified_fusion.sh` 会重新设置 DDS 配置，热点模式请使用本节列出的入口。

建图启动后，在工作站另开终端启动 MITI：

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
source ~/ros2_wifi_env.sh
bash scripts/start_miti.sh
```

浏览器打开 <http://localhost:3000/rtabmap.html>，应用项目布局。已有 SLAM、RViz 或 MITI 进程时，需要从热点环境重新启动对应进程，终端里的 `export` 不会改变已运行进程的环境。

本节的热点命令按上述顺序执行；上文旧命令中仅指定有线接口的 `export CYCLONEDDS_URI=...` 会覆盖单播 `Peers` 配置。更换热点或地址后，需要同步更新两端 XML 中的对端 IP 和对应路由，不能只验证 SSH 连通。

本次已实测工作站接收到雷达点云、IMU、彩色与原始深度图。雷达一次 12 秒采样收到 37 帧点云和 1040 条 IMU 消息，点云接收速率约 5 Hz，低于驱动设置的 10 Hz；热点带宽和链路质量仍会影响实际接收速率。

######热点建图#####

说明：以下是本次热点建图的日常操作汇总，按顺序执行即可。机载电脑连接热点 `yxm`，通过 `wlan0`（`172.29.38.125`）将雷达、IMU 和图像数据传给工作站（`192.168.123.220`）。雷达本体仍通过机载电脑的 `eth1`（`192.168.123.18`）有线接入，RealSense 通过 USB 接入；热点用于机载电脑到工作站之间的数据传输。

两端的单播 DDS、转发路由和机载雷达网口已完成配置。雷达和相机分别在两个 SSH 终端运行，数据检查、建图、RViz 和 MITI 在工作站本地运行。登录机载电脑出现 ROS 版本选择时输入 `1`（Foxy）；已有同类驱动或建图进程时先停止已有实例，再启动一次。每个终端都要加载自己的环境，`source ~/ros2_wifi_env.sh` 放在其他 setup 命令之后。

**1．雷达启动命令（终端 1）**

在工作站打开终端，连接机载电脑：

```bash
ssh unitree@172.29.38.125
```

登录成功后，在机载电脑执行：

```bash
source ~/yao_space/ws_livox/install/setup.bash
source ~/ros2_wifi_env.sh
/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

这三条命令也可用 `bash ~/start_livox_wifi.sh` 一条代替，两种方式选一种。当前机载版本输出 `/livox/lidar`（PointCloud2）和 `/livox/imu`，不启动 RViz。

**2．图像启动命令（终端 2）**

在工作站另开终端，再连接一次机载电脑：

```bash
ssh unitree@172.29.38.125
```

登录成功后，在机载电脑执行：

```bash
source ~/ros2_wifi_env.sh
/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py \
  config_file:="'/home/unitree/realsense_slam_sensor_qos.yaml'" \
  rgb_camera.profile:=424x240x6 \
  depth_module.profile:=640x480x6 \
  align_depth.enable:=false \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=false
```

输出彩色图 `/camera/color/image_raw` 和原始深度图 `/camera/depth/image_rect_raw`；建图时由工作站完成深度配准。保留 `config_file` 的两层引号。

**3．本机检查雷达和图像（终端 3，不进入 SSH）**

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
source ~/ros2_wifi_env.sh
ros2 daemon stop
ros2 daemon start
ros2 topic list

# 确认点云类型，并分别采样 10 秒。
ros2 topic type /livox/lidar
timeout 10s ros2 topic hz /livox/lidar
timeout 10s ros2 topic hz /livox/imu
timeout 10s ros2 topic hz /camera/color/image_raw
timeout 10s ros2 topic hz /camera/depth/image_rect_raw
```

点云类型应为 `sensor_msgs/msg/PointCloud2`，四个话题都应实际收到数据。首次切换 DDS 环境或话题列表异常时，执行上面的 daemon 重启命令。

**4．本机启动建图（终端 4，不进入 SSH）**

沿用项目已安装的建图环境，启动前用 `export 变量名=标定值` 设置 `CAM_X`、`CAM_Y`、`CAM_Z`、`CAM_ROLL`、`CAM_PITCH`、`CAM_YAW`。YOLOv8 语义建图命令：

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
source ~/ros2_wifi_env.sh
bash scripts/start_semantic_slam.sh
```

若使用已安装的 YOLOE 模型，则选择下面这一组命令；与 YOLOv8 建图任选一种：

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
source ~/ros2_wifi_env.sh
bash scripts/start_semantic_yoloe.sh
```

**5．本机启动 MITI 展示（终端 5，不进入 SSH）**

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
source ~/ros2_wifi_env.sh
bash scripts/start_miti.sh
```

浏览器打开 <http://localhost:3000/rtabmap.html>，点击应用项目布局。需要 RViz 时，在工作站另一个图形桌面终端执行：

```bash
source /home/jyfight/Project/rtabmap_slam/setup.bash
source ~/ros2_wifi_env.sh
ros2 run rviz2 rviz2
```

**连接异常时检查：** 工作站先执行 `ping -c 3 172.29.38.125`；机载电脑执行 `ip -br addr` 和 `ping -c 3 192.168.123.20`。如果机载 `eth1` 没有 `192.168.123.18/24`，执行 `sudo nmcli connection up livox-eth1`。若仍提示 `eth0: does not match an available interface`，在对应启动终端重新执行 `source ~/ros2_wifi_env.sh`。本流程使用已部署的用户目录脚本和配置；更换热点或 IP 后，需要同步更新两端 DDS 配置及路由。

## EMSANet RGB-D 实例分割（可选）

EMSANet 让 RGB 与注册深度共同参与网络分割，实例结果继续接入现有三维标签地图。使用独立启动文件，原 YOLOv8 / YOLOE 命令及墙面模块保持原状。

```bash
# 首次准备；本机 Clash 代理端口为 7897
bash scripts/install_emsanet.sh --proxy http://127.0.0.1:7897
bash scripts/build_semantic.sh

# 加载当前连接方式对应的 ROS 环境后运行；先退出已有建图进程
bash scripts/start_semantic_emsanet.sh
```

网线连接可先 `source scripts/ros2_wired_env.sh`；热点连接沿用已经验证的热点环境。新模型使用 NYUv2 室内类别和独立的 `.emsanet.semantic.sqlite`，不需要开启近红外。配置、离线推理与验证说明见 [EMSANet RGB-D 建图](docs/emsanet_semantic_mapping.md)。
