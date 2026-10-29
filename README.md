# RTAB-Map + ROS 2 Humble 建图项目

目标目录：`/home/jyfight/Project/rtabmap_slam`。

## YOLO 语义地图标签

新增 YOLOv8n-seg 关键帧识别、RGB-D 三维定位、跨帧对象融合和语义地图保存。
标签直接显示在 RTAB-Map 原生三维地图窗口，并随回环优化更新。

```bash
bash scripts/install_semantic.sh
bash scripts/build_semantic.sh
# 启动原有 Livox / RealSense 驱动、启用对齐深度并设置实际 CAM_* 外参后：
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
请先启动 Livox 和 RealSense 驱动，并启用深度对齐。脚本使用项目内的
`check_rgbd_input.py` 与 `verify_fusion_input.py` 采样 5 秒，确认 RGB、对齐深度和内参的
尺寸、坐标系、时间戳一致，且没有旧建图节点后，启动融合建图。

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
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
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


ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true


## 相机命令
ssh unitree@192.168.123.18
source /opt/ros/foxy/setup.bash
/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 depth_module.profile:=640x480x30 align_depth.enable:=true rgb_camera.enable_auto_exposure:=false rgb_camera.exposure:=10000 rgb_camera.gain:=128


ssh unitree@192.168.123.18
source /opt/ros/foxy/setup.bash

/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py \
  config_file:="'/home/unitree/realsense_slam_sensor_qos.yaml'" \
  rgb_camera.profile:=424x240x6 \
  depth_module.profile:=424x240x6 \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=true


/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.profile:=424x240x6 \
  depth_module.profile:=424x240x6 \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=true

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
source go2env.sh
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