# 纯雷达与相机辅助建图对比

> 更新：用户确认应以 `/home/jyfight/livox_realsense_3dslam.launch(1).py` 为准。下文 `slam_comparison.launch.py` 是先前基于另一个版本的流程，当前不建议继续用它排查实机重影。原文件已补齐 `use_camera` 的纯 ICP 配置；直接使用原文件的最新命令见 [原文件开关对比](original_camera_switch.md)。

可以实现。当前目录中的实际文件名是 `launch/livox_realsense_3dslam.launch.py`，没有找到带 `(1)` 的副本。本次修改以此文件为基础。

原代码默认 `use_color_loop=false, fuse_depth=true`，主要使用 RealSense **深度**点云补充 LiDAR 几何，并使用空间临近 ICP 回环；开启 `use_color_loop=true` 后，才使用 RGB 词袋识别重访地点、深度支持视觉特征的三维匹配，再结合 ICP 验证并优化位姿图。两种相机辅助方式不能混称为同一种优化。

## 三种可运行配置

| mode | 建图输入 | 回环方式 | 对比目的 |
| --- | --- | --- | --- |
| lidar | LiDAR | 空间几何回环 | 公共基线 |
| depth | LiDAR + 深度点云 | 空间几何回环 | 对比原代码默认的深度融合 |
| rgbd | LiDAR；RGB-D 提供回环信息 | 视觉回环 + 空间几何回环 | 展示 RGB-D 辅助全局优化 |

三组使用同一份 launch 的 ICP 和点云处理参数。对比入口强制保留各组空间几何回环，避免把“无回环基线”和“有回环融合”混为相机的贡献。`rgbd` 不把深度点云拼入地图，适合观察相机约束对 LiDAR 地图几何一致性的作用；`depth` 会改变地图采样密度，点数变多本身不能证明定位更准确。

## 不录制 rosbag：实时建图，保存两组最终地图

可以直接用实时传感器，分两次保存地图。对比入口中的记录器只保存地图输出、位姿图和回环统计，不录制原始传感器 rosbag。

每个终端先执行：

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
```

终端 1 启动 MID360，必须使用输出 PointCloud2 的配置：

```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

终端 2 启动相机，显式使用根命名空间，匹配 `/camera/color/...` 话题：

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=/ camera_name:=camera \
  enable_color:=true enable_depth:=true \
  align_depth.enable:=true publish_tf:=true
```

终端 3 先启动纯雷达建图：

```bash
ros2 launch ./launch/slam_comparison.launch.py \
  mode:=lidar run_dir:=logs/comparison/live_lidar_run1 \
  use_sim_time:=false use_imu:=false
```

走完路线后停住，等待至少 5 秒，在终端 3 按 Ctrl+C 正常保存并退出。传感器驱动保持运行。将设备放回相同起点和朝向，再在终端 3 启动融合组。先输入实际标定外参，再执行命令：

```bash
read -r -p "输入 x y z roll pitch yaw（米、弧度）: " \
  CAM_X CAM_Y CAM_Z CAM_ROLL CAM_PITCH CAM_YAW

ros2 launch ./launch/slam_comparison.launch.py \
  mode:=rgbd run_dir:=logs/comparison/live_rgbd_run1 \
  use_sim_time:=false use_imu:=false \
  cam_to_lidar_x:="$CAM_X" \
  cam_to_lidar_y:="$CAM_Y" \
  cam_to_lidar_z:="$CAM_Z" \
  cam_to_lidar_roll:="$CAM_ROLL" \
  cam_to_lidar_pitch:="$CAM_PITCH" \
  cam_to_lidar_yaw:="$CAM_YAW"
```

沿尽量相同的路线、速度和方向再走一次，经过之前访问的区域以提供回环机会。结束时停住，等待至少 5 秒，然后 Ctrl+C 保存。两组地图分别保存在两个目录中的 `map.db`，同时自动保存 `cloud.npz`、`graph.json` 等出图文件。

离线生成最终地图对比，不需要传感器继续运行：

```bash
python3 scripts/plot_slam_comparison.py \
  logs/comparison/live_lidar_run1 \
  logs/comparison/live_rgbd_run1 \
  --output logs/comparison/live_figures
```

输出 `lidar_map.png`、`fusion_map.png` 和 `comparison.png`。出图脚本读取两个实验目录中的地图点云和位姿图快照，不能只传入或只保留两个 `map.db` 文件；请保留整个实验目录。

也可以在两个终端分别打开数据库，使用 Database Viewer 的三维地图视图交互查看：

```bash
rtabmap-databaseViewer logs/comparison/live_lidar_run1/map.db
```

```bash
rtabmap-databaseViewer logs/comparison/live_rgbd_run1/map.db
```

实时走两次适合展示最终地图外观，但两次传感器输入不同，差异也会受路线、速度和环境变化影响。如果需要严格量化相机贡献，再使用下述相同 rosbag 回放流程。

## 数据准备（可选：使用同一份 rosbag 做受控对比）

建议录制一次包含重访路段的闭合路线，然后分别回放。开始录制时只运行传感器驱动，避免将其他 SLAM 实例的 TF 混入数据。需要：

- `/livox/lidar`，类型为 `sensor_msgs/PointCloud2`。
- RGB、对齐深度及其 CameraInfo（默认话题见下面命令）。
- `/tf_static` 中 RealSense 的 camera_link 到 optical_frame 变换；有运动传感器内部 TF 时还需要对应 `/tf`。
- 真实的 LiDAR 到 camera_link 外参；两次相机辅助实验必须使用同一份标定。默认全零只是占位值。
- RGB 和对齐深度必须使用可比较的时间戳。原 RGB-D 同步器允许较大的时间差，不能据此认为采集数据已经正确同步。

在项目根目录执行，例如：

```bash
source setup.bash
ros2 bag record -o logs/comparison_input \
  /livox/lidar \
  /camera/color/image_raw \
  /camera/aligned_depth_to_color/image_raw \
  /camera/color/camera_info /tf /tf_static
```

录制结束后停止实时传感器和其他建图节点。只保留当前实验，依次运行两组，不能把这些配置直接同时启动：它们共用话题和 TF 名称。

## 分别回放建图

第一轮，终端 A：

```bash
source setup.bash
ros2 launch ./launch/slam_comparison.launch.py \
  mode:=lidar run_dir:=logs/comparison/lidar_run1 \
  use_sim_time:=true use_imu:=false
```

终端 B（每一轮都从头播放同一个 bag）：

```bash
source setup.bash
ros2 bag play logs/comparison_input --clock --rate 0.5
```

播放结束后等待至少 5 秒，再在终端 A 按 Ctrl+C 正常结束，保存数据库和最后一帧地图。确认原建图节点全部退出。

第二轮，终端 A：

```bash
source setup.bash
ros2 launch ./launch/slam_comparison.launch.py \
  mode:=rgbd run_dir:=logs/comparison/rgbd_run1 \
  use_sim_time:=true use_imu:=false
```

**第二轮启动命令还必须追加实测的** `cam_to_lidar_x/y/z` 和 `cam_to_lidar_roll/pitch/yaw` 六个参数，例如参数形式为 `cam_to_lidar_x:=测量值`（单位 m、rad）。这里不提供虚构外参。相机话题与默认不同时，两轮都应传入相同的话题覆盖参数。随后在终端 B 再次执行同一个播放命令。

若需要对比原代码的默认深度融合，将第二轮改为 `mode:=depth run_dir:=logs/comparison/depth_run1`，也必须提供真实外参。可额外运行这一组，形成三组消融实验。

两轮都显式关闭 IMU，以免原代码根据是否启用相机自动改变 IMU 链路。其他参数（体素、装配时间、重叠率、回放速度等）保持一致。资源不足时降低两组的回放速度，并检查是否有掉帧或同步丢失。

每轮必须使用全新的结果目录。原始 launch 建图模式带 `-d`，会清空传入的已有数据库；新的对比入口在发现结果目录已经存在时直接拒绝启动，避免覆盖已有结果。

## 生成两张地图和一张并排对比图

```bash
source setup.bash
python3 scripts/plot_slam_comparison.py \
  logs/comparison/lidar_run1 logs/comparison/rgbd_run1 \
  --output logs/comparison/figures
```

深度融合实验将第二个目录换为 `logs/comparison/depth_run1`。输出：

- `lidar_map.png`：纯雷达地图。
- `fusion_map.png`：相机辅助地图。
- `comparison.png`：两组地图的三维视图及俯视图，统一坐标范围、高度配色和视角。
- `summary.json`：保存点数、节点数及最终图中的回环约束数量。

蓝线表示里程计图边，红线表示全局回环，橙线表示空间临近回环，紫线表示时间临近回环。连线来自实际 MapGraph 约束，而不是根据轨迹相近自行猜测。图上的连线数量是最终图中去重后的约束数，不等同于回环事件次数。

每轮目录还包含 `map.db`、`manifest.json`（完整启动参数）、`graph.json`（最近一次优化图）、`cloud.npz`（最近一次地图）和有回环时生成的 `events.jsonl`（接受的回环事件、时间、优化误差、迭代次数、视觉内点及原始统计）。没有发布的指标保留为空，不填成零。记录器每 5 秒保存一次，正常退出时再保存一次；地图与图是各自最近一次收到的消息，不保证严格同时间戳。

默认开启 RTAB-Map 可视化窗口；无显示环境可加 `viz:=false`。两组地图都以相同的地图点云生成配置保存，便于出图。

## 如何判断“有优化”

展示时可以用相同区域的墙面重影、闭合处错位及轨迹形态作定性比较，再用 `events.jsonl` 和回环连线说明接受了哪些约束。回环次数更多、图更稠密或优化内部误差更小，都不能单独证明绝对精度更高。若需要“误差下降百分比”，必须有真值轨迹或独立测量参考。

当前只完成配置构建、记录和出图组件的自检，测试数据仅用于验证工具，不代表真实建图效果。尚未用同一份 Livox + RealSense 实测数据跑完对比，因此没有生成或宣称真实精度改善结果。
