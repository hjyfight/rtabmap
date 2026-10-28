# Livox IMU 与运动补偿修复

2026-10-27，修复 `launch/livox_realsense_3dslam.launch.py`。
此前 `lidar_reference_restore.md` 记录的是历史配置，不是当前默认参数。

## 修改结果

- 默认 `use_camera=false`、`use_imu=true`、`deskewing=true`、`assemble=false`。
- 去掉强制关闭去畸变的赋值，以及纯雷达模式下把默认 `/imu/data` 禁用的判断。
- Madgwick 订阅 `/livox/imu`，明确输出到 `imu_topic`，包含模拟时钟参数。
  `filter_imu=false` 可接入已有方向估计，避免重复启动滤波器。
- ICP 的 `wait_imu_to_init=true` 会实际创建 IMU 订阅；仅改 remap 不能替代它。
  ICP 与 RTAB-Map 各只有一条 `imu` remap。
- 自动参考系采用 `icp_odom -> livox_frame_stabilized -> livox_frame`。
  ICP 的 `guess_frame_id` 使用稳定参考系，避免 ICP 和 IMU 同时给雷达发布两个父节点。
- 用 `scripts/livox_pointcloud_time.py` 将 FLOAT64 逐点绝对时间归一为秒。
  单位通过 header 判断，支持设备启动时钟和 Unix 时钟；保留 header、坐标、强度及点序。
  本机原去畸变函数误把约 `6.9e15` 的启动时钟纳秒值当成微秒，会出现负时间异常。
- 前级统一去畸变，输出 `/livox/lidar/deskewed`，ICP、可选装配器、后端均使用它。
  ICP 内部去畸变关闭，避免重复补偿。默认逐点查询 IMU TF；CPU 不足时可测试
  `deskewing_slerp=true`，改为插值扫描两端姿态。
- 纯雷达模式关闭空间、时间邻近回环；`Rtabmap/LoopThr` 与
  `RGBD/AggressiveLoopThr` 均为 1.0，保持视觉特征提取关闭，不启动回环诊断。
  相机模式继续使用视觉回环与 ICP 验证，并打印检测状态、候选结果和成功事件，
  详见 [融合回环诊断](fusion_loop_diagnostics.md)。
- 保留 `Reg/Force3DoF=false` 的三维运动，启用本机已安装的 g2o 优化器，
  IMU 开启时 `Optimizer/GravitySigma=0.3`，关闭 IMU 时自动为 0。
- 装配默认关闭；开启时只装配补偿后的点云，默认窗口 0.3 秒。
- 按官方 LiDAR 示例将 `Odom/ScanKeyFrameThr` 调至 0.4，使匹配率下降时较早更新几何关键帧。
  `OdomF2M/BundleAdjustment` 改为合法整数枚举 0；纯 ICP 不依赖视觉 BA。
  ICP 限制 OpenMP/BLAS 线程数为 1，避免此前实测的线程开销。

## 对原分析的补充

原默认配置确实没有使用 IMU 和去畸变，纯雷达没有视觉输入，空间回环和重力约束也关闭。
但这里恢复的是 IMU 方向预测、旋转补偿和后端重力约束，不是紧耦合激光惯性预积分。
没有磁力计时 yaw 仍可能缓慢漂移，IMU 也不能替代正确的轴向、静态 TF 和时间同步。

ROS 2 对同一名称的 remap 采用首条匹配规则，追加的 `imu_not_used` 不一定覆盖前条规则；
原默认配置的禁用分支已经把第一条映射设成了 `imu_not_used`。本次删除重复映射消除歧义。

装配器会把里程计误差带入后端点云，但 ICP 输入来自装配器之前，因此这不是直接反馈到
ICP 的死循环。关闭默认装配可以先隔离运动跟踪问题。

## 运行

先在现有建图终端 Ctrl+C，让数据库保存。使用 PointCloud2 模式的 Livox 驱动，
例如本机的 `rviz_MID360_launch.py`（`xfer_format=0`）。
`msg_MID360_launch.py` 默认是 CustomMsg，不能直接输入此链路。

在新建图终端运行：

```bash
source /home/jyfight/Project/rtabmap_slam/setup.bash
ros2 launch /home/jyfight/Project/rtabmap_slam/launch/livox_realsense_3dslam.launch.py \
  use_camera:=false use_imu:=true deskewing:=true assemble:=false \
  database_path:=/home/jyfight/Project/rtabmap_slam/maps/livox_imu_$(date +%Y%m%d_%H%M%S_%N).db
```

设备先静止数秒，让方向估计初始化，然后缓慢移动、转弯，检查墙面重影和跟踪。
原 launch 的建图 `-d` 行为保留，会清空指定的同名数据库；命令用新文件名开始验证。
之前已经损坏的地图需要重新采集。

有效四元数已由外部节点提供时，传 `filter_imu:=false imu_topic:=/your/imu`。
IMU frame 与 `frame_id` 不同时，需要真实的静态 TF。自定义其他型号点云格式时可传
`normalize_livox_timestamps:=false`，并确认该格式已由 RTAB-Map 正确支持。
独立外部里程计可在 `use_imu=false` 时提供去畸变参考系。

## 验证范围

12 项回归检查覆盖设备启动/Unix 时钟、多种时间单位、大小端与行填充、输入保留、无效时间，
以及默认、装配、关闭 IMU、关闭去畸变、自定义 IMU、模拟时钟、相机/定位和外部里程计分支。
另做隔离 ROS domain 178 中的合成旋转测试，检查去畸变几何、ICP 姿态及 TF 父子关系。
8 秒的无噪声测试收到 59 帧旋转补偿点云和 77 条里程计消息，姿态跟踪检查通过，
数据库保存 7 条重力约束。无负时间异常。可在 source ROS 后重跑：

```bash
python3 -m unittest discover -s /home/jyfight/Project/rtabmap_slam/tests -p test_livox_motion_config.py
python3 /home/jyfight/Project/rtabmap_slam/tests/run_livox_rotation_check.py
```

运行测试需要本机 ROS 通信；运行时验证只在独立 domain 中发布合成传感器数据，并写入 /tmp。
这些检查验证软件链路；实际设备运动建图仍需重新采集验收。
