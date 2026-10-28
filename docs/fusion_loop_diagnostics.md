# 融合回环诊断与纯雷达建图

`launch/livox_realsense_3dslam.launch.py` 的 `use_camera` 决定回环行为：

- `false`：只进行 LiDAR + IMU 建图，关闭空间、时间邻近回环，关闭视觉特征提取，
  全局回环阈值及跨会话激进阈值设为 1.0，不启动回环诊断节点。
- `true`：保留视觉候选 + ICP 验证，并默认开启空间邻近回环。空间检测含独立的扫描 ICP 分支，
  不依赖相机同向观察；候选仍受里程计位置、搜索半径和工作记忆等条件限制。
  每条新的 `/info` 输出检测状态、候选结果，
  接受回环后单独输出成功事件；每 5 秒输出传感器频率和 SLAM 状态。
  短期记忆为 10 个有效节点，相似节点合并阈值恢复为 0.6；全局回环阈值仍为 0.11。
  `Mem/RehearsalSimilarity` 控制节点合并，降低它会更容易合并相似节点。

## 纯雷达建图

```bash
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
```

## 融合建图

先启动 Livox PointCloud2 与 RealSense RGB、对齐深度、CameraInfo 的驱动。
以下示例外参必须替换为实际测量的 `livox_frame -> camera_link` 变换；
平移单位为米，旋转为弧度。参数名 `cam_to_lidar_*` 在此启动文件中实际用于这段静态 TF。

```bash
source /home/jyfight/Project/rtabmap_slam/setup.bash
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
```

## 日志含义

- `[回环检测状态] WAITING_FOR_SLAM`：诊断已启动，尚未收到 SLAM 处理结果。
- `[回环检测] COLLECTING_HISTORY`：正在积累可供比较的历史关键帧。
- `[回环检测] SEARCHING`：收到本帧搜索结果，当前无候选。
  日志同时显示工作记忆、短期记忆、视觉词数量和实际全局搜索耗时。
- `[回环检测] SEARCH_SKIPPED`：后端报告本帧全局搜索计时均为零，未执行搜索。
- `[回环候选]`：显示当前帧、候选历史帧、候选分数、接受/拒绝/未接受结果和视觉匹配内点。
  小位移时最高候选可能沿用上一帧，日志会明确提示；未接受原因以 RTAB-Map 原始日志为准。
- `[回环成功] GLOBAL_LOOP`：后端报告 `loop_closure_id > 0`，显示匹配帧与位姿图优化统计。
- `[回环成功] PROXIMITY_ICP`：后端报告 `proximity_detection_id > 0`。
  这是诊断程序沿用的事件名称；仅凭此 ID 不能区分空间回环来自视觉还是扫描 ICP，
  需要结合 RTAB-Map 原始日志中的 `[Visual]` / `[Scan matching]`。
- `[回环检测状态] SLAM_TIMEOUT`：超过 7.5 秒未收到新的 SLAM 数据。

JSON Lines 日志保存到
`/home/jyfight/Project/rtabmap_slam/logs/loop_closure_events.jsonl`，事件类型为
`DETECTION_STATUS`、`LOOP_CANDIDATE`、`DETECTION_HEARTBEAT`，以及成功时的
`GLOBAL_LOOP` / `PROXIMITY_ICP`。时间戳采用带时区的 UTC ISO 格式。
`/info` 与 `/rtabmap/info` 转发的同一帧会去重。统计值缺失时显示 `N/A`，
不使用里程计 ICP 内点冒充回环匹配内点。

查看新增日志：

```bash
tail -f /home/jyfight/Project/rtabmap_slam/logs/loop_closure_events.jsonl
```

外参未标定时仍能验证日志输出，但融合配准与实际回环准确性需要真实标定和实地轨迹验证。

## 复查

在 ROS 环境中执行回归检查：

```bash
python3 -m unittest discover -s /home/jyfight/Project/rtabmap_slam/tests -p 'test_*.py'
```

验证诊断进程实际订阅、输出及去重行为：

```bash
python3 /home/jyfight/Project/rtabmap_slam/tests/run_loop_diagnostics_check.py
```

后者在独立 domain 179 中仅通过本机通信发送合成 `/info`，结果写到 `/tmp`；
可用 `RTABMAP_DIAGNOSTICS_TEST_DOMAIN_ID` 指定其他独立 domain。它验证日志功能，
实际融合配准和回环准确性仍需实地验证。

## 2026-10-28：回环配置核查与调整

本机安装的 RTAB-Map 为 0.23.13，以下结论依据同版本本地源码与已有诊断日志。
10 月 28 日日志中存在 3 个 `GLOBAL_LOOP` 接受事件（北京时间 15:31:46、15:31:56、15:32:00）；
这说明后端曾接受回环，不能据此认定外参已标定或回环几何一定正确。

- 单帧扫描与 0.2 对应率不存在必然冲突。前端是 scan-to-map，回环输入和初值不同，
  两者无需采用相同阈值。`Icp/CorrespondenceRatio` 是匹配对应点数与扫描点数基准的比例，
  不是两个体素集合的相交比例。`Icp/VoxelSize=0.05` 也不是匹配距离；本配置的
  `Icp/MaxCorrespondenceDistance` 为体素大小的 10 倍（默认 0.5 m）。
  当前日志没有提供足够的回环 ICP 对应率证据，默认继续使用 0.2。
- `rgbd_sync` 的最大同步时间差由 10 s 收紧为 0.05 s，避免接受相隔很久的图像与深度。
  大窗口只是允许较大时间差，不代表每次都会错配。可用 `rgbd_sync_max_interval` 调整。
- 词袋与视觉配准的最大关键点深度均设为 4 m，可用 `visual_max_depth` 调整。
  这是室内初始设置；远景为主时可能减少可用特征，需结合视觉词和内点统计评估。
- 删除安装版本中不存在的 `Kp/WordsUseDepthCorrection`。
- 融合模式默认开启 `RGBD/ProximityBySpace`，可传 `proximity_by_space:=false` 关闭。
  保留 `Reg/Strategy=2` 的全局视觉+ICP 验证；源码 `Rtabmap.cpp` 的空间扫描分支会单独
  调用 `Memory::computeIcpTransformMulti()`，无需为此把全局策略改成 1。
  策略 1 在没有初值的全局回环中也可由 `Memory::computeTransform()` 先求视觉初值，
  但切换策略并不是开启空间扫描回环的必要条件。
- 装配器和入库雷达扫描均使用 `voxel_size`（默认 0.05 m）降采样。
  前端 ICP 仍使用逐帧去畸变点云，装配只影响后端；默认仍不开装配，按实际运动表现选择。
- `Mem/RehearsalSimilarity` 保持 0.6。源码按 `similarity >= threshold` 合并，
  降到 0.3 会更容易合并，无法解决“有效节点过少”。
  `Mem/STMSize` 默认保持 10，新增 `fusion_stm_size` 供小场景实验调整。
- 外参保持用户确认的原值。`livox_frame -> camera_link` 应填写安装外参，
  不能把驱动内部的光学坐标旋转重复加到这里，也不能仅凭坐标命名把实际安装偏航置零。
- `Vis/InlierDistance` 保持 0.1 m，没有证据支持把容差直接放宽为 0.15 m。

推荐仍使用项目内的检查入口：

```bash
bash /home/jyfight/Project/rtabmap_slam/scripts/start_verified_fusion.sh
```

确认运动跟踪稳定后，单独开启后端 0.3 s 点云装配（默认 5 cm 体素滤波）：

```bash
ASSEMBLE=true bash /home/jyfight/Project/rtabmap_slam/scripts/start_verified_fusion.sh
```

只有原始日志明确提示回环 ICP 对应率不足时，才用以下命令对照实验；
降低门槛可能接受错误约束，不应把“通过更多”当作“匹配更准”：

```bash
ASSEMBLE=true MIN_LOOP_CLOSURE_OVERLAP=0.08 \
  bash /home/jyfight/Project/rtabmap_slam/scripts/start_verified_fusion.sh
```

小房间中若统计持续为 `COLLECTING_HISTORY` 且 WM 没有可比较节点，可单独实验
`FUSION_STM_SIZE=5`。`PROXIMITY_BY_SPACE=false` 可用于对照关闭空间回环。
以上脚本环境变量不修改文件；每次启动创建新数据库。旧数据库不会被重新滤波。
验证覆盖参数解析、数据链路和日志处理，不替代实地回环正确性验证。

## 2026-10-28：可视化配置未加载与双点云叠加修复

`rtabmap_viz` 通过 `-d <ini>` 读取显示配置；此前 launch 只传入 ini 文件路径，
遗漏了 `-d`，实际回退为 `~/.ros/rtabmapGUI.ini`。
本机最近的 `rtabmap_viz` 日志确认读取了该默认文件；其中 `showClouds0/1=true`、
`showScans0/1=true`、`voxelSizeScan0/1=0`，因此同时显示 RGB-D 云与扫描云，且显示层未降采样。

现已修正参数传递，项目默认显示配置为：

- 关闭 RGB-D 地图云和里程计云；显示雷达地图云及当前雷达扫描，便于观察实时更新。
- 雷达显示体素为 0.10 m，点大小为 1 像素。
- 开启显示节点筛选：位置半径 0.30 m、方向阈值 30 度；选择代表节点显示，
  不删除数据库中的节点，不改变位姿图或后端回环输入。
- 仍是每份显示扫描的体素滤波加代表节点选择，不是整个地图跨节点的全局体素合并；
  多个保留节点间仍可能有重叠，不能保证彻底消除重影。

`subscribe_rgbd=false` 只关闭 GUI 直接接收的实时 RGB-D 流；后端 `mapData` 仍携带图像与深度，
GUI 可以据此重建彩色云。因此必须用真正生效的 `showClouds0/1=false` 关闭地图渲染。
相机订阅、视觉词袋与后端视觉回环保持启用。

此前的 0.05 m 入库滤波处理每个节点内部的点；同时显示多份历史扫描仍然会很稠密。
`Camera/Scan/voxelSize` 是相机源扫描预处理设置，不能代替
`Gui/General/voxelSizeScan0` 的地图显示滤波设置。
本次不做天花板裁剪；截图无法可靠确定所需的地图高度阈值。

重新启动可视化后生效。若需要重新开始一轮建图，先在旧建图终端 Ctrl+C 正常保存退出，
再运行项目脚本。启动日志应出现：

```text
rtabmap_viz: Using configuration from ".../config/livox_realsense_gui.ini"
```

当前窗口不会因修改文件自动更新配置。也可在当前窗口的 Preferences / 3D Rendering
手动关闭 Map、Odometry 的 RGB-D Clouds，并调节 Scan 的 voxel size；不必立即终止当前建图。

## 2026-10-28：恢复当前雷达扫描实时显示

此前设置 `showScans1=false` 隐藏了实时里程计扫描，只显示后端保留的地图节点。
当设备停留在原地、连续相似帧合并到同一个节点时，累计地图自然看起来不动。
现恢复 `showScans1=true`，当前雷达扫描采用 0.10 m 显示体素与 2 像素点大小。
`showClouds0/1=false` 保持不变，因此不会恢复相机 RGB-D 彩色点云。

只读采样确认 `/icp_odom`、`/odom_info`、去畸变点云、`/info` 和 `/mapData` 都有数据；
同一次采样中后端地图只含 1 个节点。独立 GUI 从只读数据库成功绘制了 `scan1`，
因此“点云没有保存”或“显示滤波清空了点云”不符合当时证据。
传感器时间戳使用设备启动时钟，日志打印的巨大 delay 和超时警告不能单独作为断流证据。

## 2026-10-28：网格、回环提示与运动拖影

默认关闭底部网格：`widget_cloudViewer/grid=false`。
此前为减少重影同时设置了 `General/showScans1=false`，用户反馈点云看不到后，
已恢复 `General/showScans0=true`、`General/showScans1=true`，保留地图与实时扫描，
两者均使用 1 像素点。隐藏实时扫描不是修复运动拖影的方法。
静止时相邻帧被合并，地图可能不再刷新，但实时扫描仍应持续可见。
当前已打开窗口可在 Preferences → 3D Rendering 中勾选 Odometry 的 Scans，
同时确认 Map 的 Scans 已勾选；无需重新启动建图。

`loop_closure_diagnostics.py` 现在可通过 `--status-window true` 显示置顶状态窗，
融合 launch 默认随 `rtabmap_viz` 开启（参数 `loop_status_window` 可单独覆盖）。
两个已验证启动入口默认都开启提示窗，环境变量 `LOOP_STATUS_WINDOW=false` 可关闭。
提示来自本次收到的 `/info` 或 `/rtabmap/info` 接受结果：

- 全局回环与空间邻近回环分别计数，只对已接受事件显示“回环成功”。
- 成功保持绿色 8 秒，后续普通帧或被拒候选不会立即覆盖成功提示。
- 最近成功时间、节点对持续保留；提示音可在窗口取消，声音是否播放取决于系统设置。
- 数据中断会显示红色；关闭状态窗只隐藏窗口，诊断和建图继续运行。
- 无桌面环境时继续输出终端日志；独立重启诊断后的计数仅代表本次监控。

用户原来的命令继续有效：

```bash
bash /home/jyfight/ros2_ws/rgbd_input_check_20261027/start_verified_fusion.sh
```

显示文件不会自动更新已经打开的窗口。优先在当前窗口修改显示选项；
不要为了刷新界面重新执行建图入口，因为它每次创建新数据库，旧地图不会自动加载。
文件配置在下次启动可视化时生效。旧 GUI 若询问保存界面设置，只在其显示选项
符合当前需求时保存，否则选择 Discard；界面设置与地图数据库保存是两回事。

这轮 19:30:52 启动后的只读日志采样含 1304 条检测结果，全部为 `COLLECTING_HISTORY`，
图最多 2 个节点，1233 帧记录相邻帧合并，未记录成功回环。用户确认大部分时间静止；
源码也表明相邻帧合并同时考虑位移，因此这些记录不构成合并阈值错误的证据。
保留回环阈值与记忆参数，实际验证需移动积累历史，然后返回此前观察过的位置。

移动时点云变厚仍有以下限制：

1. 当前 `livox_frame_stabilized` 只提供 IMU 旋转补偿，缺少扫描内平移去畸变。
   10 Hz 扫描期间，0.5 m/s 平移约 5 cm，这是可能产生拖影的运动量尺度，
   并非测得的地图误差。完整平移补偿需要独立的高频外部里程计 TF；
   现有 `external_odom_frame_id` / `external_odom_topic` 分支可接入该输入。
2. 当前输入预检只验证 RGB、对齐深度和内参，不证明雷达与图像时间同步或外参已标定。
   0.05 s 的 `rgbd_sync_max_interval` 仅限制相机三路同步。
   本机 0.23.13 后端的 `CommonDataSubscriber` 未实现同名最大配对时间差参数，
   本次没有添加无效参数，也没有改变时间戳或同步数据链路。
3. ICP 漂移、雷达与相机时间偏差及真实外参，需要运动数据进一步区分。
   目前仅完成显示与通知改进，尚未验证真实运动地图厚度减小或真实回环成功。

单独执行 `export CAM_X CAM_Y ...` 只导出变量，未赋值时仍使用启动脚本示例值。
两个入口现会明确提示未赋值的变量，launch 拒绝 NaN/Inf 外参。

验证：39 项 Python 回归通过；Qt 离屏检查覆盖中文成功提示、后续帧保持、
关闭窗口后继续轮询；启动脚本语法与 diff 空白检查通过。
Qt 检查使用模拟事件，不能作为现场回环成功的证据。
实时点云刷新与累计地图新增节点是不同现象，设备发生有效运动后才会逐步形成多节点地图。
