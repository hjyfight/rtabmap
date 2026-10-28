# 融合回环诊断与纯雷达建图

`launch/livox_realsense_3dslam.launch.py` 的 `use_camera` 决定回环行为：

- `false`：只进行 LiDAR + IMU 建图，关闭空间、时间邻近回环，关闭视觉特征提取，
  全局回环阈值及跨会话激进阈值设为 1.0，不启动回环诊断节点。
- `true`：保留视觉候选 + ICP 验证，每条新的 `/info` 输出检测状态、候选结果，
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
  诊断仍兼容 `proximity_detection_id > 0` 的空间回环事件；本入口不启用空间回环。
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

在 ROS 环境中执行 26 项回归检查：

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
