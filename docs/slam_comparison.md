# 纯雷达与 RGB-D 辅助回环的地图对比

本流程使用当前的 `launch/livox_realsense_3dslam.launch.py`，支持两种完整配置：

| 模式 | 共同输入与处理 | 模式差异 |
| --- | --- | --- |
| `lidar` | Livox 点云、内置 IMU、旋转去畸变、ICP 里程计 | 不启用回环 |
| `rgbd` | 同上 | RGB-D 提供视觉候选及深度验证，并开启空间邻近回环 |

两组地图几何均来自雷达，深度点云不拼入地图。旧对比入口中 `depth`、
`use_color_loop`、`fuse_depth`、`geometry_loop` 等参数已不适用于当前主 launch，
现已移除，避免参数被忽略却被当作实验条件。

## 直接比较已有数据库

必须先正常结束两轮建图并保存数据库，保证最终优化位姿已经落盘。运行：

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
python3 scripts/compare_slam_databases.py \
  maps/livox_imu_20261028_202634.db \
  maps/livox_realsense_20261028_202218_584028279.db
```

第一个参数始终为纯雷达数据库，第二个为融合数据库。默认在
`logs/comparison/saved_maps_日期时间/` 创建全新结果目录；也可用
`--output /完整路径/新目录` 指定。已有目录会拒绝覆盖。

本机这两份已完成的比较结果位于：
`logs/comparison/walk_20261028_2022_2026/comparison_report.md`。
纯雷达 31 个节点、无回环；融合 14 个节点、5 条空间邻近回环、0 条全局回环。
两组保存的轨迹覆盖不同，不能用这次结果直接给出相机带来的精度改善百分比。

## 后续重新各走一圈

先启动 Livox 驱动，保证 `/livox/lidar` 为 PointCloud2、`/livox/imu` 持续发布。
融合组还需已对齐的 RealSense RGB、深度、内参和相机内部 TF。
以下命令在同一个终端依次运行；每轮结束后停稳等待约 5 秒，再按 Ctrl+C，
等所有建图节点退出并保存数据库。传感器驱动保持运行。

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
PAIR="$PWD/logs/comparison/walk_$(date +%Y%m%d_%H%M%S)"

# 第一轮：雷达 + IMU
ros2 launch ./launch/slam_comparison.launch.py \
  mode:=lidar run_dir:="$PAIR/lidar" \
  use_imu:=true filter_imu:=true deskewing:=true assemble:=false
```

第一轮结束后回到相同起点及朝向。保留已有的实测 `CAM_*` 外参赋值，
不要仅执行 `export CAM_X ...` 而未赋值。融合组启动前检查相机输入：

```bash
python3 scripts/check_rgbd_input.py --seconds 5 --output /tmp/comparison_rgbd_input.json
python3 scripts/verify_fusion_input.py /tmp/comparison_rgbd_input.json

# 确认输入检查通过，再运行第二轮。缺失外参时下面的 shell 检查会指出缺失项。
ros2 launch ./launch/slam_comparison.launch.py \
  mode:=rgbd run_dir:="$PAIR/rgbd" \
  use_imu:=true filter_imu:=true deskewing:=true assemble:=false \
  cam_to_lidar_x:="${CAM_X:?请先给 CAM_X 赋值}" \
  cam_to_lidar_y:="${CAM_Y:?请先给 CAM_Y 赋值}" \
  cam_to_lidar_z:="${CAM_Z:?请先给 CAM_Z 赋值}" \
  cam_to_lidar_roll:="${CAM_ROLL:?请先给 CAM_ROLL 赋值}" \
  cam_to_lidar_pitch:="${CAM_PITCH:?请先给 CAM_PITCH 赋值}" \
  cam_to_lidar_yaw:="${CAM_YAW:?请先给 CAM_YAW 赋值}"
```

沿尽量相同的路线、速度和方向行走，结束位置和朝向也保持相同。两组共用话题和 TF，
不能同时启动。每轮目录内保存 `map.db` 和 `manifest.json`（实际传入的 launch 参数）。
完成第二轮并正常保存退出后：

```bash
python3 scripts/compare_slam_databases.py \
  "$PAIR/lidar/map.db" "$PAIR/rgbd/map.db" --output "$PAIR/comparison"
```

默认显示实时雷达扫描与地图，底部网格关闭。融合组自动带回环状态窗。

## 输出与比较方法

- `figures/top_view.png`：统一比例、统一高度配色的并排俯视图。
- `figures/comparison.png`：三维地图、俯视图及同高度水平切片。
- `figures/trajectories.png`：各自 ICP 原始轨迹与已保存优化轨迹。
- `figures/lidar_map.png`、`figures/fusion_map.png`：独立三维视图。
- `comparison_report.md`、`metrics.json`：点数、节点数、回环约束及轨迹指标。
- `parameter_differences.json`：两份数据库记录的后端参数差异。
- `lidar/`、`rgbd/`：数据库快照、PLY、带节点编号的位姿、导出日志及出图数据。

蓝线为相邻关键帧图边，红线为全局回环，橙线为空间邻近回环；
约束来自实际数据库，同一约束的两个方向只计一次。
这些是最终图中的约束数，不是累计触发事件数。

默认两组都以 5 cm 全局导出体素、0.3–20 m 扫描范围处理。
地图和轨迹变换到各自第一帧坐标系，未对地图做 ICP 对齐或缩放。
导出的 PLY 本身保留原始数据库坐标系，可用于独立三维查看。
水平切片默认在初始雷达坐标 Z=0 附近，宽度 0.3 m，可用
`--slice-z 0.5 --slice-width 0.2` 调整。仅重绘已有导出结果时运行：

```bash
python3 scripts/plot_slam_comparison.py \
  "$PAIR/comparison/lidar" "$PAIR/comparison/rgbd" \
  --output "$PAIR/comparison/figures" --slice-z 0.5 --slice-width 0.2
```

此流程从 SQLite 一致性快照读取已保存优化位姿，不修改原数据库、不重做回环、
不依赖 `/cloud_map`（当前主 launch 关闭了占据栅格生成）。未保存最终优化位姿时会报错，
不会静默重新优化后把结果当作在线建图结果。

点数多不等于更准确；关键帧折线长度不是完整实际行走距离；只有实际回到同一点时，
估计首尾距离才适合讨论闭合误差。两次真实采集还受视角、速度和掉帧影响。
若要严格控制传感器输入，可录制同一份包含雷达、IMU、RGB-D、相机内参和传感器 TF
的 rosbag 后分别回放。绝对精度结论还需要真值或独立测量。

当前已用用户指定的两份真实数据库完成导出与出图，并通过相关回归测试；
图中差异不被解释成已经验证的精度提升。
