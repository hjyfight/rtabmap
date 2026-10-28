# 按指定参考脚本恢复纯雷达建图

2026-10-26，修改 `launch/livox_realsense_3dslam.launch.py`。
参考文件为 `/home/jyfight/livox_realsense_3dslam.launch(1).py`，参考文件未修改。
修改前的工程文件备份为
`launch/livox_realsense_3dslam.launch.py.before_reference_restore_20261026_183916`。

默认只用 `/livox/lidar` 的 PointCloud2：原始点云送入 ICP 产生 `icp_odom`，
同一原始点云在该里程计参考系下累积 0.3 秒，`assembled_cloud` 送入 RTAB-Map。
相机、深度融合、IMU 默认关闭，不启动时间戳转换节点，不使用
`odom_filtered_input_scan` 代替原始输入。去畸变与参考脚本一样强制关闭，
即使传入旧命令中的 `deskewing:=true` 也会关闭，并打印说明。

| 参数 | 恢复后的参考值 |
| --- | --- |
| `Odom/ScanKeyFrameThr` | `0.15` |
| `OdomF2M/ScanSubtractRadius` | `voxel_size`，默认 `0.05` 米 |
| `OdomF2M/ScanMaxSize` | `15000` |
| `OdomF2M/BundleAdjustment` | 字符串 `true`，原样保留参考值 |
| ICP 里程计 `Icp/CorrespondenceRatio` | `0.01` |
| SLAM `Icp/CorrespondenceRatio` | `0.2` |
| `Icp/OutlierRatio` | `0.5` |
| `Icp/MaxTranslation` | `3` |
| `Icp/MaxCorrespondenceDistance` | `voxel_size * 10`，默认 `0.5` 米 |
| `Icp/Iterations` / `Icp/Epsilon` | `20` / `0.001` |
| `RGBD/AngularUpdate` / `RGBD/LinearUpdate` | `0.05` 弧度 / `0.05` 米 |
| `RGBD/ProximityBySpace` | `false` |
| `RGBD/ProximityPathMaxNeighbors` | `10` |
| `Reg/Strategy` / `Reg/Force3DoF` | `1` / `false`（纯 ICP、六自由度） |

参考脚本关闭空间回环；纯雷达又没有视觉词袋输入，因此该默认配置不启用这两种回环。
本次也移除了工程版额外指定的优化器、鲁棒核及其他回环阈值，沿用参考脚本的默认行为。
`OdomF2M/BundleAdjustment` 在 RTAB-Map 中是整数枚举，不能仅凭参考文件中的
字符串 `true` 就断言启用了 BA；本次未额外改变它的含义。

保留的两项运行兼容处理：ICP 进程限制 `OMP_NUM_THREADS=1`、
`OPENBLAS_NUM_THREADS=1`，避免此前记录的线程开销；旧参数
`Grid/FromDepth=false` 换成现行等价参数 `Grid/Sensor=0`，删除本机不再支持的
`Grid/FullUpdate`。这些没有改变参考脚本的 ICP 配准设置。
已有对比实验的显式开关仍可使用，但默认不覆盖参考策略。

## 运行

先在原建图终端 Ctrl+C，等数据库保存且节点退出。雷达驱动继续提供 PointCloud2。
在建图终端运行：

```bash
source /home/jyfight/go2env.sh
source /home/jyfight/Project/rtabmap_slam/setup.bash
export ROS_LOCALHOST_ONLY=0

ros2 launch /home/jyfight/Project/rtabmap_slam/launch/livox_realsense_3dslam.launch.py \
  use_camera:=false \
  use_imu:=false \
  deskewing:=false \
  database_path:=/home/jyfight/Project/rtabmap_slam/maps/lidar_reference_$(date +%Y%m%d_%H%M%S_%N).db \
  gui_cfg:=/home/jyfight/Project/rtabmap_slam/config/livox_realsense_gui.ini
```

与参考文件一致，建图模式使用 `-d` 清空指定的同名数据库，以上命令每次生成新文件名。
此前的地图没有修改，已花的地图也不会因这次配置恢复而自动修复。

## 验证范围

- Python 语法与 `ros2 launch --show-args` 检查通过。
- 核对纯雷达的默认、关闭装配、定位、外部里程计、自定义体素/匹配阈值、旧去畸变开关等 7 种配置；
  除上面的栅格参数版本兼容处理外，节点、有效参数、输入话题与参考文件相符。
- 核对默认只启动 `point_cloud_assembler`、`icp_odometry`、`rtabmap`、`rtabmap_viz`，
  并核对既有三种显式对比模式的分支仍可选择。
- 在隔离 ROS domain 91 中关闭可视化，使用临时数据库启动约 7 秒；
  ICP、装配与 SLAM 正常启动和退出，无错误日志。此测试没有传感器输入。

配置核对结果：`logs/reference_restore_validation_20261026.json`。
启动日志：`logs/reference_restore_startup_20261026.log`。
这些检查不代表真实移动建图已验收，地图重影是否消失需要重新采集移动、转弯过程确认。
