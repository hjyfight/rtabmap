# 直接使用原文件切换相机

使用 `/home/jyfight/livox_realsense_3dslam.launch(1).py`，不再以另一个版本的 `slam_comparison.launch.py` 作为实机对比入口。两组依次运行，驱动保持启动；每组结束时停住后 Ctrl+C 正常保存，确认上一组建图节点已退出。

原文件的相机关闭分支仍硬编码视觉+ICP 配准。本次只让配准策略和视觉特征配置跟随 `use_camera`，并明确数据库参数、修正原诊断脚本执行路径。未改变原 ICP 数值参数、Force3DoF、几何回环开关、装配时间或节点生成阈值。融合分支保持原视觉+ICP 策略，纯雷达分支使用 ICP；原文件没有开启空间几何回环，此次也没有另行启用。备份在原文件旁的 `.backup_20261026_123111_315596` 文件中。

每个终端先执行：

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
```

纯雷达：

```bash
ros2 launch '/home/jyfight/livox_realsense_3dslam.launch(1).py' \
  use_camera:=false use_imu:=false \
  database_path:=/home/jyfight/Project/rtabmap_slam/maps/lidar_original_run2.db \
  gui_cfg:=/home/jyfight/Project/rtabmap_slam/config/livox_realsense_gui.ini
```

融合：

```bash
ros2 launch '/home/jyfight/livox_realsense_3dslam.launch(1).py' \
  use_camera:=true use_imu:=false \
  database_path:=/home/jyfight/Project/rtabmap_slam/maps/rgbd_original_run2.db \
  gui_cfg:=/home/jyfight/Project/rtabmap_slam/config/livox_realsense_gui.ini
```

六个外参可以省略，延用原文件的全零默认值；这表示采用相机与雷达同位置同朝向的假设，不代表已标定。有实际外参时再覆盖。相机话题须匹配原文件默认值或显式覆盖。

原文件建图使用 `-d`，会清空同名已有数据库。每次实验换新的数据库文件名，不要重复覆盖需要保留的地图。

保存后在两个终端分别打开数据库，交互查看最终地图：

```bash
rtabmap-databaseViewer maps/lidar_original_run2.db
```

```bash
rtabmap-databaseViewer maps/rgbd_original_run2.db
```

也可导出两个使用数据库最终优化位姿的扫描点云，供点云查看器对比：

```bash
rtabmap-export --scan --cloud --opt 2 maps/lidar_original_run2.db
rtabmap-export --scan --cloud --opt 2 maps/rgbd_original_run2.db
```

原文件不自动生成先前出图脚本所需的 `cloud.npz` / `graph.json`，不能把这两个数据库直接传入 `plot_slam_comparison.py`。

已检查两种相机开关下的 launch 配置构建；尚未用真实传感器确认重影问题消失。之前结果库查询到一个 map_id，界面中看到多个重叠结构的具体原因仍需结合实际轨迹、TF 和运行日志判断。

## 2026-10-26：实机重影诊断及6DoF修正

`maps/lidar_original_run2.db` 中有 11 个节点，全部属于 map_id=0，只有里程计相邻边，没有回环边。每个节点的 z、roll、pitch 均为零。原配置中 ICP 使用 `Reg/Force3DoF=false`，后端却使用 `true`，会压平传入的三维姿态。实际移动/转动雷达时，这种不一致可能导致点云拼接重影。

已将原文件的后端 `Reg/Force3DoF` 改为 `false`，与 ICP 保持一致；除此之外未修改数值参数。修改前备份为 `/home/jyfight/livox_realsense_3dslam.launch(1).py.before_6dof_20261026_123758_046787`。这条修正替代上文“未改变 Force3DoF”的先前状态描述。

相机开/关两组配置构建已检查，尚未实机复测。请使用新的数据库文件名（如 `lidar_original_run3.db`）重跑，避免清空需要保留的旧结果。旧地图不会因修改启动参数自动重新变好。

此外，相关日志里传感器时间戳约为 6915601 秒，主机时间约为 1792989190 秒，时基明显不同；ICP 的841次处理耗时中位数约0.250秒。需要进一步检查设备时间同步和实时处理能力，不能把日志中十亿秒的 delay 当作真实排队时间。三维姿态修正并不证明其为全部重影的唯一原因。

## 2026-10-26：RGB-D同步的QoS对照结果

用户在独立测试节点中设置 `qos=2, qos_camera_info=2`，其余输入话题、30消息队列和10秒近似同步窗口与原同步节点一致，测得 `/rgbd_probe/rgbd_image` 约11Hz。原节点未观测到输出。该结果说明传感器三路数据能够形成RGB-D，显式QoS配置是已验证的有效配置；可靠性策略对具体接收行为的影响仍不能仅由发布端Reliable就完全解释。

原文件中的 `qos` 参数此前只传给共享参数，未传给 `rgbd_sync` 和 `point_cloud_assembler`。已补齐这两处参数传递，包括 `qos_camera_info` 和 `qos_odom`，以免同步器输出BestEffort而后端仍要求Reliable。默认值保留1；重启命令须显式加 `qos:=2`，使整条链路一致。修改前备份：`/home/jyfight/livox_realsense_3dslam.launch(1).py.before_qos_20261026_135400_033452`。相机开关两分支的参数传递已检查；完整后端实机结果待重启确认。

用户测得同步输出11Hz时，同一个测试节点仍持续报告“5秒未收到数据”。源码的SyncDiagnostic把主机当前时间和消息header时间比较；当前两者时基不同，此警告不能单独判断无消息。不要为了消除警告给单一路径随意重写时间戳。

## 2026-10-26：RealSense实机链路恢复

已通过 SSH 在 `unitree@192.168.123.18` 操作相机驱动，未重启 Livox 驱动。相机只有一台 RealSense D435i；RGB Camera 和 Stereo Module 都是它的内部模块。实测安装的 ROS 驱动为 4.51.1、SDK 为 2.51.1、固件为 05.15.00.02。

640×480、开启深度对齐时，重启和降至6帧仍未恢复持续深度输出；关闭对齐时原始深度能够输出。直接调用同一SDK的独立程序，彩色、深度以及深度对齐都能工作。将ROS驱动降为424×240、6帧并使用SensorData QoS后，正式链路恢复。该对照支持使用低负载配置恢复，但尚未唯一确定是驱动内部处理还是通信负载造成640×480配置失败。

连续20秒正式链路采样：彩色4.19Hz、对齐深度6.03Hz、CameraInfo 4.19Hz、`/rgbd_image` 4.24Hz、`/info` 0.80Hz。结果文件 `logs/sensor_sync_recovery_stable.json`。后端日志持续出现新的处理编号，`maps/rgbd_original_run6.db` 已写入1个地图节点；尚未验证移动建图效果或回环。

### 当前相机启动配置

2026-10-29 修正：用户通过 `ros2 param describe /camera/camera depth_module.profile`
确认这台相机不支持深度 `424x240x6`，支持 `640x480x6`。旧命令中的深度模式会被
驱动拒绝并回退，本次日志实际打开的是 `640x480x15`。下方命令已改为支持的深度
`640x480x6`，彩色仍为 `424x240x6`；相机 YAML 只有 QoS 参数，没有覆盖模式。
上文历史记录的 `424×240` 彩色／对齐输出不能证明原始深度也采用了该模式。

已保持该驱动在机载电脑运行，当前无需再启动第二个。以后重启相机时，先在原驱动终端Ctrl+C；本次后台恢复的launch PID为5615，可核实后用 `kill -INT 5615` 正常停止，PID会随重启变化。然后在机载电脑运行：

```bash
source /opt/ros/foxy/setup.bash
/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py \
  config_file:="'/home/unitree/realsense_slam_sensor_qos.yaml'" \
  rgb_camera.profile:=424x240x6 \
  depth_module.profile:=640x480x6 \
  align_depth.enable:=true \
  enable_color:=true enable_depth:=true enable_sync:=true
```

此旧版本launch读取config_file需要上述两层引号。机载电脑配置文件已保存，其内容同本项目 `config/realsense_slam_sensor_qos.yaml`。本次后台日志：`/tmp/realsense_slam_small.72xm2A.log`。

### 工作站检查环境

正式建图使用CycloneDDS，并指定连接机器人的网卡。仅执行项目setup而终端DDS环境不同，会出现能发现话题却完全采不到消息的误导结果。在工作站另开终端检查：

```bash
cd /home/jyfight/Project/rtabmap_slam
source setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enxec1ac312a316"/></Interfaces></General></Domain></CycloneDDS>'
python3 scripts/check_sensor_sync.py --require-slam --seconds 20
```

下次融合启动仍使用原文件，参数加 `use_camera:=true use_imu:=false qos:=2`。当前已有融合节点运行，无需重复启动，也不要重新启动同名run6数据库（原文件 `-d` 会清空）。结束采集时在建图终端Ctrl+C，等待正常退出保存。

机载电脑系统时间仍处于1970年，工作站时间为2026年；因此“5秒未收到数据”和巨大delay可能仍出现。传感器各自时间戳关系、同步配对精度和地图重影尚需后续验证；本次恢复证明数据链路已接通，不证明回环优化或地图精度已经通过验证。
