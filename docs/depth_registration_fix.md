# RealSense 深度配准修复（2026-11-06）

机载 RealSense 发布原始彩色、已校正的原始深度和相机内参，工作站使用官方 ROS
`depth_image_proc/register_node`，根据内参及相机内部 TF 把深度投影到 RGB 坐标系。
融合及语义 launch 默认启动配准，RGB-D 同步器和输入检查继续逐帧验证。

本次版本保存 USB 转发接入后的多路图像与 SLAM 配置。15 秒同时采样收到
91 帧 RGB（424×240，约 6.06 Hz）、90 帧右近红外（640×480，约 5.97 Hz）和
56 帧显示用配准深度（424×240，约 3.80 Hz）。配置模板保留双路红外启用设置。
相机朝向、深度、三维地图和二维栅格已通过桥接检查，并经用户确认显示恢复。

## 分辨率

| 数据 | 当前配置 |
| --- | --- |
| RGB | 424×240，6 FPS |
| 原始深度 | 640×480，6 FPS |
| 配准深度 | 424×240，自动跟随 RGB CameraInfo |

两路原始图像不必同分辨率。配准先通过深度内参还原三维点，再用两个镜头之间的
旋转和平移变换到 RGB 坐标系，最后按 RGB 内参投影。这使用相机内部标定，和相机到
雷达的安装外参不同。本机两路 CameraInfo 的畸变系数实测均为零；更换相机后，需要
保证 RGB 图像与其校正后的相机模型一致，非零畸变应先校正图像。

可以先缩放图像，但必须同步变换 CameraInfo 的焦距、主点、投影矩阵及裁剪信息。
单纯 resize 无法补偿镜头基线和视场差异，普通平滑插值还可能混合前后景深度。
当前保留原始深度采样，通过标定重投影生成目标尺寸。

RGB 支持多种采集模式，可以在机载电脑查询：

```bash
ros2 param describe /camera/camera rgb_camera.profile
ros2 param describe /camera/camera depth_module.profile
```

当前设备报告支持 424×240、640×480 等模式。修改下面命令的
`rgb_camera.profile:=640x480x6`，重启相机后配准输出会自动成为 640×480。
此次稳定性验证使用表格中的配置；其他尺寸和帧率需要重新检查传输、处理负载。

## 启动与检查

工作站已安装项目内依赖。迁移到另一台已具备 ROS Humble Desktop 的机器时运行：

```bash
bash scripts/install_depth_registration.sh
source setup.bash
```

机载配置 `/home/unitree/realsense_slam_sensor_qos.yaml` 已更新。原文件备份为
`realsense_slam_sensor_qos.yaml.before_depth_registration_20261106_af5dec4e`。
配置模板为 `config/realsense_slam_sensor_qos.yaml`，不覆盖采集模式参数。
先在旧相机驱动终端按 Ctrl+C，再在机载电脑运行：

```bash
source /opt/ros/foxy/setup.bash
/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py \
  config_file:="'/home/unitree/realsense_slam_sensor_qos.yaml'" \
  rgb_camera.profile:=424x240x6 depth_module.profile:=640x480x6 \
  align_depth.enable:=false enable_sync:=false \
  enable_color:=true enable_depth:=true
```

工作站重新 `source setup.bash` 后，原融合/语义 launch 会启动配准。直接启动语义
launch 时还需要 `source install/semantic/local_setup.bash`；语义启动脚本已处理。
`scripts/start_verified_fusion.sh`、`scripts/start_semantic_slam.sh` 会先启动配准和
检查输入，再启动 SLAM，并在退出时关闭配准进程。

机器人上的雷达和相机已运行时，只需在工作站启动建图。先在上一轮建图终端按
Ctrl+C，等待正常保存退出，再运行下面的 YOLOE 语义建图入口：

```bash
cd /home/jyfight/Project/rtabmap_slam
source ~/go2env.sh
source setup.bash
bash scripts/start_semantic_yoloe.sh
```

不需要物体识别时，将最后一行换成 `bash scripts/start_verified_fusion.sh`。
两种入口只运行一个，都自动完成配准；默认开始新的地图数据库。
当前修复已接入的建图会话可以继续使用，无需为了启用配准而重启。

仅查看配准、不启动建图：

```bash
source ~/go2env.sh
source /home/jyfight/Project/rtabmap_slam/setup.bash
ros2 launch /home/jyfight/Project/rtabmap_slam/launch/realsense_depth_registration.launch.py
```

在另一个已 source 环境的终端检查：

```bash
python3 scripts/check_rgbd_input.py --seconds 15
ros2 topic hz /camera/depth_registered/image_rect
```

单独配准和包含配准的 SLAM 不应同时向同一话题发布。已有外部配准时，为 SLAM 设置
`register_depth:=false depth_topic:=/你的配准深度话题`。软件配准保留原始深度时间戳，
预检查和 RGB-D 同步使用默认 0.05 秒时间差容差，不伪造时间戳。

## 实测与限制

旧 RealSense ROS 驱动为 4.51.1，对齐话题交替输出 424×240 和 640×480。
同分辨率对照收到的三帧“对齐深度”与原始深度完全相同，因此不能仅凭尺寸一致
认定完成几何对齐。

新配准接入原 SLAM 后，连续 30 秒收到 155 帧配准深度、25 次 SLAM 输出。
最大接收间隔分别为 0.45 秒、1.712 秒，输入检查没有新增尺寸错误，ICP 未报告丢失。
记录为 `logs/miti/motion_20261106_114459.summary.json`。
未执行机器人运动，也未验证新的回环成功或地图几何精度。

新启动的会话由 launch 或启动脚本管理配准，正式输出为
`/camera/depth_registered/image_rect`。MITI 主深度面板继续使用
`/miti/aligned_depth/image_raw`，由显示适配器从校验后的 RGB-D 消息发布。
桥接白名单已包含新的 `image_rect` 话题；展示检查同时覆盖深度、地图与相机 TF，
操作见 [MITI 实时展示](miti_live_demo.md)。

机器人时间仍为 1970 年，工作站为 2026 年。旧 RTAB-Map 的部分诊断比较主机时间与
消息时间戳，仍可能报告“5 秒未收到数据”。需保存并停止建图后同步机器时间、重启
传感器和 SLAM，避免在建图期间突然跳变时钟。
