# 原文件的性能和运动补偿处理路径

后续更新（2026-10-26）：工程 launch 已按用户要求恢复指定参考脚本的纯雷达逻辑，
去畸变强制关闭。当前运行命令见 [参考脚本恢复说明](lidar_reference_restore.md)。
下文保留为此前诊断记录，不代表当前 launch 配置。

2026-10-26。本轮未修改原launch、运行中的节点、雷达驱动或系统库。

## 1. 处理速度：已验证线程限制

独立诊断使用正式ICP的同一组参数，保留`Odom/ScanKeyFrameThr=0.15`等数值，只限制`OMP_NUM_THREADS=1`和`OPENBLAS_NUM_THREADS=1`，关闭诊断节点的TF发布、使用独立命名空间。

12秒运行109帧，完整处理耗时中位数0.081482秒。结果是独立对照，不能当作重跑正式节点后的频率或移动地图验收结果。日志：`logs/icp_single_thread_original_20261026.log`。

先在当前建图终端Ctrl+C一次，等待数据库保存和所有建图节点退出，再在工作站运行同一原文件：

```bash
source /home/jyfight/go2env.sh
source /home/jyfight/Project/rtabmap_slam/setup.bash
export ROS_LOCALHOST_ONLY=0
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1

ros2 launch '/home/jyfight/livox_realsense_3dslam.launch(1).py' \
  use_camera:=false use_imu:=false \
  database_path:=/home/jyfight/Project/rtabmap_slam/maps/lidar_threads_run1.db \
  gui_cfg:=/home/jyfight/Project/rtabmap_slam/config/livox_realsense_gui.ini
```

原文件`-d`清空同名数据库，须确保新文件名未用于需要保留的实验。相机模式下同样适用线程限制；融合链路还应沿用先前确认的`qos:=2`配置。

## 2. 运动补偿：发现了必须先修复的逐点时间兼容问题

仅在独立诊断中直接设置`deskewing=true`，原ICP二进制在处理两帧后异常退出：`cannot store a negative time point in rclcpp::Time`。正式建图未受影响。日志：`logs/icp_single_thread_deskew_20261026.log`。

实机单帧读数：

- header时间：6932864.055079328秒；
- 首点`timestamp`：6932864055079328，FLOAT64、纳秒；
- 两者满足首点timestamp / 1e9 = header时间；
- 一帧逐点时间跨度：99440109纳秒，约0.09944秒。

因此逐点时间和header指向同一时刻，只是单位不同。当前安装的RTAB-Map补偿函数不能正确解析这一输入。在单帧C++测试中，直接调用同一RTAB-Map补偿函数：原始数据抛出上述异常；测试副本只将逐点timestamp除以1e9转成秒，函数成功返回20064个点。测试没有发布修正数据，没有改动正式话题或坐标。日志：`logs/livox_deskew_format_check_20261026.log`。

RTAB-Map官方记录了Livox去畸变中同样的负时间异常和时间转换修复：<https://github.com/introlab/rtabmap_ros/issues/1267>。不能据此保证某个升级版本一定覆盖本机的设备启动计时格式，须实测。

完整修复顺序：

1. 在现有时间转换实现中兼容Livox逐点纳秒数据，保留真实采集时间；不应简单把header改为当前系统时间。
2. 恢复原launch已有deskewing开关。目前文件内部强制关闭它，仅传`deskewing:=true`不会生效。
3. 确认点云装配/建图使用经过补偿的扫描。只补偿ICP输入、继续装配未补偿的原始点云，并不能保证最终地图也消除运动畸变。
4. 重新采集移动、转动过程，检查正式ICP频率、装配延迟及墙面重影。已有异常地图不会随配置更改自动修复。

这些涉及现有模块的时间格式兼容和原有开关/数据连接，不需要另写SLAM算法。本轮没有将这些兼容性修改应用到正式链路，也未验证地图重影已消失。
