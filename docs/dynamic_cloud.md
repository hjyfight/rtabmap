# 动态点云清理

MITI 三维地图默认开启“动态清理”。它保留环境地图，叠加整帧替换的当前扫描；原始 `/cloud_map` 与地图数据库保持原样。右下角“动态清理”开关可对照原始累积地图。关闭时历史残影会再次可见，这是原始数据的内容。

## 原因与处理

原来的三维视图使用 `/cloud_map`，它由历史关键帧点云拼接得到。当前主 launch 的 `Grid/RayTracing=false`，`map_always_update=false`。静止机器狗不一定触发新地图，移动人的旧点会一直保留；网页刷新率无法解决这一点。

`scripts/dynamic_cloud_node.py` 使用 `/cloud_map` 完整快照及 `/livox/lidar/deskewed` 扫描，输出：

- `/miti/clean_map`：保持原坐标的环境点云，去掉已被连续射线证据否定的历史点，最多 2 Hz。
- `/miti/live_scan`：最多 5 Hz 处理最新可配对扫描；每条消息替换前一帧，不累积；中断 1.5 秒发布空帧。
- `/miti/dynamic_cloud_status`：实际输出频率、耗时、等待位姿时间、清除历史点数。

清理使用 OctoMap 的真实三维体素射线遍历。默认 10 cm 体素、12 m 范围、20 cm 回波终点保护距离，以及终点邻域保护。一个历史体素需在 3 次不同采集时间的扫描中被射线穿过，才从显示地图移除；同帧的多条射线和重复数据只记一次。新回波命中时恢复该体素。未完成确认的证据间隔超过 5 秒重新计数，已确认空闲的记录持续保留，原始地图重发不会重新加回残影。

不按时间删除整个地图，也不把遮挡、未观测当作空闲。清理适用于有运动的物体，不依赖“person”分类是否正确。它是体素尺度的几何判断，玻璃、极细结构及配准误差仍可能造成误清理；原图可随时对照。

扫描必须取得采集时刻的里程计变换，再组合最新 `map → icp_odom` 校正。不会拿最新位姿代替缺失的历史位姿。输入队列有上限并优先处理最新可配对帧，超时数据丢弃。地图会话变化、传感器时间回退或显著回环校正会重置清理证据；回环后等待新的地图快照。

墙面和二维栅格显示读取 `/miti/clean_map`，避免继续从已清除的历史点提取虚假墙面或障碍。栅格仍沿用已有的地面点投影算法；这项修改不等同于补齐导航所需的全部可通行空间，也不修改控制或 SLAM 配准。

## 启动与验证

`bash scripts/start_miti.sh` 自动启动模块；首次运行或 C++ 源更新后执行 `scripts/build_dynamic_cloud.sh`，使用本机已安装的 OctoMap 库构建到 `build/dynamic_cloud/`。参数位于 `config/dynamic_cloud.yaml`。

单元测试：

```bash
bash scripts/build_dynamic_cloud.sh
source setup.bash
/usr/bin/python3 -m unittest discover -s tests -p 'test_dynamic_cloud*.py' -v
```

现场读数（展示服务已启动）：

```bash
source scripts/ros2_wired_env.sh
source install/semantic/local_setup.bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/check_dynamic_cloud_live.py \
  --seconds 25 --output logs/miti/dynamic_cloud_check.json
```

2026-11-10 首次 25 秒试跑：原始地图 88,354 点，清理后 81,553 点，清理历史点 6,801；实时扫描约 3.05 Hz、清理地图约 1.61 Hz，状态报告的最高处理耗时 35.40 ms。这些点是射线否定的历史几何，不能等同于 6,801 个人体点；没有逐点真值精度评估。三次确认需要重复扫到相应体素，遮挡处不能保证固定时间内清除。报告为 `logs/miti/dynamic_cloud_audit_20261110/filter_trial.json`，并保留前后点云快照和对比图。

正式部署后又检查 20 秒，此时活动地图已是 `semantic_20261110_161052_153595.db`：原始地图 27,641 点、清理后 27,610 点，当前扫描约 2.41 Hz，清理地图约 1.68 Hz；最后一帧处理 73.49 ms，窗口报告的最高值 82.46 ms，末尾队列为 0。每个派生点云话题只有 1 个发布者。已通过 ROS 参数服务确认墙面、栅格输入均为 `/miti/clean_map`，并核对实际 HTTP 资源哈希。此次展示重载前后的 SLAM 进程集合一致。记录见 `deployed_check.json`、`deployment.json` 和 `logs/miti/dynamic_cloud_restart_20261110.json`。没有可连接浏览器，未作浏览器画面验证。
