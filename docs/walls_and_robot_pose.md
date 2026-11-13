# 墙面显示与机器人位姿输出

运行原来的 `bash scripts/start_miti.sh` 时，会同时启动独立的 `map_geometry` 节点。
三维地图右下角提供“显示墙面”开关和可收起的位姿读数；设置面板顶部也直接显示
XYZ 和 Roll/Pitch/Yaw 数值。`/robot_pose` 坐标轴默认可见。已有项目布局强制刷新后
自动加入墙面和位姿图层，保留点云、语义标签、相机朝向箭头、顶部裁剪及其他布局设置。
对先前只启用了墙面的布局，一次性打开位姿图层；之后用户自行关闭的设置会继续保留。

## 墙面

`/miti/clean_map` 的完整地图快照经过 10 cm 降采样、局部法向估计、平面拟合和连通区域
筛选，输出 `/miti/walls`（`visualization_msgs/MarkerArray`）。蓝色半透明面只覆盖
有观测支持的 20 cm 单元，空洞不会用整块包围矩形填满。门洞边缘存在一个单元的
离散误差；它不是建筑尺寸测量或精确门洞重建工具。

默认筛选近竖直、至少高 1.4 m、宽 0.9 m、观测面积 1.2 m² 的墙面候选。
这是几何提取，尚未使用图像语义确认，较大的柜面、隔板可能包含在内。曲面、玻璃、
稀疏或严重重影的区域可能漏检。输入已经过[动态残影清理](dynamic_cloud.md)，只有被新射线
反复证实为空的历史点才会移除；遮挡区域的残影可能暂时保留。

墙面在后台单线程处理，默认最多每 2 秒启动一次，忙时仅保留最新输入。
每次按完整地图重建并替换显示，不累积旧墙面；回环后的点云更新会重新提取墙面。
空地图会发出定向删除消息。地图不再更新时保留最后墙面，与原地图一致。
顶部裁剪也作用于墙面及边界面；文字标签沿用原有语义文字的行为，不参与高度裁剪。
输出使用标准 ROS 消息，也可在 RViz 添加 MarkerArray 显示。

## 位姿

`/robot_pose`（`geometry_msgs/PoseStamped`）以最多 10 Hz 发布，参考系默认 `map`：

```text
T_map_tracked = T_map_odom(latest) × T_odom_lidar(acquisition stamp) × T_lidar_tracked
```

最后一项使用已有 TF 的实际安装变换，包括旋转和平移。
默认 `robot_frame: auto` 优先查找 `base_link`；不存在时输出里程计消息真实的
`child_frame_id`，界面明确注明该参考点。当前实机没有 `base_link` TF，输出的是
`livox_frame` 的位置和姿态，不能把它称为机身中心。也没有把前视相机箭头的方向
当作机身航向。将来补齐外参即可自动输出机身位姿。

如要求只允许特定机身坐标系，将配置中的 `robot_frame` 改为实际名称，例如
`base_link`。此时 TF 缺失就停止输出并显示等待状态，不回退到雷达。
`PoseStamped` 本身没有 child_frame_id；消费该话题时应同时读取
`/miti/map_status` 的 `pose.tracked_frame`，或使用上述显式坐标系配置。

位置单位为米，ROS 消息姿态为四元数 XYZW；界面显示固定轴 XYZ 的
Roll/Pitch/Yaw，单位为度，满足 `R = Rz(yaw) Ry(pitch) Rx(roll)`。
Yaw 相对地图坐标系的 X 轴，不代表绝对地理北向。

现场 Livox 里程计使用设备启动时间，而 RTAB-Map 的地图修正 TF 使用系统时间。
因此分别取最新地图修正与原始采集位姿进行组合，不对两个不同起点的时间直接作差，
也不查询在设备时间处不存在的系统时间 TF。输出保留原里程计时间戳，
`pose.stamp_clock` 提示 `device` 或 `ros`（按与系统时间的差异判断）。
这只是兼容现有链路，不构成设备时钟同步。需要与外部 UTC 记录比较时仍应先完成授时。

位姿消息到达或采集时间戳停止推进超过 1 秒时，停止发布 `/robot_pose`，界面隐藏旧
数值并显示中断。地图修正 TF 过期或缺失同样停止输出。壁面计算在独立工作线程，
不会等待它完成后才发布位姿。地图回环修正可能引起 map 坐标下的位姿跳变，这是
全局修正行为；需要连续局部里程计时使用原 `/icp_odom`。

## 配置和使用

参数文件：`config/map_geometry.yaml`，修改后重启展示服务或该独立节点生效。
修改 `odom_topic` 可接入项目已有的其他 Odometry 来源。

```bash
cd /home/jyfight/Project/rtabmap_slam
source scripts/setup_miti.bash
ros2 topic echo /robot_pose
ros2 topic echo /miti/map_status
```

只输出 ROS 数据而不启动网页时：

```bash
source scripts/setup_miti.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/map_geometry_node.py \
  --ros-args --params-file config/map_geometry.yaml
```

ROS 域、DDS 网卡配置必须与建图一致；回放 rosbag 时添加 `-p use_sim_time:=true`。
不要与已经启动该节点的 MITI 服务重复运行。若单独管理该节点，可使用
`MITI_GEOMETRY=0 bash scripts/start_miti.sh`，避免展示脚本再启动一份。

## 验证

`bash scripts/test_miti.sh` 在独立 ROS domain 91 上验证消息到 Foxglove WebSocket
的传输。新增检查覆盖带门洞墙面、地面和小物体排除、旋转后的墙面、安装外参变换、
四元数/RPY、设备时间戳、位姿中断、冻结帧、参考点回退、墙面删除、点云字节序与行填充、
界面数值/单位、断流提示、开关和布局迁移、墙面裁剪。既有展示回归检查一并执行。
结果在 `logs/miti/walls_pose_test.log`、`logs/miti/connection_test.json`。

实机消息核查（只读）：

```bash
source scripts/setup_miti.bash
python3 scripts/check_map_geometry_live.py --seconds 8 --output logs/miti/walls_pose_live.json
```

脚本保存对应地图采样为同名 `.npz`，报告明确区分消息传输检查与浏览器渲染检查。

2026-11-03 上线验证：50 项离线/组件检查及独立 ROS 域联调通过。更新正在运行的
MITI 展示服务后，实机 8 秒采样收到 80 条 `/robot_pose`、32 条状态消息及完整墙面
MarkerArray；该次 36,320 点地图提取出 10 处几何墙面候选，处理耗时约 0.136 秒。
当前跟踪参考点为 `livox_frame`，`base_link` 不存在，设备时间戳兼容路径已生效。
报告为 `logs/miti/walls_pose_live.json`。本次工具没有可用浏览器，未直接验收网页画面，
已核对前端组件、打包代码和实机消息链路；没有重启传感器或 SLAM。
