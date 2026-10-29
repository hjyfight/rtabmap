# YOLO 物体标签与 RTAB-Map 原生三维窗口

新增 `rtabmap_semantic_msgs`、`rtabmap_semantic`，并扩展 `rtabmap_viz`。
使用 YOLOv8n-seg 在 RTAB-Map 保存的 RGB-D 关键帧上识别物体，通过深度定位，
在当前三维地图窗口中显示 `chair #1 0.92` 一类的文字和定位点。
工具栏的 **Object labels** 可以切换显示。标签默认使用模型英文类别名。

## 安装与编译

在项目目录运行：

```bash
bash scripts/install_semantic.sh
bash scripts/build_semantic.sh
```

如果当前网络无法直连 GitHub，可使用已有代理下载模型：
`SEMANTIC_DOWNLOAD_PROXY=http://127.0.0.1:7897 bash scripts/install_semantic.sh`。

推理依赖安装在 `.venv-semantic`，模型在 `models/yolov8n-seg.pt`。
NumPy 固定在 1.x，保持与 ROS Humble 的 cv_bridge 兼容。默认使用 CPU。
编译输出在 `install/semantic`，只在语义启动脚本中加载这个 overlay。
使用系统已有 RTAB-Map GUI 库的派生窗口，不改变系统安装，也不修改几何后端。

## 实机启动

当前机器人上的 Foxy / RealSense 驱动使用以下配置。先在相机驱动终端正常停止旧
相机进程，在同一 SSH 终端分别执行：

```bash
source /opt/ros/foxy/setup.bash
```

```bash
/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py \
  config_file:="'/home/unitree/realsense_slam_sensor_qos.yaml'" \
  rgb_camera.profile:=424x240x6 \
  depth_module.profile:=640x480x6 \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=true
```

2026-10-29 查询的设备支持列表中没有深度 `424x240x6`，有 `640x480x6`。
旧命令使用不支持的深度模式后，实际回退为 `640x480x15`；当时的对齐话题混入了
`424x240` 和 `640x480` 两种尺寸，被启动检查拒绝。此处使用支持的深度模式并将
彩色和深度设为 6 Hz。确认 `Open profile` 日志显示 Depth `640×480@6`、Color
`424×240@6`，再运行工作站上的启动检查。对齐深度应稳定为 `424×240`，原始深度
仍为 `640×480` 是正常的。模式被支持不等于已经通过实机对齐验证，以采样结果为准。

先按原流程启动 Livox 与 RealSense，启用对齐深度。将实际相机外参写入
`CAM_X CAM_Y CAM_Z CAM_ROLL CAM_PITCH CAM_YAW`，然后：

```bash
cd /home/jyfight/Project/rtabmap_slam
```

```bash
bash scripts/start_semantic_slam.sh
```

脚本复用 RGB-D 输入检查和现有融合配置，每次默认新建地图。
启动检查要求至少 3 组尺寸、坐标系与时间戳一致的 RGB-D；运行时继续逐帧校验。
当前相机偶发在对齐深度话题发布 `640×480` 原始尺寸，校验器会丢弃这些帧并记录错误，
保留后续有效帧。它不会缩放错误深度来伪装对齐；相机驱动的异常输出本身仍需排查。
相机外参未赋值时沿用原融合脚本示例值并提示，示例值不等于标定结果。
原窗口必须由语义 overlay 中的 `rtabmap_viz` 启动，系统原版没有语义订阅。

同一 ROS 域的启动脚本使用独占锁，并检查遗留 IMU、点时间转换、去畸变等节点。
启动失败后，即使 `/rtabmap` 已退出，也要结束旧处理节点后再启动，否则两个独立 IMU
滤波器可能为同一时间戳给出不同航向。RTAB-Map 后端或 RGB-D 校验进程退出时，
语义启动链会整体结束，避免窗口仍运行却已经停止建图。

指定数据库或计算设备：

```bash
bash scripts/start_semantic_slam.sh database_path:=/absolute/path/session.db
bash scripts/start_semantic_slam.sh semantic_device:=cpu
# 在 CUDA 环境已经配置好、且安装匹配的 GPU PyTorch 后才使用：
# bash scripts/start_semantic_slam.sh semantic_device:=0
```

如果已经有一套运行中的 SLAM，先正常结束、保存，再通过新入口启动，避免重复
里程计、TF 和数据库写入。标签属于地图中的物体，不会写入机器人轨迹节点的 label。

## 重新加载与已有地图补标

```bash
source setup.bash
source install/semantic/setup.bash
export YOLO_CONFIG_DIR="$PWD/models/.ultralytics"
ros2 launch "$PWD/launch/semantic_slam.launch.py" \
  database_path:=/absolute/path/session.db localization:=true
```

语义模块启动后通过 `/rtabmap/get_map_data` 请求已有关键帧，按节点编号和时间戳
去重。已有语义观测直接恢复；缺失观测可以重新识别。地图必须保存过 RGB、对齐深度
和相机标定信息，纯 LiDAR 数据库无法通过 YOLO 补出物体类别。
上述入口仍包含传感器侧处理节点；后端服务和保存图数据可用于补标，显示全部旧地图
可在原窗口中执行下载完整地图。不要使用删除数据库的启动参数。

## 数据、配置与行为

几何数据库旁自动保存同名 `*.semantic.sqlite`。每次完成关键帧后通过 SQLite 事务
保存，记录模型指纹、配置、物体和节点局部观测；动态对象不持久化。
它与 RTAB-Map 数据库是配套文件，复制地图时应一起复制，并调整数据库关联路径
后使用（当前版本对路径不一致会拒绝载入）。复用节点 ID 但时间戳变化时创建新语义
epoch，旧 epoch 保留，防止重建地图时沿用错误标签。

默认配置在 `src/rtabmap_semantic/config/semantic.yaml`：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| confidence | 0.5 | YOLO 检测阈值 |
| min_depth / max_depth | 0.3 / 4.0 m | 有效深度范围 |
| min_observations | 3 | 确认对象需要的不同关键帧数量 |
| association_distance | 0.45 m | 对象关联的基础距离阈值 |
| max_observations | 30 | 每个对象保留的局部观测上限 |
| dynamic_classes | person, cat, dog | 临时目标类别 |
| dynamic_ttl | 3 s | 动态目标消失后的保留时间 |
| classes | ['*'] | 检测类别白名单，默认全部模型类别 |

可复制 YAML 后使用 `semantic_config:=/path/config.yaml`。模型或影响融合的配置发生变化时，
应设置新的 `semantic_database_path`，避免混用不同定义的数据。

流程：`/mapData` → `keyframe_bridge` → `/semantic/keyframes` → YOLO + 深度反投影
→ `/semantic/objects` → 原生三维窗口。`/mapGraph` 与 `/semantic/graph` 提供优化位姿。
相机像素在掩膜内筛选，处理 mm/m 深度、无效深度、畸变和相机到节点的局部变换。
关联使用类别、三维距离、可见范围和一对一匹配；同帧出现过的对象不会自动互相合并。
回环后重投影各关键帧的观测，也可合并满足条件的重复对象。

GUI 在自己的当前优化图上计算锚点位置，通过 Qt 队列切换到 GUI 线程。
它单独缓存语义对象，在地图重绘清除文字后恢复语义标签。无需改 RTAB-Map 地图格式，
语义观测也不作为几何图优化约束。

桥接器每次只向推理节点发送一个未确认关键帧，以确认消息提供背压。
实时待处理队列上限 32，过载会跳过最旧待处理帧并打印诊断；完整旧地图补标按序处理。
YOLO 运行在独立线程，推理期间仍处理位姿图和发布标签。

诊断与手动保存：

```bash
ros2 topic echo /semantic/status
ros2 service call /semantic/save std_srvs/srv/Trigger '{}'
```

查看输入校验累计通过/拒绝帧数及最近原因：

```bash
ros2 topic echo /semantic/input_status
```

定位是物体可见表面的估计，不能当作完整物体尺寸。COCO 之外的类别需要自定义权重。
相机与雷达外参、原始 RGB-D 同步和 SLAM 本身的质量决定地图标注精度。
静态类别的物体搬离原位置后，新观测可能形成新对象；当前版本没有完整的场景变化检测，
不会仅凭物体离开视野就删除历史对象。

## 验证

```bash
bash scripts/test_semantic.sh
# 同时验证真实 YOLO 推理与压缩关键帧：
bash scripts/test_semantic.sh --real-yolo
# 已安装 Xvfb 时，验证原生窗口的文字、重绘与回环更新：
bash scripts/test_semantic_gui.sh
```

使用独立 ROS domain（默认 93）和本机通信，检查几何、关联、回环、持久化及编译后的
关键帧桥接器。测试中的合成检测器只用于几何断言，生产节点始终加载真实 YOLO 权重。
真实相机定位精度及实地回环效果需要在传感器接通后验收。

本次构建与验证结果见 [语义地图验证记录](semantic_mapping_validation.md)。
