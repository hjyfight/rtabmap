# 语义地图验证记录（2026-10-29）

## 环境与构建

- ROS 2 Humble，系统 RTAB-Map 与 rtabmap_ros 0.23.13。
- 构建通过：`rtabmap_semantic_msgs`、`rtabmap_semantic`、扩展后的 `rtabmap_viz`。
- Python 3.10，PyTorch 2.5.1+cpu，torchvision 0.20.1+cpu，Ultralytics 8.3.40。
- NumPy 1.24.4、OpenCV Python 4.10.0.84；ROS 的原有系统依赖继续保留。
- 模型为官方 `yolov8n-seg.pt`；SHA-256：
  `a7cd8f929e1903d78a12a48efecab430209f18dc46cb96c3599a5980c63c423c`。
- 推理环境、模型、编译产物均位于本项目，系统 RTAB-Map 安装未被替换。

## 已通过检查

| 检查 | 结果 |
| --- | --- |
| 16 项语义单元测试 | mm/m 深度、无效深度、降采样内参、外参、背景过滤、同类对象关联、重复关键帧、回环重投影、合并保护、异常观测过滤、动态目标、保存恢复、地图/模型匹配均通过 |
| 17 项现有 Livox 配置回归测试 | IMU、去畸变、同步、融合、定位和 GUI 启动配置均通过 |
| 编译后的关键帧桥接器 + 语义节点 | 原始图像与 PNG 压缩关键帧均处理成功；3 个关键帧融合成一个已确认对象 |
| 真实 YOLO + ROS 集成 | 官方 bus 样例识别人和公交车，人工已知深度平面用于验证投影；公交车跨 3 帧获得稳定 ID |
| 优化图更新与恢复 | 各节点优化后物体位置按锚点重算，重新启动后 ID 与语义 epoch 保持一致 |
| 地图重置 | 节点编号复用且时间戳变化时清空旧待处理队列、建立新 epoch，旧语义记录保留 |
| 原生三维窗口 | 真实 VTK 文字 actor 出现；清除文字后自动恢复；节点平移 1 m 时标签同样平移 1 m；删除对象后文字和定位点移除 |
| 4 项嵌套启动回归测试 | 执行真实 include 动作，验证默认参数、自定义路径与定位模式；SLAM、语义节点和 GUI 使用同一地图；原始 RGB-D 必须经过校验器 |
| 7 项输入保护回归测试 | 有效帧保持不变，错误尺寸/标定/坐标系/存储/时间戳被拒绝；后端不存在时仍能检出遗留处理节点；带持续校验器的入口要求至少 3 组有效 RGB-D |
| 完整启动与运行时故障 | 独立 ROS domain 94、Xvfb、临时数据库中，全链启动；按正常/错误尺寸/正常顺序输入 RGB-D，2 帧通过、1 帧拒绝，后端继续运行；停止测试后端后所有子进程退出 |
| 实机静止检查 | 临时数据库、真实传感器运行约 48 秒；去掉初始化段后 43 秒内 237 条里程计，最大位移 3.2 mm、最大旋转 0.26°；后端 39 次更新，持续运行无崩溃 |

真实 YOLO 的最后一次集成测试中，首次处理约 0.59 s，随后两帧约 0.11 / 0.13 s。
这些是该 CPU、该样例下的测量，不代表所有分辨率和场景的吞吐率。

原生窗口测试使用隔离的 Xvfb 和软件 OpenGL。视觉检查确认截图中可见
`chair #1 0.92` 标签与定位点；这是合成地图上的绘图测试，不是实机建图照片。
为使 VTK 9 及时更新画面，语义层使用 `CloudViewer::refreshView()` 显式渲染。

### 启动参数修复

实机相机检查通过后，旧入口在包含语义子 launch 时出现
`Included launch description missing required argument 'database_path'`。
原因是顶层声明了参数，却未通过该 include 的 `launch_arguments` 显式传入。
现已补传 `database_path`、`model_path` 和 `semantic_python`，无需重新编译。

此前的 `--show-args` 只检查参数展示，不能替代实际执行 include 的验证。
新增 `tests/test_semantic_launch.py` 的 3 项检查在修复前均复现同一异常，修复后均通过，
并纳入 `scripts/test_semantic.sh`。完整启动检查另外通过 ROS 参数服务核对路径和 GUI
语义开关，收到 `/semantic/status` 后确认节点持续运行，再正常关闭测试进程。
测试域未接入实机传感器，因此本项只验证初始化与连接，不代表已经完成实机建图。

### 静止时环状重影与崩溃

2026-10-29 15:34 启动的语义任务中，上一轮失败启动留下的 Madgwick 和点时间转换
进程仍在运行。只读采样发现 `/imu/data` 有两个发布端，1510 个重复时间戳；同一时间戳
姿态差最大 104.48°。机器人静止时 `/icp_odom` 旋转变化达 104.35°，已保存的节点位姿
也有大角度跳变，足以解释环状叠影。

同一任务在 15:35:04 因 RGB 为 `424×240`、深度为 `640×480` 触发
`Memory.cpp::createSignature()` 断言。启动前 5 秒一致并不能保证后续帧持续有效。

已通过核验 PID、命令和父进程后向三个遗留进程发送 SIGINT。新入口补充独占启动锁、
遗留处理节点检查、持续 RGB-D 校验及后端退出联动。独占锁检查已实测通过。
语义入口允许采样中存在被拒绝的深度帧，但要求至少 3 组有效配对，并始终经过运行时
校验器；原融合入口保留严格预检查。

实机复测使用 `/tmp/semantic_stationary_ssza5zj1/stationary.db`。有效里程计区间为
43.001 秒，最大位移 0.003206 m，最大旋转 0.25557°。IMU、点时间转换、里程计和
已校验 RGB-D 均仅一个发布端。后端处理 39 次，静止图保留 1 个节点；此段校验器通过
175 帧、拒绝 0 帧。错误尺寸帧拦截另由隔离注入测试验证。测试结束后所有测试进程正常
退出。此结果仅覆盖静止场景，未验证移动建图或真实相机/雷达外参精度。

本地生成的验证文件：

- `logs/semantic_validation/yolo.json`
- `logs/semantic_validation/ros_pipeline.json`
- `logs/semantic_validation/ros_real_yolo.json`
- `logs/semantic_validation/native_gui.png`
- `logs/semantic_validation/launch_arguments.txt`
- `logs/semantic_validation/launch_smoke.json`
- `logs/semantic_validation/launch_smoke.log`
- `logs/semantic_validation/live_audit_before_cleanup.json`
- `logs/semantic_validation/stationary_live.json`
- `logs/semantic_validation/stationary_live.log`
- `logs/semantic_validation/ready_to_restart.json`

## 验证范围

已验证代码、模型推理、ROS 数据传输、已知几何条件下的投影融合以及原窗口绘制。
另以临时数据库验证了实机静止状态；尚未验证移动建图、实际相机与雷达标定误差、
真实场景中的标签定位精度及物体搬动后的完整地图变化检测。原有实机地图没有被改写。
