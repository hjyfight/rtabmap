**单目标 RGB / NIR / LiDAR / 声源融合**

这条流程以一个检测目标为单位，将 RGB 实例、对应 NIR 区域、属于目标表面的点云和可唯一关联的声源组合。真实运行使用已核验的冻结 UniBind / PointBind 权重；RGB 实例由既有 YOLOE 或兼容的实例检测生产者提供。建图语义、整场四模态特征面板、原有 RGB/NIR 复核面板仍是各自的流程。

本轮任务周期未规定，精度比较与 8.6 替代增益验收按用户要求暂缓。机器狗未连接、未改动；物理标定与现场验收没有用合成数据代替。

**关联和计算**

- RGB：使用原始图像中的实例掩膜，保留类别基线、实例号、原始来源、掩膜哈希和同一目标的缩略图。
- NIR：通过注册深度、RGB 畸变参数、RealSense 立体内参及静态 TF 投影，裁出同一目标的有效区域。覆盖不足或相机校准不匹配时，该路不参与。
- LiDAR：只有 RGB 与 LiDAR 安装外参有完整标定证据时才投影；按实例掩膜及深度表面门限筛选，排除背景和遮挡后的点。至少需要 512 个不同目标点，保留 RGB 光学坐标中的点中心、距离和范围。没有安装外参时不会拿整场点云替代目标点云。
- 音频：需要至少三通道的阵列几何、阵列安装外参和有效目标公共坐标。取图像采集时刻附近最多 250 ms PCM，以 GCC-PHAT 估计方向。线性阵列、未消解的平面反射、静音、TDOA 不一致以及多个目标共用同一方位均不能唯一关联。仅对配置中的可发声目标类使用；编码器也只编码已关联的窗口。

独立编码是 4 × 1024 维，L2 归一化后按固定 RGB / NIR / LiDAR / Audio 槽位拼接为 4096 维。有效权重为任务先验 × 目标区域信号质量 × 来源连续性/信号质量，再归一化。缺失槽位和权重为零。融合向量为加权和的 L2 归一化结果；抵消时明确标记无效。质量权重是可解释工程策略，尚未通过任务精度验证。

各路和融合结果投到相同文本原型空间。分类冲突保留 RGB 检测基线，记录参与模态、各路建议、冲突类型及处理措施；余弦相似度不是校准概率。声学另输出可配置的事件候选，例如说话、犬吠、发动机、机械噪声和撞击；事件分类没有验收，口令转写和执行不在当前单目标任务范围内。声源没有独立测距，关联目标位置来自视觉深度与位姿。

**质量、状态与动态变化**

输入记录包括格式、采集时间、来源、坐标系、时长、原始数据 SHA-256、质量评分/等级、观测到的连续性异常及处理措施。图像测曝光、纹理和清晰度；点云测有限/有效/不同点；音频测 RMS、静音和削波。重复、回退和过期输入隔离；恒定画面仅标记“疑似冻结或静态”，不会伪造设备自检结果。硬件健康字段明确为未验证或需检查。

目标用 UUID 会话及序号维持 ID，按图像特征与预测位置匹配。连续观测输出公共坐标速度、方向和一秒预测；首次观测的运动状态未验证。短时遮挡恢复时输出重关联事件。保存状态受坐标系、标定文件指纹和时间轴约束；时间回退或运行几何改变会保留旧状态并开启新跟踪会话。

新增、位移、运动状态和范围变化有独立事件。消失要由有效深度射线看到旧目标体积后方背景，且连续三帧确认；检测器漏检或原处深度不变不能证明移除。静态移除还需多次静止观测，已有运动的目标离开记为瞬态离开，未知目标则记为消失待分类。缺少门窗参考状态时只报告几何变化，不声称已确定开启/关闭或坍塌。

输出明确报告视场边缘、深度缺失、NIR 覆盖不足、类别冲突、缺失/未关联模态、公共位姿及硬件同步未验证等不确定性。

**启动与显示**

主机已有 RGB 实例、注册深度及传感器话题时：

```bash
bash scripts/start_object_multimodal_fusion.sh
```

此入口只启动目标融合节点。CUDA 默认来自 `.venv-segmentation-gpu`；可用 `-p device:=cpu` 显式切到 CPU。先用 `bash scripts/build_fourmodal_features.sh` 构建，且需要现有 segmentation / semantic 消息覆盖层。

Orin 新增两个入口：

```bash
# 已有 RGB 实例生产者时只启动目标融合
bash scripts/start_orin_network_perception.sh target-fusion
# 启动 YOLOE RGB 检测和目标融合；必要时创建注册深度
bash scripts/start_orin_network_perception.sh target-perception
```

这两个命令不启动网络接收器。机器人/传感器留在机器狗并通过局域网送数据的接法沿用 `orin_network_perception.yaml`；本轮不执行 receive。当前空白安装外参仍会阻止 LiDAR/声源参与，不能仅接上新机器狗就声称完成四模态物理融合。

MITI 安装后在 `/rtabmap.html` 点击“打开单目标四模态融合”，或访问 `/rtabmap.html?view=target`。新面板显示目标 RGB/NIR 区域、ID、位置、运动、各路实际参与及权重、关联依据和不确定性；无结果时显示等待。原有融合布局继续可用。

主机专用显示入口为 `bash scripts/start_object_multimodal_miti.sh`，默认网页
`http://localhost:3003/rtabmap.html?view=target`，独立桥接端口 8767。它开放目标融合结果、
状态和计时话题，使用独立布局状态；保留已有展示服务和建图进程。检测、目标融合和这个
显示入口应使用与输入数据相同的 ROS 域及 DDS 配置。原有桥接默认白名单没有目标结果，
因此首次查看新目标输出优先使用此入口。

若主机已有本机时间戳的 RGB 原始话题，而实例检测还未运行，可以在另一个终端运行：

```bash
SEGMENTATION_PYTHON="$PWD/.venv-segmentation-gpu/bin/python3" \
  bash scripts/start_yolo_segmentation.sh \
  --params-file config/yoloe_segmentation.yaml \
  --params-file config/yoloe_segmentation_gpu.yaml
```

这个基础检测入口不会执行机器狗 SSH 时差测量。目标融合仍需要匹配的 CameraInfo 和同坐标
注册深度；没有这些输入时，网页会保持等待。

机器狗已通过原有有线网卡接入、相机与雷达已开启时，每个主机终端先执行：

```bash
cd /home/jyfight/Project/rtabmap_slam
source scripts/ros2_wired_env.sh
```

然后在四个终端依次运行：

```bash
# 1. YOLOE 语义建图，按需启动注册深度
bash scripts/start_semantic_yoloe.sh --gpu
# 2. 独立 RGB 实例检测，发布目标融合请求
bash scripts/start_yoloe_segmentation_live.sh --gpu
# 3. 单目标融合，使用机器狗墙钟估计补偿帧龄
bash scripts/start_object_multimodal_fusion_live.sh
# 4. 网页显示
bash scripts/start_object_multimodal_miti.sh
```

语义建图的检测以关键帧为输入，不发布 `/perception/segmentation/fusion_request`，因此仍需要
第 2 步；已有该请求生产者时可复用。第 2、3 步通过只读 SSH 估计机器狗墙钟偏移，不修改
任何设备时间。第 3 步将估计另存至 `logs/object_multimodal/live_*/calibration.json`，原始标定
不变；注册深度沿用 RGB 的时钟补偿。也可传 `--clock-report /path/clock.json` 复用一天内的
估计。这个入口仅适用于所有采集时间戳都使用同一机器狗墙钟的当前有线接法；已有实测
标定时使用原入口，不覆盖实测证据。网页地址仍为
`http://localhost:3003/rtabmap.html?view=target`。雷达参与仍需实测外参；没有声源输入时音频
不会参与，接通设备不等于四模态齐全。

2026-11-18 在线入口修正后，28 项离线检查通过；仅本机回环的独立 ROS 域中，合成输入
落后主机 57.5 s 的 8 项检查通过并发布 3 帧，记录另存
`logs/object_multimodal_validation/host_live_clock/ros_contract.json`。该检查使用模型替身，
验证时差补偿和接口，不替代本次真实传感器效果查看。

**标定证据**

`config/object_multimodal_calibration.json` 的安装外参和阵列几何留空。真实部署需要每个传感器的 frame_id、common_from_sensor、对应的空间/时间参考测量、时钟不确定度、采样周期及测量有效区间。

离线工具不连接任何设备：

```bash
python3 scripts/fit_object_multimodal_calibration.py --check config/object_multimodal_calibration.json
python3 scripts/fit_object_multimodal_calibration.py --measurements /path/measurements.json --output logs/object_calibration
```

输入需声明 `physical_measurements` 或 `synthetic_fixture`、参考事件方法、仪器参考、版本和公共坐标。每路提供至少四组非共线 xyz 对应、至少五组跨时间的采集事件对应、仪器不确定度及由任务指定的空间 RMSE 限值；声学还提供实测阵列几何。工具保存输入、拟合残差、时钟偏移/漂移及证据 SHA-256。合成拟合不能使硬件同步通过。

同步预算使用不同传感器两两时钟不确定度之和，并同时考虑 50 ms 与最短参与采样周期的 1/10。运行帧只有在物理证据、采样周期、证据有效时间区间和实配时间差加不确定度都满足时，才可能标记同步验证。未知采样周期、过期证据和只设置匹配容差都不会通过。外部协同授时适用项仍需实测确认。

**周期的含义与测量**

“处理周期不大于任务需求周期”要求一次任务所需的处理及时完成。例如任务要求每秒更新一次，任务周期就是 1 s；若处理要 2 s，就不能据此通过该任务的周期要求。1 s 仅为解释示例，本项目没有把它设为用户的验收限值。

要区分计算时间、排队时间、新结果间隔和从采集到显示的时延。新链路分别记录：

- `timing.core_compute_s`：关联、编码、融合、跟踪及结果构造，排除节点准备、封装校验和发布。
- `/perception/fusion/timing` 与证据日志中的 `processing_s`：本次成功准备输入开始，到最终校验、序列化及 DDS 发布入队完成；包含工作线程等待和调度。
- `queue_s`：RGB 实例请求到达节点，到成功准备开始。
- `receipt_to_result_s`：请求到达节点，到发布入队。
- `output_interval_s`：连续结果发布间隔。

该边界不包含上游检测、机器人到 Orin 的网络传输或浏览器显示。若完整任务包括这些阶段，最终验收还必须把它们计入。日志中保留每个边界，不能用核心计算时间代替全任务时延，也不能把配置的调度间隔当作处理能力。

```bash
# 只运行离线合成输入，但使用真实权重；不会采集硬件
.venv-fourmodal/bin/python3 scripts/benchmark_object_multimodal.py --real-model --device cpu --samples 3 --output logs/object_multimodal_benchmark
# 默认日志最多约 64 MiB，保留三份轮转备份
python3 scripts/measure_object_multimodal_cycle.py logs/object_multimodal/results.jsonl --output logs/object_multimodal/cycle_report.json
python3 scripts/audit_object_multimodal_results.py logs/object_multimodal/results.jsonl --output logs/object_multimodal/interface_report.json
```

未指定任务周期时报告为 `not_assessed_task_period_unspecified`。离线纯核心记录即使给了限值，也不会伪装成完整发布边界验收。少量样例不是最坏情况实时保证；真实传感器、目标数量、热状态和同时运行的模型会改变周期。

**接口与复核**

目标结果为 `/perception/fusion/targets` 上的 `std_msgs/String`，契约 `object_multimodal_fusion/1`。结构定义见 `config/object_multimodal_result.schema.json`，跨字段、归一化、加权算术、ID、来源一致性及完整性验证由 `standard_interface.validate_result` 执行，无额外 JSON Schema 运行库依赖。完整字段映射见 `docs/object_multimodal_interface.csv`。

全部未验收结果仅允许显示、记录和离线分析，明确需人工复核、冲突需独立复核、禁止自动执行。SHA-256 用于完整性，默认 unsigned；可用 HMAC helper 做认证，但无密钥不能声称来源认证通过。`require_control_approval` 会拒绝这些结果、未认证结果和违反限制的结果。节点不提供机器人控制路径。

本轮验证文件保存在 `logs/object_multimodal_validation`。核心/接口/标定工具测试、隔离 ROS 测试、真实权重离线测试和物理验收分别记录；表 5 进展见 `docs/fusion_table5_completion_20261117.md`。

Orin 的实际测量：单目标完整四模态核心计算中位约 0.70 s；隔离 ROS 的成功准备至发布入队中位约 1.08 s、最大约 1.18 s（完整四模态仅 2 帧）。这表明可以测周期并开展优化，不能证明所有情况下都满足 1 s 任务要求。尚未固定任务周期，因此报告保持“未评定”。
