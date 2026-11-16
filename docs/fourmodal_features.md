# 四模态统一特征：UniBind 预训练骨干与固定融合

本模块处理可见光 RGB、红外（默认 RealSense 近红外）、LiDAR 三维点云、声学。
输出特征向量及采集来源，不输出分割掩膜。使用 UniBind 官方 Quick Start 的
Point-Bind / ImageBind 预训练骨干；全部参数冻结，没有新增可训练投影、分类头或训练流程。

## 本机运行

独立环境为 `.venv-fourmodal`，复用项目 `.venv-emsanet` 中已验证的 CPU PyTorch
2.5.1 / torchvision 0.20.1，仅将新增依赖安装进新环境。上游源码在
`.tools/unibind`，固定提交 `7b22a7790af38d25ffa5b12d49fbe786dc302e6e`；
官方权重在 `models/unibind/pretrained_weights.pt`，约 4.86 GB。

首次安装或继续断点下载：

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/install_fourmodal_features.sh
# 使用代理时：
bash scripts/install_fourmodal_features.sh --proxy http://127.0.0.1:7897
```

安装器验证检查点关键张量的形状并记录本地 SHA-256；该哈希用于后续一致性检查，
不是发布方提供的签名。模型启动还会严格加载整个 state_dict，拒绝缺失或多余权重。
下载异常时保留断点，未通过验证的文件不能用于特征推理。

本机历史近红外图片、阵列录音与官方图像/物体点云的离线 smoke test：

```bash
bash scripts/demo_fourmodal_features.sh
```

每次生成新的 `logs/fourmodal/demo_*/features.npz` 和 `report.json`。
这些文件来自不同时间、不同场景；manifest 和报告明确标为 `unpaired_smoke_test`，
只验证真实模型的四路特征提取与融合数据流，不能作为同步、跨模态关联或精度提升的证据。
其中点云是官方物体点云，不是 MID360 扫描。

## 四路编码与输出

| 模态 | 处理方式 | 默认骨干 | 维度 |
| --- | --- | --- | --- |
| RGB | uint8 RGB → 短边缩放/中心裁剪 224×224 → 官方视觉归一化 | ImageBind vision | 1024 |
| 近红外 | mono8 → 灰度复制三通道 → 同样的视觉预处理 | 共享 vision，实验输入 | 1024 |
| LiDAR | XYZ 米制 → 有限值/距离检查 → 去重 → 单位球归一化 → 确定性采样 | Point-Bind I2P-MAE + 已训练投影 | 1024 |
| 音频 | float PCM → 多通道均值为单声道 → 16 kHz → 至多 2 秒窗口 → log-Mel | ImageBind audio | 1024 |

音频使用 Kaldi fbank：128 个 Mel 频带、25 ms 帧长、10 ms 帧移，补齐/裁剪到 204 帧，
归一化均值 -4.268、标准差 9.138。取包含 RGB 采集时刻的单个窗口；这与上游
默认对较长音频提取三个窗口再平均的策略不同，报告记录实际裁剪范围。

LiDAR 默认采样 2048 点，可通过离线参数 `--point-count 8192` 或 ROS 参数 `point_count`
调整。超过 32768 个候选点时先确定性降采样，以限制 CPU FPS 的开销。至少需要
512 个有效不同点，默认只接受 0.05–40 m 范围；原点和半径保存在预处理元数据中。
这里不执行点云运动去畸变，也不把单位球特征当作米制坐标。

NPZ 包含以下字段，固定模态顺序为 **rgb / infrared / lidar / audio**：

| 字段 | 形状 | 含义 |
| --- | --- | --- |
| `encoder_features` | 4×1024 | 骨干原始输出，保留上游输出缩放 |
| `independent` | 4×1024 | 每路 L2 归一化特征 |
| `concat` | 4096 | 按固定顺序拼接 `independent` |
| `fused` | 1024 | 固定权重平均后再归一化 |
| `valid_mask` | 4 | 该模态是否真实参与本次编码 |
| `weights` | 4 | 对有效模态重新归一化后的实际权重 |
| `cosine_similarities` | 4×4 | 各模态向量相似度；缺失项为 NaN |
| `complete` / `fused_valid` | 标量 | 四模态齐全 / 加权向量未发生零范数抵消 |

默认等权重。固定拼接和加权只是免训练融合基线，不证明跨模态语义完全对齐或精度提升。
相似度矩阵也不直接代表经过校准的可靠性分数。若两路特征完全抵消，报告 `fused_valid=false`，
同时保留各路特征及拼接结果，不返回 NaN 融合向量。

## 用自己的同步数据离线运行

以 [manifest 示例](../config/fourmodal_bundle.example.json) 为模板，填写各文件路径、
**采集时刻**、来源 ID、各自坐标系和标定版本。路径相对 manifest 所在目录。
LiDAR 需明确写 `attributes.units: m`。支持 NPY、NPZ（`points` 字段）、单 Tensor PT、
XYZ 文本、ASCII/binary PCD；不支持 binary_compressed PCD。

```bash
.venv-fourmodal/bin/python3 scripts/predict_fourmodal_features.py \
  --manifest /path/to/capture/manifest.json \
  --device cpu --threads 2 --point-count 2048 \
  --output logs/fourmodal/my_capture
```

默认需要四类全部存在，并检查采集时间。图像和 LiDAR 按时间差匹配，默认容差 50 ms；
音频首样本时间到窗口末尾形成时间区间，RGB 时刻可位于该区间内。
此容差是配置值，不是实测同步精度。

显式 `--allow-partial` 才允许缺失模态。缺失位置保持固定零块，并用 `valid_mask=false`
表示；不会伪造编码器输出或使用过期数据补齐。非同步历史文件必须在 manifest 声明
`unpaired_smoke_test` 并显式传 `--allow-unaligned`。

真正的热红外输入可使用 `--infrared-kind thermal`，采用已训练的 thermal 分支及
单通道 `(0.5, 0.5)` 归一化。不要把近红外输入直接声明为热红外。

## ROS 接入

```bash
bash scripts/build_fourmodal_features.sh
# 在与相机和 LiDAR 相同的已验证 ROS 网络环境中：
bash scripts/start_fourmodal_features.sh
```

默认订阅：

- `/camera/color/image_raw`：`sensor_msgs/Image`。
- `/camera/infra2/image_rect_raw`：`sensor_msgs/Image`，要求 mono8。
- `/livox/lidar`：`sensor_msgs/PointCloud2`，读取 XYZ，保留原 frame_id。
- `/multimodal/audio`：新增 `rtabmap_features_msgs/AudioFrame`。

发布 `/multimodal/features`（`FeatureSet`）和 `/multimodal/status`（JSON String）。
FeatureSet 包含原始/独立/联合/加权特征、源时间/时长、有效标记、实际权重、模型哈希、
标定版本及元数据。Header 是 RGB 采集锚点，发布时间不覆盖源时间。

额外发布 MITI 可直接读取的标准消息：`/miti/fourmodal_features` 和
`/miti/fourmodal_status` 为 `std_msgs/String`，`/miti/audio_spectrum/image_raw`
为 204×128 `rgb8`。现有桥接白名单已允许 `/miti/`，不需要在桥接端安装自定义特征消息。
声谱与模型使用相同的音频窗口、128 个 Mel 频带和预处理，显示固定的归一化范围 [-2, 2]，
低 Mel 频带在下方、时间向右；颜色不是声压级或 dB 标定结果。缺少音频就不产生新声谱。

配置在 [fourmodal_features.yaml](../config/fourmodal_features.yaml)。该节点使用全局观测
特征，保留各模态不同的坐标系，**不执行图像—LiDAR 的逐像素几何配准或物体身份关联**。
填写标定版本用于来源记录，不能据此声称几何配准已经完成。

推理在独立工作线程中运行；仅保留一个待处理 bundle，新的完整观测替换等待中的旧观测，
`dropped_pending` 记录主动丢弃数。已经处理/排队的相同 RGB 来源不会重复推理；
缺少输入、采集时间不匹配、数据过期或非法时，状态话题说明当前情况。
默认最多每秒接收一个新锚点，实际输出速率取决于 CPU 推理时间。

### 麦克风阵列 PCM 入口

项目历史记录中的 ALSA 设备为 `hw:CARD=XFMDPV0018,DEV=0`。实际设备名可通过
`arecord -l` 核对，再在同一 ROS 环境启动：

```bash
bash scripts/start_fourmodal_audio.sh --alsa-device hw:CARD=XFMDPV0018,DEV=0
```

通过 arecord 连续读取 16 kHz / mono PCM，默认发布 2 秒滑动窗口，每 0.5 秒一次。
可通过 `--channels` 保留多通道交织样本，编码时明确均值到单声道。
该入口的时间是主机读取末尾减去窗口时长的估计，标记为 `host_read_end_estimate`，
不能作为硬件 ADC 时间或 50 ms 同步达标的证据；已有硬件时间戳的驱动应直接发布 AudioFrame。
本模块不提供声源方位或口令识别输出。

音频采集遇到 ALSA 退出、设备暂不可用或连续读取超时后，默认每 2 秒重开同一设备，
不会因这类断流退出 ROS 音频节点或连带关闭特征节点。重连时清空旧窗口，重新采满
2 秒连续 PCM 后才恢复发布；不会填充静音或将断开前后的数据拼成连续录音。
音频等待期间，四模态默认仍要求四路齐全，面板根据节点心跳显示具体缺失输入。
`--retry-seconds` 可调整音频重试间隔。

2026-11-16 的显示中断来自 ALSA 读取错误导致旧采集进程退出，旧启动器随即关闭了特征节点。
修复后已通过 EOF、短读、卡住及重开失败的恢复测试，且重新收到连续 3 组真实四模态输出及声谱，
记录见 [audio_recovery_live.json](../logs/fourmodal_validation/audio_recovery_live.json)。

## 验证与能力边界

2026-11-16 本机 CPU、2 线程的真实检查点 smoke test 已输出四路有效特征，
`encoder_features` / `independent` 为 4×1024，`concat` 为 4096，`fused` 为 1024，
所有特征有限、各路归一化范数为 1。结果见
[report.json](../logs/fourmodal/demo_20261116_100258_2/report.json)。本次单次推理约 4.27 s：
视觉两路合批 3.48 s、点云 0.46 s、音频 0.32 s；不含模型加载，不能当作稳定吞吐率。
实际运行记录 `parameters_trainable=0`。

24 项离线检查通过；两个 ROS 包构建通过。

隔离 ROS 真实模型测试也通过，记录见
[ros_real_model.json](../logs/fourmodal_validation/ros_real_model.json)。它验证了特征消息输出、
旧音频拒绝和重复 RGB 去重，输入仍为明确标记的测试消息。

```bash
bash scripts/test_fourmodal_features.sh
.venv-fourmodal/bin/python3 scripts/check_fourmodal_runtime.py
```

前者检查特征契约、同步/缺失/旧数据处理、声谱和 CPU 点算子；后者只验证架构构造，
不加载权重、不代表预训练推理通过。

隔离 ROS 域的接口检查与真实模型检查（运行前确认测试域未占用）：

```bash
source /opt/ros/humble/setup.bash
source install/features/local_setup.bash
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$PWD/logs/fourmodal/ros_test"
.venv-fourmodal/bin/python3 scripts/test_fourmodal_ros.py --transport-only \
  --output logs/fourmodal_validation/ros_transport.json
.venv-fourmodal/bin/python3 scripts/test_fourmodal_ros.py \
  --output logs/fourmodal_validation/ros_real_model.json
```

`--transport-only` 使用明确标记的特征 fixture，不能作为真实网络推理证据。
省略该参数才加载真实官方检查点。两种模式都使用测试消息和合成采集时刻，验证
旧音频拒绝、完整特征发布、重复 RGB 去重及来源记录；不代表同步硬件采集或现场精度。

## MITI 展示

只看输入、特征与融合，直接打开
<http://localhost:3002/rtabmap.html?view=fusion>。这个入口备份当前浏览器布局，再切换到
融合专用布局：上方是特征流程，下方是 RGB、近红外、原始 LiDAR、log-Mel 声谱和辅助深度。
专用布局不包含地图和 ARC 面板；已有布局加载新版控件时也会移除 ARC，并保留其他面板设置。
入口只改变展示，不启动 SLAM，也不把融合特征写入地图。

流程面板按以下顺序展示同一份模型输出：

1. **输入**：真实尺寸、点数、音频采样率与声道数、来源话题、输入状态及匹配时间差。
2. **预处理 / 编码**：图像裁剪与像素标准化、点云单位球归一化与采样、PCM 转 log-Mel，以及对应冻结编码器。
3. **中间特征**：每路编码器输出 `h`（1024 维，含预训练投影/后处理），再做 `z = h / ‖h‖₂`。
4. **输出**：固定顺序拼接 `J = [zRGB, zNIR, zLiDAR, zAudio]`（4096 维）；或先求
   `m = Σ wᵢzᵢ`，再得到 `f = m / ‖m‖₂`（1024 维）。默认有效四路各占 25%。

面板采用简洁视图：四张输入卡片显示归一化特征、实际权重和输入状态，下方集中展示当前向量。
点击输入卡片查看该路 `z`，通过阶段下拉菜单选择 `h` / `z` / `m`，或点击“融合 / 拼接”切换输出。
“逐维检查”“输入与模型”“模态相似度”默认折叠；展开逐维检查后拖动或输入索引，
可以核对 `h[j]`、`z[j]`、`w × z[j]`、`m[j]` 和 `f[j]`。贡献条共享尺度，向左为负、向右为正。
“冻结当前帧”便于逐维检查，传感器预览和后台推理仍持续运行；它们不代表被冻结特征的原始帧。
这些是全局嵌入，不是图像局部关键点、物体标签或米制坐标；没有提取 Transformer 内部 token / 注意力。
旧记录没有的阶段显示缺失，不从最终向量伪造中间特征。

更新已有页面使用 <http://localhost:3002/rtabmap.html?refresh=1>，保留当前布局和视角。
导航连通图、稀疏特征地图在升级时隐藏一次，其配置保存在保留窗口的
`rtabmapHiddenAuxiliaryMaps` 字段中，入口同时将升级前布局备份到浏览器本地。
此操作只改变可见面板，不停止相关 ROS 节点；随后主动恢复的面板不会再次被自动隐藏。

2026-11-16 融合流程展示实机检查通过，见
[fusion_flow_live.json](../logs/fourmodal_validation/fusion_flow_live.json)：连续 3 组真实传感器特征，
编码器原值、归一化、4096 维拼接、加权中间值与最终输出均逐项匹配 ROS 消息；
单次 CPU 推理约 4.49 / 4.73 / 4.76 秒。Foxglove 传输、原生面板解析与 React 静态渲染、
3002 页面资源、无 ARC 的布局均通过。实时特征节点和 3002 服务持续运行。
本次没有可控制的浏览器，未验收浏览器画面与点击交互；也不以此证明硬件同步精度或融合效果。

```bash
bash scripts/install_fourmodal_miti.sh
# 复用已有8765桥接，3002端口使用独立浏览器布局：
bash scripts/start_fourmodal_miti.sh
```

打开 <http://localhost:3002/rtabmap.html>，点击“打开融合专用布局”。
四模态面板显示输入缺失/过期状态、实际权重、各阶段完整向量曲线、维度值和 4×4
余弦矩阵；声谱使用 MITI 原生图像面板。没有数据时明确等待，不绘制假特征。
“离线结果”读取最近保存的模型输出；非同步样本会显示“离线样本 · 未配对输入”。
保存结果不发布到实时 ROS 话题，也不会覆盖实时输入状态。

原有 3000 服务启动后，Next.js 可能保留旧静态文件目录，新增内容地址会返回 404。
独立 3002 前端用于加载新资源并复用既有桥接，保留原 SLAM、传感器驱动和 3000 浏览器布局。
正常重启原展示服务后也可以使用 3000。已有项目布局迁移只添加一次新窗口；主动删除会保留。

也可选择指定保存目录：

```bash
bash scripts/install_fourmodal_miti.sh --result /path/to/result
```

隔离域97、HTTP13001/桥接18766的真实模型测试：

```bash
bash scripts/test_fourmodal_miti.sh
```

此测试已验证 HTTP 资源、真实冻结模型输出、声谱的 Foxglove CDR 传输、MITI 原生 React
面板解析和静态渲染。其输入为测试消息，记录见
[miti_real_model.json](../logs/fourmodal_validation/miti_real_model.json)。没有浏览器 WebGL 验证。

## 实机采集与时钟估计

持续运行实时输入（保持终端开启，Ctrl+C 停止本次麦克风和特征节点）：

```bash
bash scripts/start_fourmodal_live.sh
```

该入口自动测量当前时钟偏移，持续发布四模态特征与声谱，默认不逐帧保存特征文件。
在 <http://localhost:3002> 的四模态面板选择“实时输入”。当前运行目录及进程号记在
`logs/fourmodal/live_current.json`；日志保留在对应 `live_*` 目录。无需启动建图来更新特征。

2026-11-16 15秒实机采集收到 RGB 90 帧（5.987 Hz）、近红外 90 帧（5.988 Hz）、
MID360 点云 150 帧（10.019 Hz）和工作站讯飞阵列音频 26 个窗口（2 Hz）。原始数据和
时间审计见 [capture_report.json](../logs/fourmodal/hardware_20261116_102902/capture_report.json)。
这些是真实设备输入；四路未做硬件同步与几何标定，麦克风与机器人是否同处一个场景尚未确认。

原始传感器时钟落后工作站约57.5秒；未经偏移估计的四路不能满足匹配条件。
实机原始样本已经单独通过真实模型编码，单次 CPU 推理约4.26秒，输出四路有效 1024 维
特征；报告仍标为 `unpaired_smoke_test`，不能作为同步融合效果的证据。

下面命令使用项目既有有线 DDS 设置，只读采集15秒并结束本次麦克风节点：

```bash
bash scripts/capture_fourmodal_hardware.sh
```

为了验证实时链路，增加了显式 `clock_offsets_s: [rgb, infrared, lidar, audio]` 参数，
默认全为0。原始 `Sample.stamp`、FeatureSet 的 Header 和 `source_stamps` 全部保留；
匹配与新鲜度检查使用 `alignment_stamp = stamp + clock_offset_s`。报告保存原始时间、
对齐时间和偏移值；非零偏移将 context 标为 `estimated_clock_offsets`，MITI 明确提示
“未验证硬件同步精度”。音频裁剪使用对齐后的时间差，声谱 Header 仍保留原始音频窗口时间。
加载这类离线 manifest 也需要显式 `--allow-unaligned`。

偏移由5次只读 SSH 远端墙钟查询中的最短往返估计。首次测量约57.512秒、往返区间
约±0.111秒，见 [hardware_clock.json](../logs/fourmodal_validation/hardware_clock.json)。
该区间不覆盖相机曝光、雷达扫描或 ADC 采集延迟，也不能证明50ms硬件同步精度。
RGB 与 LiDAR 用同一机器偏移；音频通过包含锚点的2秒窗口匹配。

在独立3002前端运行时，可执行有时限的实时实机验证：

```bash
bash scripts/validate_fourmodal_hardware.sh
```

该脚本测量当前时钟偏移、启动本次麦克风和冻结特征节点、检查至少3组连续完整输出与
既有桥接，结束后关闭本次采集/特征节点，保留原传感器、SLAM 和 MITI。
通过后报告写入 [hardware_live.json](../logs/fourmodal_validation/hardware_live.json)，
最近实机特征和声谱导出到 MITI 的“离线结果”。这是设备到特征再到展示的数据流验证，
不是现场识别精度、声源定位、融合增益或同场景几何配准验收。

10:42的最终实机验证通过：连续3组四路完整输出，CPU推理4.47/4.17/4.22秒，
各路及融合向量范数均为1；原始 Header/source_stamps 保留。最终偏移估计57.529秒，
往返区间约±0.114秒。最后一组匹配时间差为近红外0.38ms、LiDAR47.91ms、音频窗口4.96ms；
这是估计时基下的匹配结果，不是实测硬件同步精度。Foxglove CDR、面板解析/React静态渲染、
声谱和3002页面资源均通过。本轮没有可控制的浏览器，实际浏览器画面尚需现场确认。
验证结束后本次麦克风与特征节点已停止，独立3002前端继续提供已保存的结果。

近红外分布适配、MID360 场景点云识别效果及四模态融合增益尚需独立实测。
CPU FPS/KNN 与上游 CUDA 扩展实现相同类型的操作，但数值/相同距离时的取点顺序
未进行 CUDA 逐项等价验证，报告保留该状态。

参考：[UniBind](https://github.com/QC-LY/UniBind)、
[Point-Bind](https://github.com/ZiyuGuo99/Point-Bind_Point-LLM)、
[ImageBind](https://github.com/facebookresearch/ImageBind)。上游源码和权重遵循各自许可证，
见下载源码中的 LICENSE。
