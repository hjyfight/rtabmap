# 多源信息特征提取与融合处理周期测量

## 可直接使用的测量方法文字

在规定的传感器配置、输入频率、图像分辨率、地图规模和计算平台负载下，向受测
处理模块连续输入具有唯一关联标识的多源数据。以该模块接收到本组待处理数据的
时刻为起点，以对应特征提取、融合处理完成并提交结果发布的时刻为终点，使用
同一单调时钟测得逐组处理周期 `C_i = t_out,i - t_in,i`。计时包含模块内排队、数据
预处理、特征提取、融合及结果发布，不用配置频率的倒数或单次神经网络推理耗时
替代。记录全部原始计时、输入/输出数量、重试、丢弃、失败及测量结束时未完成任务。

分别报告首帧/初始化开销与预热后测量阶段的均值、中位数、P95、P99、最大周期，同时单独
报告相邻新结果的更新间隔。对“应不大于任务需求周期”的严格限值，按有效试验窗口内
最大周期与任务需求周期比较，并报告超限次数。平均值或 P95 达标不能覆盖最大值超限；
没有任务阈值、缺少样本、存在计时缺口或未完成任务时，不给出通过结论。预热排除方式、
试验持续时间、样本数量和容许丢帧条件应在正式试验前固定，不能看完结果后再调整。

当前用户尚未指定任务需求周期，所以本次只测实际值。工具默认的 30 个最少样本是
防止极少样本被误判的保护值，不是标准规定的充分试验规模。正式验收应覆盖代表性场景、
运动、回环和地图规模增长，并增加长时间满负载试验。

## 本工程的实际计时边界

当前 RGB-D EMSANet 确实以 RGB 和注册深度作为两个网络输入。新增逐帧主口径为：

`/mapData` 进入关键帧桥接器 → 排队与图像解码 → `/semantic/keyframes` → 等待图位姿/工作线程
→ 图像转换 → 网络内特征提取与融合及掩膜后处理 → 深度三维投影 → 跨帧物体融合
→ SQLite 保存 → `/semantic/objects` 发布调用返回。

这是 **RGB-D 语义处理子系统的周期**。起点之前的传感器采集、网络传输、RGB-D 配对、
LiDAR/IMU 去畸变与里程计、SLAM 后端处理，以及终点之后的 DDS 接收、网页渲染，
均不包含在这个数值内。多次观测后才首次确认物体的响应时间也不是单帧处理周期。
若任务指标针对传感器采集至最终应用的整个系统，必须继续为这些边界关联原始输入组，
本报告不能直接充当全系统符合性结论。

同时只读采集 `/odom_info_lite` 的 `time_estimation` 和 `/info` 的 `Timing/Total/ms`，
提供 LiDAR/IMU 里程计及 SLAM 原节点报告的阶段耗时。它们没有与上述语义周期构成
同组、串行、无重叠的计时分解，**不能将这些统计均值或最大值相加作为总周期**。

`network_and_masks` 包含 RGB/深度网络特征提取、网络内融合与后处理，不能将它命名为
“纯融合算子耗时”。现在新增以下 CPU 分层计时，并保留这个总量以校验覆盖范围。

| 新增字段 | 计时范围 |
| --- | --- |
| `emsanet_rgb_features` | RGB 分支 5 个骨干阶段的累计执行时间 |
| `emsanet_depth_features` | 深度分支 5 个骨干阶段的累计执行时间 |
| `emsanet_feature_extraction` | 上述两个分支骨干执行时间之和，排除融合层 |
| `emsanet_feature_fusion` | 5 个 RGB-D 特征融合层累计执行时间，含 SE 加权与逐元素相加 |
| `emsanet_context` | 编码器之后的上下文模块 |
| `emsanet_decode_and_postprocess` | 顶层解码器及其内部后处理 |
| `emsanet_model_forward` | 模型完整前向执行 |
| `emsanet_model_other` | 完整前向中未归入上述互不重叠模块的开销 |

`emsanet_timing.py` 使用 PyTorch 前向钩子和 `perf_counter_ns()`。当前网络融合方式为
`se-add-uni-rgb`，RGB/深度骨干和融合层逐级交替执行；RGB 后续骨干阶段接收的已经包含
前一级融合信息，因此这里按实际执行模块归类，不声称它们是两条独立、完全不交互的网络。
各分支小计与提取合计是嵌套统计，不能重复相加。`object_fusion` 仍指后端跨帧物体融合，
与网络内的特征融合分开统计。

钩子只读取执行边界，不改权重或张量。每帧检查所有选定模块是否恰好执行一次，并校验
分项与总量；缺失、重复或重叠的记录不作为有效分层样本。失败或空深度帧清空前次结果，
避免旧耗时误归到新帧。CPU 推理中的调度等待也计入墙上执行时间；钩子有少量测量开销。
GPU 模式明确标为 `unsupported_device`，不把异步提交耗时冒充设备执行时间。
当前掩膜在 CPU 上取回，外围 `network_and_masks` 边界在结果取回之后。

分层结果随 `/semantic/timing` 的 `stages.model_timing` 发布；原始记录含各层边界、融合方式、
有效状态和累计时间，统计脚本自动生成提取与融合的均值、分位数和最大值。新增代码安装后
需要新启动的语义进程才会加载，已经运行的 Python 模型不会自动更新。

## 实现

- C++ 桥接器发布 `/semantic/bridge_timing`，记录原始排队入口、解码及发布边界、队列深度，
  并报告队列溢出、地图重置丢弃、无效输入和解码失败。
- Python 语义节点发布 `/semantic/timing`，每次实际完成产生一条记录，分列各阶段。
  相同关键帧重传不重置原始等待时间；缓存命中、繁忙未接受等另记事件。
- 使用 Linux `CLOCK_MONOTONIC` / `time.monotonic_ns()`。跨进程相减前校验相同 `boot_id`。
  以 `session_id + node_id + 原始 double node_stamp` 关联，不作最近时间配对。
  传感器时间戳仅作数据身份，不与电脑当前时间相减。
- `scripts/measure_fusion_cycle.py` 只订阅诊断信息，生成 `.jsonl` 原始记录、`.json` 统计与
  `.md` 中文报告。重传事件去重、序号缺口、无匹配桥接记录、未完成任务均可见。
- 尚未重启的旧语义进程只有 `/semantic/status`。工具只在完成计数增加时采一次
  `last_inference_seconds`，重复状态不重复计数；多帧被跳过时记录缺失的耗时条数。
  该字段始终列为局部“推理与三维投影”，不能替代完整周期。

这些计时信息不进入模型或语义数据库配置指纹，不改变已保存地图的模型兼容性。

## 使用方法

正常构建语义包后，下一次启动语义建图将自动发布新增计时话题：

本机已经安装本次验证后的桥接器和 Python 计时代码，未重启当前建图。安装记录及
旧文件备份位于 `logs/fusion_cycle/deployment_20261110.json` 所列位置；下次正常启动
`scripts/start_semantic_emsanet.sh` 即可启用。下面构建命令用于后续源码更新或其他机器。

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/build_semantic.sh
```

保持当前建图运行，在另一个终端测量。输出前缀须为新路径，避免覆盖旧记录：

```bash
source scripts/ros2_wired_env.sh
source install/semantic/local_setup.bash
/usr/bin/python3 scripts/measure_fusion_cycle.py --seconds 60 \
  --output logs/fusion_cycle/my_measurement
```

如果实际里程计统计话题不同，用 `--odom-info-topic /实际话题` 指定。工具不会同时订阅
完整和 lite 统计而重复计数，也不会为了测量额外订阅庞大的局部点云。

任务周期确定后可对原始记录重新统计。下例 1000 ms 仅演示参数，**不是当前验收要求**：

```bash
/usr/bin/python3 scripts/measure_fusion_cycle.py \
  --from-jsonl logs/fusion_cycle/my_measurement.jsonl \
  --task-period-ms 1000 --minimum-samples 30 \
  --output logs/fusion_cycle/my_measurement_limit_1000
```

`within_limit_for_measured_scope` 只表示这个计时边界内的有效样本未超限；JSON 的
`whole_system_compliance_proven` 始终为 false。需要逐阶段评价其他模块时，应使用它们各自
的需求周期和完整边界，不套用语义模块限值。

## 不中断正式建图的实际数据回放

先只读采集当前真实关键帧，保留 RGB-D 原始压缩/未压缩内容及其优化位姿：

```bash
source scripts/ros2_wired_env.sh
source install/semantic/local_setup.bash
/usr/bin/python3 scripts/capture_cycle_keyframes.py --frames 32 --seconds 90 \
  --output logs/fusion_cycle/my_capture
```

构建可放到独立前缀，避免替换正在运行的程序：

```bash
source /opt/ros/humble/setup.bash
source install/semantic/local_setup.bash
colcon --log-base logs/cycle_build build --base-paths src/rtabmap_semantic \
  --packages-select rtabmap_semantic --build-base build/cycle --install-base install/cycle \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

在单独终端设置隔离域，运行相同真实模型、桥接器、语义节点和临时数据库。输入周期
是试验负载，与验收限值独立；默认每 1000 ms 输入一个关键帧，并排除前两个成功样本
的汇总统计，但保留它们和初始化耗时；排除预热不表示队列已达到稳态。终点为真实 ROS 发布调用返回：

```bash
source /opt/ros/humble/setup.bash
source install/semantic/local_setup.bash
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
export PYTHONNOUSERSITE=1
.venv-emsanet/bin/python3 scripts/benchmark_fusion_cycle.py \
  --capture logs/fusion_cycle/my_capture --input-period-ms 1000 --warmup 2 \
  --output logs/fusion_cycle/my_replay
```

验证分层计时时追加 `--require-model-timing`，缺少任何有效编码器/融合层样本会明确失败。
要降低队列积压对本次计算耗时试验的干扰，可设 `--input-period-ms 1500`；这只是输入负载，
不代表任务需求周期或达标标准。

回放结果说明这批实际图像在该测试负载下的处理能力，不等于生产系统端到端时延。
回放采用新的小规模语义数据库，工作站上正式建图仍在运行；报告保留输入实际发送间隔、
配置和源数据标识。这些条件需要随数值一起引用。

## 本次验证

- 计时汇总测试覆盖包含排队、严格限值、缺少阈值/样本、时基不一致、身份不匹配、
  重复事件、非单调边界、未完成任务、预热、更新间隔、丢失事件和处理失败。
- ROS 固定检测器集成验证包含真实桥接解码、逐帧计时关联、三维投影、回环、保存恢复和地图重置。
- 在线采样：`logs/fusion_cycle/live_20261110.{jsonl,json,md}`，30 秒，旧进程的阶段统计。
- 实际关键帧采集：`logs/fusion_cycle/capture_20261110/`。
- 真实模型回放报告：`logs/fusion_cycle/replay_20261110.{jsonl,json,md}`。

本次尚无用户指定的任务周期，报告不得表述为“符合处理周期要求”。

### RGB-D 特征提取和特征融合分开计时（2026-11-10）

新采集 32 个真实 RGB-D 关键帧，在同一工作站上用 EMSANet R34-NBt1D、CPU 2 线程、
640×480 网络输入、batch=1 独立回放。输入间隔设为 1500 ms；正式建图同时运行，
这些数值是该并发负载下的回放测量，不能直接套回先前那次 60 秒在线报告。排除前 2 帧
预热，汇总 30 帧。32 帧的模块覆盖均有效，没有处理失败、丢弃或未完成任务。

| 计时范围 | 均值 ms | P95 ms | 最大 ms |
| --- | ---: | ---: | ---: |
| RGB 骨干特征提取 | 321.34 | 366.93 | 374.35 |
| 深度骨干特征提取 | 309.73 | 348.49 | 354.56 |
| 多源特征提取合计 | **631.08** | **702.04** | **719.24** |
| 网络内 RGB-D 特征融合 | **15.39** | **21.82** | **22.18** |
| 解码器及其后处理 | 929.93 | 1056.12 | 1085.74 |

特征融合为 5 个 `se-add-uni-rgb` 融合模块，包含两路 SE 加权与相加。各指标最大值可能
出现在不同帧，不能将分支的最大值相加替代“合计”的逐帧最大值。
同轮后端跨帧物体关联与融合平均 4.96 ms，属于另一个阶段。

原始与汇总记录：`logs/fusion_cycle/encoder_timing_20261110.{jsonl,json,md}`；
源数据：`logs/fusion_cycle/capture_encoder_timing_20261110/`。
测试同时验证了开启/关闭分层计时后官方样例的类别、置信度和逐像素掩膜完全一致。

计时代码已同步至本机安装包；下一次正常启动 `scripts/start_semantic_emsanet.sh` 后，
原有 `scripts/measure_fusion_cycle.py` 会自动汇总新增的两项。无需另加启动参数。

### 先前的整体周期测量

30 秒在线采样得到 ICP 估计耗时 81 个样本，中位数 113.02 ms、最大 232.99 ms；
SLAM 后端 26 个样本，中位数 40.20 ms、最大 56.26 ms；旧语义节点推理加三维投影
14 个样本，中位数 1460.10 ms、最大 1779.81 ms。它们是当时工作站负载下的不同阶段统计。

12 个真实关键帧以约 1000 ms 间隔进入独立回放；排除前 2 帧后，10 个样本的语义节点
周期中位数 1513.51 ms、最大 1790.24 ms。桥接排队持续增长，计入排队的主口径中位数
5750.99 ms、最大 8164.09 ms。没有失败、丢弃、计时序号缺口或未完成样本。
本批数据没有产生有效三维物体检测，不能代表物体密集场景的三维投影和对象融合负载；
这次回放也不能证明生产地图规模下的最大耗时上界。
