# YAMNet 声音事件识别与哨兵巡逻展示

YAMNet 直接订阅现有 `/multimodal/audio`，独立输出 521 类声音的模型分数、
Top-5 候选和可配置的巡逻声音记录。没有视觉目标时仍能识别声音。
这是声音事件分类，不是语音转写；声源方向、距离与目标 ID 关联由其他模块提供。

## 启动

主机运行。安装一次：

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/install_yamnet.sh
```

终端 1：复用已经在运行的麦克风采集，启动声音识别。

```bash
cd /home/jyfight/Project/rtabmap_slam
source scripts/ros2_wired_env.sh
bash scripts/start_yamnet_audio.sh
```

终端 2：独立的哨兵巡逻网页。

```bash
cd /home/jyfight/Project/rtabmap_slam
source scripts/ros2_wired_env.sh
bash scripts/start_yamnet_miti.sh
```

打开 <http://localhost:3004/rtabmap.html?view=audio>。
布局包含三维地图、彩色画面、YOLO 分割和声音面板；地图与视觉区域使用原有节点的输出。
声音识别本身只需要麦克风。

### 在现有 3002 融合页面中查看

声音面板已接入 <http://localhost:3002/>。VS Code 网页和外部浏览器各自保存
`localStorage`，因此同一地址也可能显示不同布局。
`config/miti_workspace_sync.json` 指定 3002 的一次性更新：各浏览器首次加载新版页面时，
备份其原有布局与连接，保留已有融合／相机控件，补上地图和声音面板，
将主地图固定坐标系设为 `map`，并连接支持声音话题的 8768 桥接。
未配置过项目布局的浏览器会获得项目融合布局。
同步完成后仍可调整布局和视角，后续刷新不会重复覆盖。

日常启动 3002 页面：

```bash
bash scripts/start_fourmodal_miti.sh
```

已经运行的旧版 3002 前端可只重载网页服务：

```bash
python3 scripts/reload_miti_frontend.py --port 3002 --bridge-port 8768
```

在浏览器刷新原有的 3002 地址即可；旧 `rtabmap_ui` 查询参数不影响新版资源加载。

如果 `/multimodal/audio` 没有发布者，另开一个终端执行现有采集入口：

```bash
source scripts/ros2_wired_env.sh
bash scripts/start_fourmodal_audio.sh --alsa-device hw:CARD=XFMDPV0018,DEV=0
```

同一个 ALSA 设备只启动一个采集进程。当前输入由 XFM 麦克风发布，
YAMNet 与四模态特征节点可以同时订阅。
Ctrl+C 分别停止本轮识别和独立网页进程。

## 模型与预处理

- Google 官方 YAMNet TFLite 模型，约 4 MB；模型 SHA-256 固定校验。
- 16 kHz 单声道、浮点 PCM `[-1, 1]`。多声道求平均，其他采样率用多相滤波重采样；不自动增益。
- 模型图内包含 log-Mel 预处理。每次输入 15,600 个采样点（0.975 秒），
  对应 0.96 秒特征片段；相邻片段间隔 0.48 秒。
- 默认分析最新 2 秒录音，对完整片段的 521 类输出求均值。
  每个类别的分数独立，不将它们做 softmax，也不解释为风险概率。
- 独立 `.venv-yamnet`，仅使用 CPU TFLite 推理；默认 2 个线程。
- 单工作线程与一个待处理窗口。繁忙时用新窗口替换待处理窗口，拒绝重复、过期和格式错误的输入。

YAMNet 的 521 类分数作为分类结果发布。原有 UniBind 声学语义特征仍走原有编码器，
其共享空间中的 1024 维向量与 YAMNet 分类分数含义不同。

## 输出与巡逻记录

| 话题 | 类型 | 内容 |
|---|---|---|
| `/multimodal/audio` | `rtabmap_features_msgs/msg/AudioFrame` | 现有归一化 PCM 输入，时间戳为首个采样点 |
| `/perception/audio/events` | `std_msgs/msg/String` | JSON：`yamnet_audio_events/1`、全部 521 类分数、Top-5、巡逻事件、来源、耗时 |
| `/perception/audio/status` | `std_msgs/msg/String` | 模型加载、等待音频、识别、错误及丢弃计数，每秒发布 |

结果保存到 `logs/audio/yamnet/events.jsonl`。
原始采集时间、分析起点、时钟偏移和 `stamp_quality` 随结果保留。
本机 ALSA 采集仍是 `host_read_end_estimate`，不代表硬件同步已经验证。
远程音频的偏移可通过 `audio_clock_offset_s` 设置。

`config/yamnet_patrol.json` 提供人声、脚步、犬吠、车辆／发动机、警报、
敲击／撞击和破碎声七组规则。持续声音默认连续两个窗口达到阈值才确认；
短促敲击和破碎声默认一个窗口。确认后同一段声音保留同一事件 ID，
首次确认进入网页最近巡逻记录。可调整类别、阈值、确认窗口数及最低音量。
这些阈值用于演示起点，需在巡逻现场录音上评估误报与漏报。

声音面板显示原始类别及模型分数。断流后显示“音频已中断”，保留的上一帧候选不会标为实时结果。
现有项目布局刷新时会增加声音面板；也可使用独立的 `view=audio` 布局。
已有桥接服务需允许 `/perception/audio/(events|status)`，独立网页启动入口已包含这两个话题。

## 录音检查与测试

```bash
.venv-yamnet/bin/python3 scripts/predict_yamnet_audio.py --check
.venv-yamnet/bin/python3 scripts/predict_yamnet_audio.py \
  --wav logs/audio/m260c_comparison_cjkh2_c2/array.wav \
  --output logs/audio/yamnet/recording_check.json
bash scripts/test_yamnet.sh
```

主机已经有实时麦克风与识别节点时，只读复核：

```bash
source scripts/ros2_wired_env.sh
source install/features/local_setup.bash
.venv-yamnet/bin/python3 scripts/check_yamnet_live.py --seconds 6
.tools/miti/root/usr/lib/miti/node/bin/node tests/check_yamnet_bridge.js
```

分别保存 `logs/audio/yamnet/live_check.json` 和 `logs/audio/yamnet/bridge_check.json`。
前者按 `source_id` 对照实际接收的麦克风帧；后者读取网页静态资源，
从实际 Foxglove 桥接解码声音结果，并用同一 React 面板解析与渲染。

ROS 测试固定使用隔离域 96 和本机通信，使用保存的麦克风录音回放验证分类接口，
覆盖无输入、坏格式、旧帧、未来帧、重复帧、事件确认、断流与恢复。
默认测试录音是本项目已有的 12 秒 XFM 录音；另一份测试录音可通过 `--wav` 指定，
其中 6–8 秒应包含人声。
录音回放与静音检查证明运行链路，不构成 521 类或现场巡逻准确率验收。

### 2026-11-18 主机验证记录

- 模型及输入／事件规则：13 项通过；声音面板解析与渲染：5 项通过。
- 共享前端安装：7 项通过；现有网页入口与栅格显示回归：7 项通过。
- 隔离 ROS 域 96：7 组检查通过，共发布 5 帧；包括人声录音回放、断流与恢复。
- 真实 XFM 麦克风只读采样 6 秒：收到 12 个音频窗口，
  12 条分类结果按来源 ID 对应成功，格式及时间字段检查无错误。
- 该次真实输入的声音核心计算中位耗时 17.81 ms、最大 24.70 ms；
  相对采集端估计窗口结束时间的发布延迟中位数 98.58 ms。
  这些数据只覆盖声音分支，时间来源仍是主机读取估计，不包含网页显示耗时。
- `http://localhost:3004` 静态资源和 `ws://127.0.0.1:8768` 桥接检查通过；
  实际桥接的 3 条声音结果通过同一 React 面板的解析与渲染。
  本轮环境未提供可用浏览器，因此未执行浏览器截图或交互检查。

详细记录在 `logs/audio/yamnet/ros_check.json`、`live_check.json` 和 `bridge_check.json`。

## 公开来源

- [TensorFlow 官方 YAMNet 说明与 Apache-2.0 源代码](https://github.com/tensorflow/models/tree/master/research/audioset/yamnet)
- [Google 官方 TFLite 模型](https://storage.googleapis.com/mediapipe-models/audio_classifier/yamnet/float32/1/yamnet.tflite)
- [官方 521 类标签表](https://github.com/tensorflow/models/blob/master/research/audioset/yamnet/yamnet_class_map.csv)

加载时同时核对模型内附标签的顺序与项目标签表，防止类别索引错配。
