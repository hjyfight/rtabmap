# Go2 → Orin → 主机 MITI 3001

2026-11-22 实机部署。Go2 采集，Orin 建图与推理，主机展示。研究服务不发送机器人运动指令。

## 当前接线与网络

| 设备 | 地址 | 功能 |
| --- | --- | --- |
| Go2 板载电脑 | Wi-Fi `192.168.1.104`，维护有线 `192.168.123.18` | MID360、RealSense 采集 |
| Orin 计算机 | `192.168.1.103` | 当前 WheelTec USB 采集、RGB-D/LiDAR/IMU 建图、YOLOE 三维语义、ODIN/PTv3、RGB/NIR、AVSegFormer |
| 主机 | `192.168.1.101` | MITI `http://localhost:3001/`、ROS WebSocket 桥 `8766` |

三者连接 `TP-LINK_6E07_2.4G`。Go2 的 `eth0` 保留在雷达网段，MID360 为 `192.168.123.20`。Go2 独立供电拓展坞连接相机、无线网卡。WheelTec 在 Go2 上只有串口接口，用户随后将其直接接到 Orin，已成功枚举并采集 PCM。录音必须出现 USB `2207:0001` 和 ALSA `XFMDPV0018`；`1a86:55d4` 只证明串口存在。

当前配置 `audio.source=orin`，话题为 `/multimodal/audio/orin`，音频时钟偏移为 0；Go2 视频/雷达仍使用 Orin-minus-Go2 偏移。音频来源和偏移写入模型结果，两个采集时钟不会混入同一个 PCM 话题。将来恢复 Go2 USB 音频后，改为 `audio.source=go2` 并重启本次 Orin 服务。

## 实际传输路径

```text
Go2 ROS domain 42（本机）
  雷达 + IMU + RGB + 深度 + IR2（可选 Go2 PCM）
             │ TCP 7447，字节重排 + 无损压缩
             ▼
Orin ROS domain 74（loopback）
  当前 WheelTec USB → Orin 本机 PCM
  深度配准 / 已校验 RGB-D / RGB-D + LiDAR-IMU SLAM
  MapData → 原 keyframe_bridge → 原 YOLOE semantic_mapper
  深度投影 / 跨关键帧物体关联 / 三维语义标签
  ODIN/PTv3、RGB/NIR、AVSegFormer 研究对照
  地图、轨迹、墙面、栅格和图像展示适配器
             ├─ TCP 7449：选定展示话题，无损压缩，限制展示频率
             └─ HTTP 7448：状态清单；结果序号改变时才下载完整结果
             ▼
主机 ROS domain 74（loopback）+ MITI 3001
```

ROS DDS 在每台计算机内走 loopback，避免 Orin 重新发布的全速点云/图像又经 DDS 在 Wi-Fi 上复制。主机只解压和展示，不运行研究模型。展示地图最多 1 Hz、图像最多 2 Hz；这些限制不改变 Orin 的传感器采集与模型输入。RGB、深度、IR2、点云、标定与 ROS 时间戳均保留。只有模型结果的展示时间额外换算到主机时钟，原始采集时间不改写。

地图语义沿用项目原有 **YOLOE prompt-free 实例分割 → RGB-D 三维投影 → RTAB-Map 关键帧关联 → 物体标签**。配置 `semantic.enabled=true`；CUDA 运行原 `semantic_mapper` 与 C++ `keyframe_bridge`。语义存入与当前 `map.db` 配对的 `map.yoloe.stable.semantic.sqlite`，发布 `/semantic/objects`，由原显示适配器转换成 `/miti/research/display/semantic_markers` 并经 TCP 7449 发到主机。ODIN/PTv3 的研究结果不替代这条 YOLOE 地图语义链路。

语义模式由原 SLAM launch 统一启动 `rgbd_sync` 与 `guard_rgbd_input`，避免独立研究同步器重复发布；已有相机外参 TF、深度配准和私有回环诊断继续复用。YOLOE 原稳定确认门槛未降低：在最新 6 个已处理关键帧中，静态对象需要至少 4 次确认。

Go2 只开启 IR2，RGB `424×240@6`，深度和 IR2 `640×480@6`。USB 设备掉线会使相机驱动重启；雷达、音频重试和传输继续运行。音频采用 16 kHz 单声道，0.5 秒窗口每 0.1 秒发布一次，接收端按样本时间保留至少 10 秒连续数据；缺失音频不会用零或重复片段替代。

## 启动

先确认 IP 与配置 `config/go2_orin_research_network.json` 一致。路由器重新分配地址后需更新配置。当前时钟偏移为 SSH 往返估计，不是硬件同步；设备重启或时钟调整后应重新测量。

Go2：

```bash
bash /home/unitree/go2_orin_network/scripts/start_go2_research_sensors.sh
```

Orin：

```bash
cd /home/qiyuan/Project/rtabmap_slam
bash scripts/start_orin_research_wireless.sh --check
bash scripts/start_orin_research_wireless.sh
```

主机：

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/start_go2_orin_research_display.sh
```

启动脚本通过锁避免重复实例。前台运行时 `Ctrl+C` 停止本次守护进程及其子进程。

## 状态和日志

| 设备 | 当前运行记录 | 日志 |
| --- | --- | --- |
| Go2 | `/home/unitree/go2_orin_network/running.json` | 记录中的 `run` 目录，含 `camera/audio/lidar/transport.log` |
| Orin | `logs/go2_orin_research/current_run.json` | 记录中的 `run` 目录及 `workers.json` |
| 主机 | `logs/research/supervisor.json` | `remote-display.log`、`remote-research.log`、`web.log`、`bridge.log` |

后台实例可以读取对应记录中的守护 PID，并对这个 PID 发送 `SIGTERM`；不要用全局 `pkill ros2`。模型源端 JSON 位于 Orin 当次运行的 `public/`，主机副本位于 `.tools/miti-research/root/usr/lib/miti/public/research/`。两端 `session_id` 和递增 `sequence` 应一致。`remote_transport.json` 中 `connected` 表示 HTTP 结果链路状态；模型等待原因以各任务 JSON 的 `detail` 为准。

传输日志每 10 秒记录 `interval_mbps` 和各话题 `interval_hz`；累计平均包含重连等待，不宜用来判断当前速率。

## 已有验证和实际限制

- 无损传输原有 9 项测试、展示传输 4 项测试通过；音频输入测试包括 Go2 10 Hz 短窗口的连续 10 秒重建与旧数据清理。
- 主机只读实测见 `logs/research/go2_orin_live_probe.json`：收到 RGB、配准深度、IR2、地图、实时扫描、轨迹、栅格、位姿和完整 TF 链。该次地图约 1 万点、实时扫描约 1.9 万点，墙面状态为 ready，位姿有效。
- YOLOE 源码、权重与编译产物已同步；6 个关键语义源文件的主机/Orin SHA256 一致。恢复启动链路后，实测已处理 93 个关键帧，失败 0 次，确认 7 个对象；主机收到 14 个三维标记，包含工作台、椅子等预测标签。原始验证记录为 `logs/research/yoloe_semantic_orin_probe.json` 与 `logs/research/yoloe_semantic_host_probe.json`。类别是模型预测，标签确认不等于人工真值验证。
- 实时模型测速采样见 `logs/research/go2_orin_live_speed.json`，统计实际递增结果序号的推理耗时，不包括 Wi-Fi 等待和浏览器绘制。
- 第一次视觉测速见 `logs/research/go2_orin_vision_speed.json`：45 秒取得 10 个不同序号，ODIN+PTv3 中位 3.4666 秒，RGB/NIR 中位 0.2621 秒；当时 Go2 麦克风接口缺失，AVSegFormer 尚未执行。
- WheelTec 直接接 Orin 后，最终 45 秒实测取得 ODIN+PTv3 10 组、RGB/NIR 7 组、AVSegFormer 7 组，推理中位分别为 **3.2054 秒、0.2317 秒、0.8951 秒/10 帧音视频**。三条结果序号均持续增加，来源为同一实机会话；HTTP 3001 的音视频结果保存于 `logs/research/go2_orin_avs_verified.json`。
- WheelTec 直接接 Orin 后，3 秒实际 WAV 检查为 16 kHz、16 位、单声道、48000 样本，其中 42547 个非零样本；RMS 约 -44.55 dBFS，峰值 0.0703。文件位于 Orin `logs/research/audio/wheeltec_orin_live_20261122.wav` 及同名 JSON。Go2 拓展坞上的音频路径仍未验证成功。
- 音视频窗口以实际到达的共同采集时间范围选取，可适应 Orin 本机音频早于 Go2 Wi-Fi 视频到达；仍要求连续 10 秒 PCM、10 张不同曝光图像和不超过 0.25 秒的配对偏差，不补造缺失样本。
- 预测图字体会校验中文、英文与数字字形，优先选择完整的 Noto Sans CJK；避免 Droid 中文备用字体把英文和数字绘制成方框。
- `research_camera_tf.py` 根据当前研究外参与 RealSense 工厂 TF 发布 `livox_frame → camera_link`，用于三维面板相机朝向。来源与矩阵记录在当次运行的 `camera_tf.json`；主机所需的相机到雷达变换验证见 `logs/research/camera_tf_verified.json`。自动外参仍保留 `geometry_verified=false`。
- 相机曾重新枚举为 USB2；低分辨率 6 fps 已恢复，但接线与拓展坞稳定性仍需持续确认。整坞掉线时先隔离相机和 WheelTec，只保留无线网卡，必要时更换上行数据线或拓展坞。
- ODIN 使用已做独立帧复核的自动外参估计，`geometry_verified=false`。此链路可用于研究展示，尚未完成物理标定验收和融合精度评价。ODIN/PTv3 比较也不是独立单模态基线。
- 原始源端时间戳、估计的时钟偏移及不确定度都写入结果。音频时间是主机读出时间估计，并非 ADC 硬件时间。动态点云输入本身没有在研究链路中按点去畸变，运动场景需进一步验证。

Orin 环境、官方权重、离线基准与 CUDA 扩展详情见 `docs/orin_research_deployment_20261122.md`。
