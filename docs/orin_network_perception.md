# Orin 局域网感知部署

当前目标是传感器留在机器狗，Orin 通过路由器局域网接收 MID360、RealSense
RGB/深度/近红外和可选音频，运行建图、YOLOE 分割及 UniBind 融合，主机运行 MITI。
本次部署只操作 Orin 和主机，没有连接或更改机器狗，也没有启动机器狗传感器。

## 已配置的运行环境

项目目录为 `/home/qiyuan/Project/rtabmap_slam`。三个 Python 入口
`.venv-fourmodal`、`.venv-yoloe`、`.venv-segmentation-gpu` 复用 Orin 已验证的
NVIDIA CUDA PyTorch 2.5、CUDA 12.6 和自行编译的 torchvision 0.20；
新增依赖在感知虚拟环境中，原 EMSANet 环境保留。
TorchAudio 2.5.0 在 Orin 上针对同一 PyTorch ABI 编译。音频声谱预处理使用 CPU，
UniBind 的图像、音频和点云模型推理使用 `cuda:0`。

UniBind 使用与主机相同的固定源码提交和官方权重，YOLOE 使用
`yoloe-11s-seg-pf.pt`。UniBind 检查权重清单的 SHA-256；YOLOE 的预期 SHA-256 为
`292bdf157a9ec7315f34b567cb93467c5043cd1889a1cc18abbfdeb88d7a948d`。

离线依赖和 TorchAudio 源码保存在 `.cache/orin-perception/`。在已有 JetPack
GPU 基础环境的这台 Orin 上，可以离线重建：

```bash
cd /home/qiyuan/Project/rtabmap_slam
ORIN_PERCEPTION_WHEELHOUSE="$PWD/.cache/orin-perception/wheels" bash scripts/install_orin_perception.sh
bash scripts/build_fourmodal_features.sh
bash scripts/build_yolo_segmentation.sh
bash scripts/build_semantic.sh --headless
```

## 配对新机器狗

编辑 `config/orin_network_perception.yaml`：机器狗 IP、传感器 TCP 端口、Orin
网卡、显示主机 IP 和 ROS 域。默认机器狗 `192.168.1.104:7447`、Orin 网卡
`wlP1p1s0`、显示主机 `192.168.1.100`、ROS domain 74。
地址是接入配置，部署验证不会主动连接这个机器狗地址。

默认检查命令只检查本地部署文件，不连接机器狗：

```bash
bash scripts/start_orin_network_perception.sh check
```

机器狗发送端以后需要按现有 `hotspot_sensor_transport.py` 协议提供数据。
传输保留原始图像尺寸、深度像素、点云时间字段、CameraInfo、TF 和时间戳。
原有流编号 0–9 保留；新增可选音频流编号 10。
目前没有在机器狗上安装或修改这个发送端。

音频启用时，两端需要有相同的 `rtabmap_features_msgs/AudioFrame` 消息，发送端
发布 `/multimodal/audio`，并使用 `--audio`；Orin 配置中设置
`audio_transport: true`。采集设备、声道、采样率和音频时间来源必须由实际发送端提供。
没有音频输入时不会用合成音频填充生产结果。

两端采集时钟应保持同步；已有稳定时差可显式填写
`sensor_clock_offset_s`、`audio_clock_offset_s` 和测量来源。入口不通过 SSH
测量或修改机器狗时钟。原始消息时间戳始终保留。

## 手动启动

以下命令以后在机器狗发送端就绪时使用，本次部署不执行实际接收命令。
在 Orin 不同终端运行：

```bash
# 接收机器狗已启动的传感器发送流。
bash scripts/start_orin_network_perception.sh receive
# 只用雷达的基础 ICP 建图；已有 IMU 时可改成 lidar-imu。
bash scripts/start_orin_network_perception.sh lidar
# RGB/NIR 分割、对象融合预览和四模态特征；复用或启动本机注册深度。
bash scripts/start_orin_network_perception.sh perception
```

`perception` 不启动机器狗驱动或麦克风采集程序。输入缺失时等待真实输入；
默认四模态结果要求 RGB、NIR、LiDAR、音频齐全。RGB/NIR 对象融合使用实际配对
图像、注册深度和相机内部 TF，正式分割输出继续保留 RGB 基线。
运行日志在 `logs/orin_perception/`，Ctrl+C 只停止该入口启动的工作进程。

语义建图需填写新安装位置的相机到雷达实测外参 `mapping.camera_to_lidar`
和 `calibration_version`，再运行 `semantic`。默认继续使用 EMSANet；
需要 YOLOE 建图时显式将 `semantic_backend` 改为 `yoloe_pf`。
外参为空时不会沿用示例安装位姿启动语义建图。

主机继续使用独立域 74 的 MITI 显示入口：

```bash
bash scripts/start_go2_orin_display_miti.sh 192.168.1.103 wlx6c1ff712636d
```

网页地址为 `http://localhost:3001`。更换显示主机时替换实际网卡和 Orin 地址。

## 2026-11-17 部署验证

源码、标签显示和融合代码已同步到 Orin。UniBind 固定源码提交为
`7b22a7790af38d25ffa5b12d49fbe786dc302e6e`，约 4.86 GB 的权重 SHA-256 为
`9c4793c7bd5c56d3fc948769f953f45a2e1cee0af1a068137e9407c1ba86d9b0`。

| 验证项 | 结果 |
| --- | --- |
| CUDA 环境与 TorchAudio 预处理 | 通过；保留 NVIDIA 原 PyTorch |
| UniBind 真实权重 GPU ROS 链路 | 四路各 1024 维、拼接 4096 维、融合 1024 维；拒绝过期音频与重复 RGB |
| YOLOE + UniBind GPU 对象融合 | 样例输出 4 个融合对象；NIR 与几何使用测试数据 |
| YOLOE GPU 语义建图 | 通过三维投影、回环重投影、重载和地图重置检查 |
| TCP → ROS 回环传输 | 11 路流通过；保留深度、点云时间字段和音频来源 |
| 统一感知启动与退出 | 5 个工作进程启动；无输入时等待；退出后所属工作进程已停止 |
| 本地部署检查 | `deployment_ready: true`；相机到雷达外参尚未填写 |

14 项部署检查和额外的统一启动检查均通过。Orin 原始记录位于
`logs/orin_perception/`，主要报告为 `tests.json`、`environment.json`、
`unibind_gpu_ros.json`、`fusion_models.json`、`network_transport.json`、
`launcher_startup.json` 和 `deployment_check.json`。

这次验证使用离线样例和 Orin 本机回环网络，没有连接机器狗，没有启动长期接收或
建图服务。新机器狗的实际数据发送、持续带宽、相机/音频输入、安装外参和时间同步
仍需接入后验收；上述结果不代表硬件精度或实时性能已经通过。
