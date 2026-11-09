# EMSANet RGB-D 实例分割建图

这是新增的可选语义后端。RGB 和注册深度分别进入 EMSANet 的编码器，在网络内部融合特征，输出物体实例掩膜，再接入现有三维定位、物体关联和回环修正流程。

原来的 `scripts/start_semantic_slam.sh`（YOLOv8）和 `scripts/start_semantic_yoloe.sh`（YOLOE）继续使用原有默认模型和环境。墙面提取、配置、显示代码保持原状；EMSANet 的 wall、floor、ceiling 等 stuff 类别不进入物体实例标签流程。

## 启动

本机完成安装、构建后，网线连接环境使用：

```bash
cd /home/jyfight/Project/rtabmap_slam
source scripts/ros2_wired_env.sh
bash scripts/start_semantic_emsanet.sh
```

若已有建图进程，先在原建图终端按 `Ctrl+C` 并等待退出，再启动新入口。沿用原有相机、雷达驱动和 MITI 启动方法。这里使用 RGB 和深度，近红外不是模型输入。

热点连接时，沿用当前已经验证可用的热点 ROS 环境，然后运行同一个 `start_semantic_emsanet.sh`。新入口不主动切换网络。

已有地图可使用同一 RTAB-Map 数据库重新计算 EMSANet 语义：

```bash
bash scripts/start_semantic_emsanet.sh \
  database_path:=/完整路径/已有地图.db \
  localization:=true
```

已有地图必须含 RGB-D 关键帧。回放读取各节点保存的 RGB 和深度，不依赖实时近红外。默认允许回放；只处理之后的新关键帧可以传入 `semantic_replay_existing:=false`。

### 切换建图后显示为空或出现旧点云

网页 `Ctrl+R` 是正常刷新，不会修改地图或传感器标定。若 MITI 在上一轮建图时已经启动，重新订阅后仍可能收到桥接缓存中的旧点云；先核对 `/cloud_map` 的点数和时间戳。2026-11-08 的现场检查中，ROS 当前地图为 9,192 点，而网页桥接还回放上一轮的 119,019 点。保持原网络配置重启网页桥接后，网页恢复为当前地图，SLAM 无需重启。

原生 RTAB-Map 只有坐标轴时，可用 **Edit → Download all clouds (update cache) → Local map optimized → OK** 重新加载后端已有地图。这是显示缓存操作，不是重新建图。若还未积累足够的不同关键帧，物体标签会继续等待多次观测确认。地图和标签恢复显示并不等于分割精度、点云配准质量已经验收。

## 首次安装与构建

```bash
bash scripts/install_emsanet.sh
bash scripts/build_semantic.sh
```

本机 Clash 代理端口为 7897，下载时可使用：

```bash
bash scripts/install_emsanet.sh --proxy http://127.0.0.1:7897
```

代理设置只作用于安装进程。安装脚本校验权重压缩包完整性，并保留下载断点。

安装位置：

- `.venv-emsanet/`：Python 3.10、CPU PyTorch 2.5.1 和固定版本依赖，独立于两个 YOLO 环境。
- `.tools/emsanet/`：官方 EMSANet 源码及匹配的两个子模块。
- `models/emsanet/nyuv2_r34_NBt1D_pre.pth`：官方 NYUv2 R34-NBt1D 权重，使用过 Hypersim 预训练。
- `src/rtabmap_semantic/config/emsanet.yaml`：新后端的默认配置。

需要可访问 GitHub、PyTorch/PyPI 下载站和 Google Drive。安装脚本可重新执行；权重尚未下载成功时，新入口会提示安装，不会自动切换成 YOLO。

使用其他路径下、相同结构和类别体系的权重：

```bash
EMSANET_MODEL_PATH=/完整路径/nyuv2_r34_NBt1D_pre.pth \
  bash scripts/start_semantic_emsanet.sh
```

当前适配器固定为 NYUv2 的 R34-NBt1D。权重采用严格加载，SUNRGB-D、EMSAFormer 或任意其他结构的权重不能直接替换。

## 输入、输出和融合

输入来自 `/semantic/keyframes`，与 RTAB-Map 的具体节点关联：

1. RGB：ROS `bgr8`，进入网络前转换为 RGB。
2. 深度：已经配准到彩色相机视角。`16UC1` 按毫米处理，`32FC1` 按米转换为毫米；非有限值、非正值和超过 10 米的值作为无效深度。
3. 使用官方 NYUv2 RGB/深度归一化和后处理。默认网络输入 640×480；实例掩膜恢复到原 RGB 分辨率后再做三维投影。

深度图和 RGB 必须属于同一相机视场；缩放不是配准。新入口复用现有深度注册与输入检查，不另加一套配准节点。

输出沿用 `label / confidence / mask` 契约。类别来自 NYUv2 的 40 类体系，其中可数物体才产生实例标签。实例置信度采用官方 panoptic score，即“实例中心分数 × 该实例的平均语义概率”，默认阈值 0.35，与 YOLO 的分数定义不同。

适配器直接解码完整 panoptic ID，绕过固定版本上游 CPU 后处理中 `semantic_idx` 被原地减一的问题；否则可能将 sofa 错标成 chair。相应回归测试覆盖了类别索引、背景过滤和同类多实例区分。

默认使用 2 个 CPU 线程；静态物体需要最近 5 个已处理关键帧内至少 3 次观测确认。person 为动态类别。中文显示补充 NYUv2 特有名称，不改变 YOLO 的翻译策略。

同一画面中的不同实例继续采用现有的物体身份约束，避免仅因类别相同就合并。三维位置仍由 `lift_mask()` 和相机到节点的变换计算，回环后按图优化位姿重新投影。

## 语义数据库

同一个 `map.db` 会默认使用：

| 后端 | 语义数据库 |
| --- | --- |
| YOLOv8 | `map.semantic.sqlite` |
| YOLOE 稳定配置 | `map.yoloe.stable.semantic.sqlite` |
| EMSANet | `map.emsanet.semantic.sqlite` |

新数据库指纹包含模型 SHA-256 和 EMSANet 适配版本。显式指定 `semantic_database_path` 时，应指定对应后端自己的文件。更新模型或推理阈值后，使用新的语义数据库路径，保留旧结果用于比较。

## 验证与离线推理

原后端和公共代码回归：

```bash
bash scripts/test_semantic.sh
```

新后端的单位转换、掩膜适配和 ROS 三维链路测试：

```bash
bash scripts/test_emsanet.sh --fixture-only
```

包含官方权重真实推理的检查：

```bash
bash scripts/test_emsanet.sh
```

ROS 测试使用隔离域 95 和本机通信。先用固定检测器检查数据传递、已知三维位置、中文标签、压缩关键帧、回环、重载和地图重置，再运行官方权重的离线推理及真实模型 ROS 全链路。真实模型测试使用官方 RGB-D 样例和合成相机内参，验证投影、标签与回环一致性，不代表样例的真实三维定位精度。

自己的离线图像可这样测试：

```bash
.venv-emsanet/bin/python3 scripts/predict_emsanet.py \
  --rgb /完整路径/rgb.png \
  --depth /完整路径/registered_depth.png \
  --output logs/emsanet/my_sample
```

深度 PNG 应为毫米单位的 uint16；也支持米单位的 float32 `.npy`。输出包括 `instances.png`、`masks.npz` 和记录类别、置信度及推理时间的 `report.json`。

## 版本与能力边界

- [EMSANet 官方仓库](https://github.com/TUI-NICR/EMSANet)，固定提交 `9d2ade475ba44c6bd82f6fac6d44ac82086bcd53`，沿用官方推荐的 2023 代码版本，适配 ROS Humble 的 Python 3.10。
- `nicr-multitask-scene-analysis`：`834bd58c3e79f30153525950057a45a33f446f0f`。
- `nicr-scene-analysis-datasets`：`90344ecea1209ab9a5cf7ae8cf4d936e5835a8f0`。
- 默认官方权重文件 ID：`1QbOJXVrOzsVM8ltX7AxqFSVLsY6vzvNX`。
- 权重 SHA-256：`1814ef0b3173e6e9d16b11a66027ec41477b5dc100e9f8b100a91e2786bc951a`，安装脚本会校验。

本次接入使用官方 RGB-D 权重，不包含近红外分支或额外训练。NYUv2 类别范围比 YOLOE 小；例如它没有独立的 laptop、keyboard、monitor 类别。官方样例运行通过不能代替机器狗实际场景的精度测试，模型吞吐也应以本机测量为准。

## 本机验证记录（2026-11-08）

- 60 项启动配置、输入保护、语义几何、存储及检测器单元测试通过。
- 原有 ROS 几何链路、EMSANet 固定检测器链路和官方权重的真实模型 ROS 链路通过；覆盖 PNG 压缩关键帧、中文标签、三维投影、回环位置修正、保存重载及地图重置。
- 官方 RGB-D 样例输出 8 个实例；本机 CPU、2 线程、640×480 网络输入下，完整 ROS 推理及几何处理约 1.5–1.7 秒/关键帧。该样例原图为 1440×1080，不是机器狗实时帧率测试。
- 单图报告与叠加图：`logs/emsanet/sample/report.json`、`logs/emsanet/sample/instances.png`。
- 真实模型 ROS 报告：`logs/semantic_validation/ros_real_emsanet.json`。
- 修复了上游 CPU 类别索引偏移的适配问题；墙面模块没有改动，也未替换正在运行的机器人建图进程。
