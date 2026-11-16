**真实 RGB＋近红外分割融合**

已实现同帧目标级融合、独立近红外分割、MITI 三路对照、同步采集，以及人工标注后的精度评估。当前没有人工标注，正式 `/perception/segmentation/selected` 保留 RGB 基线；融合预览标为“精度未验证”。本实现复核实例类别，保留 RGB 的原始掩膜、框和置信度，不能补回漏检或改善掩膜边界。

RGB 掩膜经注册到 RGB 的深度、相机内参、静态外参和遮挡检查，关联真实近红外像素。D435 的右近红外 CameraInfo 使用左相机参考系，P[3] 包含约 5 cm 基线；投影到左参考系后仅应用一次 P 平移，同时验证右图像坐标系的实际 TF 基线与 P 一致。不能直接复制 RGB 框作为近红外框。

UniBind 对两路对应目标区域分别编码，按 RGB 权重 1 和近红外几何覆盖比例归一加权，再与明确配置的 18 类文本原型比较。近红外是灰度输入复用视觉分支，当前未经过近红外域精度验证。权重、文本原型、最小覆盖、类别映射和模型组成固定策略指纹。独立近红外 YOLO 只读取近红外图像，原生预测投影到 RGB 网格作对照；它与目标特征中的近红外分类建议是不同结果。LiDAR 和音频当前权重为零。

**运行与查看**

在工作区已有模型、虚拟环境和相机话题可用时：

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/build_yolo_segmentation.sh
# 第一终端；已有 RGB 分割服务时跳过，文件锁禁止重复启动。
bash scripts/start_yoloe_segmentation_live.sh
# 未运行含深度配准的建图链路时，在另一个终端先启动独立深度预览。
# 已有 /camera/depth_registered/image_rect 和 /rgbd_image 发布端时复用现有链路。
bash scripts/start_depth_preview.sh
# 第二终端；启动独立近红外分割、目标融合和有上限的同步采集。
bash scripts/start_segmentation_fusion.sh
```

工作站的 RTX 3090 可使用独立 CUDA 环境。首次安装后，以 GPU 入口启动融合和独立近红外分割：

```bash
bash scripts/install_segmentation_gpu.sh
bash scripts/start_segmentation_fusion.sh --gpu
```

GPU 入口叠加 [segmentation_fusion_gpu.yaml](../config/segmentation_fusion_gpu.yaml)，两路使用 `cuda:0`，最小计算间隔为 1 秒。模型使用 float32，关闭 TF32；目标数量、裁剪、词表和类别门限保持原配置。CPU 入口仍为 `bash scripts/start_segmentation_fusion.sh`。CUDA 依赖安装在 `.venv-segmentation-gpu`，复用已有普通依赖。模型运行设备纳入评估流水线指纹；当前正式输出仍保留未评估的 RGB 基线。

配置见 [segmentation_fusion.yaml](../config/segmentation_fusion.yaml)。输入为 RGB 分割请求、近红外原图、注册深度、两路 CameraInfo 和 TF。采集时间差超过 50 ms、深度未注册或标定不匹配时等待有效输入，正式 RGB 输出继续发布。节点保留一帧计算和一帧最新等待请求，单工作线程限制资源；每次启动默认最多采集 60 帧，不会无限写盘。

原始深度在线不等于注册深度在线。注册节点退出后，右侧深度预览和融合都会等待；早先保存的“已采集 60 组”仍会保留。达到采集上限仅停止写入新样本，实时融合继续运行。独立深度预览入口检查既有深度/RGB-D 发布端，避免与建图配准重复发布。

在 MITI 打开 <http://localhost:3002/rtabmap.html?refresh=1>，点击目标分割面板的“融合对比”，查看同一采集帧的 RGB 单模态、独立近红外单模态和融合预览。CPU 模式融合约 16～25 秒/帧；当前工作站已切换 GPU 模式，连续 12 次推送的中位更新间隔为 1.14 秒，融合计算中位耗时为 0.32 秒，独立近红外推理为 0.03 秒，见 [fusion_gpu_live.json](../logs/segmentation_validation/fusion_gpu_live.json)。面板显示帧号、各路目标数量、耗时、有效覆盖、采集数量及断流状态。主叠加页独立实时更新，帧号不同的图片不能直接比较目标数量。Ctrl+C 停止融合入口会一起停止其近红外子进程。

三路预览保留 YOLO 的完整原生词表，并逐实例显示中文名称，文字使用深色底衬，字号随面板宽度调整。RGB 使用对应采集帧的全部原始实例，不按 18 类任务表、区域面积或同类连通区域过滤标签；融合仅修改实际通过候选门限的实例类别，保留其他 RGB 实例。标签优先落在实例可见区域，完全重叠时使用原始掩膜内的位置。近红外使用独立 YOLO 实例和标定投影，只展示投影后仍有像素的目标。`image_labels` 中的 `class_id` 为 YOLO 原生 ID，应使用明确的 `label` 显示，不能直接索引任务表 `class_names`；`image_instance_counts` 为三路图各自标签数。

人工标注、评估和目标级文本原型仍使用配置的 18 类任务 ID。预览的完整词表与评估映射独立；显示更多标签不代表精度提高。目标级文本特征的分类建议仍显示在详情中。

目标分割面板顶部的“融合对比”按钮用于进入三路对照。如果更新资源后仍只有“原图／叠加／类别图”三个按钮，Next 服务可能仍缓存旧页面脚本。对于当前 3002 的独立网页实例，执行 `python3 scripts/reload_miti_frontend.py --port 3002`，再打开刷新入口。该工具按原配置重载网页实例，并验证实际返回的页面及两份脚本与安装版本完全一致；拥有 ROS 桥接的实例会拒绝此重载方式。

新增 ROS 接口：

| 话题 | 消息与内容 |
| --- | --- |
| `/perception/segmentation/nir_request` | `SegmentationImageRequest`，会话/帧序号与真实近红外图像。 |
| `/perception/segmentation/nir_baseline` | `SegmentationFrame`，独立近红外原生掩膜与类别图。 |
| `/perception/segmentation/candidate` | `SegmentationFusionCandidate`，目标级类别复核；未评估时得分校准为 unverified。 |
| `/miti/segmentation/fusion_view` | String JSON，同帧三张 JPEG、目标建议、耗时、覆盖和策略指纹。 |
| `/miti/segmentation/fusion_status` | String JSON，心跳、输入/模型状态、最近错误与采集目录。 |

**采集、标注与评估**

每次运行创建 `logs/segmentation_fusion/capture_日期_时间/`，当前目录记录在 `live_current.json`。每个 samples 子目录包含真实 rgb.png、nir.png、arrays.npz 和 metadata.json。数组包含注册深度、两路预测、融合预测、几何有效区域、原生近红外图、实例图、两路目标特征及分类 logits；新采集还保存两路裁剪实例掩膜、原始参数和词表，可保留重叠区域重新分析。metadata 记录原始采集时间、标定、模型哈希、策略指纹和实例信息。

`annotations/` 初始为空；预测不会被写成真值。采集索引和模型预测可以用于人工检查，但所有用于验收的像素标签必须经人工审核。

已存在 2026-11-16 的 60 组实机采集：`logs/segmentation_fusion/capture_20261116_170034/`。已为它们建立 [待标注索引](../logs/segmentation_fusion/evaluation/index.json) 和 [当前评估状态](../logs/segmentation_fusion/evaluation/report.json)。这批数据来自同一次采集，不能把相邻帧随机拆成验证集和测试集。

1. 在不同场景、光照、距离和遮挡条件下分别采集。相同物理场景的多次采集应归为同一个 group_id。模型、参数、词表和标定保持一致；混合策略指纹会拒绝评估。
2. 用下面的命令汇总采集目录，创建评估索引。生成索引时所有 split 为 unassigned，annotation_source 为 pending。
3. 对原始 RGB 网格制作单通道 uint16 类别 ID PNG，尺寸必须等于 RGB 图像。类别 ID 按 [segmentation_fusion_task.json](../config/segmentation_fusion_task.json) 的 classes 顺序，从 0 到 17；65534/65535 表示忽略区域。完整标注任务类别，不得仅标注模型正确的区域。
4. 在索引里填写 annotation 路径、真实场景 group_id 和 validation/test；标签审核后设置 annotation_source 为 human_reviewed。同一 group_id 不能跨集合。需要独立验证场景及至少 30 张测试图、5 个独立测试场景。
5. 执行评估。温度校准和融合置信度门槛仅在 validation 选择；test 只用于最终比较。

```bash
.venv-semantic/bin/python3 scripts/prepare_segmentation_evaluation.py \
  logs/segmentation_fusion/capture_日期_时间 更多采集目录 \
  --output logs/segmentation_fusion/evaluation/index.json
.venv-semantic/bin/python3 scripts/evaluate_segmentation_fusion.py \
  --index logs/segmentation_fusion/evaluation/index.json \
  --output logs/segmentation_fusion/evaluation/report.json
```

评估同时报告 RGB、近红外、未门控融合候选及最终选定策略的 mIoU 和逐类 IoU。未知预测计为漏分；人工忽略像素不计入。验收要求完整 RGB 网格及共同几何有效区域上的融合 mIoU 均不低于最佳单模态，按场景分组的 1000 次 bootstrap 单侧 95% 差值下界也均不小于 0。没有标签、划分不足或不满足要求时退出码为 2，报告 passed=false。

如果验证集选择 RGB 直通，报告明确记录 selected_policy=rgb_baseline；这不代表真实融合提高了精度。如果近红外比 RGB 更准确而融合不能达到它，评估会失败，不会用 RGB 直通冒充满足目标。

仅当真实融合策略通过评估且 fusion_enabled=true，先停止原有两个分割入口，再使用：

```bash
bash scripts/start_segmentation_fusion_guarded.sh logs/segmentation_fusion/evaluation/report.json
```

生产者核对完整模型/参数/标定指纹，选择器核对 RGB 模型及报告校准标识。guarded 在候选计算时保留对应参考帧，最长等待 30 秒，原始相机回调仍保留最新帧；候选缺失、超时或不合格时保留基线。该结果只证明已标注留出分布上的统计比较，不保证每一帧、传感器缺失或未来场景的精度。

**验证**

```bash
bash scripts/test_yolo_segmentation.sh
source scripts/ros2_wired_env.sh
source install/segmentation/local_setup.bash
.venv-semantic/bin/python3 scripts/check_segmentation_fusion_live.py
# 采集尚未达到帧数上限时，核对当前预览与同帧原始实例和展示图片。
.venv-semantic/bin/python3 scripts/check_segmentation_fusion_live.py --verify-native-preview
# 用相同的 CPU 实机采集复核 GPU 速度、特征误差及类别与几何图。
.venv-segmentation-gpu/bin/python3 scripts/benchmark_segmentation_fusion.py \
  logs/segmentation_fusion/capture_20261116_194838 --device cuda:0 --samples 3
# 相机离线时，仅复核已保存的真实数据。
.venv-semantic/bin/python3 scripts/check_segmentation_fusion_live.py \
  --capture-only logs/segmentation_fusion/capture_20261116_170034 \
  --output logs/segmentation_validation/fusion_capture_audit.json
```

隔离 ROS 测试明确使用合成模型，验证真实消息收发、独立近红外输入、错时拒绝、shadow 基线保持、采集原始实例掩膜和 guarded 保留参考帧。评估测试覆盖漏分计数、无标注拒绝、场景泄漏、双基线比较和报告完整性。实机检查使用真实图像与模型产物，重算投影、遮挡、近红外类别投影和加权特征，验证三路展示图；它不使用人工真值，不作为精度测试。验证记录在 `logs/segmentation_validation/`。

已完成 60 组真实采集的投影及融合数值复核，记录见 [fusion_capture_audit.json](../logs/segmentation_validation/fusion_capture_audit.json)。恢复注册深度后，已重新核对实机采集帧 #75074，RGB 的 6 个原始实例在三路对比中全部保留，独立近红外投影显示 1 个目标，融合预览显示 6 个目标；三路 JPEG 与同帧原始实例重新渲染的结果完全一致，见 [fusion_native_preview.json](../logs/segmentation_validation/fusion_native_preview.json)。前端解析和 React 渲染复核见 [fusion_native_preview_frontend.json](../logs/segmentation_validation/fusion_native_preview_frontend.json)。GPU 切换前，对同一批 3 帧实机数据的数值对照通过，特征最大误差小于 1.5e-6，类别和几何图一致，计算耗时中位数从 25.33 秒降为 0.39 秒，见 [fusion_gpu_benchmark.json](../logs/segmentation_validation/fusion_gpu_benchmark.json)。GPU 在线检查保留了对应帧的全部 7 个 RGB 实例，并核对三路 JPEG，结果见 [fusion_gpu_live.json](../logs/segmentation_validation/fusion_gpu_live.json)。3002 网页已核验实际返回资源；没有浏览器截图验证，当前仍无人工真值，精度状态保持未验证。
