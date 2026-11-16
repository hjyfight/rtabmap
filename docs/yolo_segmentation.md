**独立 YOLO 分割与融合接口**

已实现 RGB 实例分割、MITI 掩膜显示，以及可实际收发的融合请求、候选和结果选择接口。分割进程独立于建图；当前输出来源在界面标为 `RGB · YOLOv8` 或 `RGB · YOLOE`。默认 `shadow` 模式发布复核请求、检查候选，但正式结果始终采用原始 RGB 分割。

现已接通独立近红外分割和真实 RGB＋近红外 UniBind 目标级融合，提供同帧三路预览、同步采集和人工标注后的评估入口。运行方法、当前实机数据和精度验收见 [segmentation_fusion.md](segmentation_fusion.md)。已有 `/multimodal/features` 是整场景向量，本接口单独按实例计算特征；音频不参与对象类别改写。

本次当前服务选用 YOLOE，保留 YOLOv8 入口。2026-11-16 实机连续三帧均得到 6 个实例，模型推理约 0.41 / 0.24 / 0.17 秒；入口限制为最多每秒 1 帧。该结果验证非空掩膜的数据链路，不证明每个类别正确，现场仍有将设备误识别成安全头盔等情况。测试记录见 [live_yoloe.json](../logs/segmentation_validation/live_yoloe.json)、[ros_contract.json](../logs/segmentation_validation/ros_contract.json) 和 [bridge.json](../logs/segmentation_validation/bridge.json)。

**运行与显示**

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/build_yolo_segmentation.sh
# 工作站连接机器人：读取时钟估计、加载有线 ROS 环境，运行 YOLOv8n-seg。
bash scripts/start_yolo_segmentation_live.sh
# 可选：使用项目已有 YOLOE-11s-seg-pf 和独立虚拟环境。
bash scripts/start_yoloe_segmentation_live.sh
```

同一 ROS 域只允许一个独立分割进程，通过文件锁避免重复启动。模型启动入口保留前台进程，停止时使用 Ctrl+C。基础配置是 [yolo_segmentation.yaml](../config/yolo_segmentation.yaml)，YOLOE 覆盖配置是 [yoloe_segmentation.yaml](../config/yoloe_segmentation.yaml)。使用同机相机或自行设置的 ROS 环境时可直接运行 `bash scripts/start_yolo_segmentation.sh`。

更新已有 MITI 页面：<http://localhost:3002/rtabmap.html?refresh=1>。分割面板与四模态特征并排加入已有布局，保留原有视角和隐藏地图面板的设置；一旦用户主动移除分割面板，后续刷新不会反复加回。完整融合布局入口为 <http://localhost:3002/rtabmap.html?view=fusion>。

面板支持原图、掩膜叠加、类别图切换，显示目标数、推理耗时及实际输出来源。详细置信度和融合模式折叠显示。三种图片和类别信息封装在同一展示消息中，防止将旧掩膜画到新相机帧。正常空检测显示 0 个目标；相机断流、节点退出或时间戳不匹配显示明确状态，保留的旧画面不会标为实时。

**接口契约**

算法话题前缀为 `/perception/segmentation/`。

| 后缀 | 消息 | 行为 |
| --- | --- | --- |
| `baseline` | `rtabmap_semantic_msgs/SegmentationFrame` | 原始 RGB YOLO 输出。 |
| `fusion_request` | `SegmentationFusionRequest` | 同帧原始 RGB、完整 baseline、类别 ID/名称词表、处理时限；`off` 模式不发布。 |
| `candidate` | `SegmentationFusionCandidate` | 外部分割复核器发布的类别修改建议。该接口不接受新掩膜。 |
| `selected` | `SegmentationFrame` | 对外采用的结果；基线 revision=0，采用候选时 revision=1。 |
| `fusion_decision` | `std_msgs/String` JSON | 每个候选对应的会话、序号、提供方、采用/拒绝理由，便于调试和记录。 |

展示话题 `/miti/segmentation/view` 为标准 String JSON，包含同帧三张 JPEG 及实例元数据；`/miti/segmentation/status` 提供心跳和状态。也提供标准 `sensor_msgs/Image` 话题 `/miti/segmentation/source`、`overlay`、`class_image`。这些图片均保留输入 RGB 的采集时间戳。

`SegmentationFrame` 的关联键为 session_id、sequence、header.stamp、header.frame_id，另带模型哈希和 vocabulary_version。`provenance_json` 记录模型参数、输入时钟偏移和估计方法。实例 ID 只在当前帧内有效；原始 YOLO 置信度保存在 confidence，复核得分另存 classification_score（不存在时为 -1）。框是原图 xyxy 像素坐标。每个实例的 mono8 掩膜按自身有效范围裁剪，mask_offset 是它在原图中的左上角，保留原始重叠区域。

类别图是 32SC1，-1 表示未知；实例图是 32SC1，0 表示未覆盖。单值图发生重叠时按原始 YOLO 置信度从高到低分配，同分使用较小实例 ID。原始实例掩膜不会因单值图覆盖规则而被修改。类别 ID 使用当前模型的编号；YOLOE 的显示名称有别名归一，多个编号可能显示同一名称，跨模型比较时仍需制定任务词表映射。候选必须使用本次请求的词表编号和版本。

**接入真实融合复核器**

复核器订阅 fusion_request，以 request.baseline 的实例掩膜裁剪原始 request.rgb，再关联该帧对应的近红外区域和 LiDAR 点集。融合模型得到结果后，在 candidate 上发布以下内容：

```python
from copy import deepcopy
from rtabmap_semantic_msgs.msg import SegmentationFusionCandidate, SegmentationRefinement

def publish_refinement(publisher, request, model_output):
    # model_output 必须来自真实目标级复核，包含实测关联与校准信息。
    base = request.baseline
    candidate = SegmentationFusionCandidate(
        header=deepcopy(base.header), session_id=base.session_id,
        sequence=base.sequence, model_sha256=base.model_sha256,
        vocabulary_version=base.vocabulary_version,
        provider_id=model_output['provider_id'],
        calibration_version=model_output['calibration_version'],
        score_calibration_version=model_output['score_calibration_version'])
    candidate.refinements = [SegmentationRefinement(**row)
                            for row in model_output['refinements']]
    publisher.publish(candidate)
```

每条 refinement 包括 instance_id、class_id、score，以及固定顺序 RGB/NIR/LiDAR/Audio 的 valid_mask、weights、source_stamps，还需提供 spatial_alignment_valid 和 time_alignment_valid。source_stamps 使用秒，必须先换算到 RGB 原始时钟域；不要直接使用各传感器未校正的时钟。缺失模态权重为 0，有效模态权重和为 1。目前至少需要两路有效非音频模态，且不接受音频参与对象类别复核。请求中的原始实例置信度不能被视为已校准的复核分数。

支持三种模式：off 只输出 RGB；shadow 发布请求并检查候选，正式输出保持 RGB；guarded 才允许符合条件的候选修改类别。guarded 同时要求提供方获准、完整评估报告通过且模型匹配、得分校准版本与报告一致。使用 [start_segmentation_fusion_guarded.sh](../scripts/start_segmentation_fusion_guarded.sh) 同时启动经过评估的生产者和选择器；仅设置 approved_fusion_providers 不会启用未经评估的融合。模式在启动时设置。

候选必须匹配完整帧键、模型和类别表；实例 ID 不可重复、不可新增；类别和得分须有效，几何/时间关联需声明有效且时间差通过检查，标定及得分校准版本不得为 unknown/unverified。默认复核分数门槛为 0.8。只有最新已显示且尚未采用过候选的帧可被修订一次。超时、旧帧、未批准提供方及不合格候选保持基线，拒绝原因由 fusion_decision 给出。候选缺失不会阻塞 RGB 输出。

这些检查是接口约束；复核器提供的标定声明和得分仍需独立验证，不能将它们当作精度保证。此版本只支持修改类别，不能改进掩膜边界或补回 YOLO 漏检目标。

**时钟、资源与验证**

工作站与机器人当前存在约 57.5 秒墙钟偏移，直接使用工作站时间减相机时间会错误地拒绝新图像。工作站入口通过只读 SSH 估计偏移，用于帧龄检查；相机原始 header 不改动。也可显式指定 `--clock-report 路径` 复用一天内的测量文件，测量时间与估计方法会记录下来。复用值仍须结合实际帧龄核验，不能作为硬件同步或 50 ms 精度证明。相机和主机时钟改变后应重新测量。

推理使用一个工作线程，保留一帧正在处理和一帧最新等待图像；候选缓存按时间和帧数双重限制。实际速率取决于模型和 CPU，界面显示实际推理耗时。YOLOv8n 与 YOLOE 的词表、速度和错误模式不同；词表更大、阈值更低都不代表更准确。未识别区域始终为未知，当前能力属于实例分割及其类别图，不是完整场景语义分割。

```bash
bash scripts/test_yolo_segmentation.sh
source scripts/ros2_wired_env.sh
source install/segmentation/local_setup.bash
.venv-semantic/bin/python3 scripts/check_yolo_segmentation_live.py
.tools/miti/root/usr/lib/miti/node/bin/node scripts/check_yolo_segmentation_bridge.js 8765
```

隔离 ROS 测试使用明确标注的合成检测器，覆盖基线逐字节直通、真实消息收发、类别复核、错帧/过期拒绝、几何保持、断流状态和远端时钟。实机检查另用真实相机与模型，验证图像、掩膜、类别图、请求和展示像素的一致性；nonempty_masks_seen 区分空结果检查与真实非空掩膜检查。验证记录在 logs/segmentation_validation。浏览器自动化当前没有可用浏览器，显示验证包括桥接传输、实际面板解析和 React 渲染，不声称完成浏览器截图核验。

精度尚未评估。未来比较实例分割的 mask AP 或类别图的 mIoU，需固定任务词表、模型、阈值、后处理和同一测试集，再比较融合与最佳单模态。当前默认直通能保持所选 RGB 基线的预测，不证明达到“融合优于所有单模态”。
