# 3001室内RGB/LiDAR候选阶段评估

本阶段解决了显示链路，并改善了投影深度一致性；尚未证明分割精度提升。全部回放使用实机数据，结构测试与实测评估分别记录。

| 路线 | 当前实机观察 | 结论 |
|---|---|---|
| PMF / SemanticKITTI | 一个真实帧约97%的投影点被判为其他车辆；与同帧RGB分支差异约93.5% | 道路类别与室内任务不匹配，继续保留作对照 |
| 调整PMF输入坐标轴 | 同帧回放仍几乎全部判为其他车辆 | 没有作为修复部署 |
| YOLOE无提示完整词表 | 同一帧出现健身房、汽车制造等不适合的标签 | 不能据此宣称室内目标识别更好 |
| EMSANet / NYUv2 RGB-D | 原生RGB-D可输出墙、门、地面等，但有柜台等误分；将深度直接换成稀疏LiDAR后输出明显退化 | RGB-D权重不能作为即插即用的RGB/LiDAR权重 |
| PointTransformerV3 / ScanNet20 | 488个张量严格加载；单帧约600点仍误分。9组真实位姿对齐帧形成约4900个有色点，出现椅子、柜子、墙等室内类别，仍有误分 | 作为3001室内实验候选，保留PMF视觉对照；需要标注与独立基线验收 |
| ODIN | 官方有ScanNet完整2D/3D任务权重，依赖有位姿的RGB-D帧和相应几何输入 | 已核对源码/模型目录，未运行；不能宣称直接适配Livox |

PointTransformerV3的RGB颜色和LiDAR法向是6通道实际输入，XYZ用于空间位置。颜色来自对应曝光中的像素；法向来自原始LiDAR扫描的局部几何。多帧使用时间戳对应的ICP位姿，保留源帧和扫描索引。注册相机深度只用于20cm表面一致性剔除，不替代原始LiDAR。

作者完整权重：[Pointcept ScanNet20](https://huggingface.co/Pointcept/PointTransformerV3/tree/main/scannet-semseg-pt-v3m1-0-base)。源码固定提交 `77f87abe69bf098ebccf3e9b08400963954af6ac`。本机采用作者原生PyTorch注意力，关闭可选FlashAttention；没有新增未训练层。基准数据完整场景与本机稀疏局部场景不同，单次体素采样也不同于官方全部测试投票。

固定安装已由用户确认。相机／LiDAR外参经实测RGB-D与原始扫描自动估计，不能从安装照片获得精确矩阵。拟合后在另一真实帧的同一组954个点上，投影深度残差中位数约73cm→14cm，20cm内比例约22%→55%。剩余尾部误差仍大；这是深度一致性指标，不是外参真值RMSE或语义准确率。估计仅供3001实验使用，保留未验收标记，未改原SLAM的TF。

主要记录位于 `logs/research/`：

- `pmf_quality_diagnostics.json`、`pmf_frame_replay.json`：同帧分支及坐标轴回放。
- `yoloe_indoor_replay.json`、`emsanet_indoor_evaluation.json`：候选真实输入回放。
- `extrinsic_estimate.json`、`extrinsic_independent_validation.json`：估计及独立帧复核。
- `indoor_point_scene.npz`、`ptv3_scene_replay.json`：真实多帧点云和模型输出。

后续精度评价应在相同类别、留出场景和同一有效区域上标注，比较独立单模态及融合模型的mIoU。当前没有这组真值，界面框、类别占比和较低的分支差异都不是准确率证明。ScanNet20没有机器狗类别，未知物体仍可能被强制归入已有类别。
