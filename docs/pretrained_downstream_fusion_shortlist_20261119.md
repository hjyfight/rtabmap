# 面向检测／分割的免训练多模态融合项目清单

核对日期：2026-11-19。根据当前工作区代码，并联网读取作者仓库 README、模型源码、配置和公开权重目录。这里的“免训练”是加载作者已经训练好的完整任务模型进行推理；本次没有安装新增模型、下载大型权重、运行候选模型或修改当前服务。

用户要求：覆盖 RGB、近红外 NIR、LiDAR、音频，可以分组覆盖；用于研究特征提取、拼接、尺度匹配和语义交互；下游选择目标检测或语义／实例分割；比较融合结果与同任务的最佳单模态。

## 结论与推荐顺序

**需要选择完整的任务融合网络，而不是继续把全局向量拼接后直接送给现有 YOLO。** 已找到可免训练复现下游任务的项目，但目前没有核实到一套完整权重能直接消费现有 D435 近红外、单 RGB 相机、Livox 与 M260C 四路数据，并保证本机场景融合精度不低于最佳单模态。

建议分为三组任务实验，共同覆盖四种模态：

1. **RGB＋LiDAR：BEVFusion 先复现公开基准；PMF 再尝试当前单相机与 Livox。** BEVFusion 的融合和两种单模态基线均有完整任务权重，最适合验证研究流程。
2. **RGB＋近红外：GF-Net 做含 NIR 的任务特征融合；Pixel 做明确的 RGB/NIR 图像融合再接冻结分割器。** GF-Net 的类别和 DIN 编码有条件限制；Pixel 的最终任务精度需要另做对照。
3. **RGB＋音频：AVSegFormer 做音视频分割；需要发声实例与跟踪时选 AVISM。** 二者均发布完整任务权重，不需要用当前 YAMNet 分数充当音频特征。

这些实验的融合张量形状和类别空间不同，应分别保存、分别评价，不能把不同任务的 mAP、mIoU 直接比较，也不能把三个模型合称为已完成四模态联合分割。

## 当前项目与下游任务的差距

- `src/rtabmap_features/rtabmap_features/backbone.py` 使用冻结 UniBind/PointBind 编码器。NIR 复用 `vision`，并非专门训练的 NIR 分支。
- 四路全局向量的拼接适合记录联合表示；它不包含能直接用于像素分割的空间金字塔和已训练的任务解码器。
- `src/rtabmap_semantic/rtabmap_semantic/object_fusion.py` 当前以 RGB 目标区域为基础，复核 RGB/NIR 的类别，输出权重 `[RGB, NIR, 0, 0]`；保留 RGB 掩膜。它不能据此补回漏检、改进掩膜边界或证明 LiDAR／音频参与了这条分割任务。
- 当前 18 类文本原型是本地任务配置，不能替代下面项目的原始任务分类头和类别映射。

## 候选总表

| 项目 | 实际任务输入／输出 | 完整任务权重 | 对当前项目的价值 | 必须保留的限制 |
|---|---|---|---|---|
| [BEVFusion](https://github.com/mit-han-lab/bevfusion) | 多相机 RGB＋LiDAR → 3D 检测／BEV 地图分割 | 有融合、相机单模态、LiDAR 单模态权重 | **最完整的免训练融合精度对比与 BEV 特征拼接实验** | 发布配置面向 nuScenes 多相机、车载点云；先用原始数据复现，不能当作单 D435＋Livox 即插即用模型 |
| [PMF](https://github.com/ICEORY/PMF) | RGB＋投影 LiDAR 特征 → 点云语义分割，同时有视觉分支预测 | 有 SemanticKITTI-PMF-ResNet34；另有 nuScenes-EPMF-ResNet34 | **单相机＋LiDAR 更接近当前设备；能研究多尺度特征拼接与残差注意力** | 作者权重面向室外；需标定、视野筛选、5 通道点云特征和原预处理；Livox 采样模式与 HDL64 不同 |
| [GF-Net / MM5](https://github.com/martinbrennernz/MM5-Dataset/tree/main/GF-Net) | RGB＋DIN 等图像流 → 像素语义分割 | 已在公开目录看到 RGB3-DIN 完整 `.pth`，71.5 MB | **真正覆盖 NIR 的多分支任务模型；研究分层编码、特征校正、拼接与门控** | DIN 是配置中的 3 通道打包图像，须复现逐通道编码；公开配置为 32 类水果、器物等；单模态完整任务权重尚未核实 |
| [AVSegFormer](https://github.com/vvvb-github/AVSegFormer) | RGB 视频＋音频 → 发声区域分割；AVSS 为带语义分割 | S4、MS3、AVSS 均有完整权重 | **音频／视觉通道匹配、多尺度 token 拼接、音频查询与跨模态注意力** | S4/MS3 是发声前景分割，不能直接当作通用语义分割；使用 AVSS 权重才能做其语义任务；不能定位画面外或墙后的物体 |
| [AVISM](https://github.com/ruohaoguo/avis) | RGB 视频＋音频特征 → 发声实例类别、掩膜与跟踪 | R50、Swin-L 的 IN／COCO 版本均发布完整权重 | **需要“哪个物体在发声”及稳定实例身份时优先** | 26 个任务类别；官方 demo 使用帧序列及预提取音频特征；音视频同步与音频预处理须适配 |
| [Pixel-aligned RGB-NIR](https://github.com/divisonofficer/Pixel_aligned_RGB_NIR_Stereo) | 已配准 RGB＋NIR → 融合三通道图，再接冻结 RGB 检测／分割器 | 有图像融合权重；下游模型使用各自完整任务权重 | **NIR 输入明确，作为免训练下游对照的补充路线** | 外部任务入口是图像级融合；其立体特征融合版需要左右 RGB＋左右 NIR，不适配当前单 RGB 相机；不能替代特征级任务网络实验 |

“有权重”表示作者公开列出了完整任务 checkpoint；除 GF-Net 目录文件名／大小外，本次未下载和严格加载这些权重，不将其标为已在本机部署通过。

## 1. BEVFusion：最适合先验证“融合不低于最佳单模态”

官方 [README 模型表](https://github.com/mit-han-lab/bevfusion#3d-object-detection-on-nuscenes-validation) 同时给出以下三组配置与完整权重：

| nuScenes 验证任务 | RGB 单模态 | LiDAR 单模态 | 融合 | 相对最佳单模态差值 |
|---|---:|---:|---:|---:|
| 3D 检测 mAP | 35.56 | 64.68 | 68.52 | +3.84 个百分点 |
| BEV 地图分割 mIoU | 57.09 | 48.56 | 62.95 | +5.86 个百分点 |

这是作者公开基准结果，不是本项目的实测。BEV 地图分割也不同于图像语义分割或逐点物体分割。

完整权重：

- [融合检测 bevfusion-det.pth](https://www.dropbox.com/scl/fi/ulaz9z4wdwtypjhx7xdi3/bevfusion-det.pth?rlkey=ovusfi2rchjub5oafogou255v&dl=1)
- [相机检测 camera-only-det.pth](https://www.dropbox.com/scl/fi/pxfaz1nc07qa2twlatzkz/camera-only-det.pth?rlkey=f5do81fawie0ssbg9uhrm6p30&dl=1)
- [LiDAR 检测 lidar-only-det.pth](https://www.dropbox.com/scl/fi/b1zvgrg9ucmv0wtx6pari/lidar-only-det.pth?rlkey=fw73bmdh57jxtudw6osloywah&dl=1)
- [融合 BEV 分割 bevfusion-seg.pth](https://www.dropbox.com/scl/fi/8lgd1hkod2a15mwry0fvd/bevfusion-seg.pth?rlkey=2tmgw7mcrlwy9qoqeui63tay9&dl=1)
- [相机 BEV 分割 camera-only-seg.pth](https://www.dropbox.com/scl/fi/cwpcu80n0shmwraegi6z4/camera-only-seg.pth?rlkey=l60kdaz19fq3gwocsjk09e60z&dl=1)
- [LiDAR BEV 分割 lidar-only-seg.pth](https://www.dropbox.com/scl/fi/mi3w6uxvytdre9i42r9k7/lidar-only-seg.pth?rlkey=rve7hx80u3en1gfoi7tjucl72&dl=1)

具体研究结构：

- 相机特征通过 view transform／BEV pooling 变换到鸟瞰网格；LiDAR 经体素与编码器形成 BEV 特征。这是在任务坐标系里匹配空间尺度。
- [`mmdet3d/models/fusers/conv.py::ConvFuser`](https://github.com/mit-han-lab/bevfusion/blob/main/mmdet3d/models/fusers/conv.py) 明确执行 `torch.cat(inputs, dim=1)`，再用 `Conv2d(sum(in_channels), out_channels, 3)`、BatchNorm 和 ReLU。
- 两路 `[B,C_i,H_bev,W_bev]` 沿通道拼接为 `[B,C_camera+C_lidar,H_bev,W_bev]`，融合卷积输出任务头所需通道。融合卷积和任务头都已有训练权重。

官方 `tools/test.py` 支持加载任务配置与 checkpoint 做 `--eval bbox` 或 `--eval map`；无需执行 `tools/train.py`。原依赖较旧且含 CUDA 算子，建议独立环境；先做主机离线复现，不在当前运行环境直接覆盖依赖。

## 2. PMF：更适合单相机＋LiDAR 的任务路线

- [完整 SemanticKITTI-PMF-ResNet34 权重](https://drive.google.com/file/d/1-dgvl1mlweasZNCQXw6WPR6OF64hTIuU/view?usp=drive_link)
- [完整 nuScenes-EPMF-ResNet34 权重](https://drive.google.com/file/d/19hgcF91q3rEXyURGev0zLJ50c19b_WSF/view?usp=sharing)
- [SemanticKITTI 推理入口](https://github.com/ICEORY/PMF/blob/master/tasks/pmf_eval_semantickitti/infer.py)
- [匹配推理配置](https://github.com/ICEORY/PMF/blob/master/tasks/pmf_eval_semantickitti/config_server_kitti.yaml)

这里优先讨论原 PMF 的 SemanticKITTI 模型：一张前视 RGB 图＋相机视野内 LiDAR 点，可以直接使用作者已经训练好的完整模型。作者报告验证集 mIoU 为 63.9，但本次未复跑。

[`pc_processor/models/pmf_net.py`](https://github.com/ICEORY/PMF/blob/master/pc_processor/models/pmf_net.py) 的可检查结构：

- `PMFNet.camera_stream_encoder`：ResNet34，四级输出通道为 `[64,128,256,512]`。
- `PMFNet.lidar_stream`：`SalsaNextFusion`，点云先变换到透视图像空间，默认 `pcd_channels=5`。
- `ResidualBasedFusionBlock`：`cat(pcd_feature,img_feature,dim=1)` → 3×3 卷积压回 `C_lidar` → 注意力与残差连接。
- `fusionblock_1...4`：在对应的四个特征尺度融合，而不是拼接两个整场向量。
- `PMFNet.forward` 同时输出 `lidar_pred` 与 `camera_pred`，可以检查视觉分支预测及融合点云预测。

接入 D435＋Livox 时，需要相机／雷达外参，按相机视野投影点云，复现通道含义、反射强度范围、归一化、尺寸与有效像素 mask。不能直接把 `/livox/lidar` 的 XYZ 塞进五通道网络；也不能只 resize 图像就认为完成投影。

**基线限制：** 官方 README 未列出一套与该 PMF 权重对应的全部单模态任务权重；融合模型视觉分支可做有明确名称的分支对照，但不是独立训练的视觉基线。若要严格证明融合 ≥ 最佳独立单模态，仍须补齐匹配的基线模型及统一任务评价，不能把 `camera_pred` 在任意像素集上的精度直接和点云 mIoU 比较。

## 3. GF-Net：含近红外的原生分割模型

官方 [权重目录](https://drive.google.com/drive/folders/1Ea-OFVtBz5uzMSFG7sYCOjvq9i5uhYqc) 当前确有：

`GFNET-I-ADD-CEDICE-FRM-FFM-RGB3-DIN_mit_b0.pth`，页面显示 71.5 MB。

该版本的名称不含 T24／U8，对应 [RGB3-DIN 配置](https://github.com/martinbrennernz/MM5-Dataset/blob/main/GF-Net/configs/gfnet-b0-RGB3-DIN.py) 只启用 RGB 与一个 DIN 图像流；不需要额外热红外或紫外设备。MM5 明确包含 Near-Infrared，不能把这里的 I 当作 LWIR 热图。

重要部署条件：

- DIN 被配置为 **3 通道 PNG**。本次未找到完整公开的 DIN 逐通道生成脚本，因此不能写成“把 NIR 灰度复制三通道就是 DIN”。需要先用作者 MM5 原始 DIN 样本确认编码，再决定如何从本机深度／NIR 构造输入。
- 当前配置为 32 个类别，包含柠檬、橘子、碗、壶、葡萄、苹果、梨、胡萝卜、镜面和背景等。类别头不是通用人／车／猫／狗模型。
- 权重文件名带 `-I-`，当前配置里的 `use_intensity_enhancement` 为 False。部署时要核对对应代码版本、实际参数键与开关；不能用 `strict=False` 吞掉融合层不匹配后宣称模型可用。
- 本次公开目录只看到五种融合权重，没有确认同任务 RGB-only 与 NIR-only 完整权重。GF-Net 可列为下游融合候选，但其单模态验收尚有缺口。

具体结构与研究位置：

- [`GF-Net/models/builder.py`](https://github.com/martinbrennernz/MM5-Dataset/blob/main/GF-Net/models/builder.py)：`mit_b0` 对应四尺度通道 `[32,64,160,256]`，按当前配置选择 `dual_segformer`。备用 `dual_segformer_genSGate` 不能一概当作当前配置所用编码器。
- [`models/encoders/dual_segformer.py`](https://github.com/martinbrennernz/MM5-Dataset/blob/main/GF-Net/models/encoders/dual_segformer.py)：RGB／附加模态分别 `OverlapPatchEmbed`，形成同尺度特征；按配置选择特征校正与融合路径。
- [`models/cmm5_fusion.py`](https://github.com/martinbrennernz/MM5-Dataset/blob/main/GF-Net/models/cmm5_fusion.py)：`StageWiseRGBIntensityFusion` 将两路 `[B,C,H,W]` 拼为 `[B,2C,H,W]`，卷积还原 C 通道；`SigmoidGateModule` 根据拼接特征生成逐像素权重，`transform_conv` 匹配输出通道。具体哪条路径有效，须以匹配权重和开关为准。

建议先用带像素标注的 [MM5 数据](https://doi.org/10.6084/m9.figshare.28722164) 复现完整分割，再尝试实机。附加深度模态不能记为已经覆盖 LiDAR。

## 4. AVSegFormer：音视频语义／发声区域分割

官方 [README](https://github.com/vvvb-github/AVSegFormer) 给出三种任务的配置和完整模型：

| 任务／推荐权重 | 输出性质 | 作者报告 mIoU | 下载 |
|---|---|---:|---|
| PVTv2-B5 S4 | 单声源前景 mask | 83.06 | [S4 完整权重](https://drive.google.com/file/d/1ZJ55jxoHP1ur-hLBkGcha8sjptE_shfw/view?usp=drive_link) |
| PVTv2-B5 MS3 | 多声源前景 mask | 61.33 | [MS3 完整权重](https://drive.google.com/file/d/1iKTxWtehAgCkNVty-4H1zVyAOaNxipHv/view?usp=drive_link) |
| PVTv2-B5 AVSS | 带类别的音视频语义分割 | 37.31 | [AVSS 完整权重](https://drive.google.com/file/d/1P8a2dJSUoW0EqFyxyP8B1-Rnscxnh0YY/view?usp=drive_link) |

若用户希望下游语义分割，应优先 AVSS；不能把 S4 的 83.06 与 AVSS 的 37.31 直接比较，因为任务和类别不同。

具体研究结构：

- [`model/AVSegFormer.py`](https://github.com/vvvb-github/AVSegFormer/blob/master/model/AVSegFormer.py)：VGGish 音频 128 维，`audio_proj=Linear(128,256)`。
- [`model/head/AVSegHead.py`](https://github.com/vvvb-github/AVSegFormer/blob/master/model/head/AVSegHead.py)：各视觉尺度用 1×1 Conv＋GroupNorm 匹配到 256 通道；将多尺度特征 flatten，按 token 维 `cat`，保留 `spatial_shapes`、层级嵌入与位置编码。
- 音频生成 query，与视觉特征通过多尺度 Transformer 交互；解码时重新恢复空间尺度并插值融合。
- [`model/utils/fusion_block.py::CrossModalMixer`](https://github.com/vvvb-github/AVSegFormer/blob/master/model/utils/fusion_block.py)：音频／视觉 Q、K/V 交互，进一步调制视觉特征。

M260C 当前单声道 PCM 可以用于此类非空间阵列音频编码；仍需要匹配采样率、窗口、时间片和 log-Mel/VGGish 预处理。YAMNet 的 521 类分数不是 VGGish 128 维特征。

官方完整权重可通过 `test.sh avss CONFIG CHECKPOINT` 进行免训练测试。旧版 PyTorch／MMCV 及 MSDeformAttention 需独立环境适配。本次没有核实与其全部单模态消融对应的完整权重；声音单模态分类准确率与画面 mask mIoU 也不是同一任务。

## 5. AVISM：发声实例分割与跟踪

官方 [README](https://github.com/ruohaoguo/avis) 发布四组完整 checkpoint。建议先用 ResNet50 降低环境和资源适配难度：

- [AVISM_R50_IN.pth](https://1drv.ms/u/c/3c9af704fb61931d/EYyAuCNpRjxDqEohJfoDLO0BYgw0lbwKqQ1lwVXe_kIPVQ?e=PeRlyx)：作者报告 mAP 40.57。
- [AVISM_R50_COCO.pth](https://1drv.ms/u/c/3c9af704fb61931d/EX0snZsxQwdBswQFdG4sc9kBd-Bd7lw5zaTGR6FvrSxinQ?e=bdZF5G)：作者报告 mAP 45.04。

[`avism/modeling/transformer_decoder/avism_transformer_decoder.py`](https://github.com/ruohaoguo/avis/blob/main/avism/modeling/transformer_decoder/avism_transformer_decoder.py) 明确包含：

- 视觉 `input_proj`：将各尺度视觉特征用 1×1 Conv 映射为统一 hidden 通道。
- `av_pre_proj=Linear(128,hidden_dim)`：音频／视觉通道匹配。
- 三个尺度的 `av_sf` 跨注意力。
- `torch.cat((av_feats[0],av_feats[1],av_feats[2]),dim=-1)` 与 `av_post_proj=Linear(3*hidden_dim,hidden_dim)`：可直接观察三尺度交互后的拼接及压缩。

它比普通音视频前景分割更接近“识别哪个物体正在发声，并保持实例标签”。但它依赖物体在视频里可见，不会单凭听到声音在墙后生成可靠地图坐标。官方 demo 从图像序列与预提取音频 `.npy` 读取输入；需先回放同步视频／PCM，按官方方法生成音频特征。

## NIR 补充：Pixel＋冻结下游模型

作者明确提出 RGB/NIR 图像融合允许已有 RGB 任务模型在无需额外微调时使用 NIR 信息。融合网络是已经训练好的模型，区别于本项目当前 ROI 向量加权。

- [`net/image_fusion.py`](https://github.com/divisonofficer/Pixel_aligned_RGB_NIR_Stereo/blob/main/net/image_fusion.py)：共享 `BasicEncoder(output_dim=256)`、`BAttentionFeatureFusion`、亮度融合与三通道图像重建。
- [权重目录](https://github.com/divisonofficer/Pixel_aligned_RGB_NIR_Stereo/tree/main/weights)：图像版权重需与 `HSVNet` 匹配。
- [图像融合示例](https://github.com/divisonofficer/Pixel_aligned_RGB_NIR_Stereo/blob/main/visualize/image_fusion_viewer.ipynb)。

可对同一配准场景用相同冻结分割器比较 RGB、NIR 适配输入和融合图；统一类别与真值。NIR 灰度复制三通道只可明确称为 RGB 预训练模型的 NIR 输入基线，不等于专门训练的最佳 NIR 模型。如果没有强 NIR 单模态模型，该实验只能声明相对已测试基线的结果。

## 精度验收与特征研究怎么安排

每个任务独立保存原始输入、时间戳、标定、类别表、checkpoint 哈希、层名与特征形状。使用 forward hook 导出模型**已有**融合层前后的张量；不新增未训练的投影或注意力层。

特征研究至少分清：

1. **几何／时间对应：** 配准、投影、BEV 网格、音视频片段匹配。相同尺寸或相近网络到达时间不能证明这一项。
2. **尺度匹配：** 1×1 Conv／Linear 匹配通道；池化／插值匹配 H/W；token 拼接保留尺度元数据；归一化匹配数值范围。
3. **任务语义交互：** 已训练的融合卷积、跨注意力、门控与解码器建立任务相关联系。它不同于 UniBind 的公共对比学习语义空间；`cat` 本身也不证明语义对齐。

验收在**同一任务、同一类别映射、同一留出数据和同一评价区域**上计算：

`Δ = score(fusion) - max(score(single_modal_1), score(single_modal_2), ...)`。

- 语义分割使用 mIoU 与逐类 IoU；实例分割／检测使用匹配的 mask AP／box AP，必要时报告跟踪指标。
- 先用作者公开带真值的数据验证模型和评价流程，再用实机人工标注留出场景检验迁移效果；无需训练，但需要真值。
- 参数选择只在验证集进行；测试集用于最终比较。按场景／视频分组，避免相邻帧泄漏。
- 缺失模态另设实验组和可用单模态集合；不要用测试集真值逐帧选最优分支，也不要把补零后运行称为已验证的缺模态能力。
- 除点估计 Δ≥0 外，若要统计性证明不劣，可报告按场景重采样的单侧置信下界。未通过就报告未达到目标，不能用分数更高、向量更长或观感更清晰替代准确率证据。

**当前最明确可免训练验证“融合≥最佳单模态”的是 BEVFusion 的公开基准路线。** GF-Net、PMF、AVSegFormer 和 AVISM 能提供真实下游模型与可研究的融合结构，但它们在当前传感器／类别上达到目标，及全部独立单模态权重的完备性，仍需逐项验证。

## 没有列为当前优先直接方案的项目

- CMX／CFT／多数 RGB-T 模型：不能把 MFNet、LLVIP、FLIR 等热红外完整权重当作 D435 NIR 已适配权重。
- 2DPASS：其图像信息主要用于训练阶段辅助，不能仅凭多模态训练论文就称为推理时持续 RGB＋LiDAR 融合。
- MCubeS：包含 NIR，但完整多模态权重还涉及偏振等输入；缺失输入效果未确认。
- Sound3DVDet、AV-PedAware：本次未核实到适配当前条件的完整任务权重及输入链路。
- EchoSegnet／音视频 3DGS：场景级训练或重建要求与当前免训练优先条件不同。

本清单不改变 3000 网页、建图、传感器或现有语义数据库。新增任务模型建议先在工作站独立环境中离线复现，再选择接入 ROS／MITI 的分支。
