# 室内 RGB＋深度＋LiDAR / AVSegFormer / RGB＋NIR 独立研究页面（3001）

入口：<http://localhost:3001/>，<http://localhost:3001/rtabmap.html> 会进入同一页面。

3001复制现有MITI程序并使用独立前端、缓存版本、状态与进程锁；3000程序目录及其配置不变。3001的ROS桥独立使用8766，读取传感器／地图并发布 `/miti/research/pmf`、`/miti/research/avs`。它复用3000的场景、动态点云、墙面／定位和栅格显示代码，输出独立使用 `/miti/research/display/*`，不启动、重启或控制SLAM／机器人。

2026-11-21增加 `/miti/research/rgbnir` 实验面板，布局共9块，保留原有两张地图、建图状态、RGB、NIR和对齐深度。旧3001进程需重新运行下方启动命令才能使用新的推理模块。

## 启动与停止

在硬件驱动和原建图服务就绪后，另开终端：

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/start_fusion_research.sh
```

保持该终端运行。Ctrl+C停止本次3001的网页、推理、独立数据桥与自行启动的音频采集。另一个终端也可执行：

```bash
cd /home/jyfight/Project/rtabmap_slam
python3 scripts/stop_fusion_research.py
```

已有麦克风采集时复用 `/multimodal/audio`；否则在原设备锁下采集主机M260C。`--audio reuse` 禁止本启动器新增音频采集。重复启动拒绝占用，不会跳到别的网页端口。

两张地图、回环、RGB、NIR和对齐深度保留原话题。原建图命令仍为：

```bash
bash /home/jyfight/Project/rtabmap_slam/scripts/start_semantic_yoloe.sh --gpu
```

3001不自行生成地图；它自动启动四个地图／深度显示适配器和一个独立回环诊断服务，原建图运行后读取 `/cloud_map`、`/mapGraph`、`/icp_odom`、`/rgbd_image` 和 TF，显示两张地图、轨迹、定位及对齐深度。原建图没有数据时这些面板仍等待数据。此前3001遗漏适配器，导致SLAM实际有地图而网页等待；2026-11-20已修复。

如果3001晚于建图启动，场景适配器通过只读 `/rtabmap/get_map_data` 请求恢复优化图与轨迹；不需要移动机器人来等待下一次图更新。

## 第二组：ODIN-Swin 室内 RGB＋深度＋LiDAR 融合

2026-11-21实机恢复后的默认版本为 **ODIN-Swin 室内语义权重**，899个张量严格加载，SHA256 `fc4e7ffd4507ea334ed7d96c5be146b786fd18c4abd42cb39138bd39c71856f5`。当前低机位实机片段中，ResNet版本把大片地面判成桌子／家具；同一保存的RGB-D、LiDAR和位姿回放中，Swin版本主要将该区域判为地面。另一保存场景也输出较合理的地面、墙和家具区域，仍有柜子、书架等误分。模型与PTv3对照GPU约1.8–2.5秒；尚无人工真值，定性改善不能当作准确率提升。模型类别仍为ScanNet20，机器狗不在词表内，当前被归到其他家具。

两份回放对照分别位于 `logs/research/odin/swin/`、`logs/research/odin/swin_other_scene/`，保留完整特征NPZ及同帧旧模型预测。全图分割仅使用真实有效深度，雷达点仍单独标明。3001重新加载Swin版本后，保留音视频及90/10 RGB-NIR实验、两张地图与建图状态。使用 `models/odin_scannet_resnet.pth.download` 可回到下面的初始ResNet版本。

实机数据存在传输和注册深度延迟，当前使用5秒真实历史窗口选择至少4组有对应位姿的RGB-D／LiDAR帧，采集时间配对仍不超过100ms；没有复制帧或放宽同步容差。模型完成后间隔1秒再检查新帧。原回环诊断进程在Qt状态窗启动时以127退出，3001另行启动无Qt窗口的诊断实例，输出独立 `/miti/research/display/loop_status`。不会更改原建图或3000的诊断配置。

2026-11-21在用户确认深度图可用后，初始候选为 **ODIN-ResNet / ScanNet20 的 RGB-D＋LiDAR**，使用作者完整ResNet50任务权重，807个张量严格加载，SHA256 `ecbf1292b60fe2ccbc301be283739eb08313f7c59a7b1ffc172d010b34d49c21`；当前默认已改为上面的Swin版本。两者在骨干和像素解码器中按真实三维邻域交互RGB-D特征，并插值到真实Livox点／有效深度点上解码。页面左侧为有效深度区域分割，右侧可切换原始LiDAR点预测、原PTv3相同LiDAR点对照以及投影。

同一4组真实帧离线回放：1949个有色Livox点、1689个可见雷达投影点，加上当前帧92152个有效注册深度点。主要类别为地面、墙、家具、椅子；已不再出现此前大片浴帘、冰箱预测，仍有窗户等误分。GPU完整模型及PTv3对照约1秒，CPU仅ODIN约4.3秒。这是已有片段的定性观察，没有人工真值，不能宣称mIoU提升或实机持续运行已经验收。

ODIN输入新增深度已由用户授权：使用对应曝光的注册深度及内参、时间戳位姿。沿用作者ScanNet补洞预处理（毫米值、OpenCV Telea半径5），仅作编码几何上下文；额外分割查询与显示只使用原始有效深度，补洞像素不充当实测。真实LiDAR点与深度点在NPZ中分别标记来源，保留原始扫描及位姿；没有用深度点云冒充LiDAR。没有ScanNet网格分段，关闭作者点段聚合；这是适配Livox的推理设置，未复现完整公开基准。

源码固定于 [作者ODIN](https://github.com/ayushjain1144/odin/tree/b8225c8879bb1e563fc85b5ea423ee950af8005a)，权重来自 [作者发布目录](https://huggingface.co/katefgroup/odin/blob/1def72841925a64069dd42eceb9f0fde2ea01a57/scannet_resnet_47.8_73.3_32k_1.5k.pth)。使用作者已有`grid_sample`注意力参考路径；两个pointops邻域／插值算子按原公式用精确欧氏kNN替代CUDA核，已与穷举距离及加权公式核对。神经网络类和所有参数保持作者代码，不新增训练层。依赖单独放在 `.tools/odin-research-deps`。

原PMF和PTv3继续保留：配置 `pmf_backend: ptv3_compare` 可回到下述PTv3候选＋PMF对照，`pmf_backend: pmf` 回到PMF。

2026-11-20曾默认改为**室内 PointTransformerV3 候选＋原 PMF 对照**，不是精度验收通过。选择该后端时下述PMF完整模型继续加载，显示在右侧“PMF对照”；“雷达投影”可切换几何查看。要回到原PMF单帧实验，在配置中设 `pmf_backend: pmf` 并按需启用 `allow_pmf_rgb_preview` 后重启3001。

室内候选使用作者完整 ScanNet20 权重，488个张量严格加载，SHA256 `40206376ff2f83f48e4d1bc27d5c5d96be7c87c5d11eb45fe7be501959040e7f`。输入是投影到真实LiDAR点上的RGB颜色与LiDAR表面法向，XYZ用于点空间位置；它没有独立RGB图像金字塔，不能称为PMF同样的双编码器残差融合。使用作者原生PyTorch注意力路径及2cm单次体素采样，未复现基准测试的全部增强投票。

约3秒内的真实RGB-D/LiDAR帧按扫描时间的ICP位姿变换到同一坐标；重复曝光或扫描不能充当新帧。模型使用原始LiDAR几何，既有注册深度只用于剔除投影深度误差超过20cm的遮挡／不一致点，不用相机深度点云冒充LiDAR。实例框是预测点区域的显示聚合。

外参只在3001内使用自动估计：[fusion_research_estimated_calibration.json](../config/fusion_research_estimated_calibration.json)。同一批954个新帧点的深度残差中位数从约73cm降到14cm，20cm内比例由约22%升至55%。仍有较大尾部误差，文件保留 `auto_estimated`、`geometry_verified: false`；它不是已验收的实测标定，也没有写入SLAM的全局TF或3000配置。原 `fusion_research_calibration.json` 的实测标定仍优先。

候选单帧仅约600个点时仍会明显误分；真实位姿对齐的多帧回放约4900个有色点可输出椅子、柜子、墙等类别，但仍有误分类。ScanNet20没有机器狗、路锥等类别，不能保证识别未知物体。PMF与它的类别空间不同，不能比较类别占比宣称精度增益。阶段评估见 [室内候选评估](fusion_research_indoor_evaluation_20261120.md)。

新增默认“移除RGB对照”：保持相同LiDAR位置、法向、体素索引与ScanNet20类别，把RGB颜色通道置为作者归一化后的0，使用相同串行点注意力随机种子。可切换PMF道路类别对照及雷达投影。此对照是冻结联合模型的输入消融，没有独立训练LiDAR模型；类别差异只说明颜色输入影响输出，不是精度提升。NPZ保留两种点标签以及完整输出logits的体素映射。

- 作者网络：[ICEORY/PMF](https://github.com/ICEORY/PMF)，commit `73dc74e57109c3439965761d137ee1e76cd03503`。
- 完整SemanticKITTI任务权重：654个张量严格加载，SHA256 `024860251e7df936616fc468cc002765010ae64a290b0f481b1ffc2f5963f246`。
- 输入为相机RGB与原始LiDAR的XYZ/intensity，不使用地图颜色点云代替原始测量。
- 使用实测 `camera_from_lidar`、RGB内参与畸变投影点云，同像素保留最近相机深度的点。右／下补齐到16倍数，不改内参；五通道依作者设置为range、x、y、z、intensity，使用作者均值／标准差。Livox强度转换须显式配置。
- 四个尺度真实拼接通道分别为128、256、512、768；输出为残差注意力融合后的64、128、256、256通道。页面显示实际shape、通道均值与空间能量图。
- 实测标定优先使用；没有实测标定时，默认 `pmf_extrinsics_mode: tf_preview`，读取与3000建图相同的当前SLAM相机／LiDAR TF，以真实原始LiDAR执行完整PMF融合。页面明确显示“实验融合、安装外参尚未实测标定”，NPZ记录矩阵、坐标系、内参、畸变及实际强度转换。当前TF包含示例安装位姿，不构成实测标定证据。
- 实机Livox驱动 `src/comm/pub_handler.cpp` 将 reflectivity 原样赋给 intensity，`msg/CustomPoint.msg` 定义为0–255的uint8；实验配置显式乘以1/255。该数值规范化不证明与训练雷达的光学响应一致。
- 若需仅允许实测外参，将 `pmf_extrinsics_mode` 设为 `measured_only`。缺失TF、投影不足、输入过期或无法同步时，回退冻结PMF的RGB分支，页面显示具体原因。视觉分支是联合训练模型的分支，不等同于独立训练的单模态基线。

标定文件：[fusion_research_calibration.json](../config/fusion_research_calibration.json)。填入实测刚体矩阵、准确frame_id、强度转换比例、标定记录，设 `status: measured`。服务每次检查该文件；满足同步和输入条件后自动改用实测标定。当前项目示例安装位姿或自动估计不会被当作标定通过。配对选择时间容差内完整RGB／LiDAR帧组，不因传输先后放宽100ms容差。

SemanticKITTI是道路场景19类＋忽略类；这些权重在室内的任务效果尚未评估。

## 第三组：AVSegFormer

- 作者网络：[vvvb-github/AVSegFormer](https://github.com/vvvb-github/AVSegFormer)，commit `7083cb6586d1bc06d174a34be3c34bb5814cf0c3`。
- R50 AVSS完整权重：853个张量严格加载，包含视觉、音频编码器与融合头；SHA256 `a32cf9f47b43c24e96da7c221640a53f97f435c0f29e54ae2581f85a7600c87f`。
- 拼接重叠PCM的原始采集时间轴，必须覆盖连续10秒；缺口和不足帧等待，不重复2秒音频或图像凑10帧。
- 作者VGGish预处理输出 `[10,1,96,64]`，每段0.96秒；RGB按对应log-Mel段中心匹配10次真实曝光，缩放224×224并用ImageNet均值／标准差。中心采样是实机适配，与作者数据集名义1fps采样方式有所区别。
- 音频特征128维经Linear到256维；视觉四尺度均投影到256通道，后三尺度展平后拼接成1029个token。音频条件查询为300×256，Mixer输出256×56×56；最终输出 `[10,71,224,224]`。
- 本机使用作者提供的PyTorch `grid_sample` deformable attention参考实现，保持数学运算和任务权重。没有编译原生CUDA扩展；参考实现仍在GPU上运行。
- 71类名称与颜色索引根据公开 [OV-AVSS类别定义](https://github.com/ruohaoguo/ovavss/blob/main/ov_avss/data/datasets/avsbench.py)整理，并与 [Stepping-Stones AVSS配置](https://github.com/GeWu-Lab/Stepping-Stones/blob/main/avss/.models/config.json)逐项核对，背景索引0。没有使用项目UniBind的18类文本原型。

此任务分割画面中的发声区域，未输出画面外物体位置或墙后声源地图标签。

## 第四组：RGB 90%＋NIR 10% YOLOv8-seg实验

复用已有 `models/yolov8s-seg.pt` 完整COCO80分割权重，冻结全部参数，不新增训练层。RGB与真实mono8 NIR分别通过同一个骨干。P3、P4、P5保留纯RGB骨干输出，只在原neck读取这三个尺度时替换为固定加权特征。NIR先在有效区域进行RMS尺度匹配，再按 `0.9 RGB + 0.1 NIR` 加权；无效区域保持原RGB特征。该权重没有针对NIR训练，COCO80也没有机器狗类，不能保证提高识别精度。

配准复用项目既有RGB-D到NIR的几何代码，使用同一RGB-D帧的注册深度、CameraInfo、工厂相机TF及NIR右目P矩阵，处理遮挡与基线。禁止把不同尺寸图像直接缩放后当成配准。RGB-D/NIR采集时间差不超过50ms，配准覆盖不足35%、某尺度没有有效区域、缺少标定或输入过期时等待。页面提供同帧原RGB基线、配准NIR、中文类别与实例区域标注，以及12个真实特征张量的展示和NPZ下载。

## 离线检查

联网恢复前可用已保存真实输入检查模型与页面，无需ROS：

```bash
cd /home/jyfight/Project/rtabmap_slam
.venv-indoor-research/bin/python3 scripts/check_fusion_research_offline.py --device cuda:0
.tools/miti/root/usr/lib/miti/node/bin/node scripts/check_research_offline_ui.cjs
```

PTv3稀疏卷积需要GPU；只有CPU时可执行 `--device cpu --tasks rgbnir avs`。结果写入 `logs/research/offline/index.html`、`report.json`、三组图片及NPZ；不替换实时页面的推理结果。UI检查使用同一个研究组件，读取离线快照，明确标为回放。它验证组件与交互，不能证明当前ROS、地图传输或九块完整建图页面在实机上正常。

2026-11-21最终Swin默认版本的实测回放：RGB/NIR配准覆盖49.6%，90/10实验与同帧RGB基线均输出6个实例，仍有误分；NIR权重归零时模型输出逐值等于原RGB输出。ODIN使用9组有位姿的真实RGB-D上下文、4920个有色LiDAR点及95381个有效深度查询点，4011个雷达点可见投影；输出以地面、墙、柜子和其他家具为主。音视频标签与此前导出结果约99.988%一致，调整真实log-Mel输入会改变融合特征。三组回放检查通过，详细结果见 `logs/research/offline/report.json`；这些是可复现性／敏感性和定性检查，仍无人工真值。

同日实机只读采样确认：清理地图10080点、原始地图10109点；二维栅格为243×266，含228个空闲格和1032个占用格；定位有效，回环诊断处于积累历史关键帧阶段。该次采样保存在 `logs/research/live/map_status.json`，数值随后续建图更新。

## 时间、特征导出与精度

两种融合的“分割结果”默认开启画面标注：中文预测类别、区域编号、轮廓、框及支持像素／投影点数量。可用“标注：开／关”对照原着色结果。PMF框与轮廓聚合有效雷达投影点，不表示框内每个像素都有雷达测量；AVSegFormer没有可标注前景时明确显示“本段未检出可标注的发声区域”。这些是模型预测，不能作为人工真值。当前道路任务PMF权重在室内会误分为车辆等道路类别；增加标注仅改善可读性，不提高模型精度。

启动时通过SSH只读机器狗时钟，记录偏移与往返不确定度到 `logs/research/clock.json`。SSH不可用时，复用项目现有RGB／NIR／LiDAR最小接收延迟估计；此回退包含未知采集和网络延迟，没有SSH往返不确定度。校正只作用于3001内存中的配对时间，不修改原消息、系统时钟和3000配置。该估计及主机读音频结束时间不能证明硬件同步；页面详情与NPZ保留时间来源。

“下载特征NPZ”包含实际模型张量、输入、输出标签、原始结果时间、模型hash、序号和配对信息。PMF保存本帧四级特征；AVSS保存末帧中间特征，同时保存全部10帧输入与标签。文件始终保留最新一次结果，原子替换，文件内metadata用于复核来源。

室内候选NPZ另保存每个真实RGB-D帧、原始XYZ/intensity扫描、扫描位姿、输入有色点／法向、来源索引、原生分类头logits、点标签及当前投影标签。类ID原生为0–19；显示标签加1，0只表示没有投影点。

页面类别百分比为有效预测像素／点的占比，**不是准确率或置信度**。当前没有人工真值及独立RGB、LiDAR、音频任务基线，不能宣称融合精度不低于最佳单模态。后续需在相同类别、同一采集片段和同一有效区域上标注，分别评价融合与独立基线的mIoU，再比较结果。

当前验证：完整权重与冻结状态、PMF真实拼接张量、RGB分支与完整网络的RGB输出一致、音频变化影响AVSS真实融合张量、投影遮挡处理／补齐／强度通道、PCM重叠与缺口、曝光配对。`logs/research/model_smoke.json` 明确标为合成输入的结构检查，不能当作融合精度实验。

日志目录 `logs/research/`：`inference.log`、`bridge.log`、`audio.log`、`web.log`，以及显示适配器 `scene.log`、`cloud.log`、`geometry.log`、`grid.log` 和独立诊断 `loops.log`、`loops_events.jsonl`。配置位于 `config/fusion_research.json`，实际启动配置与测量记录写入 `logs/research/runtime.json`。
