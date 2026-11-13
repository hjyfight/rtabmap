**任务书、两份验证大纲与当前项目的覆盖检查（2026-11-13）**

当前项目还没有完整覆盖任务书，也不能认定两份验证大纲已经全部通过。已经具备的是多传感器数据接入、RGB-D 网络特征提取与拼接、LiDAR/IMU/视觉 SLAM、物体语义关联、墙面提取、地面栅格、稀疏视觉特征、导航连通图及部分动态点云清理。主要缺口是声学处理链、近红外参与融合模型、完整地图形式的验证、跨模态接口与异常状态表达，以及定量精度、实时性和野外复杂地形试验。

本次按用户确认只做检查，没有修改任务书、验证大纲、项目实现或运行服务。复跑了 149 项已有离线测试，全部通过且无跳过；这些测试验证代码行为，不能替代规定场景的精度、成功率或适应性试验。检查包含当前源码、配置、已有实机/回放记录和本日上一轮地图传输检查，没有新增机器人运动试验。

用户确认已有一个 Wheeltec 麦克风阵列，口令样本尚未准备，真值地图数据没有。麦克风硬件可用性列为用户提供的信息；型号、驱动、音频接口、阵列几何标定及声源定位输出尚未在本项目中核实。不能将“项目未接入声学”写成“没有麦克风硬件”。

**检查依据与口径**

主依据是用户指定目录中的 [任务书标准要求.docx](/home/jyfight/项目标准文件/最新/融合和建图标准-最新/任务书标准要求.docx)、[融合验证大纲 v5](</home/jyfight/项目标准文件/最新/融合和建图标准-最新/机器人具身智能多源异构感知融合要求_标准验证 - v5.docx>)、[建模验证大纲 v2](/home/jyfight/项目标准文件/最新/融合和建图标准-最新/机器人具身智能环境感知与建模要求_标准验证_v2.docx)。为核对大纲引用的“标准草案”，补充读取同目录的 [融合草案 V12](</home/jyfight/项目标准文件/最新/融合和建图标准-最新/1.1 多源异构融合标准大纲-V12.docx>) 和 [建模草案 v12](/home/jyfight/项目标准文件/最新/融合和建图标准-最新/环境感知与建模技术要求_v12.docx)。

任务书包含 A1—A12、B1—B15 共 27 个测试项，并含两组流程说明。下表同时核对功能和证据：“具备”只表示有实现；“部分”表示该整项中仍有子要求未覆盖；“未见实现”表示本项目检查范围内没有找到相关链路。未准备真值、未进行规定统计或未固定限值时，不写成“已通过”。历史测试只适用于其当时版本和条件，不推定当前各项性能与历史值相同。

**任务书 A 组：多源异构感知融合**

| 项目 | 任务书要求 | 当前实现与证据 | 未完成部分 / 检查结论 |
| --- | --- | --- | --- |
| A1 | 可见光、近红外、LiDAR、声学四类传感器及数据类型、编码 | RGB、IR1/IR2、深度和 LiDAR 已有接入及传输记录；IR2 为 mono8。Wheeltec 阵列硬件由用户确认。E1、E2。 | 部分。未见 Wheeltec 音频进入本项目的驱动、话题和编码记录；RGB-D 和 IMU 不能代替声学这一类。 |
| A2 | 口令识别正确率、声音分类正确率、声源定位角度误差 | 本项目未见声学 encoder、口令识别、声音分类、声源定位处理链。 | 明确缺项。没有口令样本，尚未找到声音类别标注、标准声源和角度真值记录。硬件已有不等于这三项功能已经完成。 |
| A3 | 本体位置、姿态的量纲、频率、精度 | /icp_odom、IMU、/robot_pose 和位姿状态输出；单位与四元数/RPY 有实现。E3。 | 部分。自动模式可能输出实际跟踪的雷达坐标系而非 base_link；需按 body_frame 标记核对。本体安装外参、频率验收与绝对精度没有完整证据。 |
| A4 | 内外参标定流程与标定误差 | 相机内参、RGB/深度内部配准及相机—节点/雷达变换入口存在；非法标定可拒绝。E1、E4。 | 部分。厂商内参和可配置外参不能代替相机—LiDAR—机体—声阵列的完整实测标定及误差报告。 |
| A5 | 同步方式、各模态时间对齐精度、同步误差上限 | RGB-D 配对、时间戳检查、点时间单位转换和同步诊断存在。E1、E5。 | 部分，指标未证实。融合草案为 50 ms，建模草案为 10 ms；当前 RGB-D 默认配对容差 50 ms 不是实测同步误差，也不覆盖声学或所有传感器采集时钟。 |
| A6 | 去噪、去畸变、坐标统一、采样率归一 | 点云降采样/离群点处理、IMU 辅助旋转去畸变、深度有效性处理、坐标投影和时间窗口匹配。E1、E4、E6。 | 部分流程具备。缺四类模态逐步骤的完整预处理记录，尤其声学预处理和异步采样归一链路。 |
| A7 | 特征提取流程及维度/格式，定位模型中的模块 | EMSANet RGB/Depth 双骨干及五级特征提取有代码、分层计时和真实张量记录；RTAB-Map 保存三维视觉特征。E7、E8。 | RGB-D 子链路具备。NIR、声学的特征提取未接入；不能认定四类任务模态均覆盖。 |
| A8 | 跨模态语义对齐、特征尺度匹配和模型拼接结构 | 已实现 SE → torch.cat → 固定投影，五级 RGB/Depth 同尺度特征联合；输出用于实例分割和三维关联。E7、E8。 | RGB-D 尺度匹配与拼接有实证；NIR/声学的联合特征与语义关联没有实现或验证。固定投影等价于原相加，不证明新增精度增益。 |
| A9 | 融合检测/识别/定位精度及相对单模态提升 | 有两种 SLAM 配置的地图结构对比、相加与固定拼接的逐元素等价对比。E8、E9。 | 缺验收证据。没有同场景、同样本、同真值的最佳单模态基线；相加 vs 拼接是两种融合实现，不是单模态 vs 融合。 |
| A10 | 融合接口形式、格式、速率、时序约束 | ROS 2 标准消息、自定义 Keyframe/Observation/SemanticObject/SemanticMap 和状态/计时输出。E10。 | 部分。没有完整的四模态原始特征、联合特征、融合结果字段映射；健康、来源链、冲突状态、不确定性等字段未完整贯通。 |
| A11 | 丢帧/丢包率与端到端时延的长时统计 | 有传输完整性测试、在线流诊断及语义桥接排队到结果发布的计时。E5、E6、E11。 | 部分。语义计时不含采集、网络和先前 SLAM；接收空档不能直接当丢包率。缺统一采集身份/时基下的全链路长时统计。 |
| A12 | 方法与步骤总体确认、工作量可控 | 已有多份运行/验证说明和两份用户提供的大纲。 | 尚无完整 27 项覆盖追踪和正式总体确认记录。本次完成对照检查，不等同于专家评审通过。 |

**任务书 B 组：环境感知与建模**

| 项目 | 任务书要求 | 当前实现与证据 | 未完成部分 / 检查结论 |
| --- | --- | --- | --- |
| B1 | 相机、LiDAR、RGB-D 与图像/点云/深度完整性 | RealSense RGB/深度、LiDAR 接入；RGB-D 配准、关键帧保存和真实链路记录存在。E1、E2、E4。 | 基础功能具备，有链路验证。正式被试品齐套记录及同一固定版本连续完整性记录尚需归档。 |
| B2 | RGB-D 同时用于 SLAM | 主启动链 subscribe_rgbd 与 subscribe_scan_cloud 同时开启；RGB-D 参与视觉特征/回环，LiDAR 提供几何。另有深度 ICP 入口。E1、E12。 | 功能具备。主配置几何地图主要来自 LiDAR，不能表述为“所有深度点都拼入 LiDAR 地图”；不同融合路径应按实际配置检查。 |
| B3 | 位姿、速度、IMU/里程计完整性与精度 | Odometry 含 pose/twist/covariance，IMU 与地图位姿输出存在。E1、E3。 | 基础接口具备；机体参考系标定和这些参量的实测精度、完整频率统计尚未完成。 |
| B4 | 环境感知数据要素与规格 | 图像、点云、深度、CameraInfo、物体语义、墙面、栅格与导航图各有数据结构。E4、E10、E13—E15。 | 部分。缺统一的环境要素清单/字段映射；野外地形、水坑、功能区域等尚未形成完整输出。 |
| B5 | 噪声过滤效果、时间对齐精度 | 过滤、配准、无效输入拒绝及单元测试存在。E4—E6。 | 部分。缺真实噪声前后对照指标及满足建模 10 ms 的跨传感器采集同步测量。 |
| B6 | 墙壁/障碍物/通道正确率、漏检误检率 | 墙面拟合、地面/障碍投影、区域/通道/连接口几何提取已实现；通道有合成验证。E13、E14。 | 几何候选能力具备，真实识别指标未测。候选门口没有语义确认；本日实机导航图为 1 区域、0 边，不能据此证明现场通道识别正确率。 |
| B7 | 分割、聚类正确性与高层结构理解 | EMSANet 实例分割、掩膜深度前景聚类、墙面聚类和自由空间几何分区。E4、E7、E13、E14。 | 部分。EMSANet stuff 类别没有进入完整环境语义地图；区域编号不等于房间功能或地形/通行语义。缺真实分割、聚类和结构理解评价。 |
| B8 | 特征—地图元素关联准确性、图优化收敛与残差 | 语义观测绑定关键帧节点，回环后重投影；视觉特征按优化图重新生成。E4、E8、E15。 | 部分。没有完整输出每个视觉特征与墙/通道等稳定地图元素的对应及关联正确率/残差报告；图优化过程存在不等于收敛性已验收。 |
| B9 | 栅格、稀疏特征、拓扑、三角网格四种地图 | 已有栅格、真实三维视觉特征、关键帧图和导航连通图。墙面为 TRIANGLE_LIST；原生 rtabmap-export 支持 --mesh。E13—E16。 | 前三类已有项目链路。第四类有局部三角墙面和上游导出基础，但未找到完整场景三角网格地图的项目输出/保存/加载及验收记录；该整项仍为部分，不能按“至少一种”免除其余形式。 |
| B10 | 几何精度、地图一致性与闭环误差 | 建图、回环及优化已有运行记录；两配置比较工具存在。E9、E12。 | 缺真值验收。用户确认没有真值地图；检查范围内也未见可替代的经测量控制点/真值轨迹评估。点数、首尾距离、图优化修正量均不能代替绝对精度。 |
| B11 | 绝对/相对轨迹误差与位姿更新频率 | ICP + IMU 辅助 + 图优化输出位姿，具备相关记录与比较工具。E1、E3、E9。 | 位姿功能具备，ATE/RPE/频率验收未完成。当前没有 Kalman/粒子滤波实现；最新建模草案允许图优化等方法，不应仅因未采用两个示例算法就判无定位功能。 |
| B12 | 动态目标处理、帧率和端到端时延 | 动态类位置更新/TTL；射线清理生成 clean_map，并供墙面/栅格使用；计时链路存在。E6、E11、E13。 | 部分。已确认静态语义标签缺完整消失处置；没有完整速度/方向/预测轨迹输出。动态清理展示不能代替所有地图持久更新；全系统实时性未证实。 |
| B13 | 结构化/非结构化同指标对比 | 有室内结构化场景演示和部分合成测试。 | 未找到按同一套指标完成的结构化—非结构化分组对比，不能整项覆盖。 |
| B14 | 野外非结构化地形构建 | 通用三维点云重建具备基础能力。 | 未找到规定野外场景的地形建图、地形语义/通行属性输出及实测记录。室内点云不能代替野外验证。 |
| B15 | 不平地面、水坑等复杂地形适应性 | 地面投影允许一定倾斜，当前 max_ground_tilt 为 20°。E13。 | 参数能力不等于复杂地形适应性。未见水坑、坑洼/落差处理链或相应实测，也未见复杂地形对照结果。 |

任务书的正文要求也已纳入以上条目：传感器与“添加红外参与深度/可见光检测或分割”对应 A1、A7、A8；特征提取—拼接—融合流程对应 A4—A10；本体/环境要素、结构化/非结构化流程和地形对应 A3、B3—B7、B13—B15；分割聚类、SLAM/图优化关联对应 B7、B8；四种地图、位姿与动态实时建模对应 B9—B12。正文里“人影子/点云”的问题已有部分射线清理基础，不能据此将整个动态建模项填为完成。

**两份验证大纲的技术条款**

| 大纲 / 条款 | 已有支撑 | 整项不能直接通过的原因 |
| --- | --- | --- |
| 融合 v5 / 5.1 特征提取 | RGB-D、LiDAR/IMU 接入预处理与特征；标定参数入口 | 5.1.1 a、b 明列四类模态；声学链路、NIR 模型特征和完整质量/健康传递缺项。 |
| 融合 v5 / 5.2 特征拼接 | RGB-D 五级拼接、共同实例输出、跨帧对象 ID | 跨四类模态的关联/重关联、共享语义、模态冲突类型/参与模态/处理结果尚未完整覆盖。 |
| 融合 v5 / 5.3 感知融合 | LiDAR/IMU/视觉 SLAM、RGB-D 网络融合、物体多帧融合 | 模态缺失/质量下降下的完整策略验证、结构持久变化与不确定性输出缺项。 |
| 融合 v5 / 5.4 同步与精度 | 同步检查、阶段/语义链路计时、地图结构比较 | 无全模态同步证据、任务周期基线及同真值最佳单模态对照。 |
| 融合 v5 / 5.5 接口 | ROS 与语义消息可传递数据 | 原始特征—联合特征—融合结果的完整字段/来源/质量/冲突映射和异常结果标记未完成。 |
| 建模 v2 / 5.1 预处理 | 配准、去畸变、过滤及部分数据检查 | 10 ms 全传感器同步未证实；来源、质量、策略及冻结/漂移等异常处置链不完整。 |
| 建模 v2 / 5.2 环境解析 | 物体分割、墙面、地面/障碍、几何通道与回环关联 | 目标速度/方向/预测、完整场景分类与环境语义、地形/通行分析、持久/瞬态变化区分及实测指标缺项。 |
| 建模 v2 / 5.3 地图构建 | 连续位姿、地图更新、四类形式的部分基础、优化和保存 | 地图老化/持久语义变化、完整子图/多时相对齐验收、统一地图版本与质量输出未完成；任务书另要求四种形式。 |
| 建模 v2 / 5.4 性能 | 位姿、回环、地图载入入口和已有短时记录 | 无约 100 m² 真值试验、规定次数的初始/重定位与成功率统计、地图精度及加载计时验收。 |
| 建模 v2 / 5.5 接口 | 通用消息、UUID、保存恢复、导航图 revision | 缺统一地图要素增删改查、版本回退/更新记录及跨环节完整字段、异常/兼容验证；UUID 不等于可回退修订版本。 |

大纲第 1—4 章和第 6 章也进行了文本检查：第 1 章的标准草案研究/形成及标准验证属于标准化交付，软件功能不能单独证明其完成；第 2 章人员资质、设备校准和组织过程没有项目内完整记录，原列日期为 2026-10-08—2026-11-07，日期已过去并不证明测试已完成；第 3 章只有笔记本设备，尚不能支撑标定、声源角度、真值精度等试验；第 4 章四类传感器和场景条件未全部在当前项目验证；第 6 章要求的中止/恢复和异常试验只能部分由输入保护测试支撑，尚无完整现场过程记录。

**文档之间的覆盖差异（仅指出，不修改）**

1. 融合大纲表 5 第 1 项写“至少接入两种模态”，但任务书 A1 与大纲 5.1.1 a、b 明列可见光、红外、LiDAR、声学四类。当前 RGB-D 双输入不能据此免除 NIR 和声学。
2. 建模大纲表 5 第 3 项、5.3.3 写“至少一种地图形式”，任务书 B9 明确要求栅格、稀疏特征、拓扑、三角网格四种。原大纲没有确保这四种都被逐项检查。
3. 任务书 A2 明列口令、声音类别和声源角度三个量化项；融合大纲主要写声学关键要素和通用融合精度，没有对应的独立样本、指标和声源定位步骤。笼统提到声学不等于覆盖 A2 的完整测试。
4. 任务书 B13—B15 要求结构化/非结构化对比、野外地形及不平地面/水坑。大纲列室内外和石子/土路等场景，但没有确保水坑和同指标分组对比的独立证据。楼宇场景包含室外路段不等于这些项已经完成。
5. 任务书 A12 的方法/步骤总体确认，大纲没有独立的覆盖评审与确认记录。
6. 两份大纲额外列出的完整场景分类、动态轨迹预测、模态冲突/不确定性、子图/多时相对齐、地图回退等，当前也有缺项。用户允许以后调整大纲，但本次不据此删减任务书要求或判为已完成。

**需要使用当前文件版本的性能口径**

| 指标 | 本次读取的依据 | 当前证据 |
| --- | --- | --- |
| 融合跨传感器同步 | 最新融合草案：应 ≤50 ms；同时另有参与同步传感器对最短采样周期 1/10 的要求，高速任务宜收紧 | 默认 RGB-D 50 ms 配对上限不是全模态实测值；还需核对采集时钟、采样周期及适用条件。 |
| 建模跨传感器同步 | 最新建模草案及建模大纲：应 ≤10 ms；外部协同时应 ≤1 ms | 未证实。外部授时是条件适用项，是否适用须说明，不能仅因本机演示未使用就删掉。 |
| 全链路处理周期 | 两份草案均要求不大于任务需求周期 | 任务周期未固定；当前分层计时范围并非整个系统。 |
| 语义响应时间 | 建模草案：应 ≤500 ms；单类语义理解延迟宜 ≤300 ms | 历史 CPU 固定拼接现场窗口中，完整模型平均 1251.37 ms，语义节点平均 1375.19 ms；含桥接排队到发布平均 24468.14 ms、最大 29847.88 ms。这批记录不能支持 500 ms 达标。E11。 |
| SLAM / 局部地图更新 | 当前建模草案：本地 SLAM ≥1 Hz、局部地图 ≥1 Hz | 当前源码后端检测配置 1 Hz；实际输出频率仍需统计。11 月 3 日旧评估记载的 5 Hz 不适用于本次指定草案。 |
| 初始定位 / 重捕获 | 建模大纲：无先验 ≤20 s，有有效信息 ≤2 s；草案重捕获 ≤3 s | 功能入口存在，规定条件下的重复计时证据不足。 |
| 成功率 / 加载时间 | 建模大纲：静态定位和一次建图 ≥98%；地图加载 ≤20 s；局部变化 ≤15% 时成功率下降 ≤5% | 缺固定规模、试验次数、成功定义和对应记录。不能用几个成功示例计算 98%。 |
| 轨迹/地图精度 | 建模大纲：ATE 宜 <5 cm，垂直误差宜 <2 cm，角度误差宜 <1° | 用户没有真值地图；未见达到这些口径的控制点/轨迹真值评估。推荐性“宜”和强制性“应”须区别。 |
| 类别/功能属性 | 建模草案：至少 10 种任务相关类别、至少 5 种功能属性标注 | 模型词表数量不等于已验证地图标注能力；当前未见完整功能属性输出和逐项验收。 |

上述毫秒值是 2026-11-10 的历史配置/负载测量，不作为 2026-11-13 全部平台的当前性能值。Orin 的既有 GPU 官方样例报告是另一个条件，不能替代连续真实建图的处理周期与精度验收。

**已核查的证据索引**

| 编号 | 位置 | 能证明的范围 |
| --- | --- | --- |
| E1 | [融合主启动文件](/home/jyfight/Project/rtabmap_slam/launch/livox_realsense_3dslam.launch.py:347)；[RGB-D 默认同步容差](/home/jyfight/Project/rtabmap_slam/launch/livox_realsense_3dslam.launch.py:811) | RGB-D/LiDAR 同时订阅、IMU/ICP/图优化链与配置。配置值不等于实测性能。 |
| E2 | [近红外实机传输记录](/home/jyfight/Project/rtabmap_slam/logs/miti/infrared_stream_check.json)；[近红外模型接入设计](/home/jyfight/Project/rtabmap_slam/docs/multimodal_instance_segmentation_plan.md) | 红外图像已接入；设计文档明确其尚未进入当前分割模型。 |
| E3 | [地图位姿输出](/home/jyfight/Project/rtabmap_slam/scripts/map_geometry_node.py:182)；[位姿接口记录](/home/jyfight/Project/rtabmap_slam/logs/miti/walls_pose_live.json) | 位姿及真实跟踪坐标系标记；没有真值精度结论。 |
| E4 | [深度配准说明](/home/jyfight/Project/rtabmap_slam/docs/depth_registration_fix.md)；[三维投影](/home/jyfight/Project/rtabmap_slam/src/rtabmap_semantic/rtabmap_semantic/geometry.py:26)；[物体关联](/home/jyfight/Project/rtabmap_slam/src/rtabmap_semantic/rtabmap_semantic/fusion.py) | RGB/深度配准、前景聚类、局部锚点/图优化重投影；不证明完整传感器标定和关联准确率。 |
| E5 | [同步诊断工具](/home/jyfight/Project/rtabmap_slam/scripts/check_sensor_sync.py)；[输入保护](/home/jyfight/Project/rtabmap_slam/scripts/guard_rgbd_input.py) | 格式/时间差检测；不能把近同时到达配对结果当采集同步精度。 |
| E6 | [去畸变说明](/home/jyfight/Project/rtabmap_slam/docs/livox_imu_motion_fix.md)；[传输实现](/home/jyfight/Project/rtabmap_slam/scripts/hotspot_sensor_transport.py)；[动态清理实机记录](/home/jyfight/Project/rtabmap_slam/logs/miti/dynamic_cloud_audit_20261110/deployed_check.json) | 时间单位转换、去畸变基础、传输数据保持和射线清理；不代表全系统丢包/实时性验收。 |
| E7 | [RGB-D 模型入口](/home/jyfight/Project/rtabmap_slam/src/rtabmap_semantic/rtabmap_semantic/emsanet_detector.py:87)；[固定投影拼接](/home/jyfight/Project/rtabmap_slam/src/rtabmap_semantic/rtabmap_semantic/emsanet_fusion.py:125)；[模型能力说明](/home/jyfight/Project/rtabmap_slam/docs/emsanet_semantic_mapping.md) | 真实双输入和五级拼接；不包含 NIR/声学模型。 |
| E8 | [真实关键帧张量对比](/home/jyfight/Project/rtabmap_slam/logs/emsanet/concat_fixed_live_20261110/paired_comparison.json)；[真实模型 ROS 链路记录](/home/jyfight/Project/rtabmap_slam/logs/semantic_validation/ros_real_emsanet_fixed_concat.json) | 三个真实关键帧的五级拼接/投影、输出等价及 ROS 投影/回环/重载。样例内参或无标注数据不是真值精度试验。 |
| E9 | [两配置比较说明](/home/jyfight/Project/rtabmap_slam/docs/slam_comparison.md)；[地图比较报告](/home/jyfight/Project/rtabmap_slam/logs/comparison/walk_20261028_2022_2026/comparison_report.md) | 地图结构、覆盖、约束比较。两次场景覆盖不同，无共同真值，不能证明融合精度增益。 |
| E10 | [Keyframe](/home/jyfight/Project/rtabmap_slam/src/rtabmap_semantic_msgs/msg/Keyframe.msg)；[SemanticMap](/home/jyfight/Project/rtabmap_slam/src/rtabmap_semantic_msgs/msg/SemanticMap.msg)；[SemanticObject](/home/jyfight/Project/rtabmap_slam/src/rtabmap_semantic_msgs/msg/SemanticObject.msg)；[实际发布字段](/home/jyfight/Project/rtabmap_slam/src/rtabmap_semantic/rtabmap_semantic/node.py:248) | 当前实际接口；对象 ID、UUID、置信度与观测锚点不能代替完整标准字段链。 |
| E11 | [计时边界说明](/home/jyfight/Project/rtabmap_slam/docs/fusion_cycle_measurement.md)；[现场计时分析](/home/jyfight/Project/rtabmap_slam/logs/emsanet/concat_fixed_live_20261110/analysis.md) | 分层耗时与排队积压。27.11 ms 是拼接融合层平均耗时，不是完整模型或端到端周期。 |
| E12 | [融合/回环说明](/home/jyfight/Project/rtabmap_slam/docs/fusion_loop_diagnostics.md)；[语义链路验证](/home/jyfight/Project/rtabmap_slam/docs/semantic_mapping_validation.md) | 真实 SLAM/回环与接口基础；非现场精度或 98% 成功率验收。 |
| E13 | [墙面提取](/home/jyfight/Project/rtabmap_slam/scripts/wall_geometry.py)；[地面栅格](/home/jyfight/Project/rtabmap_slam/scripts/grid_projection.py)；[动态清理](/home/jyfight/Project/rtabmap_slam/scripts/dynamic_cloud_node.py)；[栅格配置](/home/jyfight/Project/rtabmap_slam/config/grid_projection.yaml) | 几何候选、地面投影、实时清理及 clean_map 接入；没有水坑或完整野外地形语义。 |
| E14 | [导航图实现](/home/jyfight/Project/rtabmap_slam/scripts/navigation_topology.py)；[导航图说明](/home/jyfight/Project/rtabmap_slam/docs/navigation_topology.md)；[本日传输检查](/home/jyfight/Project/rtabmap_slam/logs/miti/map_display_validation_20261113.json) | 区域/通道/连接路径、净空约束及图 revision；本日输入仅 1 区域 0 边。 |
| E15 | [真实稀疏视觉特征](/home/jyfight/Project/rtabmap_slam/scripts/graph_feature_map_node.py:141)；[特征地图说明](/home/jyfight/Project/rtabmap_slam/docs/graph_feature_maps.md) | 优化后的关键帧三维视觉特征；不是 LiDAR 下采样伪装的视觉特征。 |
| E16 | [墙面三角消息](/home/jyfight/Project/rtabmap_slam/scripts/map_geometry_node.py:64)；[原生 mesh 导出能力](/home/jyfight/Project/rtabmap_slam/src/rtabmap/tools/Export/main.cpp:92) | 有三角片和上游完整网格导出能力；未找到项目完整场景网格验收记录。 |

本次检查记录：[源文档哈希与版本清单](/home/jyfight/Project/rtabmap_slam/logs/standards_coverage/20261113/source_manifest.json)、[任务书 27 项原文提取](/home/jyfight/Project/rtabmap_slam/logs/standards_coverage/20261113/task_requirements.json)、[85 项几何/输入测试结果](/home/jyfight/Project/rtabmap_slam/logs/standards_coverage/20261113/geometry_and_input_tests.json)、[64 项语义/模型测试结果](/home/jyfight/Project/rtabmap_slam/logs/standards_coverage/20261113/semantic_and_model_tests.json)。本报告可用于后续逐项补齐和安排验证，不能作为已完成全部任务书要求的验收证明。
