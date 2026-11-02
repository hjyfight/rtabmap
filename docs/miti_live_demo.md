# MITI 实时验收展示

本机为 Ubuntu 22.04 amd64 / ROS 2 Humble，已在项目内安装 MITI **3.3.6**、
foxglove_bridge **3.6.0** 和所需的 rosx-introspection / RapidJSON 包。
使用官方预编译包，无需 npm、Docker 或 sudo。

## 启动与使用

1. 按项目原来的流程启动 Livox、RealSense（启用对齐深度），并检查实际相机外参。
2. 启动建图，任选对应入口：

   ```bash
   cd /home/jyfight/Project/rtabmap_slam
   # 雷达 + RGB-D 融合
   bash scripts/start_verified_fusion.sh
   # 或启用 YOLO 语义标签
   bash scripts/start_semantic_slam.sh
   ```

3. 新开终端启动展示：

   ```bash
   cd /home/jyfight/Project/rtabmap_slam
   bash scripts/start_miti.sh
   ```

4. 本机浏览器打开 **http://localhost:3000/rtabmap.html**，点击
   **应用项目布局并进入展示**。已有浏览器布局会先备份在该地址的 localStorage，
   也可以点击“继续使用当前布局”保留自己的面板设置。
   局域网另一台电脑使用 `http://本机实际IP:3000/rtabmap.html`。
   若端口被占用，MITI 会选择其他端口，以终端输出为准。

展示脚本只启动 MITI、foxglove_bridge 和显示消息转换节点。关闭此终端或按 Ctrl+C
停止展示，传感器和 SLAM 仍由各自终端管理。需要切换地图会话时，建议同时重启展示脚本。

## 默认面板

| 面板 / 图层 | 话题 | 说明 |
| --- | --- | --- |
| 三维地图 | `/cloud_map` | 固定坐标系 `map`，按高度着色，最多显示 30 万点 |
| 优化轨迹 | `/miti/trajectory` | 从 `/mapGraph` 的当前优化位姿生成 Path，回环后随图更新 |
| 语义标签 | `/miti/semantic_markers` | 将 `/semantic/objects` 的节点锚点转换为地图坐标下的 MarkerArray |
| 当前位姿 | `/icp_odom` | 当前项目的 ICP 里程计，位置通过 TF 映射到地图 |
| 实时雷达 | `/livox/lidar` | 默认隐藏；需要时在三维面板 Topics 中打开 |
| 彩色画面 | `/camera/color/image_raw` | 默认最多绘制 15 FPS |
| 对齐深度 | `/camera/aligned_depth_to_color/image_raw` | 默认 10 FPS、16UC1 深度显示范围 0.3–4 m |
| 回环检测日志 | `/rosout` | 底部左侧，14 px 字体；默认筛选 `loop_closure_diagnostics` 节点的 INFO 及以上日志 |
| 回环状态 | `/miti/loop_status` | 底部右侧的原生面板，显示检测状态、成功次数、最近匹配节点和输入频率；支持独立放大 |

回环日志显示检测状态、候选及接受/拒绝结果、成功记录和输入频率，默认自动滚动。
在日志面板清除 `loop_closure_diagnostics` 搜索条件，可恢复查看所有节点的运行日志。
它直接使用已有 ROS 日志话题，不依赖 Qt 窗口；回环诊断进程需要保持运行。
日志面板显示接入后收到的消息，不会自动读取本地历史 JSONL 日志。
更新布局后，打开 `/rtabmap.html` 并点击“应用项目布局并进入展示”；只刷新主页面会
继续使用浏览器已保存的旧布局。应用前会自动备份旧布局。

### 回环状态面板

`loop_closure_diagnostics.py` 现在独立于 Qt 发布 `std_msgs/msg/String` JSON 状态快照。
与 Qt 共用 `LoopStatus` 的计数和提示逻辑：仅接受事件累计成功，候选/拒绝不计数，
同一回环去重，成功提示保留 8 秒，SLAM 超时优先显示中断。全局与空间邻近分别计数；
`PROXIMITY_ICP` 沿用原诊断事件名，界面显示“空间邻近回环”，不据此推断一定是 ICP 分支。

快照每 0.5 秒发布一次，使用 Reliable + Transient Local + depth 1，晚打开或刷新网页
仍能看到诊断进程本次运行的累计记录。诊断重启后开始新的 `monitor_id` 和计数。
网页 3 秒收不到更新时显示“诊断更新已中断”，保留最近记录并隐藏过期输入频率；
桥接断开时显示“等待连接”。提示音默认关闭，需要用户勾选后由浏览器播放。
它不会替换已有 `/rosout` 日志或 `logs/loop_closure_events.jsonl` 持久化记录。

本机 MITI 3.3.6 是预编译包，没有外部面板加载接口。项目将可读的 React 面板代码保存在
`config/miti_loop_status.js`，由 `scripts/install_miti_loop_panel.py` 注册原生面板类型、
渲染入口和图标；现有 MITI 的拖动、布局保存与独立放大继续可用。
安装器只接受已核对的 3.3.6 前端文件 SHA-256，备份原文件并校验修改后的 JavaScript。
未知版本或文件修改会拒绝覆盖，需要核对新的接入点后升级；不会修改系统安装。
安装和启动脚本都会安装/更新扩展。首次加入扩展后需要重启 MITI 展示服务，让 Next.js
识别新增的公开资源清单；应用布局前会刷新扩展资源缓存。已有诊断进程需按原流程
重启后才会发布新增状态话题。

检查：`node tests/test_miti_loop_panel.js`（使用 MITI 随附的 Node）、
`python3 tests/run_loop_diagnostics_check.py`（独立域 179）和
`bash scripts/test_miti.sh`（独立域 91，含状态话题到 Foxglove 的传输）。
这些检查验证状态逻辑、React 输出、面板安装与通信，不替代浏览器实际视觉验收。

2026-11-02 部署验证：26 项诊断/Qt 状态检查、7 项网页面板检查通过；独立域 179
验证了晚订阅快照、去重和超时，独立域 91 验证了包括回环状态在内的 7 类消息传输。
本机运行中的 MITI 已重启并核对面板资源，实机状态采样结果在
`logs/miti/loop_panel_live.json`。当前浏览器不可由工具控制，未执行实际页面视觉验收。

上线时已有 Qt 诊断进程仍在运行，本次会话使用 `.tools/miti/existing_loop_session.py`
临时桥接：只重放该进程启动后的 JSONL 记录，并持续跟随日志，保留当前成功计数。
它会在旧诊断退出或原生状态发布器出现时结束。新启动的诊断进程直接发布状态，
常规启动不需要此兼容脚本；既有 Qt、SLAM 与地图数据库未因接入面板而重启。

### 隐藏天花板与墙体顶部

三维地图右上角新增“隐藏顶部”开关。默认关闭；打开后拖动“最高显示高度”滑块，
或输入数值后按 Enter / 移开焦点，隐藏高于该高度的点云。关闭后立即恢复完整点云，
无需等待新的地图消息。初始值为 2.00 m，只是可调起点，不是已测量的天花板高度。

高度使用当前 Fixed frame 的 Z 坐标（项目默认为 `map`），不是离地高度。
滑块初始范围覆盖 -5～5 m；数值输入可扩大范围。旋转观察视角或切换机器人跟随模式
不会改变切面的地图高度。该设置随 MITI 布局保存，位于 `scene.heightClip`。

实现保存在 `config/miti_height_clip.js`，通过现有版本校验安装器接入点云着色器，
在 GPU 绘制时丢弃切面以上的像素；不修改 PointCloud2 数据、SLAM 参数或地图数据库。
裁剪也作用于点云的 stixel 竖线，不作用于语义标签、轨迹或机器人模型。
它按高度截取，会同时隐藏同一高度以上的天花板、墙顶和其他物体。

更新后从 `/rtabmap.html` 应用项目布局会刷新控件缓存；要保留自己调整过的布局，
也可在 MITI 主页面按 Ctrl+Shift+R 强制刷新。此次无需重启 SLAM 或增加 ROS 话题。
检查命令：使用 MITI 随附 Node 运行 `tests/test_miti_height_clip.js`。

自定义语义消息无法由 MITI 直接绘制，因此新增 `scripts/miti_scene_adapter.py`。
它按最新 `/mapGraph` 重算观测锚点的加权位置，发布标准 Path 和 MarkerArray；
缺少锚点或坐标系不一致时隐藏相应标签。每次发送完整标签快照，移除已消失的对象。
标签沿用模型类别名称，通常为英文。它不更改 SLAM 算法、外参或地图数据库。
无语义消息包时仍可展示轨迹；没有运行语义建图时不会出现标签。

相机话题随驱动版本可能带 `/camera/camera/` 前缀。若面板无画面，点击面板标题，
从 Topic 下拉框选择实际发布的话题。深度若使用 `32FC1` 米单位，应把显示范围改为 0.3–4。

## 网络和性能

默认 domain 为 0，DDS 使用项目同款 CycloneDDS。已连接时优先使用
`enxec1ac312a316` 传感器网卡，否则自动选网卡；显式 `CYCLONEDDS_URI` 优先。
桥接默认只发布项目显示相关的话题，避免未安装的 Unitree 自定义遥测消息产生 schema 错误；
如需其他话题，可通过 `BRIDGE_TOPIC_WHITELIST` 覆盖正则列表。
换网卡时可指定：

```bash
MITI_INTERFACE=eno1 ROS_DOMAIN_ID=0 bash scripts/start_miti.sh
```

传感器、SLAM、转换节点和桥接必须位于相同 ROS 域，并能够通过 DDS 互相发现。
不要只在 MITI 网页中更改桥接的 ROS domain；应在终端设置环境后重启整个展示脚本。
若网页保存的 domain 与脚本不同，脚本会提示修正 `.tools/miti/state/domain-*/bridge.json`
或指定新的 `MITI_STATE_DIR`，避免转换节点留在旧域。

面板 Max FPS / Max rate 只限制浏览器解码和绘制，不会限制网络发送量。
建议验收时使用本机浏览器或有线网络；无线展示优先选择驱动已经发布的
`.../compressed` 彩色话题，并关闭暂时不看的深度或点云面板。
正式验收需观察移动过程中点云、TF 与图像是否同步，确认回环后地图和标签一起更新。

## 安装文件与验证

安装文件位于 `.tools/miti/`，下载包在 `downloads/`，解包内容在 `root/`，
已加入 Git 忽略。Node.js 由 MITI 包自带。没有写入 `/usr`、`/opt/ros` 或系统服务，
也没有添加开机自启。可重跑：

```bash
bash scripts/install_miti.sh
```

MITI 安装包 SHA-256：
`69b996bff7d9fc2567d16433d7e025f7d4488663d4466c85470d67fa8ebc68d4`。
ROS 包通过 apt 索引校验下载，所有缓存包哈希记录在 `.tools/miti/package-sha256.txt`。
缓存齐全时可在本机离线重新部署；换机器仍需先装 ROS 2 Humble Desktop 及其运行库。

```bash
bash scripts/test_miti.sh
```

测试使用仅本机的 domain 91、网页端口 13000、桥接端口 18765，启动后自动清理。
它检查网页和项目布局响应，并发布合成点云、彩色图、深度图、TF、图节点与语义对象，
通过 Foxglove WebSocket 接收 CDR 消息，再由 ROS 反序列化校验实际数值。
结果在 `logs/miti/connection_test.json`，服务器日志在 `logs/miti/test_server.log`。
另有三项转换单元测试覆盖回环位姿更新、无效/缺失锚点、标签删除和大对象 ID。
2026-11-01 本机验证结果：三项单元测试通过，六种显示话题端到端校验通过（`passed=true`）。

这些测试不等于浏览器 WebGL 渲染、真实传感器吞吐或实机建图验收通过。
本次环境没有可控制的浏览器，浏览器视觉效果需现场打开页面检查。

## 2026-11-01 真实设备连接验证

设备接通后，已接入用户终端中运行的语义融合建图，保留原有建图进程。
传感器侧采样：Livox 点云约 10 Hz、IMU 约 200 Hz，RealSense RGB 和对齐深度
均为 424×240、约 6 Hz，RGB-D 输入检查通过。

19:32 的 10 秒 WebSocket 只读采样通过：收到 38,768 点地图、7 个轨迹位姿、
2 个 `chair` 标签，以及 TF、里程计和彩色/深度图。语义状态为已处理 25 个关键帧、
失败 0 个。该时段网页链路彩色约 6 Hz、对齐深度约 4 Hz；这是采样结果，不是持续性能保证。
当前地图路径和节点记录在 `logs/miti/existing_slam.json`。

可在展示服务运行时再次执行只读检查（不发布测试数据）：

```bash
source scripts/setup_miti.bash
python3 scripts/check_miti_live.py
```

最新报告写入 `logs/miti/live_stream_check.json`。该检查证明真实 ROS 数据可经
桥接传输和解码，尚未验证浏览器 WebGL 画面、运动过程地图精度或闭环效果。

### 截图显示检查与时间基准

19:34 的实机截图已确认浏览器能显示点云、彩色图、深度图和日志，但左下角出现
`/icp_odom: TF history unavailable: icp_odom`，且语义文字太小。
只读检查发现：机器人里程计时间戳约为 7,459,471 秒，工作站发布的 `map -> icp_odom`
TF 时间戳约为 1,793,533,061 秒。二者不在同一时间基准上，无法按消息时间查历史 TF；
日志中非常大的 `delay` 也不能解释为实际网络延迟。

项目展示布局已将三维面板的 **Sync timestamps 关闭**，使用最新 TF 展示当前位姿，
并将 **Label scale 调为 6**，让远距离语义文字可读。地图、轨迹和语义坐标本身已经在
`map` 系中。正在打开的浏览器需要回到 `/rtabmap.html`，重新应用项目布局才能更新；
也可在三维面板的 Frame / Scene 设置中手动修改这两项以保留当前视角。

这是当前实时展示的兼容设置，不代表传感器与工作站时钟已经统一。若启用运动中原始
点云叠加或相机三维投影，需要先统一时间基准，再开启按时间戳同步并做运动/回环验证。

官方资料：[MITI 文档](https://miti.readthedocs.io/en/latest/)、
[项目源码](https://github.com/hiran-t/MITI)。
