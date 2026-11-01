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
| 日志 | `/rosout` | INFO 及以上，查看建图、回环及异常信息 |

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
