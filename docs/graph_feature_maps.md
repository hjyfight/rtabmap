# Miti 导航连通图、关键帧位姿图与稀疏特征地图

2026-11-10：拓扑窗口默认改为从二维空闲栅格提取的**导航连通图**，展示区域、通道、
候选门口和可通行连接；保留“关键帧位姿图”切换。算法、净空配置、当前实机覆盖限制
见[导航连通图说明](navigation_topology.md)。下文的 RTAB-Map 图接口仍供位姿图与
稀疏视觉特征使用。

运行 `bash scripts/start_miti.sh` 时自动启动两种地图的显示节点。已有网页打开
<http://localhost:3000/rtabmap.html?refresh=1>，会补上两个独立面板，并保留原地图视角、
相机、回环面板和其他设置。完整项目布局左侧上排为三维点云、二维栅格，下排为拓扑、
稀疏特征；右侧继续显示相机。每个地图面板都可独立放大。

- **关键帧位姿图**（切换后显示）：青色球为关键帧，蓝色线为相邻里程计约束，粉色线为全局回环，
  黄色线为空间/时间邻近回环。用户约束、虚拟连接、地标连接保留独立命名空间。
  不按节点编号猜测连线；只显示 RTAB-Map 实际输出的约束，双向重复约束只计一次。
- **稀疏特征地图**：显示 RGB-D 关键帧保存的三维视觉特征点，颜色表示高度。
  不将雷达点云下采样后当作视觉特征。无有效深度的特征不显示；纯雷达建图可能没有视觉特征。
- 两个面板支持“适应地图”和俯视/三维视角切换；图例显示当前节点、约束或特征数量。

仅展示实时数据，不自动加载保存的数据库。SLAM 未启动时显示“等待 RTAB-Map 位姿图”；
启动建图后自动接入。显示节点不会启动传感器、改变建图参数或重置地图。

输入为 `/mapGraph`、`/mapData`，使用 `/rtabmap/get_map_data2` 补齐全局图和历史特征。
该服务仅设置 `global_map=true`、`optimized=true`、`with_words=true`；不请求图像、扫描、
栅格或其他传感器数据。默认每 15 秒读取一次完整快照，其间使用流式更新；局部子图
缺少的节点保留，完整快照负责处理图压缩或删除。延迟服务响应不会覆盖更新后的位姿。
特征保存在关键帧局部坐标中，回环优化后重新投影到 `map` 坐标系。

输出为标准 ROS 消息，原生 Miti 三维窗口直接绘制：

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/miti/topological_map` | `visualization_msgs/msg/MarkerArray` | 关键帧和各类约束 |
| `/miti/sparse_features` | `sensor_msgs/msg/PointCloud2` | 有效 XYZ 视觉特征 |
| `/miti/graph_feature_status` | `std_msgs/msg/String` | 数量、范围和更新时间 |

地图使用 Reliable + Transient Local + depth 1，刷新或晚打开的窗口可收到快照。
信息栏只订阅独立状态话题，避免抢先消耗原生绘图器的保留地图消息。

配置在 `config/graph_feature_map.yaml`：默认每秒最多生成一次显示快照，按 2 cm 空间格
去重重复观察点，最多显示 150000 点。该显示限制不影响 RTAB-Map 中的特征。
设置 `feature_voxel_size: 0.0` 可关闭去重；`MITI_GRAPH_FEATURES=0` 可禁用显示节点。

验证命令：

```bash
source scripts/setup_miti.bash
python3 -m unittest discover -s tests -p 'test_graph_feature_maps.py' -v
.tools/miti/root/usr/lib/miti/node/bin/node tests/test_miti_graph_feature_maps.js
python3 scripts/check_graph_feature_maps_live.py --seconds 12 \
  --output logs/miti/graph_features_check.json
```

建图未启动时可加 `--allow-waiting` 验证空地图与状态的桥接传输。报告分别记录
`transport_passed` 与 `map_data_ready`，不会将传输成功当作已有地图。
2026-11-09 部署时没有 SLAM 发布端，按用户选择保持等待实时数据；未加载保存地图。
当前工具没有可用浏览器，实际页面绘制尚未进行视觉验收。

2026-11-10 实机复核：页面已有新面板，但 MITI 仍由 11 月 8 日启动的旧进程运行，
未启动 `graph_feature_map_node.py`。重启 MITI 展示服务后，保持原 SLAM 进程及数据库
不变，桥接采样收到 5 个关键帧、4 条相邻约束和 2034 个有效视觉特征点；
`transport_passed`、`map_data_ready` 均为 `true`。报告见
`logs/miti/graph_features_recovery_20261110.json`，进程核对记录见
`logs/miti/graph_maps_recovery_20261110.json`。浏览器需要重新加载新版组件：打开
`/rtabmap.html?refresh=1`，必要时按 Ctrl+Shift+R。当前仍无可用浏览器控制接口，
此记录确认真实地图数据传输，不代表已完成页面视觉验收。

同日用户截图确认稀疏特征已出现，但两个新窗口仍显示普通墙面控件。继续修正前端：
按独立地图话题识别并修复缺失的 `rtabmapGraphMap` 标记，保留现有布局和视角；
图面板不再挂载墙面与高度裁剪控件。MITI 的固定组件文件名使用一年 `immutable`
缓存，安装器现将三维组件的内容摘要加入延迟加载 URL，避免新版页面与旧渲染器混用。
前端 22 项测试通过，实际 HTTP 资源与安装文件一致；验证记录为
`logs/miti/graph_frontend_fix_20261110.json`。更新已安装，用户重新打开
`/rtabmap.html?refresh=1` 后应看到数量、图例和“适应地图”。此轮页面效果仍待用户确认。

后续截图显示运动后拓扑轨迹与视觉特征均已更新，但普通墙面控件仍在。仅修改延迟
组件的 URL 没有覆盖同样采用固定文件名的首页脚本，之前的刷新措施不充分。
安装器现生成以内容摘要命名的首页脚本，同时更新 Next.js 的页面入口清单，并保留
经校验的原始清单。重载展示服务后，实际 HTTP 首页已只引用新文件
`page-8a71873d9e043968.js`；SLAM 进程保持不变。安装、升级和意外清单变更的 3 项
集成测试通过；对实际返回的原生三维组件执行 4 组带标记/缺失标记的渲染分支检查，
均选择地图专用控件并关闭普通墙面及高度控件。记录见
`logs/miti/graph_frontend_entrypoint_20261110.json`。重新打开项目刷新入口后加载；
这些检查仍不等同于真实浏览器页面视觉验收。
