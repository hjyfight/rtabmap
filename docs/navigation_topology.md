# 导航连通图：区域、通道与候选门口

MITI 的拓扑窗口现在默认显示从 `/miti/grid_map` 提取的几何导航图，原关键帧位姿图
保留在同一窗口的“关键帧位姿图”按钮下。刷新入口：
<http://localhost:3000/rtabmap.html?refresh=1>。稀疏视觉特征窗口保持原数据来源。

导航信息栏右侧的“隐藏”按钮可收起整块状态、图例及视角按钮，只保留“展开导航信息”
小按钮。开关保存在该面板的布局设置中，刷新或切换导航/位姿图后仍保留，不改变地图数据。
区域和连接口标签使用固定 12 像素文字，缩放、俯视或放大面板不会使文字遮住地图。
没有有效导航节点时，信息栏明确显示“当前仅显示栅格背景，尚未生成导航节点和连线”。

显示含义：

| 显示 | 含义 |
| --- | --- |
| 青色“区域”节点 | 具有较大净空的空闲空间，属于几何分区，尚未赋予房间功能名称 |
| 蓝色“通道”节点 | 连接分区的狭长空闲空间 |
| 黄色“候选门口”节点 | 分区间较窄的几何连接口，未经过门的语义确认 |
| 紫色“连接口”节点 | 分区间较宽的连接口 |
| 绿色折线 | 经过已知空闲格且满足配置净空的连接路径 |
| 背景栅格 | 原始地面投影栅格；未知区域保留为未知 |

处理过程为：以未知格和障碍格作为边界，计算距离场，按机器人半径和余量得到可通行
中心位置；从宽阔空间提取核心，分出剩余通道，再生成区域—连接口—区域图。
每条边都通过同一区域内的四邻接路径检查；显示折线只合并共线段，不能穿墙或跨越
未知区域。关闭门口、缩小间隙或改变栅格后重新计算，旧的节点和连接会清除。

这层输出用于观察和图上路径查询，没有接入机器人运动控制。它不把“区域”直接
命名为办公室、仓库等房间，也不把门口候选当作已识别的门。区域编号是当前
`revision` 内的标识，重新分割后可能变化；调用方应同时记录图版本。

## 输入和参数

`config/navigation_topology.yaml` 的默认值：

- 输入 `/miti/grid_map`，每 2 秒最多处理一次变化的栅格。
- 机器人圆形包络半径 `robot_radius: 0.40` m，余量 `safety_margin: 0.05` m。
  这是可调整的保守预设，需按实际足迹确定；不是从机器狗尺寸自动标定的结果。
- 宽阔空间核心的净空 `room_core_clearance: 1.0` m。
- 连接口宽度下界不超过 `doorway_max_width: 1.6` m 时标为候选门口。
- 最多处理一百万格和 256 个区域；过度碎片化时明确报告错误。

`scripts/start_miti.sh` 自动启动 `navigation_topology_node.py`。
`MITI_NAVIGATION=0` 可禁用该节点。它不改变 RTAB-Map、传感器和原二维展示栅格。

当前地面投影只有被实际观测的地面格为自由格，不从全局点云虚构射线来填充空闲区。
2026-11-10 的实机输入有 4042 个空闲格，但最大保守净空半径约 0.365 m，低于
默认 0.45 m，所以此输入没有可确认的导航节点或连接。窗口会显示
“已知空闲区域不足以容纳当前机器人净空”，并保留背景栅格及原位姿图切换。
这不代表实际房间无法通行，而是当前栅格的已知空闲覆盖不足。需要更完整的自由空间
观测，或将 `grid_topic` 指向经过验证的导航占据栅格；不要仅为显示连线而缩小机器人包络。

## 输出与验证

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/miti/navigation_map` | `visualization_msgs/msg/MarkerArray` | 区域、连接口、标签和经过检查的折线路径 |
| `/miti/navigation_graph` | `std_msgs/msg/String` | JSON 图，含版本、坐标系、节点类型、边长、最小净空和路径坐标 |
| `/miti/navigation_status` | `std_msgs/msg/String` | 区域/通道/门口数量、连通分量、输入状态、当前位置所属区域 |

最新 JSON 同时保存到 `maps/current_navigation_graph.json`；错误或无有效栅格时
清除旧连接。`navigation_topology.shortest_route(graph, start_id, goal_id)` 按边长查询
图上的最短路，断开时返回 `None`。该查询只产生结果，不发送导航目标。

```bash
source scripts/setup_miti.bash
python3 -m unittest discover -s tests -p 'test_navigation_topology*.py' -v
.tools/miti/root/usr/lib/miti/node/bin/node tests/test_miti_graph_feature_maps.js
python3 scripts/check_navigation_topology_live.py --seconds 8 --allow-waiting \
  --output logs/miti/navigation_topology_live_20261110.json
```

合成场景验证了两个区域、一个走廊、两个连接口构成的 5 节点/4 边图，以及关门、
未知缺口、过窄通道、旋转栅格和节点清除。示例明确区分合成场景与实机输入：
[验证图](../logs/miti/navigation_topology_validation.png)。检查报告分别记录
`transport_passed` 和 `navigation_ready`，不会把零节点的传输成功当成可导航地图。

部署后的实机检查已收到全部三个导航话题，`transport_passed=true`，当前状态为
`insufficient_free_space`、`navigation_ready=false`，节点和边均为零。
HTTP 检查确认首页引用新版页面脚本，两个前端脚本的响应内容与安装文件一致；
这项检查不包含浏览器画面验收。检查记录分别保存在
`logs/miti/navigation_topology_live_20261110.json` 和
`logs/miti/navigation_topology_http_20261110.json`。

## 2026-11-10 点击切换后的背景显示修复

二维栅格和导航窗口共享 `/miti/grid_map`。MITI 原客户端只缓存 TF；二维窗口已接收
静态地图后，导航窗口新加入的监听器拿不到之前的快照，直到机器人移动导致地图更新。
现在只对该完整 OccupancyGrid 话题，在活动订阅中保留最新一份解码消息，并立即补发
给后加入的渲染器。空地图会替换旧快照；取消最后一个监听器、断线、重新绑定或撤销
话题时清除缓存。没有缓存相机图像、点云或增量 MarkerArray。

这项改动解决静止时切换面板的背景缺失，不会生成原本不存在的导航连线。本次实机
采样收到 2851 个空闲格，但最大保守净空半径约 0.256 m，小于配置的 0.45 m，
可通行中心格仍为零。点云投影的已知地面格较碎，不能把全部未知格填成空闲。
记录：`logs/miti/navigation_click_audit_20261110.json`。

回归覆盖隐藏/展开与布局状态保留，以及实际 MITI 客户端的晚订阅、重复切换、
地图重置、断线、取消订阅和缓存释放。浏览器控制接口当前不可用，因此代码与 HTTP
检查不代替用户浏览器上的画面验收。
