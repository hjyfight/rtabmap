# 3001建图流程与验证

在`/research-process.html`顶部切换“建图标准”，或直接打开`/research-process.html?standard=mapping`。原实时监控的九个面板保持不变。

六个步骤对应建图大纲5.1.1—5.1.4及5.2：输入及预处理、环境解析、地图关联与优化、地图与位姿、接口及地图操作、场景与性能验证。各步骤的局部按钮集中展示一种过程，避免将所有图固定堆在监控主页。

只读采集器`export_research_mapping.py`读取RGB-D、真实LiDAR、去畸变扫描、IMU、里程计、地图位姿、点云、栅格、墙面、语义对象及实际图约束。读取`/rtabmap/get_map_data2`仅请求关键帧和视觉词，不请求图像、扫描或栅格，不写数据库、发布传感器数据、调用运动控制或重置地图。正常启动沿用原3001命令，由监督器管理采集器。

优化对照来自`Node.pose`中未优化里程计位姿与`MapGraph.poses`，先使用当前`map_to_odom`统一坐标。图边残差根据实际约束和节点位姿计算，不表示绝对轨迹误差。三维视觉特征来自关键帧`word_pts`，不由LiDAR下采样生成。对象位置来自原语义模块的关键帧锚点。

通道分析对实际栅格运行既有几何分区方法，只读生成区域、通道和连接口；门口是几何候选，未知格不补为自由格。零节点和净空不足保留原状态。

场景登记为浏览器中的人工记录，包括结构化、非结构化、复杂地形与地面条件，随JSON导出保存，不等同于自动场景分类或场景验证通过。样本冻结停止页面取数，适合展示和留证。

性能仅采用`config/mapping_research_verification.json`中对应当前`map_uuid`的已验证记录。每项需包含`verified`、`scene_id`、`method`、`evidence`和`value`；成功率另需`success`、`total`，且与百分比一致；同步误差另需测量不确定度。没有固定限值的项目需记录`task_limit`与`comparison`。大纲中的“宜”值保留推荐属性。采样频率不当作全系统处理频率，修正量、约束残差和截图不当作精度。

当前没有性能报告时全部留为待验证。数据库文件可见不等于保存或加载验收通过；持久变化、子图、多时相对齐和版本兼容保留专项验证项。

无当前输入时保留历史记录，并标明历史来源和时间。初始化可读取`logs/research/live/map_inputs.npz`与同次`map_status.json`；这份历史点云未记录`frame_id`，因此不与其他坐标系数据叠加判定一致性。未归档的栅格、墙面顶点、分割图和图约束不伪造。ROS环境暂不可用时采集器保持等待并重试，不因该错误关闭全部3001服务。

开发检查：

```bash
.venv-indoor-research/bin/python3 -m pytest -q tests/test_research_mapping.py
.tools/miti-research/root/usr/lib/miti/node/bin/node scripts/check_research_mapping_ui.cjs
```

浏览器检查使用真实保存的地图采样，测试步骤切换、场景保存、冻结、导出和窄屏显示；`--live`额外检查运行中的3001。截图与报告在`logs/research/mapping-ui/`。
