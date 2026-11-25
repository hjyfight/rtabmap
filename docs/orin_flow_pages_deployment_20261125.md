# Orin流程与验证页面部署

目标为`qiyuan@192.168.1.103:/home/qiyuan/Project/rtabmap_slam`。融合与建图页面共用3001：

- 融合：`http://192.168.1.103:3001/research-process.html`
- 建图：`http://192.168.1.103:3001/research-process.html?standard=mapping`

同步页面、安装器和只读采集器源码，由Orin原生ARM64 Node运行；模型权重、CUDA环境、传感器配置和地图没有同步覆盖。设备上的验证配置和相机显示参考配置存在时保留原文件。备份与安装记录位于Orin的`logs/visualization_deploy/20261125_091330/`，最新清单为`logs/visualization_deploy/latest.json`。

仅重载`orin_research_web.py`所属网页、显示桥和只读采集器。原计算监督器277442及14个工作进程保持同一PID和启动时间，包括传感器接收、SLAM、语义、研究推理和地图显示节点。

新增`process_export`读取融合结果及可用特征文件，`mapping_export`在现有计算域74只读获取地图、语义对象、图约束与关键帧；浏览器显示域仍为75。完整特征文件存在时严格检查JSON与NPZ身份一致性。当前Orin计算配置未开启NPZ保存，所以融合使用原模型JSON中的真实图片、特征预览和统计，缺失输入与完整张量明确标注，接口不判为完整通过。不使用工作站样本补填，不重启原计算服务来改变归档设置。

适配了Orin RTAB-Map服务的嵌套地图消息：内层Graph没有header时使用外层MapData的真实坐标系和时间信息。该适配在Orin原生ROS消息上检查通过。

验证包括17项Orin原生数值及记录检查、实际HTTP资源、浏览器九面板、融合18步骤、建图6步骤、缺失归档提示、冻结、导出和窄屏显示。实际采集器取得原地图、语义对象与视觉特征；没有新传感器输入时展示保存记录，未测精度保持待验证。

工作站部署清单、浏览器截图与报告位于`logs/orin_visualization_deploy/20261125/`。本次没有提交Git、重新编译SLAM、安装模型依赖或切换传感器来源。
