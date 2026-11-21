# 统一实时控制台（3000）

当前入口：http://localhost:3000/ 。旧 `rtabmap_ui` 查询参数可继续使用。

2026-11-19 已将 3000 更新为三维语义地图、二维栅格、回环状态、RGB／NIR／对齐深度、
UniBind 特征、目标分割／融合及声音事件的统一布局。首次更新备份浏览器原布局，保留
同名地图的视角与相机设置；话题侧栏默认收起，通过左侧按钮展开。

当前接法：机器狗采集相机与雷达，主机运行 YOLOE 建图、UniBind、M260C 声音识别和 3000 展示。
先在机器狗上启动原有传感器驱动，M260C 的 USB 音频和串口保持接在主机。主机执行：

```bash
cd /home/jyfight/Project/rtabmap_slam
source scripts/ros2_wired_env.sh
bash scripts/start_semantic_yoloe.sh --gpu
```

YOLOE 建图入口现在自动补齐主机音频、GPU UniBind 四模态特征、RGB/NIR 目标感知和
3000 展示；已运行的服务按实际 ROS 发布端复用，不重复打开麦克风或加载相同模型。
只读的 `--check-input-only` 不启动这些感知服务。停止建图时清理本次自动启动的服务，
此前独立运行并复用的服务保持运行。模型首次加载需要等待。

浏览器直接打开 http://localhost:3000/ 。更新后按 Ctrl+Shift+R 加载最新页面资源。
已有统一展示时不用再另开 MITI；只需单独启动展示时仍可执行：

```bash
source scripts/ros2_wired_env.sh
bash scripts/start_miti.sh --port 3000
```

展示使用 ROS domain 0、8765 桥接，与建图及主机音频使用同一份有线 DDS 配置。
`start_dashboard_miti.sh`、`start_host_m260c.sh` 尊重显式 `CYCLONEDDS_URI`；未设置时，
检测到 `enxec1ac312a316` 在线且有 IPv4 就选择 `config/wired_cyclonedds.xml`，否则选择
本机回环配置。切换配置需要重启对应节点，另一个终端的环境不会改变已运行的节点。
缺少输入时面板显示等待状态。`start_yamnet_miti.sh` 默认也进入这个控制台。

人员及身体、衣物、鞋袜标签只显示最近 3 秒内的新观测；即使旧建图记录把衣物标为静态，
显示层也按临时观测处理，并使用最新观测位置。重复 ROS 快照、图优化和旧关键帧不能刷新
它的存活时间。显示适配器每 0.5 秒清理过期标签，并反复发送定向删除，使断线重连的
页面也能清理旧标签。动态目标缺少新锚点时不沿用旧位置；桌椅等静态物体继续保留。

M260C 目前接在主机。启动音频采集、YAMNet 分类和串口方位服务：

```bash
bash scripts/start_host_m260c.sh
```

启用前不要同时运行向 domain 0 发布相同话题的 Orin 音频转发。2026-11-19 已停止该旧
转发，主机实时音频约每 0.5 秒更新，分析窗口为 2 秒。声音区域显示当前 Top-5 候选，
类别随真实模型结果更新，不固定猫或狗；历史帧明确显示为上一帧。

四模态特征复用 `/multimodal/audio`，无需另开麦克风采集。只补启特征时可以执行：

```bash
bash scripts/start_fourmodal_live.sh --reuse-audio --gpu
```

入口优先只读测量机器狗与主机的时钟差；SSH 查询失败时，使用真实传感器的到达时延
估计进行预览匹配，原始采集时间戳始终保留。估计方法和样本写入当前运行的 `clock.json`，
此预览不证明硬件同步精度，也不能用于声源与地图目标的精确位置绑定。

串口服务只发送握手确认和版本查询，不更改唤醒词、波束或固件。
`/perception/audio/direction` 提供 M260C 语音唤醒的阵列自身角度、来源、序号及年龄。
超过 5 秒、连接中断或状态异常时移除实时方向箭头。重连和重复消息不会刷新旧角度。

重新插拔后若串口权限丢失，在主机终端执行：

```bash
sudo setfacl -m u:jyfight:rw /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0004-if00
```

2026-11-19 握手、版本查询和页面订阅均通过，设备返回 `circle6:v1.0.2`，
`se_circle.1004`。用户选择稍后进行真实唤醒测试，目前未验证角度上报或定位精度。
阵列 0° 与机器人朝向尚未确认，因此不投影到地图，也不输出距离或绑定猫等实体目标。
该固件接口不能直接当作任意环境声音／猫叫的连续定位接口。

前端检查：浏览器加载全部 9 个面板，没有控制台错误或横向溢出；实时声音话题、心跳与
串口状态可通过 8765 订阅。检查资料保存在 `logs/miti/dashboard-browser.json`、
`logs/miti/dashboard-3000-streams.json` 及 `logs/miti/dashboard-3000.png`。
