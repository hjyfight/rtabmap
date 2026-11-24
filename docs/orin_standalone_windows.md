# Orin 独立采集与 Windows 网页显示

准备的目标是：RealSense、MID360、WheelTec 直接接 Orin，Orin 运行驱动、SLAM、YOLOE 语义建图和研究模型，Windows 只打开网页并通过 SSH 启停。Go2 可以作为载体；独立模式没有 Go2 的网络接收进程，不依赖机身 IMU、内置雷达、关节或运动状态。

Orin 项目路径为 `/home/qiyuan/Project/rtabmap_slam`。当前局域网 IP 为 `192.168.1.103`。Windows 不需要安装 ROS、CUDA、模型或 Python；使用 Edge/Chrome 和 Windows 的 OpenSSH 客户端即可。

## 目前已经准备的内容

- Orin 上已有模型、推理环境和传感器驱动；独立模式的软件预检通过。
- Orin 已运行网页 `http://192.168.1.103:3001/`，浏览器自动连接同一地址的 WebSocket 端口 `8766`。
- 独立模式使用 `config/orin_direct_research.json`，统一入口是 `scripts/start_orin_standalone.sh`。这个入口负责启动传感器、计算及网页服务，无需重复运行原来的两段 Foxy 驱动命令。
- 当前计算仍接收 Go2 的外接传感器数据，准备工作没有切换采集来源。
- 相机尚未接入 Orin，雷达以太网接口尚未配置，实际直连采集与最终安装外参仍待验证。网页部署成功不代表这些硬件检查已经通过。

## 接线与第一次启动

1. Orin 和 Windows 接入同一个 TP-Link 局域网。建议在路由器给 Orin 保留固定 DHCP 地址；如果 IP 改变，下面的 SSH 和网页地址也相应更改。
2. 将 RealSense 接 Orin 的 USB 3 接口，WheelTec 的录音 USB 接 Orin。若使用拓展坞，保持独立供电，并确认其 USB 数据链路正常。
3. 将 MID360 的以太网数据线接 Orin 的有线网口或 USB 千兆网卡，并保持雷达独立供电。准备的配置使用雷达 `192.168.123.20`、Orin 雷达网口 `192.168.123.30/24`。Wi-Fi 继续用于网页访问。
4. 在 Windows PowerShell 登录 Orin，密码在本地终端输入：

   ```powershell
   ssh qiyuan@192.168.1.103
   ```

5. 以下命令均在登录后的 **Orin 终端**执行。先配置实际连接雷达的接口；只有一个有线接口时会自动选择，多个接口时需指定名称，例如 `enx...`：

   ```bash
   cd /home/qiyuan/Project/rtabmap_slam
   ip -br -4 addr
   bash scripts/configure_orin_mid360.sh
   # 多个有线接口时：bash scripts/configure_orin_mid360.sh 实际接口名
   bash scripts/start_orin_standalone.sh --check
   ```

   网口配置需要在 Orin 输入 sudo 密码。它设置 `192.168.123.30/24`，且不将雷达接口设为默认路由，保留 Wi-Fi 出口。预检会分别显示 `software_ready` 和 `hardware_ready`；硬件均已连接时应为 `true`。预检本身不会启动采集。

6. 硬件预检通过后，先停止原来的 Go2 输入计算，再启动独立模式。停止脚本只处理本项目记录的监督进程：

   ```bash
   python3 scripts/stop_orin_services.py --compute
   nohup bash scripts/start_orin_standalone.sh > logs/orin_standalone_launch.log 2>&1 < /dev/null &
   ```

   已有的 Orin 网页服务会被复用，并跟随新计算会话读取结果。不要同时运行两个计算入口。`nohup` 允许关闭 SSH 窗口后继续运行，当前没有配置开机自动启动。

7. 在 Windows 浏览器打开 `http://192.168.1.103:3001/`，查看相机、地图、语义标签和推理结果。无需启动当前 Linux 主机的网页、转发器或 ROS。局域网应允许访问 Orin 的 `3001` 和 `8766` 端口。

## 后续启停与检查

硬件与网口配置保持不变时，再次登录 Orin 后执行：

```bash
cd /home/qiyuan/Project/rtabmap_slam
bash scripts/start_orin_standalone.sh --check
nohup bash scripts/start_orin_standalone.sh > logs/orin_standalone_launch.log 2>&1 < /dev/null &
```

停止独立计算及网页：

```bash
python3 scripts/stop_orin_services.py --standalone --web
```

查看状态：

```bash
tail -n 60 logs/orin_standalone_launch.log
tail -n 60 logs/orin_standalone/compute.log
cat logs/orin_web/current_run.json
cat logs/go2_orin_research/current_run.json
```

`10404`、`26046` 是之前 Go2 上那次运行的进程编号，不能当作启动命令；重启后编号会变化。

## 外参与时间检查

只改变 USB/网线接在哪台电脑，而相机与雷达相对安装位置不变，通常无需重新求外参。重新安装改变了位置或朝向，就需要重新标定。当前使用的外参仍是自动估计，`geometry_verified=false`，需要在最终安装位置验证，不能当作已完成的正式标定。

研究配置的正式标定入口为 `config/fusion_research_calibration.json`，目前估计值为 `config/fusion_research_estimated_calibration.json`；相机 TF 使用同一配置。独立模式移除原来的 Go2 网络时钟偏移，但仍需实际检查设备时间戳、RGB/深度/LiDAR 对齐和音视频窗口，不能把同机采集等同于硬件同步。

## 本次准备的验证记录

2026-11-22，独立模式软件与网页预检通过，硬件预检仍待相机和雷达接入。完整命令构建的 5 项测试通过，浏览器时间偏差测试覆盖 Windows 时间相差 ±1 小时的情况。

从另一台电脑直接访问 Orin，HTTP 页面与资源返回 200，WebSocket 收到并成功解码 424×240 RGB 图像、58,996 点地图和 14 条语义标记。实时状态持续更新；抽样期间 AVSegFormer 推理结果递增，PMF 与 RGB/NIR 因当前 Go2 无线输入延迟处于等待状态。这是网页传输验证，尚未在实际 Windows 浏览器进行画面验收，也未验证 Orin 直连传感器采集。

记录保存于 `logs/orin_standalone_validation/web_transport.json` 和 `bridge_decoded.json`。
