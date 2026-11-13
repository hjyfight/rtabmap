# M260C 六麦阵列在当前 Ubuntu 主机上的使用

检查日期：2026-11-13。设备为 WHEELTEC M260C 配套主控板与环形六麦阵列。

## 已实测的状态

| 功能 | 当前结果 |
| --- | --- |
| USB 录音 | 成功采集 5 秒 WAV：16000 Hz、16 位、单声道，共 80000 帧；有非零信号，无削波。尚未评价人声清晰度。 |
| USB 声卡回放 | `aplay` 通过 C-Media 声卡正常播放完成，退出码 0；实际听感需要在音箱/耳机端确认。 |
| 华为 USB-C 耳机回放 | 已通过指定的 PulseAudio 输出播放系统英文提示音，用户确认可以听到。耳机音量已设为 50%。 |
| 串口通信 | 握手成功，版本查询返回 `code: 0`。 |
| 固件版本 | `circle6:v1.0.2`，引擎 `se_circle.1004`，固件编译日期 2022-06-16。 |
| 硬件唤醒、声源角度 | 尚未打通。用户尝试“小微小微”后无反应；30 秒监听及后续诊断均未收到角度事件。 |
| 语音转文字、口令识别 | 本次未接入上位机 ASR，不能把录音成功等同于识别成功。 |

录音入口已经可用，无需先完成硬件唤醒。当前单声道 USB 输出不是六路原始麦克风数据。

## 接口对应关系

| 用途 | 当前接口 | USB 标识 |
| --- | --- | --- |
| 阵列录音 | `hw:CARD=XFMDPV0018,DEV=0` | `2207:0001`，iflytek XFM-DP-V0.0.18 |
| 配套声卡播放 | `plughw:CARD=Device,DEV=0` | `0d8c:0012`，C-Media USB Audio Device |
| 华为 USB-C 耳机播放 | 当前系统默认输出，使用 `paplay` | ALSA 卡名 `HEADSET` |
| 主控板串口 | `/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0004-if00` | `1a86:55d4`，WCH USB Single Serial |

串口当前指向 `/dev/ttyACM0`，参数为 115200、8N1、无流控。使用上述稳定路径可避免串口编号变化。
声卡名称也应在重新连接或增减声卡后用 `arecord -l`、`aplay -l` 复核。
配套 C-Media 设备本次只枚举出播放接口，录音应选择 XFM-DP。

## 1. 录制一句话并回放

在 Ubuntu 终端执行，录音期间正常说话：

```bash
cd /home/jyfight/Project/rtabmap_slam
mkdir -p logs/audio
arecord -D hw:CARD=XFMDPV0018,DEV=0 -t wav -f S16_LE -r 16000 -c 1 -d 5 logs/audio/m260c_test.wav
paplay logs/audio/m260c_test.wav
```

重复录制会覆盖这个手动测试文件。`paplay` 使用系统默认输出，本次确认是华为 USB-C 耳机。
如默认输出后来改变，可明确指定本次识别到的耳机：

```bash
paplay --device=alsa_output.usb-Synaptics_HUAWEI_USB-C_HEADSET_0296B24a2212255324a24A24c12D1-00.analog-stereo logs/audio/m260c_test.wav
```

只有音箱或耳机实际接在配套 C-Media USB 声卡输出端时，才使用：

```bash
aplay -D plughw:CARD=Device,DEV=0 logs/audio/m260c_test.wav
```

指定 `CARD=Device` 会绕过系统默认耳机，把声音送到 C-Media 声卡。
播放采用 `plughw` 时，ALSA 会将 16 kHz 单声道录音转换到声卡支持的格式。

若录音听不到，先用已知正常的系统音频检查耳机：

```bash
paplay /usr/share/sounds/alsa/Front_Center.wav
```

本次用户确认能听到该英文提示音，但此前阵列录音的放大副本仍听不到内容，正在区分有效人声与底噪。

本次已保存的测试录音为：

- [input_5s.wav](../logs/audio/m260c_20261113_ly64lvma/input_5s.wav)
- [录音参数及信号统计](../logs/audio/m260c_20261113_ly64lvma/report.json)

## 2. 查询版本、监听唤醒及声源角度

已提供仅依赖 Python 标准库的工具：

```bash
cd /home/jyfight/Project/rtabmap_slam
python3 scripts/m260c_serial_monitor.py --seconds 60
```

工具会响应设备握手，查询固件版本，显示收到的有效协议帧；60 秒后退出，也可按 Ctrl+C 结束。
它只发送握手确认和版本查询，不包含修改唤醒词、刷固件、强制波束或机器人控制的命令。

本机账户最初不具备串口权限，本次已由用户在终端授权。若重新插拔后出现 `Permission denied`，再执行：

```bash
sudo setfacl -m u:jyfight:rw /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0004-if00
```

这是当前设备的临时访问权限。密码仅在本机终端输入。

监听期间使用设备实际配置的唤醒词。如果上报中含角度，工具会显示 `angles_deg`。
该数值是阵列坐标系下的方向，不是声源距离或地图坐标；接入机器人时还需确定阵列安装朝向。

可选日志参数：`--log logs/audio/m260c_my_session.jsonl`。日志文件须尚不存在，以免覆盖旧记录。

## 3. 当前唤醒无反应的处理

公开第三方接入资料称其设备的出厂词为“小微小微”，但这不能证明当前主控板采用相同设置。
本机说该词没有反应，实际唤醒词、阈值及板内唤醒资源仍需依据随货说明或商家配置确认。
版本查询成功说明串口链路已通，不能单独证明唤醒引擎正常。
后续诊断还读到未封装成协议帧的 `!` 和空格，现有资料不足以确定其含义。

在唤醒问题解决前，可以先使用上述录音入口采集语音样本。
识别“开始记录”等中文口令需要在 Ubuntu 上接入 ASR，再将文字映射为项目事件。
环境声音分类需要对应模型；它与语音识别、阵列声源方向是不同功能。
这次检查不代表任务书中的声学感知与多模态融合已完成。

## 检查依据

- [串口实测日志](../logs/audio/m260c_serial_check_20261113.jsonl)：握手和版本查询成功，`angle_events: 0`。
- [补充诊断日志](../logs/audio/m260c_serial_diagnostic_20261113.jsonl)：版本一致，无角度事件。
- 协议格式参照 [jt-z/m260c-voice-assistant](https://github.com/jt-z/m260c-voice-assistant)，这是第三方资料，不是本套设备的官方随货说明。
- 本地工具已检查分段收包、粘包、校验失败恢复、噪声字节处理、嵌套角度解析，以及完整帧不会被重复交付。
