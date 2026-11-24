# Orin 3001 研究模型部署与测速（2026-11-22）

目标为 `qiyuan@192.168.1.103:/home/qiyuan/Project/rtabmap_slam`。
3001 的 RGB/NIR、ODIN-Swin／PTv3 室内对照、AVSegFormer 推理环境和权重已部署。
本记录验证模型后端；测速使用已保存的实机输入。

## 环境

Orin 64 GB，JetPack 6.2 / L4T 36.4.3，MAXN。使用原 NVIDIA
`torch 2.5.0a0+872d972e41.nv24.08`、CUDA 12.6 和已有 CUDA torchvision。
`.venv-indoor-research` 与 `.venv-fusion-research` 继承该 GPU 基础环境。

新增 Cumm 0.7.11、SpConv 2.3.8、torch-scatter 2.1.2、timm 0.6.13，
以及 resampy、Numba／llvmlite 等音频依赖。Cumm／SpConv 针对 SM87 在本机生成和编译；
Torch Scatter 针对本机 NVIDIA PyTorch ABI 编译。实际 CUDA 稀疏卷积和 scatter_mean
检查均通过。ROS 特征消息／特征节点、语义消息／语义节点四个包构建通过，消息导入通过。

NVIDIA 此版 PyTorch 提供安全允许列表 API，但没有 stable 版的 `safe_globals`
上下文管理器。PTv3 适配器兼容两种 API，保持 `weights_only=True`、完整 SHA-256
及严格参数加载；正常退出及异常退出时恢复原允许列表。Orin 的兼容分支和工作站的
原生分支均检查通过。

## 权重

工作站已下载的完整官方权重经校验后传到 Orin，并在 Orin 再核对 SHA-256。
模型源码也同步，包括 AVSegFormer、Pointcept、PMF 和固定版本的 ODIN／Detectron2。
PMF 权重供旧室内／道路对照入口使用；当前默认室内流程为 ODIN-Swin＋PTv3。

| 文件 | MB（十进制） | SHA-256 |
| --- | ---: | --- |
| `models/odin_scannet_swin_semantic.pth.download` | 1421.6 | `fc4e7ffd4507ea334ed7d96c5be146b786fd18c4abd42cb39138bd39c71856f5` |
| `models/ptv3_scannet_best.pth` | 554.6 | `40206376ff2f83f48e4d1bc27d5c5d96be7c87c5d11eb45fe7be501959040e7f` |
| `models/avsegformer_res50_avss.pth` | 604.3 | `a32cf9f47b43c24e96da7c221640a53f97f435c0f29e54ae2581f85a7600c87f` |
| `models/pmf_semantickitti.download` | 146.0 | `024860251e7df936616fc468cc002765010ae64a290b0f481b1ffc2f5963f246` |
| `models/yolov8s-seg.pt` | 23.9 | `0bac0770b55e5eb5b76a61bc535673288dcec36c2bc0cd25ee0d584c632f3413` |

## 测量

各流程在独立进程内顺序执行，`cuda:0`、2 个 CPU 推理线程，float32 张量及框架默认
TF32 策略。每项预热 2 轮后测 10 轮，计时前后执行 CUDA synchronize。
下表首列推理耗时不包含初始化；初始化列包括导入模型、权重校验和加载。

| 流程 | 中位耗时（秒） | P95（秒） | 模型初始化（秒） |
| --- | ---: | ---: | ---: |
| RGB/NIR YOLOv8s 分割＋RGB基线 | 0.247 | 0.269 | 20.71 |
| PTv3 单独点云推理 | 0.205 | 0.211 | 39.80 |
| ODIN-Swin＋PTv3同点对照 | 2.478 | 2.502 | 44.24 |
| AVSegFormer（10帧音视频） | 0.559 | 0.562 | 4.92 |

输入与边界：

- RGB/NIR：保存的 240×424 实机图像，网络输入 640，NIR 投影有效覆盖 49.6%；
  测量配准后的输入至共享骨干融合、同帧 RGB 基线、解码、真实特征归档和图像生成。
- PTv3：同一保存场景中的 2624 个有色 LiDAR 点及预先生成的法向；测量体素采样、
  完整 PTv3 与原点标签解码。
- ODIN-Swin＋PTv3：5 组有位姿的实机 RGB-D 上下文、2624 个有色 LiDAR 查询点，
  当前有效深度查询 95849 点；包括场景准备、ODIN、同点 PTv3 对照、真实特征与图像生成。
- AVSegFormer：10×3×224×224 归一化视频与 10×1×96×64 真正采集后保存的 log-Mel。
  原始 PCM 未包含在该回放归档中；测量十帧推理和最后一帧显示输出，耗时为每段十帧。

文件读取、模型初始化、实时 ROS／网络传输、NPZ 压缩写盘和网页刷新另计。
这些是已保存实机样本的模型回放耗时，准确率未评估，也不是整套网页的实时帧率。

## 复测

在 Orin 项目目录执行：

```bash
.venv-indoor-research/bin/python3 scripts/benchmark_orin_research.py \
  --warmup 2 --iterations 10 --output logs/orin_research_benchmark/retest
```

环境重建使用 `bash scripts/install_orin_research.sh`，需要外网或临时代理。
ARM64 ROS 构建入口为 `bash scripts/build_fourmodal_features.sh` 和
`bash scripts/build_semantic.sh --headless`。

Orin 原始结果：`logs/orin_research_benchmark/20261122_3001/`。
工作站结果：`logs/orin_research_deploy/20261122_3001/benchmark/`，包含 JSON、CSV
与模型输出 JPEG。模型清单、权重校验和安装日志保存在同目录的上一级。
初始化 API 失败记录已单独归档；最终 `report.json` 中四项均为 passed，failures 为空。
