# YOLOE 无提示语义建图

新增 YOLOE-11s-seg-pf，与原有 YOLOv8n-seg 并存。YOLOE 使用权重内置的
4,585 个词条自动识别，不需要输入文字类别列表，不调用 `set_classes()`，
也不下载文本编码器。内置词库扩大了识别范围，并不保证所有物体都能被正确识别。

## 两个独立入口

| 项目 | 原 YOLOv8 | 新 YOLOE |
| --- | --- | --- |
| Python 环境 | `.venv-semantic` | `.venv-yoloe` |
| Ultralytics | 原环境 8.3.40 | 8.4.198 |
| 默认权重 | `models/yolov8n-seg.pt` | `models/yoloe-11s-seg-pf.pt` |
| 启动命令 | `bash scripts/start_semantic_slam.sh` | `bash scripts/start_semantic_yoloe.sh` |
| 默认语义数据库 | `地图名.semantic.sqlite` | `地图名.yoloe.stable.semantic.sqlite` |

安装新环境和官方权重：

```bash
cd /home/jyfight/Project/rtabmap_slam
bash scripts/install_yoloe.sh
```

安装脚本只写入独立环境、下载缓存和新权重，不升级原 YOLOv8 环境。
固定 PyTorch 2.5.1+cpu、NumPy 1.24.4 和 OpenCV 4.10.0.84，兼容当前 ROS Humble
Python 3.10 / cv_bridge。权重从 Ultralytics 官方 v8.4.0 assets 下载，安装时核对
官方 SHA-256 `292bdf157a9ec7315f34b567cb93467c5043cd1889a1cc18abbfdeb88d7a948d`。

相机和雷达按原流程启动；结束上一轮建图后启动 YOLOE：

```bash
bash scripts/start_semantic_yoloe.sh
```

原启动命令仍默认使用 YOLOv8；两个入口共用 ROS 域锁，不允许同一域同时启动两套建图。
新增环境不会自动替换已经运行的 YOLOv8 节点。MITI 继续订阅原来的语义消息话题，
不需要重新配置面板。相机的对齐深度和外参检查也沿用原流程。

## 输出和类别

地图标签只显示物体类别；类别编号、置信度等信息不加到标签文字中。
分割掩膜、置信度仍在程序内部使用：掩膜筛选物体对应的深度像素，置信度参与多帧融合。
否则只有类别名而没有物体区域，无法可靠地把类别放到三维地图里。

MITI 和 Qt 都显示中文类别，单图命令也只输出中文名称。模型的 4,585 个类别与 RAM
项目配套中英文词表逐项核对，词表随程序安装，运行时不联网翻译。内部融合和数据库
使用统一后的英文类别键，发布 `/semantic/objects` 时转换成中文；不改变物体 ID、
深度坐标或回环锚点。无提示模式没有提示词输入框或目标文字列表。

单独识别一张本地图片，只在标准输出中打印去重后的类别名：

```bash
.venv-yoloe/bin/python3 scripts/predict_yoloe.py /path/to/image.jpg
```

没有检测结果时标准输出为空；模型诊断写入标准错误，不生成检测框图片。

参数在 `src/rtabmap_semantic/config/yoloe.yaml`：默认 CPU、2 线程、640 推理尺寸、
0.45 置信度。`classes: ['*']` 只是允许全部输出类别，不是文字提示；这里无需填写
待识别类别。阈值和实际速度应结合现场图片评估。

## 中文与稳定性设置

- 常见细分类合并：办公椅/电脑椅/转椅统一为“椅子”，电脑屏幕/阴极射线管屏幕
  统一为“显示器”，人物职业和年龄类别统一为“人”，地板细分类统一为“地面”。
  这一步发生在推理之后，没有缩减模型的识别词库。
- 明确使用跨类别重叠框抑制（IoU 0.5），减少同一区域的重复类别。
- 静态物体在最近 6 个已处理关键帧中出现至少 4 次才确认；重复发送同一关键帧不计数，
  未确认候选没有近期观测时会被清理，零散误检不会一直累积成永久标签。
- 已确认的静态标签保留在地图中，暂时漏检或移出画面不会让它闪烁。人、猫、狗需
  近期至少 2 次观测，位置使用最新观测，超过 3 秒未再次观测则移除，不写入静态地图。
- 只改显示语言可设置 `label_language: zh/en`；确认参数是 `min_observations: 4`、
  `confirmation_window: 6`、`dynamic_min_observations: 2`。

这些设置会减少低置信度结果，并延迟新标签出现。它们不能纠正持续的高置信度误识别。
单张图片命令只执行类别归一、中文转换和置信度筛选，不具备多帧确认。

稳定模式使用新的 `.yoloe.stable.semantic.sqlite`，保留旧 `.yoloe.semantic.sqlite`
和 YOLOv8 数据。加载同一 RTAB-Map 地图时，桥接节点可回放保存的 RGB-D 关键帧，
按新规则重新生成语义标签。原 YOLOv8 的默认阈值、确认行为、数据库指纹和存储格式保持兼容。

### 同一物体的多个名称

YOLOE 后端现在优先按三维位置与尺寸关联物体，允许同一物体在不同帧被识别成不同类别。
跨类别关联要求中心足够接近（最多 20 cm，随物体尺寸收紧）、加容差后的三维包围盒
IoU 至少 0.45，且不能把动态目标关联到静态物体。同帧分别检测到的对象保留独立身份。
已确认的标签只有在另一类别得到至少 4 个不同关键帧支持、累计分数超过当前类别 25%
时才更换名称，物体 ID 保持不变。旧记录在有充分几何证据时也可合并。

每次观测的类别会保存，旧记录可兼容读取。首次以新 YOLOE 后端读取旧语义库时，
自动创建同目录的 `.before_unique_objects.sqlite` 备份；YOLOv8 的默认格式和行为保持兼容。

MITI 对已有的重复记录另做显示归组：中心相距 8 cm 内、没有同帧独立观测证据的
记录显示一个名称；已成组的记录允许 12 cm 的退出阈值，避免位置噪声造成反复拆分。
组内采用多帧支持度选择名称，并用 25% 的切换余量减少跳变。原始记录仍保留。
这一步不把“电视”和“显示器”等名称全局视作同义词，也不会把地图里所有同类物体合并。
该修复先更新当前展示服务；已运行的建图节点继续工作，后端新关联逻辑在下次启动时使用。

本轮已在线核验用户指出的三组记录：`62/79`、`63/80`、`91/156/268` 各保留一个
显示标签，邻近的独立物体 `71/133/171` 仍显示。20 秒、79 次刷新中没有全量清空、
标签集合跳变或静态标签异常消失。报告为
`logs/yoloe_validation/duplicate_objects_validation.json` 和
`logs/miti/semantic_flicker_after.json`。这验证了记录去重与消息稳定性，不代表类别识别准确率。

现场相机图片已验证能输出类别，但也出现了把人物识别成 `composer`（作曲家）、
把地面识别成 `dance floor`（舞池）等情况，部分错误结果置信度仍很高。
因此词库更大不等于识别更准确，提高置信度或多帧确认也无法消除持续误识别。
现在这张图片的默认单图输出为“人、地面、显示器、椅子”；这是筛选和类别归一的结果，
不是准确率评估。没有为了这张图片增加类别黑名单或文字提示。

高级启动参数仍可传给新入口，例如 `semantic_device:=cpu`、
`semantic_config:=/absolute/path/custom.yaml`；`YOLOE_MODEL_PATH` 可指定其他兼容的
无提示分割权重。该后端会拒绝没有内置词库的文字/视觉提示模型。

## 实现与检查

`YoloDetector` 增加 `yoloe_pf` 后端，输出仍为类别、置信度和原图尺寸的二值掩膜。
深度反投影、关键帧锚点、多帧确认、回环后的重投影及地图显示继续复用。
模型后端和权重指纹进入 YOLOE 语义数据库配置；YOLOv8 的旧配置指纹保持兼容。

```bash
# 独立 ROS 域 94：真实 YOLOE、原始/压缩关键帧、深度定位、多帧融合和保存恢复
bash scripts/test_yoloe.sh
# 独立 ROS 域 93：原 YOLOv8 回归检查
bash scripts/test_semantic.sh --real-yolo
```

上述两套检查均已通过；YOLOE 的真实权重测试覆盖了分割掩膜、深度投影、
中文输出、四帧确认、回环重投影、语义数据库保存恢复和地图重置，报告位于
`logs/semantic_validation/ros_real_yoloe.json`。测试在独立 ROS 域运行，
没有切换当前正在使用的建图后端。

本轮在线更新只重新加载了 YOLOE 语义节点和关键帧桥接节点。已核对
`/semantic/objects`、`/miti/semantic_markers` 的中文标签及当前参数，
`/icp_odom` 和 `/rgbd_image` 仍持续发布；在线报告为
`logs/yoloe_validation/live_chinese_stability.json`。

官方接口与权重说明：https://docs.ultralytics.com/models/yoloe/

词表来源、校验值及 Apache-2.0 许可证见
`src/rtabmap_semantic/third_party/recognize_anything/`。
