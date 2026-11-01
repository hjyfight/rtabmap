#ifndef RTABMAP_VIZ_SEMANTIC_LABELS_H
#define RTABMAP_VIZ_SEMANTIC_LABELS_H

#include <QString>
#include <cstdint>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>

namespace rtabmap_viz {

// Presentation only. Model class names, IDs and confidence remain in the messages/database.
inline std::string semanticClassNameChinese(const std::string & label)
{
  static const std::map<std::string, std::string> names = {
    {"person", "人"}, {"bicycle", "自行车"}, {"car", "汽车"},
    {"motorcycle", "摩托车"}, {"airplane", "飞机"}, {"bus", "公交车"},
    {"train", "火车"}, {"truck", "卡车"}, {"boat", "船"},
    {"traffic light", "交通信号灯"}, {"fire hydrant", "消防栓"},
    {"stop sign", "停车标志"}, {"parking meter", "停车计时器"}, {"bench", "长椅"},
    {"bird", "鸟"}, {"cat", "猫"}, {"dog", "狗"}, {"horse", "马"},
    {"sheep", "羊"}, {"cow", "牛"}, {"elephant", "大象"}, {"bear", "熊"},
    {"zebra", "斑马"}, {"giraffe", "长颈鹿"}, {"backpack", "背包"},
    {"umbrella", "雨伞"}, {"handbag", "手提包"}, {"tie", "领带"},
    {"suitcase", "行李箱"}, {"frisbee", "飞盘"}, {"skis", "滑雪板"},
    {"snowboard", "单板滑雪板"}, {"sports ball", "球"}, {"kite", "风筝"},
    {"baseball bat", "棒球棒"}, {"baseball glove", "棒球手套"},
    {"skateboard", "滑板"}, {"surfboard", "冲浪板"}, {"tennis racket", "网球拍"},
    {"bottle", "瓶子"}, {"wine glass", "酒杯"}, {"cup", "杯子"},
    {"fork", "叉子"}, {"knife", "刀"}, {"spoon", "勺子"}, {"bowl", "碗"},
    {"banana", "香蕉"}, {"apple", "苹果"}, {"sandwich", "三明治"},
    {"orange", "橙子"}, {"broccoli", "西兰花"}, {"carrot", "胡萝卜"},
    {"hot dog", "热狗"}, {"pizza", "披萨"}, {"donut", "甜甜圈"}, {"cake", "蛋糕"},
    {"chair", "椅子"}, {"couch", "沙发"}, {"potted plant", "盆栽"},
    {"bed", "床"}, {"dining table", "餐桌"}, {"toilet", "马桶"},
    {"tv", "电视"}, {"laptop", "笔记本电脑"}, {"mouse", "鼠标"},
    {"remote", "遥控器"}, {"keyboard", "键盘"}, {"cell phone", "手机"},
    {"microwave", "微波炉"}, {"oven", "烤箱"}, {"toaster", "烤面包机"},
    {"sink", "水槽"}, {"refrigerator", "冰箱"}, {"book", "书"},
    {"clock", "时钟"}, {"vase", "花瓶"}, {"scissors", "剪刀"},
    {"teddy bear", "玩具熊"}, {"hair drier", "吹风机"}, {"toothbrush", "牙刷"}
  };
  const auto found = names.find(label);
  if (found != names.end()) return found->second;
  // Preserve Chinese names from custom models. Unmapped names use a Chinese display fallback.
  for (const auto character : QString::fromUtf8(label.c_str())) {
    if (character.unicode() >= 0x3400 && character.unicode() <= 0x9fff) return label;
  }
  return "物体";
}

inline std::string semanticDisplayLabel(const std::string & name, int64_t id,
                                       double confidence, bool showDetails = false)
{
  std::ostringstream label;
  label << semanticClassNameChinese(name);
  // Keep detailed formatting available without exposing these fields by default.
  if (showDetails) label << " #" << id << " " << std::fixed << std::setprecision(2) << confidence;
  return label.str();
}
}
#endif
