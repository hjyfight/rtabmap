"""Offline display names and conservative category grouping for YOLOE mapping.

Grouping happens after inference. It never changes the model vocabulary or adds
prompts. Fine-grained person roles are mapped to person, not inferred identities.
"""
from .class_names_zh import CLASS_NAMES_ZH

YOLOE_LABEL_POLICY_VERSION = 1

_GROUPS = {
    'chair': ('computer chair', 'office chair', 'swivel chair'),
    'desk': ('computer desk', 'office desk', 'writing desk'),
    'monitor': ('computer monitor', 'computer screen', 'crt screen'),
    'keyboard': ('laptop keyboard',),
    'sofa': ('couch',),
    'floor': ('dance floor', 'hardwood floor', 'kitchen floor', 'tile flooring', 'wood floor'),
    'person': (
        'man', 'woman', 'boy', 'girl', 'child', 'baby', 'toddler', 'teenager',
        'adult', 'people', 'airman', 'artist', 'athlete', 'business woman',
        'businessman', 'chef', 'commander', 'composer', 'construction worker',
        'craftsman', 'dancer', 'doctor', 'driver', 'engineer', 'farmer',
        'fireman', 'firefighter', 'fisherman', 'football player', 'gardener',
        'groom', 'bride', 'hiker', 'jogger', 'manager', 'mechanic', 'musician',
        'nurse', 'painter', 'passenger', 'pedestrian', 'photographer', 'pilot',
        'player', 'plumber', 'police', 'police officer', 'politician',
        'researcher', 'rider', 'runner', 'sailor', 'scientist', 'singer',
        'skater', 'skier', 'soldier', 'student', 'surfer', 'swimmer',
        'teacher', 'tourist', 'tv personality', 'waiter', 'waitress', 'worker',
    ),
}
_ALIASES = {alias: canonical for canonical, aliases in _GROUPS.items() for alias in aliases}
_DISPLAY_OVERRIDES = {
    'desk': '办公桌', 'monitor': '显示器', 'floor': '地面', 'sofa': '沙发',
    'laptop': '笔记本电脑', 'bus': '公交车', 'mouse': '鼠标', 'remote': '遥控器',
    'tv': '电视', 'dvd': '光盘', 'ktv': '卡拉歌厅', 'wii': '游戏机',
    'musical keyboard': '电子琴', 'extension cord': '延长电源线',
    'file cabinet': '文件柜', 'storage box': '收纳箱',
}


def canonical_label(label):
    key = label.strip().lower()
    return _ALIASES.get(key, key)


def chinese_label(label):
    key = label.strip().lower()
    # Custom Chinese labels pass through; unknown custom vocabularies stay inspectable.
    return _DISPLAY_OVERRIDES.get(key, CLASS_NAMES_ZH.get(key, label))
