#!/usr/bin/env python3
"""Validate the observed aligned RGB-D input before starting the fusion launch."""

import json
from pathlib import Path
import sys


def verify(data):
    running = set(data.get('nodes', [])) & {'/rtabmap', '/rgbd_sync', '/icp_odometry'}
    if running:
        raise ValueError('已有建图节点运行，请先在旧建图终端 Ctrl+C：' + ', '.join(sorted(running)))
    topics = data['topics']
    names = ('rgb', 'aligned_depth', 'color_info')
    for name in names:
        item = topics[name]
        if item['count'] < 3 or not item['last_samples']:
            raise ValueError(f'{name} 数据不足，当前输入无法确认稳定。')
        if len(item['publishers']) != 1:
            raise ValueError(f'{name} 发布端数量为 {len(item["publishers"])}，需要确认只有一个相机驱动实例。')
        if len(item['sizes']) != 1:
            raise ValueError(f'{name} 尺寸在采样期间变化：{item["sizes"]}')
    rgb = topics['rgb']['last_samples'][-1]
    shape = (rgb['width'], rgb['height'])
    frame = rgb['frame_id']
    if min(shape) <= 0 or not frame:
        raise ValueError('RGB 图像尺寸或坐标系无效。')
    for name in names:
        for sample in topics[name]['last_samples']:
            observed = (sample['width'], sample['height'])
            if observed != shape:
                raise ValueError(f'RGB 为 {shape}，{name} 为 {observed}；拒绝启动以避免尺寸断言。')
            if sample['frame_id'] != frame:
                raise ValueError(f'{name} 坐标系为 {sample["frame_id"]}，与 RGB 的 {frame} 不一致。')
    common = set(sample['stamp'] for sample in topics['rgb']['last_samples'])
    for name in ('aligned_depth', 'color_info'):
        common &= {sample['stamp'] for sample in topics[name]['last_samples']}
    if not common:
        raise ValueError('未找到时间戳一致的 RGB、对齐深度和内参消息。')
    return shape


def main():
    try:
        shape = verify(json.loads(Path(sys.argv[1]).read_text()))
    except (ValueError, KeyError, OSError, IndexError) as error:
        print('输入检查未通过：' + str(error), file=sys.stderr)
        sys.exit(1)
    print(f'输入检查通过：RGB、对齐深度、内参均为 {shape[0]}x{shape[1]}，坐标系与时间戳一致。')


if __name__ == '__main__':
    main()
