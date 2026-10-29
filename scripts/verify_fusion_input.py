#!/usr/bin/env python3
"""Validate the observed aligned RGB-D input before starting the fusion launch."""

import argparse
import json
from pathlib import Path
import sys


def verify(data, guarded=False):
    # Failed launches may leave only preprocessing nodes alive. Two independent
    # IMU filters can publish different yaw estimates for the exact same stamp.
    running = set(data.get('nodes', [])) & {
        '/rtabmap', '/rgbd_sync', '/icp_odometry', '/rtabmap_viz',
        '/imu_filter_madgwick', '/imu_to_tf', '/livox_pointcloud_time',
        '/lidar_deskewing', '/loop_closure_diagnostics',
        '/semantic_mapper', '/semantic_keyframe_bridge', '/rgbd_input_guard',
    }
    if running:
        raise ValueError('已有建图或遗留处理节点运行，请先结束旧建图进程，避免重复发布 IMU/点云：' + ', '.join(sorted(running)))
    topics = data['topics']
    names = ('rgb', 'aligned_depth', 'color_info')
    for name in names:
        item = topics[name]
        if item['count'] < 3 or not item['last_samples']:
            raise ValueError(f'{name} 数据不足，当前输入无法确认稳定。')
        if len(item['publishers']) != 1:
            raise ValueError(f'{name} 发布端数量为 {len(item["publishers"])}，需要确认只有一个相机驱动实例。')
        if len(item['sizes']) != 1 and not (guarded and name == 'aligned_depth'):
            raise ValueError(f'{name} 尺寸在采样期间变化：{item["sizes"]}')
    rgb = topics['rgb']['last_samples'][-1]
    shape = (rgb['width'], rgb['height'])
    frame = rgb['frame_id']
    if min(shape) <= 0 or not frame:
        raise ValueError('RGB 图像尺寸或坐标系无效。')
    valid_samples = {}
    for name in names:
        valid_samples[name] = []
        for sample in topics[name]['last_samples']:
            observed = (sample['width'], sample['height'])
            if guarded and name == 'aligned_depth' and (
                    observed != shape or sample['frame_id'] != frame or
                    sample.get('encoding') not in ('16UC1', '32FC1')):
                continue
            if observed != shape:
                raise ValueError(f'RGB 为 {shape}，{name} 为 {observed}；拒绝启动以避免尺寸断言。')
            if sample['frame_id'] != frame:
                raise ValueError(f'{name} 坐标系为 {sample["frame_id"]}，与 RGB 的 {frame} 不一致。')
            valid_samples[name].append(sample)
    common = set(sample['stamp'] for sample in valid_samples['rgb'])
    for name in ('aligned_depth', 'color_info'):
        common &= {sample['stamp'] for sample in valid_samples[name]}
    if guarded and len(common) < 3:
        raise ValueError('未找到至少 3 组尺寸、坐标系和时间戳一致的 RGB、对齐深度与内参。')
    if not common:
        raise ValueError('未找到时间戳一致的 RGB、对齐深度和内参消息。')
    return shape


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sample', type=Path)
    parser.add_argument('--guarded', action='store_true',
                        help='Only for launches with a continuous RGB-D validator: require valid pairs, tolerate rejected depth frames.')
    args = parser.parse_args()
    try:
        data = json.loads(args.sample.read_text())
        shape = verify(data, guarded=args.guarded)
    except (ValueError, KeyError, OSError, IndexError) as error:
        print('输入检查未通过：' + str(error), file=sys.stderr)
        sys.exit(1)
    if args.guarded:
        print(f'输入检查通过：已采到至少 3 组有效的 {shape[0]}x{shape[1]} RGB-D；运行时继续逐帧校验。')
        if len(data['topics']['aligned_depth']['sizes']) != 1:
            print('相机警告：对齐深度仍混入其他尺寸；运行时会丢弃无效帧，不进行缩放。', file=sys.stderr)
    else:
        print(f'输入检查通过：RGB、对齐深度、内参均为 {shape[0]}x{shape[1]}，坐标系与时间戳一致。')


if __name__ == '__main__':
    main()
