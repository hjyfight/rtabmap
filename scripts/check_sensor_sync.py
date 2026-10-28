#!/usr/bin/env python3
"""Read-only sensor timing audit. Run in the same host environment as the drivers."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from rtabmap_msgs.msg import RGBDImage, Info

TOPICS = {
    'rgb': ('/camera/color/image_raw', Image),
    'depth': ('/camera/aligned_depth_to_color/image_raw', Image),
    'camera_info': ('/camera/color/camera_info', CameraInfo),
    'lidar': ('/livox/lidar', PointCloud2),
    'rgbd_main': ('/rgbd_image', RGBDImage),
    'rgbd_probe': ('/rgbd_probe/rgbd_image', RGBDImage),
    'slam_info': ('/info', Info),
}


def analyze(samples, endpoints):
    notes = []
    counts = {k: len(v) for k, v in samples.items()}
    if not any(counts.values()):
        return ['所有输入均未采到消息：不能确认传感器正常，也不能确认是同步参数问题。请检查驱动终端、ROS_DOMAIN_ID及DDS通信。']
    for name in ('rgb', 'depth', 'camera_info'):
        if not counts[name]:
            notes.append(f'{name} 未收到消息，RGB-D 三路条件不齐；检查对应话题、驱动输出及QoS。')
    if all(counts[k] for k in ('rgb', 'depth', 'camera_info')):
        # Each tuple holds host arrival time and header stamp. Compare samples from
        # approximately the same host arrival time, not arbitrary first messages.
        for a, b in [('rgb', 'depth'), ('rgb', 'camera_info'), ('rgb', 'lidar')]:
            if not counts[b]:
                continue
            deltas = []
            for arrival, stamp in samples[a]:
                peer_arrival, peer_stamp = min(samples[b], key=lambda s: abs(s[0] - arrival))
                if abs(peer_arrival - arrival) < 0.5:
                    deltas.append(stamp - peer_stamp)
            if deltas:
                delta = statistics.median(deltas)
                notes.append(f'{a} 与 {b} 的近同时到达消息，时间戳差中位数={delta:.6f}秒。')
                if abs(delta) > 10:
                    notes.append(f'{a}/{b} 存在很大的时间戳差，当前同步配置无法可靠配对；须检查两端时基。')
        if counts['rgbd_main'] and counts['slam_info']:
            notes.append('已实测 /rgbd_image 与 /info 持续输出：RGB-D 同步和建图后端处理链路已接通；这不等于地图质量或回环已验证。')
        elif counts['rgbd_main']:
            notes.append('已实测 /rgbd_image 输出，但未收到 /info；仍需检查建图后端。')
        else:
            notes.append('三路相机数据都有输入，但未收到 /rgbd_image；仍需检查同步节点。')
    if not counts['lidar']:
        notes.append('雷达输入未收到，后端也无法融合。')
    return notes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12)
    parser.add_argument('--require-slam', action='store_true',
                        help='要求正式融合节点在整个采样期间保持运行')
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'logs/sensor_sync_check.json')
    args, ros_args = parser.parse_known_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    rclpy.init(args=ros_args)
    node = Node('sensor_sync_readonly_check')
    samples = {k: deque(maxlen=5000) for k in TOPICS}
    subs = []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    for name, (topic, msg_type) in TOPICS.items():
        def callback(msg, key=name):
            samples[key].append((time.time(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
        subs.append(node.create_subscription(msg_type, topic, callback, qos))
    def graph_nodes():
        return sorted(namespace.rstrip('/') + '/' + name
                      for name, namespace in node.get_node_names_and_namespaces())
    required = {'/rgbd_sync', '/rtabmap', '/icp_odometry'}
    if args.require_slam:
        discovery_end = time.monotonic() + 3
        while time.monotonic() < discovery_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        missing = required - set(graph_nodes())
        if missing:
            print('未发现正式融合节点：' + ', '.join(sorted(missing)))
            print('请保持驱动和融合launch运行，在另一个终端执行本检查。旧结果未覆盖。')
            node.destroy_node()
            rclpy.shutdown()
            return
        for values in samples.values():
            values.clear()
    start_nodes = graph_nodes()
    print(f'正在只读采样 {args.seconds:g} 秒，请保持雷达和相机驱动运行……', flush=True)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    elapsed = time.monotonic() - start
    endpoints = {}
    for key, (topic, _) in TOPICS.items():
        endpoints[key] = [{'node': p.node_namespace.rstrip('/') + '/' + p.node_name,
                           'reliability': str(p.qos_profile.reliability)}
                          for p in node.get_publishers_info_by_topic(topic)]
    end_nodes = graph_nodes()
    result = {'elapsed_seconds': elapsed,
        'environment': {key: os.environ.get(key) for key in
                        ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI')},
        'nodes_at_start': start_nodes, 'nodes': end_nodes,
        'topics': {}, 'notes': analyze(samples, endpoints)}
    if args.require_slam and required - set(end_nodes):
        result['notes'].insert(0, '正式融合节点在检查结束时不完整，本次不能作为持续运行链路的同步结论。')
    for key, (topic, _) in TOPICS.items():
        values = samples[key]
        result['topics'][key] = dict(topic=topic, count=len(values),
                                     received_hz=len(values) / max(elapsed, 0.001),
                                     publishers=endpoints[key],
                                     first_samples=list(values)[:5], last_samples=list(values)[-5:])
        result['topics'][key]['subscribers'] = [
            {'node': p.node_namespace.rstrip('/') + '/' + p.node_name,
             'reliability': str(p.qos_profile.reliability)}
            for p in node.get_subscriptions_info_by_topic(topic)]
        print(f'{key:12} {len(values) / max(elapsed, 0.001):6.2f} Hz | {topic} | 发布端{len(endpoints[key])}')
    for note in result['notes']:
        print('- ' + note)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print('诊断结果保存至：' + str(args.output.resolve()))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
