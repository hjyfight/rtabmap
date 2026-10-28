#!/usr/bin/env python3
"""Read camera dimensions, RGB-D pairs and active driver parameters without changing nodes."""

import argparse
from collections import Counter, deque
import datetime
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import CameraInfo, Image
from rtabmap_msgs.msg import Info, RGBDImage


TOPICS = {
    'rgb': ('/camera/color/image_raw', Image),
    'aligned_depth': ('/camera/aligned_depth_to_color/image_raw', Image),
    'raw_depth': ('/camera/depth/image_rect_raw', Image),
    'color_info': ('/camera/color/camera_info', CameraInfo),
    'depth_info': ('/camera/depth/camera_info', CameraInfo),
    'rgbd': ('/rgbd_image', RGBDImage),
    'slam_info': ('/info', Info),
}
PARAMETERS = [
    'rgb_camera.profile', 'depth_module.profile', 'align_depth.enable',
    'rgb_camera.color_profile', 'depth_module.depth_profile',
    'enable_color', 'enable_depth', 'enable_sync',
    'rgb_camera.enable_auto_exposure', 'rgb_camera.exposure', 'rgb_camera.gain',
    'color_width', 'color_height', 'color_fps',
    'depth_width', 'depth_height', 'depth_fps', 'align_depth',
]


def image_metadata(message):
    return {
        'width': int(message.width), 'height': int(message.height),
        'encoding': getattr(message, 'encoding', None),
        'frame_id': message.header.frame_id,
        'stamp': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
    }


def message_metadata(message):
    if isinstance(message, RGBDImage):
        return {'rgb': image_metadata(message.rgb), 'depth': image_metadata(message.depth),
                'rgb_camera_info': image_metadata(message.rgb_camera_info),
                'depth_camera_info': image_metadata(message.depth_camera_info)}
    if isinstance(message, Info):
        return {'ref_id': int(message.ref_id), 'loop_closure_id': int(message.loop_closure_id)}
    return image_metadata(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12)
    parser.add_argument('--camera-node', default='/camera/camera')
    parser.add_argument('--output', type=Path, default=Path('/tmp/rgbd_input_check.json'))
    args, ros_args = parser.parse_known_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/rgbd_input_check_ros_logs')
    rclpy.init(args=ros_args)
    node = Node('rgbd_input_readonly_check')
    counts = Counter()
    samples = {name: deque(maxlen=4) for name in TOPICS}
    sizes = {name: Counter() for name in TOPICS}
    subscriptions = []
    for name, (topic, message_type) in TOPICS.items():
        def callback(message, key=name):
            metadata = message_metadata(message)
            counts[key] += 1
            samples[key].append(metadata)
            if key == 'rgbd':
                pair = (metadata['rgb']['width'], metadata['rgb']['height'],
                        metadata['depth']['width'], metadata['depth']['height'])
                sizes[key][str(pair)] += 1
            elif key != 'slam_info':
                sizes[key][f'{metadata["width"]}x{metadata["height"]}'] += 1
        subscriptions.append(node.create_subscription(
            message_type, topic, callback, qos_profile_sensor_data))

    client = node.create_client(GetParameters, args.camera_node.rstrip('/') + '/get_parameters')
    request = GetParameters.Request(names=PARAMETERS)
    future = None
    start = time.monotonic()
    print(f'只读采样 {args.seconds:g} 秒：保持相机驱动运行即可，不要求 /rtabmap 存在。', flush=True)
    try:
        while time.monotonic() - start < args.seconds:
            if future is None and client.service_is_ready():
                future = client.call_async(request)
            rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.monotonic() - start
        result = {
            'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
            'elapsed_seconds': elapsed,
            'environment': {key: os.environ.get(key) for key in
                            ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI')},
            'nodes': sorted(namespace.rstrip('/') + '/' + name
                            for name, namespace in node.get_node_names_and_namespaces()),
            'camera_node': args.camera_node,
            'camera_parameters': {}, 'parameter_status': '服务未发现或未响应',
            'topics': {},
        }
        if future is not None and future.done():
            if future.exception() is not None:
                result['parameter_status'] = str(future.exception())
            else:
                response = future.result()
                result['parameter_status'] = '已读取；null 表示驱动未声明该参数'
                result['camera_parameters'] = {
                    name: parameter_value_to_python(value) if value.type else None
                    for name, value in zip(PARAMETERS, response.values)}
        for name, (topic, _) in TOPICS.items():
            publishers = [
                {'node': publisher.node_namespace.rstrip('/') + '/' + publisher.node_name,
                 'gid': bytes(publisher.endpoint_gid).hex(),
                 'reliability': str(publisher.qos_profile.reliability)}
                for publisher in node.get_publishers_info_by_topic(topic)]
            item = {'topic': topic, 'count': counts[name], 'hz': counts[name] / elapsed,
                    'sizes': dict(sizes[name]), 'last_samples': list(samples[name]),
                    'publishers': publishers}
            result['topics'][name] = item
            print(f'{name:15} {item["hz"]:5.2f} Hz | 尺寸:{item["sizes"]} | 发布端:{len(publishers)}')
        print('相机实际参数:', json.dumps(result['camera_parameters'], ensure_ascii=False))
        print('参数读取状态:', result['parameter_status'])
        for name in ('rgb', 'aligned_depth', 'color_info', 'rgbd'):
            if samples[name]:
                print(f'{name} 最新元数据:', json.dumps(samples[name][-1], ensure_ascii=False))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print('结果保存至:', args.output)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
