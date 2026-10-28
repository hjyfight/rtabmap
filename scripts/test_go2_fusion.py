#!/usr/bin/env python3
"""Synthetic RGB-D + Livox test for the Go2 fusion launch."""
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import MapData
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]
W, H = 640, 480


def room_points():
    surfaces = []

    def plane(axis, value, bounds_a, bounds_b):
        other = [i for i in range(3) if i != axis]
        a, b = np.meshgrid(np.arange(*bounds_a, 0.08), np.arange(*bounds_b, 0.08))
        points = np.zeros((a.size, 3), dtype=np.float32)
        points[:, axis] = value
        points[:, other[0]] = a.ravel()
        points[:, other[1]] = b.ravel()
        surfaces.append(points)

    for x in (-4.0, 6.0):
        plane(0, x, (-3, 4), (-1, 2))
    for y in (-3.0, 4.0):
        plane(1, y, (-4, 6), (-1, 2))
    plane(2, -1.0, (-4, 6), (-3, 4))
    plane(0, 2.5, (0.8, 1.8), (-1, 0.6))
    plane(1, 0.8, (2.5, 3.5), (-1, 0.6))
    plane(2, 0.6, (2.5, 3.5), (0.8, 1.8))
    return np.concatenate(surfaces)


def textured_image():
    rng = np.random.default_rng(7)
    small = rng.integers(0, 256, (12, 16, 3), dtype=np.uint8)
    image = np.repeat(np.repeat(small, H // 12 + 1, axis=0), W // 16 + 1, axis=1)[:H, :W]
    return np.ascontiguousarray(image)


def make_image(stamp, frame, data, encoding):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.height, msg.width = data.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = msg.width * (data.shape[2] if data.ndim == 3 else data.dtype.itemsize)
    msg.data = data.tobytes()
    return msg


def make_camera_info(stamp, frame):
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.width, msg.height = W, H
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [525.0, 0.0, W / 2.0, 0.0, 525.0, H / 2.0, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [525.0, 0.0, W / 2.0, 0.0, 0.0, 525.0, H / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def main():
    (ROOT / 'logs').mkdir(exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix='go2-synthetic-', dir=ROOT / 'logs'))
    database = run / 'go2.db'
    stats = {'odometry_messages': 0, 'max_graph_nodes': 0, 'max_cloud_points': 0, 'sync_frames': 0}
    errors = []
    process = None
    node = None
    launch_exit_code = None
    rclpy.init()
    try:
        node = rclpy.create_node('go2_fusion_test')
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if any(ns == '/rtabmap' for _, ns in node.get_node_names_and_namespaces()):
            raise RuntimeError('Test domain already has /rtabmap nodes; choose another domain.')

        def receive_odom(msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                errors.append('Non-finite odometry')
                return
            stats['odometry_messages'] += 1

        def receive_graph(msg):
            stats['max_graph_nodes'] = max(stats['max_graph_nodes'], len(msg.graph.poses_id))

        def receive_cloud(msg):
            stats['max_cloud_points'] = max(stats['max_cloud_points'], msg.width * msg.height)

        node.create_subscription(Odometry, '/rtabmap/odom', receive_odom, qos_profile_sensor_data)
        node.create_subscription(MapData, '/rtabmap/mapData', receive_graph, qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/rtabmap/cloud_map', receive_cloud, qos_profile_sensor_data)

        scan_pub = node.create_publisher(PointCloud2, '/rtabmap_test/points', qos_profile_sensor_data)
        rgb_pub = node.create_publisher(Image, '/rtabmap_test/rgb', qos_profile_sensor_data)
        depth_pub = node.create_publisher(Image, '/rtabmap_test/depth', qos_profile_sensor_data)
        info_pub = node.create_publisher(CameraInfo, '/rtabmap_test/camera_info', qos_profile_sensor_data)

        with (run / 'nodes.log').open('w') as output:
            process = subprocess.Popen([
                'ros2', 'launch', str(ROOT / 'launch/go2_slam.launch.py'),
                'rviz:=false', 'publish_static_tf:=true',
                'lidar_topic:=/rtabmap_test/points', 'lidar_frame:=livox_frame',
                'frame_id:=base_link', 'rgb_topic:=/rtabmap_test/rgb',
                'depth_topic:=/rtabmap_test/depth',
                'camera_info_topic:=/rtabmap_test/camera_info',
                f'database_path:={database}',
            ], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 40
            while (scan_pub.get_subscription_count() < 2 or
                   rgb_pub.get_subscription_count() < 1 or
                   depth_pub.get_subscription_count() < 1 or
                   info_pub.get_subscription_count() < 1):
                if process.poll() is not None:
                    raise RuntimeError('Go2 SLAM launch exited before subscribing; see nodes.log')
                if time.monotonic() > deadline:
                    raise RuntimeError('DDS discovery timed out for Go2 fusion topics')
                rclpy.spin_once(node, timeout_sec=0.1)
            print('subscribers: scan=%d rgb=%d depth=%d info=%d' % (
                scan_pub.get_subscription_count(), rgb_pub.get_subscription_count(),
                depth_pub.get_subscription_count(), info_pub.get_subscription_count()), flush=True)

            world = room_points()
            image = textured_image()
            depth = np.full((H, W), 2.0, dtype=np.float32)
            stats['points_per_scan'] = len(world)
            for i in range(100):
                if process.poll() is not None:
                    raise RuntimeError('Go2 SLAM launch exited during mapping')
                x = max(0, min(i - 10, 80)) * 0.015
                y = 0.05 * math.sin(x * 2)
                points = world - np.array([x, y, 0], dtype=np.float32)
                stamp = node.get_clock().now().to_msg()
                header = Header(stamp=stamp, frame_id='livox_frame')
                cloud = point_cloud2.create_cloud_xyz32(header, points)
                cloud.is_dense = True
                scan_pub.publish(cloud)
                rgb_pub.publish(make_image(stamp, 'camera_link', image, 'rgb8'))
                depth_pub.publish(make_image(stamp, 'camera_link', depth, '32FC1'))
                info_pub.publish(make_camera_info(stamp, 'camera_link'))
                stats['sync_frames'] += 1
                end = time.monotonic() + 0.2
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)
                if i % 25 == 0:
                    print(f'Frame {i}/100: {stats}', flush=True)
            end = time.monotonic() + 3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            if stats['odometry_messages'] < 10:
                errors.append('Fewer than 10 valid ICP odometry messages')
            if stats['max_graph_nodes'] < 3:
                errors.append('Fewer than 3 SLAM graph nodes')
            if stats['max_cloud_points'] < 100:
                errors.append('No usable assembled map cloud')
    except Exception as exc:
        errors.append(str(exc))
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    errors.append('SLAM nodes failed to shut down within 20 seconds')
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            launch_exit_code = process.returncode
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    try:
        with sqlite3.connect(f'file:{database}?mode=ro', uri=True) as conn:
            stats['database_integrity'] = conn.execute('PRAGMA integrity_check').fetchone()[0]
            stats['saved_nodes'] = conn.execute('SELECT COUNT(*) FROM Node').fetchone()[0]
        if stats['database_integrity'] != 'ok' or stats['saved_nodes'] < 3:
            errors.append('Saved database failed integrity/node-count checks')
    except Exception as exc:
        errors.append(f'Database validation: {exc}')
    report = {
        'passed': not errors, 'scope': 'synthetic RGB-D + PointCloud2 -> ICP -> RTAB-Map -> SQLite',
        'physical_sensor_tested': False, 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
        'launch_exit_code': launch_exit_code, 'metrics': stats, 'errors': errors,
        'artifacts': str(run),
    }
    (run / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    (ROOT / 'logs/latest_go2_test.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
