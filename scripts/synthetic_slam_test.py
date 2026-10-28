#!/usr/bin/env python3
"""Exercise real ICP/SLAM nodes with a synthetic room; require saved map data."""
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
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def room_points():
    """Sample fixed walls/floor and an asymmetric box, in world coordinates."""
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


def main():
    root = Path(__file__).resolve().parents[1]
    (root / 'logs').mkdir(exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix='synthetic-', dir=root / 'logs'))
    database = run / 'map.db'
    stats = {'odometry_messages': 0, 'max_graph_nodes': 0, 'max_cloud_points': 0}
    odometry_positions = []
    errors = []
    process = None
    node = None
    launch_exit_code = None
    rclpy.init()
    try:
        node = rclpy.create_node('rtabmap_environment_test')
        # Refuse to mix this test with another SLAM run in the same test domain.
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if any(ns == '/rtabmap' for _, ns in node.get_node_names_and_namespaces()):
            raise RuntimeError('Test domain already has /rtabmap nodes; choose RTABMAP_TEST_DOMAIN_ID.')

        def receive_odom(msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                errors.append('Non-finite odometry')
                return
            stats['odometry_messages'] += 1
            odometry_positions.append((p.x, p.y, p.z))

        def receive_graph(msg):
            stats['max_graph_nodes'] = max(stats['max_graph_nodes'], len(msg.graph.poses_id))

        def receive_cloud(msg):
            stats['max_cloud_points'] = max(stats['max_cloud_points'], msg.width * msg.height)

        subscriptions = [
            node.create_subscription(Odometry, '/rtabmap/odom', receive_odom, qos_profile_sensor_data),
            node.create_subscription(MapData, '/rtabmap/mapData', receive_graph, qos_profile_sensor_data),
            node.create_subscription(PointCloud2, '/rtabmap/cloud_map', receive_cloud, qos_profile_sensor_data),
        ]
        publisher = node.create_publisher(PointCloud2, '/rtabmap_test/points', qos_profile_sensor_data)
        with (run / 'nodes.log').open('w') as output:
            process = subprocess.Popen([
                'ros2', 'launch', str(root / 'launch/livox_slam.launch.py'),
                'rviz:=false', 'lidar_topic:=/rtabmap_test/points',
                f'database_path:={database}',
            ], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 30
            while publisher.get_subscription_count() < 2:
                if process.poll() is not None:
                    raise RuntimeError('SLAM launch exited before subscribing; see nodes.log')
                if time.monotonic() > deadline:
                    raise RuntimeError('DDS discovery timed out; expected ICP and SLAM cloud subscribers')
                rclpy.spin_once(node, timeout_sec=0.1)
            print(f'Input subscribers: {publisher.get_subscription_count()}; publishing synthetic room', flush=True)
            world = room_points()
            stats['points_per_scan'] = len(world)
            # Initial stationary frames, then 1.2 m of slow forward travel.
            # Odometry must be computed by ICP; no ground-truth odometry is sent.
            for i in range(100):
                if process.poll() is not None:
                    raise RuntimeError('SLAM launch exited during mapping')
                x = max(0, min(i - 10, 80)) * 0.015
                y = 0.05 * math.sin(x * 2)
                points = world - np.array([x, y, 0], dtype=np.float32)
                header = Header(stamp=node.get_clock().now().to_msg(), frame_id='livox_frame')
                publisher.publish(point_cloud2.create_cloud_xyz32(header, points))
                next_scan = time.monotonic() + 0.2
                while time.monotonic() < next_scan:
                    rclpy.spin_once(node, timeout_sec=0.02)
                if i % 25 == 0:
                    print(f'Scan {i}/100: {stats}', flush=True)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if stats['odometry_messages'] < 10:
                errors.append('Fewer than 10 valid ICP odometry messages')
            if stats['max_graph_nodes'] < 3:
                errors.append('Fewer than 3 SLAM graph nodes')
            if stats['max_cloud_points'] < 100:
                errors.append('No usable assembled map cloud')
            if odometry_positions:
                start, end = np.array(odometry_positions[0]), np.array(odometry_positions[-1])
                stats['estimated_translation_m'] = float(np.linalg.norm(end - start))
                if not 0.7 < stats['estimated_translation_m'] < 1.7:
                    errors.append('ICP translation does not match the 1.2 m synthetic trajectory')
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

    # Shutdown flushes the real RTAB-Map SQLite database.
    try:
        with sqlite3.connect(f'file:{database}?mode=ro', uri=True) as conn:
            stats['database_integrity'] = conn.execute('PRAGMA integrity_check').fetchone()[0]
            stats['saved_nodes'] = conn.execute('SELECT COUNT(*) FROM Node').fetchone()[0]
        if stats['database_integrity'] != 'ok' or stats['saved_nodes'] < 3:
            errors.append('Saved database failed integrity/node-count checks')
    except Exception as exc:
        errors.append(f'Database validation: {exc}')
    report = {
        'passed': not errors, 'scope': 'synthetic PointCloud2 -> real ICP -> real SLAM -> SQLite',
        'physical_sensor_tested': False, 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
        'launch_exit_code': launch_exit_code, 'metrics': stats, 'errors': errors,
        'artifacts': str(run),
    }
    (run / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    (root / 'logs/latest_test.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
