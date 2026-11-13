#!/usr/bin/env python3
"""Check the exact Orin display launch beyond ICP's first two frames, locally."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import MapGraph
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from synthetic_slam_test import room_points


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '88' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires isolated ROS_DOMAIN_ID=88 and ROS_LOCALHOST_ONLY=1.')
    run = Path(tempfile.mkdtemp(prefix='orin-display-check-', dir=ROOT / 'logs'))
    rclpy.init()
    node = rclpy.create_node('orin_display_launch_check')
    process = None
    samples, stats = [], {'max_graph_nodes': 0, 'max_cloud_points': 0}
    errors = []

    def odom(msg):
        q = msg.pose.pose.orientation
        valid = sum(v * v for v in (q.x, q.y, q.z, q.w)) > .9
        p = msg.pose.pose.position
        samples.append((valid, p.x, p.y, p.z))

    def graph(msg):
        stats['max_graph_nodes'] = max(stats['max_graph_nodes'], len(msg.poses_id))

    def cloud(msg):
        stats['max_cloud_points'] = max(stats['max_cloud_points'], msg.width * msg.height)

    subscriptions = [node.create_subscription(kind, topic, callback, qos_profile_sensor_data)
        for kind, topic, callback in [(Odometry, '/icp_odom', odom),
            (MapGraph, '/mapGraph', graph), (PointCloud2, '/cloud_map', cloud)]]
    publisher = node.create_publisher(PointCloud2, '/livox/lidar', qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if publisher.get_subscription_count():
            raise RuntimeError('Another lidar consumer is using the test domain.')
        with (run / 'nodes.log').open('w') as output:
            process = subprocess.Popen(['ros2', 'launch',
                str(ROOT / 'launch/orin_lidar_display_test.launch.py'),
                f'database_path:={run / "map.db"}'], stdout=output,
                stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 15
        while publisher.get_subscription_count() < 2:
            if process.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('Launch did not create both pointcloud subscriptions.')
            rclpy.spin_once(node, timeout_sec=.1)
        world = room_points()
        for i in range(40):
            if process.poll() is not None:
                raise RuntimeError('Launch exited during mapping.')
            x = max(0, min(i - 5, 30)) * .025
            points = world - np.array([x, .04 * math.sin(x), 0], dtype=np.float32)
            publisher.publish(create_cloud_xyz32(Header(
                stamp=node.get_clock().now().to_msg(), frame_id='livox_frame'), points))
            deadline = time.monotonic() + .3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        stats.update(odom_messages=len(samples), valid_odometry=sum(s[0] for s in samples),
                     lost_odometry=sum(not s[0] for s in samples))
        valid = [s for s in samples if s[0]]
        stats['estimated_translation_m'] = math.dist(valid[0][1:], valid[-1][1:]) if valid else 0
        if len(valid) < 20 or not all(s[0] for s in samples[-10:]):
            errors.append('ICP failed to track beyond initialization.')
        if not .5 < stats['estimated_translation_m'] < 1:
            errors.append('ICP did not recover the synthetic 0.75 m movement.')
        if stats['max_graph_nodes'] < 3 or stats['max_cloud_points'] < 100:
            errors.append('No growing map graph or usable map cloud.')
    except Exception as exc:
        errors.append(str(exc))
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                errors.append('Test nodes did not stop normally.')
        node.destroy_node()
        rclpy.shutdown()
    report = dict(passed=not errors, metrics=stats, errors=errors,
                  scope='Workstation synthetic check of the Orin launch; no live robot traffic',
                  artifacts=str(run))
    (run / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
