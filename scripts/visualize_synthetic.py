#!/usr/bin/env python3
"""Publish a synthetic room, run the Livox SLAM stack, and render the result."""
import math
import os
import sys
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'logs' / 'visualization'
OUT.mkdir(parents=True, exist_ok=True)


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


def cloud_xyz(msg):
    if msg is None:
        return np.zeros((0, 3), dtype=np.float32)
    names = {f.name for f in msg.fields}
    fields = ('x', 'y', 'z', 'rgb') if 'rgb' in names else ('x', 'y', 'z')
    points = point_cloud2.read_points(msg, field_names=fields, skip_nans=True)
    if fields == ('x', 'y', 'z', 'rgb'):
        xyz = np.column_stack([points['x'], points['y'], points['z']]).astype(np.float32)
        packed = points['rgb'].astype(np.float32).view(np.uint32)
        colors = np.column_stack([
            (packed >> 16) & 255, (packed >> 8) & 255, packed & 255
        ]).astype(np.float32) / 255.0
        return xyz, colors
    xyz = np.column_stack([points['x'], points['y'], points['z']]).astype(np.float32)
    return xyz, None


def main():
    run = Path(tempfile.mkdtemp(prefix='visual-', dir=ROOT / 'logs'))
    database = run / 'map.db'
    clouds = {'map': None, 'ground': None, 'obstacles': None}
    odom = []
    gt = []
    process = None
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node('synthetic_visualizer')

        def on_odom(msg):
            p = msg.pose.pose.position
            if math.isfinite(p.x) and math.isfinite(p.y):
                odom.append((p.x, p.y, p.z))

        def keep(name):
            def cb(msg):
                if msg.width * msg.height:
                    clouds[name] = msg
            return cb

        node.create_subscription(Odometry, '/rtabmap/odom', on_odom, qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/rtabmap/cloud_map', keep('map'), qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/rtabmap/cloud_ground', keep('ground'), qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/rtabmap/cloud_obstacles', keep('obstacles'), qos_profile_sensor_data)
        pub = node.create_publisher(PointCloud2, '/rtabmap_test/points', qos_profile_sensor_data)

        with (run / 'nodes.log').open('w') as output:
            process = subprocess.Popen([
                'ros2', 'launch', str(ROOT / 'launch/livox_slam.launch.py'),
                'rviz:=false', 'lidar_topic:=/rtabmap_test/points',
                f'database_path:={database}',
            ], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 40
            while pub.get_subscription_count() < 2:
                if process.poll() is not None:
                    raise RuntimeError('SLAM launch exited early; see nodes.log')
                if time.monotonic() > deadline:
                    raise RuntimeError('DDS discovery timeout')
                rclpy.spin_once(node, timeout_sec=0.1)

            world = room_points()
            for i in range(100):
                if process.poll() is not None:
                    raise RuntimeError('SLAM launch exited during mapping')
                x = max(0, min(i - 10, 80)) * 0.015
                y = 0.05 * math.sin(x * 2)
                gt.append((x, y, 0.0))
                points = world - np.array([x, y, 0], dtype=np.float32)
                header = Header(stamp=node.get_clock().now().to_msg(), frame_id='livox_frame')
                cloud = point_cloud2.create_cloud_xyz32(header, points)
                cloud.is_dense = True
                pub.publish(cloud)
                end = time.monotonic() + 0.2
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)
            end = time.monotonic() + 3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    map_xyz, map_colors = cloud_xyz(clouds['map'])
    ground_xyz, _ = cloud_xyz(clouds['ground'])
    obstacle_xyz, _ = cloud_xyz(clouds['obstacles'])
    odom = np.asarray(odom, dtype=np.float32)
    gt = np.asarray(gt, dtype=np.float32)
    np.savez_compressed(OUT / 'synthetic_result.npz', map_xyz=map_xyz,
                        map_colors=map_colors if map_colors is not None else np.zeros(0),
                        ground_xyz=ground_xyz, obstacle_xyz=obstacle_xyz,
                        odom=odom, gt=gt)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    def subsample(xyz, limit=25000):
        if len(xyz) <= limit:
            return xyz
        idx = np.linspace(0, len(xyz) - 1, limit).astype(int)
        return xyz[idx]

    subprocess.run([sys.executable, str(ROOT / 'scripts/plot_synthetic_result.py')], check=True)
    print((OUT / 'synthetic_visualization.png').resolve(), flush=True)
    print('map_points=', len(map_xyz), 'odom_samples=', len(odom), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
