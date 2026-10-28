#!/usr/bin/env python3
"""Isolated synthetic Livox rotation check; only writes to /tmp."""
from array import array
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
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage

project_root = Path(__file__).resolve().parents[1]
root = Path(tempfile.mkdtemp(prefix='livox_rotation_check_'))
os.environ['ROS_DOMAIN_ID'] = os.environ.get('LIVOX_TEST_ROS_DOMAIN_ID', '178')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = str(root / 'ros_logs')
os.environ['OMP_NUM_THREADS'] = '1'
os.environ['OPENBLAS_NUM_THREADS'] = '1'
qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
epoch = 6932864.055079328
omega = 0.4


def stamp(value):
    from builtin_interfaces.msg import Time
    ns = round(value * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def yaw(elapsed):
    return omega * max(0.0, elapsed - 1.0)


def quaternion_yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


a, b = np.meshgrid(np.linspace(-2.5, 2.5, 32), np.linspace(-1.5, 1.5, 24))
world = np.concatenate([
    np.column_stack([np.full(a.size, 4.), a.ravel(), b.ravel()]),
    np.column_stack([np.full(a.size, -5.), a.ravel(), b.ravel()]),
    np.column_stack([2.5 + 0.6*a.ravel(), np.full(a.size, 3.), b.ravel()]),
    np.column_stack([2.5 + 0.6*a.ravel(), np.full(a.size, -4.), b.ravel()]),
    np.column_stack([2.5 + 0.6*a.ravel(), 2.0*b.ravel(), np.full(a.size, -1.5)]),
    np.column_stack([2.5 + 0.6*a.ravel(), 2.0*b.ravel(), np.full(a.size, 2.5)]),
])
offsets = np.linspace(0., 0.0994, len(world))
dtype = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),
                  ('tag','u1'),('line','u1'),('timestamp','<f8')])


def cloud_at(elapsed):
    cloud = PointCloud2()
    cloud.header.frame_id = 'livox_frame'
    cloud.header.stamp = stamp(epoch + elapsed)
    cloud.height, cloud.width = 1, len(world)
    cloud.point_step, cloud.row_step = 26, len(world) * 26
    cloud.is_dense = True
    cloud.fields = [PointField(name=name, offset=dtype.fields[name][1], datatype=datatype, count=1)
                    for name, datatype in [('x',7),('y',7),('z',7),('intensity',7),('tag',2),('line',2),('timestamp',8)]]
    points = np.zeros(len(world), dtype=dtype)
    theta = omega * np.maximum(0., elapsed + offsets - 1.)
    points['x'] = np.cos(theta)*world[:,0] + np.sin(theta)*world[:,1]
    points['y'] = -np.sin(theta)*world[:,0] + np.cos(theta)*world[:,1]
    points['z'] = world[:,2]
    points['intensity'], points['tag'], points['line'] = 30., 16, 2
    points['timestamp'] = (epoch + elapsed + offsets)*1e9
    cloud.data = array('B', points.tobytes())
    return cloud


def run():
    rclpy.init()
    node = rclpy.create_node('rotation_check_observer')
    imu_pub = node.create_publisher(Imu, '/livox/imu', qos)
    cloud_pub = node.create_publisher(PointCloud2, '/livox/lidar', qos)
    clock_pub = node.create_publisher(Clock, '/clock', qos)
    errors, poses, tf_edges = [], [], set()

    def on_cloud(message):
        elapsed = message.header.stamp.sec + message.header.stamp.nanosec*1e-9 - epoch
        if elapsed < 2.:
            return
        xyz = np.column_stack([np.ndarray((message.width,), dtype='<f4', buffer=message.data,
                                         offset=offset, strides=(message.point_step,)) for offset in (0,4,8)])
        theta = yaw(elapsed)
        rotation = np.array([[math.cos(theta),-math.sin(theta),0.],
                             [math.sin(theta), math.cos(theta),0.],[0.,0.,1.]])
        errors.append(float(np.max(np.linalg.norm(xyz @ rotation.T - world, axis=1))))

    def on_tf(message):
        tf_edges.update((transform.header.frame_id, transform.child_frame_id) for transform in message.transforms)

    subscriptions = [
        node.create_subscription(PointCloud2, '/livox/lidar/deskewed', on_cloud, qos),
        node.create_subscription(Odometry, '/icp_odom', poses.append, qos),
        node.create_subscription(TFMessage, '/tf', on_tf, qos),
    ]
    launch_log = (root / 'rotation_startup.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', str(project_root / 'launch/livox_realsense_3dslam.launch.py'),
        'rtabmap_viz:=false', 'use_sim_time:=true',
        'database_path:=' + str(root / 'rotation_map.db')],
        stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 12.
        while time.monotonic() < deadline:
            clock_pub.publish(Clock(clock=stamp(epoch)))
            rclpy.spin_once(node, timeout_sec=0.02)
            if cloud_pub.get_subscription_count() and imu_pub.get_subscription_count():
                break
            if process.poll() is not None:
                raise RuntimeError('SLAM launch exited before receiving input.')
        else:
            raise RuntimeError('Sensor subscribers did not start.')
        started, next_cloud, next_imu = time.monotonic(), 0.3, 0.
        while time.monotonic() - started < 8.:
            elapsed = time.monotonic() - started
            clock_pub.publish(Clock(clock=stamp(epoch + elapsed)))
            if elapsed >= next_imu:
                message = Imu()
                message.header.frame_id = 'livox_frame'
                message.header.stamp = stamp(epoch + elapsed)
                message.linear_acceleration.z = 9.80665
                message.angular_velocity.z = omega if elapsed > 1. else 0.
                imu_pub.publish(message)
                next_imu = elapsed + 0.005
            if elapsed >= next_cloud:
                cloud_pub.publish(cloud_at(elapsed - 0.1))
                next_cloud += 0.1
            rclpy.spin_once(node, timeout_sec=0.002)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.01)
        assert len(errors) >= 20, f'Only {len(errors)} deskewed rotating clouds.'
        assert len(poses) >= 20, f'Only {len(poses)} odometry messages.'
        final = poses[-1]
        elapsed = final.header.stamp.sec + final.header.stamp.nanosec*1e-9 - epoch
        angle_error = math.remainder(quaternion_yaw(final.pose.pose.orientation) - yaw(elapsed), 2*math.pi)
        report = {
            'deskewed_rotating_clouds': len(errors),
            'odometry_messages': len(poses),
            'deskew_error_p95_m': float(np.percentile(errors,95)),
            'final_yaw_error_rad': angle_error,
            'tf_edges': sorted(tf_edges),
        }
        assert report['deskew_error_p95_m'] < 0.025, report
        assert abs(angle_error) < 0.05, report
        assert ('icp_odom','livox_frame_stabilized') in tf_edges, report
        assert ('livox_frame_stabilized','livox_frame') in tf_edges, report
        assert ('icp_odom','livox_frame') not in tf_edges, report
        (root / 'rotation_results.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        launch_log.close()
        node.destroy_node()
        rclpy.shutdown()
    with sqlite3.connect(root / 'rotation_map.db') as db:
        links = db.execute('SELECT type,count(*) FROM Link GROUP BY type').fetchall()
        print('graph link types:', links)
        assert dict(links).get(9, 0) > 0, 'No gravity constraints were saved.'
    log = (root / 'rotation_startup.log').read_text()
    assert 'cannot store a negative time' not in log
    assert 'Point timestamps: nanoseconds -> seconds' in log
    print('Test artifacts:', root)


if __name__ == '__main__':
    run()
