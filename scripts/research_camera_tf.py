#!/usr/bin/env python3
"""Connect the camera's factory TF tree using the research calibration."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from rtabmap_research.inputs import estimated_transform, measured_transform, NotReady


def lidar_from_camera_link(camera_from_lidar, camera_from_link):
    return np.linalg.inv(camera_from_lidar) @ camera_from_link


def calibration(config):
    path = ROOT / config['calibration_path']
    value = json.loads(path.read_text())
    if value.get('status') == 'measured' or config.get('pmf_extrinsics_mode') != 'tf_preview':
        matrix, metadata = measured_transform(path)
    else:
        path = ROOT / config['estimated_calibration_path']
        matrix, metadata = estimated_transform(path)
    return matrix, metadata, path


def main():
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
    from map_geometry_node import ros_matrix

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    matrix, metadata, path = calibration(json.loads(args.config.read_text()))
    rclpy.init()

    class CameraTF(Node):
        def __init__(self):
            super().__init__('research_camera_mount_tf')
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.broadcaster = StaticTransformBroadcaster(self)
            self.done = False
            self.camera_parent = None
            self.create_subscription(TFMessage, '/tf_static', self.parents,
                QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_timer(.5, self.connect)

        def parents(self, message):
            for transform in message.transforms:
                if transform.child_frame_id == 'camera_link':
                    self.camera_parent = transform.header.frame_id

        def connect(self):
            if self.done:
                return
            lidar, camera = metadata['lidar_frame'], metadata['camera_frame']
            try:
                if self.buffer.can_transform(lidar, 'camera_link', Time()):
                    value = self.buffer.lookup_transform(lidar, 'camera_link', Time())
                    mount = ros_matrix(value.transform)
                    state = 'reuse_existing_tf'
                else:
                    if self.camera_parent:
                        self.get_logger().warning('camera_link already has a parent: ' + self.camera_parent,
                                                  throttle_duration_sec=5.)
                        return
                    factory = self.buffer.lookup_transform(camera, 'camera_link', Time())
                    mount = lidar_from_camera_link(matrix, ros_matrix(factory.transform))
                    value = TransformStamped()
                    value.header.stamp = self.get_clock().now().to_msg()
                    value.header.frame_id, value.child_frame_id = lidar, 'camera_link'
                    for name, number in zip(('x', 'y', 'z'), mount[:3, 3]):
                        setattr(value.transform.translation, name, float(number))
                    for name, number in zip(('x', 'y', 'z', 'w'), Rotation.from_matrix(mount[:3, :3]).as_quat()):
                        setattr(value.transform.rotation, name, float(number))
                    self.broadcaster.sendTransform(value)
                    state = 'published'
            except TransformException:
                self.get_logger().info('Waiting for camera factory TF', throttle_duration_sec=5.)
                return
            report = dict(state=state, parent=lidar, child='camera_link', calibration_file=str(path),
                calibration=metadata['status'], geometry_verified=metadata.get('geometry_verified', metadata['status'] == 'measured'),
                lidar_from_camera_link=mount.tolist())
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2))
            self.get_logger().info('Camera TF connected: ' + lidar + ' -> camera_link (' + metadata['status'] + ')')
            self.done = True

    node = CameraTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
