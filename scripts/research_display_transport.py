#!/usr/bin/env python3
"""Compress Orin display outputs into a separate TCP link; preserve ROS stamps.

Both ROS domains remain local to each computer. Only this selected, rate-limited
display profile crosses Wi-Fi; full-rate sensor topics stay on the Orin.
"""
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

import hotspot_sensor_transport as transport
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

PREFIX = '/miti/research/display'
DISPLAY_STREAMS = [
    ('/tf', TFMessage), ('/tf_static', TFMessage),
    ('/camera/color/image_raw', Image),
    (PREFIX + '/aligned_depth/image_raw', Image),
    ('/camera/infra2/image_rect_raw', Image),
    ('/camera/color/camera_info', CameraInfo),
    (PREFIX + '/aligned_depth/camera_info', CameraInfo),
    ('/camera/infra2/camera_info', CameraInfo),
    (PREFIX + '/clean_map', PointCloud2), (PREFIX + '/live_scan', PointCloud2),
    (PREFIX + '/trajectory', Path), (PREFIX + '/semantic_markers', MarkerArray),
    (PREFIX + '/walls', MarkerArray), (PREFIX + '/grid_map', OccupancyGrid),
    (PREFIX + '/robot_pose', PoseStamped), (PREFIX + '/loop_status', String),
    (PREFIX + '/dynamic_cloud_status', String), (PREFIX + '/map_status', String),
    (PREFIX + '/grid_status', String), ('/icp_odom', Odometry),
]
LATCHED = {1, 8, 9, 10, 11, 12, 13, 15, 16, 18}


class DisplayTransport(transport.Transport):
    def __init__(self, args):
        self.snapshot_cache = {}
        self.dynamic_transforms = {}
        self.last_queued = {}
        super().__init__(args)

    def stream_qos(self, topic, kind):
        qos = super().stream_qos(topic, kind)
        index = next(i for i, item in enumerate(DISPLAY_STREAMS) if item[0] == topic)
        if index in LATCHED:
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            qos.depth = 1
        if index == 1 and self.args.mode == 'send':
            # RealSense retains several factory TF messages in one publisher.
            # A depth-one reader only receives its last retained message.
            # Collect the complete history before forwarding the merged tree.
            qos.depth = 100
        return qos

    def queue(self, index, message):
        if index == 0:
            for transform in message.transforms:
                self.dynamic_transforms[transform.child_frame_id] = transform
            message = TFMessage(transforms=list(self.dynamic_transforms.values()))
        if index in LATCHED or isinstance(message, CameraInfo):
            self.snapshot_cache[index] = message
        now = time.monotonic()
        hz = 10 if index in (0, 14, 19) else (1 if index == 8 else 2)
        if index != 1 and now - self.last_queued.get(index, -float('inf')) < 1 / hz:
            return
        self.last_queued[index] = now
        super().queue(index, message)

    def replay_cached(self):
        for index, message in list(self.snapshot_cache.items()):
            if index != 1:
                super().queue(index, message)


def main():
    # A separate entry point defines a fixed wire profile on both peers. It does
    # not change the original sensor protocol or its stream IDs.
    transport.STREAMS = DISPLAY_STREAMS
    transport.STATIC_STREAM_INDEX = 1
    transport.main(DisplayTransport)


if __name__ == '__main__':
    main()
