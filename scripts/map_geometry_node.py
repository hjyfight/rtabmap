#!/usr/bin/env python3
"""Wall display and map-frame pose output, independent of the SLAM writer."""
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import fields

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from wall_geometry import WallConfig, extract_walls, pose_values, transform_matrix


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def ros_matrix(transform):
    p, q = transform.translation, transform.rotation
    return transform_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def odometry_matrix(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    return transform_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def cloud_xyz(message, limit=600000):
    """Decode XYZ including organized row padding, float64 fields and big-endian clouds."""
    if not message.width or not message.height:
        return np.empty((0, 3))
    if message.point_step <= 0 or message.row_step < message.width * message.point_step or len(message.data) < message.row_step * message.height:
        raise ValueError('invalid PointCloud2 buffer or stride')
    fs = {f.name: f for f in message.fields}
    formats, offsets = [], []
    for key in ('x', 'y', 'z'):
        field = fs.get(key)
        if field is None or field.count != 1 or field.datatype not in (PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError('PointCloud2 requires scalar floating-point XYZ')
        size = 4 if field.datatype == PointField.FLOAT32 else 8
        if field.offset < 0 or field.offset + size > message.point_step:
            raise ValueError('PointCloud2 field exceeds point_step')
        formats.append(('>' if message.is_bigendian else '<') + 'f' + str(size))
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=formats, offsets=offsets, itemsize=message.point_step))
    view = np.ndarray((message.height, message.width), dtype=dtype, buffer=memoryview(message.data),
                      strides=(message.row_step, message.point_step)).reshape(-1)
    view = view[::max(1, math.ceil(len(view) / limit))]
    return np.column_stack([view[key] for key in ('x', 'y', 'z')]).astype(np.float64)


def wall_markers(walls, header, max_walls):
    markers = []
    for key, alpha, color in [('triangles', 0.25, (0.25, 0.75, 1.)), ('edges', 0.8, (0.4, 0.85, 1.))]:
        marker = Marker(header=deepcopy(header), ns='rtabmap/walls/' + key, id=0,
                        type=Marker.TRIANGLE_LIST, action=Marker.ADD if walls else Marker.DELETE)
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = 1.
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = alpha
        marker.points = [Point(x=float(x), y=float(y), z=float(z)) for wall in walls for x, y, z in wall[key]]
        markers.append(marker)
    for i in range(max_walls):
        marker = Marker(header=deepcopy(header), ns='rtabmap/walls/labels', id=i,
                        type=Marker.TEXT_VIEW_FACING, action=Marker.ADD if i < len(walls) else Marker.DELETE)
        marker.pose.orientation.w = 1.
        if i < len(walls):
            marker.pose.position = Point(**dict(zip(('x', 'y', 'z'), map(float, walls[i]['label']))))
            marker.text = '墙面'
            marker.scale.z = 0.12
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.55, 0.88, 1., 1.
        markers.append(marker)
    return MarkerArray(markers=markers)


class MapGeometryNode(Node):
    def __init__(self):
        super().__init__('map_geometry')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.map_frame = param('map_frame', 'map').strip('/')
        self.robot_frame = param('robot_frame', 'auto').strip('/')
        self.odom_topic = param('odom_topic', '/icp_odom')
        self.cloud_topic = param('cloud_topic', '/cloud_map')
        self.pose_timeout = float(param('pose_timeout', 1.0))
        self.transform_timeout = float(param('transform_max_age', 3.0))
        self.wall_interval = float(param('wall_update_interval', 2.0))
        self.wall_config = WallConfig(**{f.name: param('wall_' + f.name, getattr(WallConfig(), f.name))
                                         for f in fields(WallConfig)})
        if not self.map_frame or not self.robot_frame or any(not math.isfinite(v) or v <= 0 for v in (
                self.pose_timeout, self.transform_timeout, self.wall_interval)):
            raise ValueError('frames and timing parameters must be valid')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.wall_pub = self.create_publisher(MarkerArray, '/miti/walls', latched)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 1)
        self.status_pub = self.create_publisher(String, '/miti/map_status', 1)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, latched)
        self.odom = None
        self.odom_received = 0.
        self.odom_advanced = 0.
        self.pending_cloud = None
        self.cloud_frame = None
        self.epoch = 0
        self.job = None
        self.last_wall_start = -float('inf')
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='wall_geometry')
        self.pose_status = dict(valid=False, reason='等待里程计')
        self.wall_status = dict(state='waiting', count=0, reason='等待地图点云')
        self.last_clock = None
        self.create_timer(0.1, self.update_pose)
        self.create_timer(0.2, self.update_walls)
        self.create_timer(0.25, self.publish_status)
        self.get_logger().info('Wall display /miti/walls; pose /robot_pose (map frame, tracked frame in /miti/map_status)')

    def on_odom(self, message):
        previous = stamp_seconds(self.odom.header.stamp) if self.odom is not None else None
        stamp = stamp_seconds(message.header.stamp)
        if previous is not None and previous - 5 < stamp < previous:
            return  # A reordered packet is not a new pose.
        if previous is None or stamp != previous:
            self.odom_advanced = time.monotonic()
        self.odom, self.odom_received = message, time.monotonic()

    def on_cloud(self, message):
        if self.cloud_frame is not None and message.header.frame_id != self.cloud_frame:
            self.epoch += 1
            self.wall_pub.publish(wall_markers([], Header(frame_id=self.map_frame), self.wall_config.max_walls))
        self.cloud_frame = message.header.frame_id
        self.pending_cloud = message

    def update_pose(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_clock is not None and now < self.last_clock - 0.1:
            self.odom = None
            self.epoch += 1
            self.pending_cloud = None
            self.tf_buffer.clear()
            self.wall_status = dict(state='waiting', count=0, reason='时间已重置，等待新地图')
            self.wall_pub.publish(wall_markers([], Header(frame_id=self.map_frame), self.wall_config.max_walls))
        self.last_clock = now
        message = self.odom
        if message is None:
            self.pose_status = dict(valid=False, reason='等待里程计')
            return
        # Livox on this robot stamps odometry in device uptime, while RTAB-Map's
        # map correction is stamped in system time. Never subtract those epochs.
        # Both message arrival and acquisition timestamp progression must be fresh.
        age = max(time.monotonic()-self.odom_received, time.monotonic()-self.odom_advanced)
        self.pose_status = dict(valid=False, reason='定位数据已中断', age_seconds=max(0., age))
        if age > self.pose_timeout:
            return
        try:
            child, parent = message.child_frame_id.strip('/'), message.header.frame_id.strip('/')
            if not child or not parent:
                raise ValueError('里程计缺少参考坐标系')
            # Latest map correction + timestamped odometry; do not wait for future map TF.
            correction = self.tf_buffer.lookup_transform(self.map_frame, parent, Time())
            correction_stamp = stamp_seconds(correction.header.stamp)
            if correction_stamp and min(abs(now-correction_stamp),
                    abs(stamp_seconds(message.header.stamp)-correction_stamp)) > self.transform_timeout:
                raise ValueError('地图坐标变换已过期')
            tracked = 'base_link' if self.robot_frame == 'auto' else self.robot_frame
            if self.robot_frame == 'auto' and not self.tf_buffer.can_transform(child, tracked, Time.from_msg(message.header.stamp)):
                tracked = child
            mount = self.tf_buffer.lookup_transform(child, tracked, Time.from_msg(message.header.stamp))
            matrix = ros_matrix(correction.transform) @ odometry_matrix(message) @ ros_matrix(mount.transform)
            position, quaternion, rpy = pose_values(matrix)
            result = PoseStamped(header=Header(frame_id=self.map_frame, stamp=deepcopy(message.header.stamp)))
            for key, value in zip(('x', 'y', 'z'), position):
                setattr(result.pose.position, key, value)
            for key, value in zip(('x', 'y', 'z', 'w'), quaternion):
                setattr(result.pose.orientation, key, value)
            self.pose_pub.publish(result)
            self.pose_status = dict(valid=True, frame_id=self.map_frame, tracked_frame=tracked,
                                    body_frame=tracked == 'base_link', position_m=position,
                                    orientation_xyzw=quaternion, rpy_deg=np.rad2deg(rpy).tolist(),
                                    stamp=stamp_seconds(message.header.stamp), age_seconds=max(0., age), reason='')
            self.pose_status['stamp_clock'] = ('device' if abs(now-stamp_seconds(message.header.stamp)) > 60 else 'ros')
        except (TransformException, ValueError) as error:
            self.pose_status['reason'] = str(error) if isinstance(error, ValueError) else '等待定位坐标变换'

    def process_cloud(self, message, matrix, epoch):
        started = time.monotonic()
        points = cloud_xyz(message)
        points = points @ matrix[:3, :3].T + matrix[:3, 3]
        walls = extract_walls(points, self.wall_config)
        header = Header(frame_id=self.map_frame, stamp=deepcopy(message.header.stamp))
        result = wall_markers(walls, header, self.wall_config.max_walls)
        return epoch, result, dict(state='ready', count=len(walls), source_points=message.width*message.height,
                                   source_stamp=stamp_seconds(message.header.stamp),
                                   processing_seconds=round(time.monotonic()-started, 3), reason='')

    def update_walls(self):
        if self.job is not None:
            if not self.job.done():
                return
            try:
                epoch, markers, status = self.job.result()
                if epoch == self.epoch:
                    self.wall_pub.publish(markers)
                    self.wall_status = status
            except Exception as error:
                self.get_logger().error('Wall extraction failed: ' + str(error))
                self.wall_status = dict(state='error', count=0, reason='墙面提取失败')
                self.wall_pub.publish(wall_markers([], Header(frame_id=self.map_frame), self.wall_config.max_walls))
            self.job = None
        if self.pending_cloud is None or time.monotonic()-self.last_wall_start < self.wall_interval:
            return
        message = self.pending_cloud
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, message.header.frame_id, Time.from_msg(message.header.stamp))
            matrix = ros_matrix(transform.transform)
        except (TransformException, ValueError):
            self.wall_status = dict(state='waiting', count=self.wall_status['count'], reason='等待地图坐标变换')
            return
        self.pending_cloud = None
        self.last_wall_start = time.monotonic()
        self.job = self.pool.submit(self.process_cloud, message, matrix, self.epoch)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(dict(schema=1, pose=self.pose_status, walls=self.wall_status),
                                                       ensure_ascii=False, allow_nan=False)))

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        super().destroy_node()


def main():
    rclpy.init()
    node = MapGeometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
