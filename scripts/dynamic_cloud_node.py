#!/usr/bin/env python3
"""Preserve the environment map, clear contradicted history, replace the live scan."""
import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from dynamic_cloud_filter import DynamicCloudFilter
from map_geometry_node import cloud_xyz, ros_matrix, stamp_seconds


def xyz_message(points, frame, stamp):
    points = np.ascontiguousarray(points, dtype='<f4')
    return PointCloud2(header=Header(frame_id=frame, stamp=stamp), height=1, width=len(points),
        fields=[PointField(name=axis, offset=i*4, datatype=PointField.FLOAT32, count=1)
                for i, axis in enumerate(('x', 'y', 'z'))],
        is_bigendian=False, point_step=12, row_step=len(points)*12,
        data=points.tobytes(), is_dense=bool(np.isfinite(points).all()))


class ScanQueue:
    """A bounded latest-ready queue; waiting for a pose never builds an unbounded backlog."""
    def __init__(self, max_age=1.5):
        self.items = deque(maxlen=30)
        self.last_stamp = -math.inf
        self.max_age = max_age

    def add(self, message, received):
        stamp = stamp_seconds(message.header.stamp)
        if stamp > self.last_stamp and not any(stamp_seconds(m.header.stamp) == stamp for m, _ in self.items):
            self.items.append((message, received))

    def take(self, now, transform):
        self.items = deque(((m, t) for m, t in self.items if now-t <= self.max_age and
                           stamp_seconds(m.header.stamp) > self.last_stamp), maxlen=30)
        for message, received in sorted(self.items, key=lambda item: stamp_seconds(item[0].header.stamp), reverse=True):
            try:
                matrix = transform(message)
            except (TransformException, ValueError):
                continue
            self.last_stamp = stamp_seconds(message.header.stamp)
            self.items = deque(((m, t) for m, t in self.items if stamp_seconds(m.header.stamp) > self.last_stamp), maxlen=30)
            return message, matrix, received
        return None


class DynamicCloudNode(Node):
    def __init__(self):
        super().__init__('miti_dynamic_cloud')
        param = lambda name, value: self.declare_parameter(name, value).value
        self.frame = param('map_frame', 'map')
        self.odom_frame = param('odom_frame', 'icp_odom')
        self.scan_topic = param('scan_topic', '/livox/lidar/deskewed')
        self.max_range = float(param('max_range', 12.))
        self.update_rate = float(param('update_rate', 5.))
        self.map_rate = float(param('map_publish_rate', 2.))
        self.live_timeout = float(param('live_timeout', 1.5))
        if min(self.update_rate, self.map_rate, self.live_timeout) <= 0:
            raise ValueError('Rates and timeouts must be positive')
        self.confirmations = int(param('confirmations', 3))
        self.filter = DynamicCloudFilter(resolution=float(param('resolution', .1)),
            confirmations=self.confirmations, max_range=self.max_range,
            endpoint_margin=float(param('endpoint_margin', .2)), evidence_window=float(param('evidence_window', 5.)))
        self.queue = ScanQueue(float(param('pose_wait_timeout', 1.5)))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.map_pub = self.create_publisher(PointCloud2, '/miti/clean_map', latched)
        self.live_pub = self.create_publisher(PointCloud2, '/miti/live_scan', latched)
        self.status_pub = self.create_publisher(String, '/miti/dynamic_cloud_status', latched)
        self.map_received = False
        self.map_dirty = False
        self.last_publish = -math.inf
        self.last_live = -math.inf
        self.live_present = False
        self.correction = None
        self.await_map = False
        self.session = None
        self.scan_frame = None
        self.last_input_stamp = None
        self.last_input_received = -math.inf
        self.outputs = deque(maxlen=30)
        self.status = dict(state='waiting', reason='等待环境地图和同步扫描', removed_points=0)
        self.create_subscription(PointCloud2, '/cloud_map', self.on_map, latched)
        self.create_subscription(PointCloud2, self.scan_topic, self.on_scan, sensor)
        try:
            from rtabmap_semantic_msgs.msg import SemanticMap
            self.create_subscription(SemanticMap, '/semantic/objects', self.on_session, latched)
        except ImportError:
            pass
        self.create_timer(1/self.update_rate, self.update)
        self.get_logger().info('Dynamic cloud: /cloud_map + timestamped '+self.scan_topic+
            ' -> /miti/clean_map and /miti/live_scan; saved map is unchanged')

    def clear_live(self):
        self.live_pub.publish(xyz_message(np.empty((0, 3)), self.frame, self.get_clock().now().to_msg()))
        self.live_present = False

    def on_session(self, message):
        session = (message.map_key, message.map_uuid)
        if self.session is not None and self.session != session:
            self.filter.reset()
            self.filter.set_map(np.empty((0, 3)))
            self.map_received, self.map_dirty = False, True
            self.queue = ScanQueue(self.queue.max_age)
            self.clear_live()
        self.session = session

    def on_map(self, message):
        try:
            if message.header.frame_id.strip('/') != self.frame:
                raise ValueError('Environment map frame differs from configured map frame')
            points = cloud_xyz(message)
            if not len(points):
                self.filter.reset()
                self.queue = ScanQueue(self.queue.max_age)
                self.clear_live()
            self.filter.set_map(points)
            self.map_received, self.map_dirty, self.await_map = True, True, False
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.)

    def on_scan(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if (self.scan_frame is not None and self.scan_frame != message.header.frame_id) or (
                self.last_input_stamp is not None and stamp < self.last_input_stamp-5.):
            self.filter.reset()
            self.queue = ScanQueue(self.queue.max_age)
            self.map_dirty = True
            self.clear_live()
            self.last_input_stamp = None
        self.scan_frame = message.header.frame_id
        self.last_input_stamp = stamp if self.last_input_stamp is None else max(stamp, self.last_input_stamp)
        self.last_input_received = time.monotonic()
        self.queue.add(message, self.last_input_received)

    def transform(self, message):
        # Sensor timestamps and the map correction can use different epochs. Use
        # timestamped odometry plus latest map correction, never a latest sensor pose.
        sensor = self.tf_buffer.lookup_transform(self.odom_frame, message.header.frame_id,
                                                 Time.from_msg(message.header.stamp))
        correction = self.tf_buffer.lookup_transform(self.frame, self.odom_frame, Time())
        matrix = ros_matrix(correction.transform)
        age = min(abs(self.get_clock().now().nanoseconds/1e9-stamp_seconds(correction.header.stamp)),
                  abs(stamp_seconds(message.header.stamp)-stamp_seconds(correction.header.stamp)))
        if stamp_seconds(correction.header.stamp) and age > 3.:
            raise ValueError('Map correction is stale')
        if self.correction is not None:
            shift = np.linalg.norm(matrix[:3, 3]-self.correction[:3, 3])
            angle = math.acos(float(np.clip((np.trace(matrix[:3, :3].T@self.correction[:3, :3])-1)/2, -1, 1)))
            if shift > .15 or angle > math.radians(5):
                self.filter.reset()
                self.await_map = True
                self.map_dirty = True
        self.correction = matrix
        return matrix @ ros_matrix(sensor.transform)

    def update(self):
        now = time.monotonic()
        if self.map_received:
            ready = self.queue.take(now, self.transform)
            if ready is not None:
                message, matrix, received = ready
                started = time.perf_counter()
                try:
                    local = cloud_xyz(message, limit=40000)
                    ranges = np.linalg.norm(local, axis=1)
                    valid = np.isfinite(local).all(axis=1) & (ranges >= .3)
                    world = np.ascontiguousarray(local[valid]@matrix[:3, :3].T+matrix[:3, 3], dtype=np.float32)
                    if not self.await_map:
                        self.status.update(self.filter.scan(world, matrix[:3, 3], stamp_seconds(message.header.stamp)))
                        self.map_dirty = True
                    live = world[ranges[valid] <= self.max_range]
                    self.live_pub.publish(xyz_message(live, self.frame, message.header.stamp))
                    self.last_live, self.live_present = received, bool(len(live))
                    self.outputs.append(now)
                    self.status.update(state='ready' if not self.await_map else 'waiting',
                        reason='射线清理已启用' if not self.await_map else '等待回环后地图更新',
                        scan_points=len(live), processing_ms=(time.perf_counter()-started)*1000,
                        pose_lag_seconds=max(0., self.last_input_stamp-stamp_seconds(message.header.stamp)),
                        input_wait_ms=(now-received)*1000)
                except (ValueError, RuntimeError) as error:
                    self.status.update(state='error', reason=str(error))
                    self.get_logger().warning(str(error), throttle_duration_sec=5.)
            elif now-self.last_live > self.live_timeout:
                self.status.update(state='waiting', reason='等待新扫描与对应时刻位姿')
        if self.live_present and now-self.last_live > self.live_timeout:
            self.clear_live()
        if self.map_dirty and now-self.last_publish >= 1/self.map_rate:
            points = self.filter.filtered_map()
            self.map_pub.publish(xyz_message(points, self.frame, self.get_clock().now().to_msg()))
            self.status.update(map_points=len(self.filter.points), clean_points=len(points),
                               removed_points=len(self.filter.points)-len(points))
            self.map_dirty, self.last_publish = False, now
        times = [stamp for stamp in self.outputs if now-stamp <= 5.]
        hz = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.
        self.status_pub.publish(String(data=json.dumps(dict(schema=1, **self.status, rate_hz=round(hz, 2),
            scan_topic=self.scan_topic, confirmations=self.confirmations, queued=len(self.queue.items)), ensure_ascii=False)))


def main():
    rclpy.init()
    node = None
    try:
        node = DynamicCloudNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.filter.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
