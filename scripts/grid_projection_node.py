#!/usr/bin/env python3
"""Independent floor-relative grid display. Does not write to RTAB-Map or /map."""
from array import array
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import fields
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rtabmap_msgs.msg import MapGraph
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String

from grid_projection import GridConfig, project_grid
from map_geometry_node import cloud_xyz


def grid_message(result, header, resolution):
    message = OccupancyGrid(header=deepcopy(header))
    message.info.map_load_time = deepcopy(header.stamp)
    message.info.resolution = float(resolution)
    message.info.height, message.info.width = result['data'].shape
    message.info.origin.position.x, message.info.origin.position.y, message.info.origin.position.z = result['origin']
    message.info.origin.orientation.w = 1.0
    message.data = array('b', result['data'].ravel().tobytes())
    return message


class GridProjectionNode(Node):
    def __init__(self):
        super().__init__('grid_projection')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.map_frame = param('map_frame', 'map').strip('/')
        self.cloud_topic = param('cloud_topic', '/cloud_map')
        self.interval = float(param('update_interval', 1.0))
        self.config = GridConfig(**{field.name: param(field.name, getattr(GridConfig(), field.name))
                                    for field in fields(GridConfig)})
        if not self.map_frame or not math.isfinite(self.interval) or self.interval <= 0:
            raise ValueError('map frame and update interval must be valid')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, '/miti/grid_map', latched)
        self.status_pub = self.create_publisher(String, '/miti/grid_status', latched)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, latched)
        self.create_subscription(PoseStamped, '/robot_pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(MapGraph, '/mapGraph', self.on_graph, latched)
        self.reference = None
        self.pose_reference = False
        self.latest_cloud = None
        self.pending = None
        self.epoch = 0
        self.job = None
        self.last_start = -float('inf')
        self.last_clock = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='grid_projection')
        self.state = dict(state='waiting', reason='等待三维地图与定位参考点')
        self.create_timer(.1, self.update)
        self.publish_status()
        self.get_logger().info('Floor-relative grid /miti/grid_map; original point cloud and /map unchanged')

    def set_reference(self, point, is_pose=False):
        value = np.array([point.x, point.y, point.z], dtype=float)
        if not np.isfinite(value).all():
            return
        first = self.reference is None
        self.reference = value
        self.pose_reference = self.pose_reference or is_pose
        if first and self.latest_cloud is not None:
            self.pending = self.latest_cloud

    def on_pose(self, message):
        if message.header.frame_id.strip('/') == self.map_frame:
            self.set_reference(message.pose.position, True)

    def on_graph(self, message):
        # Saved maps can be displayed without live odometry. Use a real graph
        # node as the height reference; never assume map z=0 is the floor.
        if message.header.frame_id.strip('/') == self.map_frame and not self.pose_reference:
            pairs = [(key, pose) for key, pose in zip(message.poses_id, message.poses) if key > 0]
            if pairs:
                self.set_reference(min(pairs, key=lambda pair: pair[0])[1].position)

    def on_cloud(self, message):
        self.latest_cloud = message
        self.pending = message

    def publish_status(self):
        message = dict(schema=1, resolution=self.config.resolution,
                       min_obstacle_height=self.config.min_obstacle_height,
                       max_obstacle_height=self.config.max_obstacle_height, **self.state)
        self.status_pub.publish(String(data=json.dumps(message, ensure_ascii=False, allow_nan=False)))

    def clear(self, reason):
        result = dict(data=np.full((1, 1), -1, dtype=np.int8), origin=[0., 0., 0.])
        self.publisher.publish(grid_message(result, Header(frame_id=self.map_frame), self.config.resolution))
        self.state = dict(state='waiting', reason=reason)
        self.publish_status()

    def process(self, message, reference, epoch):
        started = time.monotonic()
        if message.header.frame_id.strip('/') != self.map_frame:
            raise ValueError('地图坐标系不匹配')
        result = project_grid(cloud_xyz(message), reference, self.config)
        grid = grid_message(result, Header(frame_id=self.map_frame, stamp=deepcopy(message.header.stamp)), self.config.resolution)
        values = result.pop('data')
        result.update(width=grid.info.width, height=grid.info.height,
                      unknown=int((values == -1).sum()), free=int((values == 0).sum()),
                      occupied=int((values == 100).sum()), processing_seconds=round(time.monotonic()-started, 3),
                      source_stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9)
        result['reason'] = {'ready': '', 'empty': '等待有效地图点云',
                            'waiting_ground': '尚未确认地面，暂不标记空闲区域'}[result['state']]
        return epoch, grid, result

    def update(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_clock is not None and now < self.last_clock - .1:
            self.epoch += 1
            self.reference = self.latest_cloud = self.pending = None
            self.pose_reference = False
            self.clear('时间已重置，等待新地图')
        self.last_clock = now
        if self.job is not None:
            if not self.job.done():
                return
            try:
                epoch, message, state = self.job.result()
                if epoch == self.epoch:
                    self.publisher.publish(message)
                    self.state = state
                    self.publish_status()
            except Exception as error:
                self.clear('栅格转换失败，请检查地图输入')
                self.state = dict(state='error', reason=str(error))
                self.publish_status()
                self.get_logger().error('Grid projection failed: ' + str(error))
            self.job = None
        if self.pending is None or self.reference is None or time.monotonic()-self.last_start < self.interval:
            return
        message, self.pending = self.pending, None
        self.last_start = time.monotonic()
        self.job = self.pool.submit(self.process, message, self.reference.copy(), self.epoch)

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = GridProjectionNode()
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
