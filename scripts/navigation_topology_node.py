#!/usr/bin/env python3
"""Publish an occupancy-derived navigation graph for MITI; no motion commands."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import fields
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from navigation_topology import NavigationConfig, build_navigation_graph

COLORS = {'area': (.25, .9, .8), 'corridor': (.35, .65, 1.),
          'doorway_candidate': (1., .75, .2), 'passage': (.75, .65, 1.)}


def navigation_markers(graph, header):
    result = [Marker(header=deepcopy(header), action=Marker.DELETEALL)]
    for i, item in enumerate(graph['nodes']):
        marker = Marker(header=deepcopy(header), ns='navigation/nodes', id=i,
                        type=Marker.CUBE if item['id'].startswith('P') else Marker.SPHERE,
                        action=Marker.ADD)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = item['position']
        marker.pose.position.z += .12
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = .22 if item['id'].startswith('P') else .35
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = *COLORS[item['kind']], 1.
        result.append(marker)
        text = deepcopy(marker)
        text.ns, text.type, text.text = 'navigation/labels', Marker.TEXT_VIEW_FACING, item['label']
        text.pose.position.z += .35
        text.scale.z = .22
        text.color.r = text.color.g = text.color.b = 1.
        result.append(text)
    lines = Marker(header=deepcopy(header), ns='navigation/paths', id=0,
                   type=Marker.LINE_LIST, action=Marker.ADD)
    lines.pose.orientation.w = 1.
    lines.scale.x = .055
    lines.color.r, lines.color.g, lines.color.b, lines.color.a = .3, .85, .45, 1.
    for edge in graph['edges']:
        for a, b in zip(edge['polyline'], edge['polyline'][1:]):
            lines.points.extend(Point(x=float(p[0]), y=float(p[1]), z=float(p[2]+.08)) for p in (a, b))
    if lines.points:
        result.append(lines)
    return MarkerArray(markers=result)


def process_grid(message, config):
    info = message.info
    if not 0 < info.width*info.height <= config.max_cells or len(message.data) != info.width*info.height:
        raise ValueError('invalid occupancy grid dimensions')
    q = info.origin.orientation
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(values).all() or norm < 1e-8:
        raise ValueError('invalid occupancy grid orientation')
    values /= norm
    if np.linalg.norm(values[:2]) > 1e-5:
        raise ValueError('navigation requires a planar occupancy grid origin')
    yaw = 2*math.atan2(values[2], values[3])
    p = info.origin.position
    data = np.asarray(message.data, dtype=np.int16).reshape(info.height, info.width)
    graph, labels, clearance = build_navigation_graph(data, float(info.resolution), (p.x, p.y, p.z), yaw, config)
    return graph, labels


class NavigationTopologyNode(Node):
    def __init__(self):
        super().__init__('navigation_topology')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.config = NavigationConfig(**{f.name: param(f.name, getattr(NavigationConfig(), f.name)) for f in fields(NavigationConfig)})
        self.topic = str(param('grid_topic', '/miti/grid_map'))
        self.interval = float(param('update_interval', 2.0))
        if not math.isfinite(self.interval) or self.interval <= 0:
            raise ValueError('update_interval must be finite and positive')
        self.export = Path(param('export_path', str(Path(__file__).resolve().parents[1]/'maps/current_navigation_graph.json')))
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/miti/navigation_map', latched)
        self.graph_pub = self.create_publisher(String, '/miti/navigation_graph', latched)
        self.status_pub = self.create_publisher(String, '/miti/navigation_status', latched)
        self.create_subscription(OccupancyGrid, self.topic, self.on_grid, latched)
        self.create_subscription(PoseStamped, '/robot_pose', self.on_pose, qos_profile_sensor_data)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='navigation_topology')
        self.job = self.pending = self.last_key = self.labels = self.graph = self.pose = None
        self.epoch = 1
        self.frame = 'map'
        self.last_start = -float('inf')
        self.status = self.waiting('等待二维空闲栅格')
        self.create_timer(.2, self.update)
        self.create_timer(1., self.publish_status)
        self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        self.get_logger().info(f'Navigation regions from {self.topic}; clearance {self.config.clearance:.2f} m; display only')

    def waiting(self, reason):
        return dict(schema=1, kind='navigation', state='waiting', reason=reason, frame_id=self.frame,
                    epoch=self.epoch, topology=dict(nodes=0, links=0, regions=0, corridors=0,
                    doorways=0, components=0, bounds=None, required_clearance_m=self.config.clearance,
                    source_topic=self.topic))

    def on_pose(self, message):
        self.pose = (message, time.monotonic())

    def on_grid(self, message):
        frame = message.header.frame_id.strip('/')
        if not frame:
            return
        info = message.info
        if not 0 < info.width*info.height <= self.config.max_cells or len(message.data) != info.width*info.height:
            self.epoch += 1
            self.pending = self.last_key = self.graph = self.labels = None
            self.status = self.waiting('栅格尺寸无效或超过处理上限')
            self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            self.publish_empty_graph()
            self.publish_status()
            return
        p, q = info.origin.position, info.origin.orientation
        key = (frame, info.width, info.height, info.resolution, p.x, p.y, p.z, q.x, q.y, q.z, q.w,
               hashlib.sha256(np.asarray(message.data, dtype=np.int8).tobytes()).hexdigest())
        if key == self.last_key:
            return
        if frame != self.frame:
            self.epoch += 1
            self.graph = self.labels = None
            self.frame = frame
            self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            self.status = self.waiting('坐标系已变化，等待新导航图')
            self.publish_empty_graph()
        self.last_key = key
        self.pending = (message, key, self.epoch)

    def update(self):
        if self.job is not None and self.job[0].done():
            future, message, key, epoch = self.job
            self.job = None
            try:
                graph, labels = future.result()
                if epoch == self.epoch:
                    graph.update(frame_id=message.header.frame_id.strip('/'), source_topic=self.topic,
                                 source_stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                                 revision=hashlib.sha256(repr(key).encode()).hexdigest()[:16])
                    self.graph, self.labels = graph, labels
                    self.markers.publish(navigation_markers(graph, message.header))
                    serialized = json.dumps(graph, ensure_ascii=False, allow_nan=False)
                    self.graph_pub.publish(String(data=serialized))
                    self.export.parent.mkdir(parents=True, exist_ok=True)
                    temporary = self.export.with_suffix('.json.tmp')
                    temporary.write_text(serialized+'\n')
                    temporary.replace(self.export)
                    kinds = [n['kind'] for n in graph['nodes']]
                    self.status = dict(schema=1, kind='navigation', state='ready' if graph['nodes'] else 'waiting',
                        reason=graph['reason'], frame_id=graph['frame_id'], epoch=self.epoch,
                        updated_at=time.time(), revision=graph['revision'], topology=dict(
                            nodes=len(graph['nodes']), links=len(graph['edges']), regions=kinds.count('area'),
                            corridors=kinds.count('corridor'), doorways=kinds.count('doorway_candidate'),
                            components=len(graph['components']), bounds=graph['bounds'] or graph['grid_bounds'],
                            required_clearance_m=graph['required_clearance_m'], source_topic=self.topic,
                            observed_free_cells=graph['observed_free_cells'], traversable_cells=graph['traversable_cells'],
                            max_observed_clearance_m=graph['max_observed_clearance_m']))
                    self.publish_status()
            except Exception as error:
                self.graph = self.labels = None
                self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
                self.status = self.waiting('导航图生成失败：' + str(error))
                self.publish_empty_graph()
                self.publish_status()
                self.get_logger().error(str(error))
        if self.job is None and self.pending is not None and time.monotonic()-self.last_start >= self.interval:
            message, key, epoch = self.pending
            self.pending = None
            self.last_start = time.monotonic()
            self.job = (self.pool.submit(process_grid, message, self.config), message, key, epoch)

    def publish_status(self):
        region = None
        if self.graph and self.pose and time.monotonic()-self.pose[1] < 2:
            pose = self.pose[0]
            if pose.header.frame_id.strip('/') == self.graph['frame_id'] and all(math.isfinite(v) for v in (
                    pose.pose.position.x, pose.pose.position.y)):
                p = pose.pose.position
                dx, dy = p.x-self.graph['origin'][0], p.y-self.graph['origin'][1]
                c, s, r = math.cos(self.graph['yaw']), math.sin(self.graph['yaw']), self.graph['resolution']
                x, y = math.floor((c*dx+s*dy)/r), math.floor((-s*dx+c*dy)/r)
                if 0 <= y < self.labels.shape[0] and 0 <= x < self.labels.shape[1] and self.labels[y, x] > 0:
                    region = 'R' + str(self.labels[y, x])
        self.status['robot_region'] = region
        self.status_pub.publish(String(data=json.dumps(self.status, ensure_ascii=False, allow_nan=False)))

    def publish_empty_graph(self):
        empty = dict(schema=1, kind='navigation', state='waiting', reason=self.status['reason'],
                     frame_id=self.frame, nodes=[], edges=[], components=[], revision=None)
        serialized = json.dumps(empty, ensure_ascii=False)
        self.graph_pub.publish(String(data=serialized))
        try:
            self.export.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.export.with_suffix('.json.tmp')
            temporary.write_text(serialized+'\n')
            temporary.replace(self.export)
        except OSError as error:
            self.get_logger().error('Cannot clear navigation export: ' + str(error))

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = NavigationTopologyNode()
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
