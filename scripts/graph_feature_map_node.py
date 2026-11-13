#!/usr/bin/env python3
"""Display the optimized RTAB-Map graph and stored visual words in MITI."""
import json
import math
import time
from copy import deepcopy

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import MapData, MapGraph
from rtabmap_msgs.srv import GetMap2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from miti_scene_adapter import pose_matrix


LINK_STYLES = {
    'neighbor': ((0, 6), (0.35, 0.65, 1.0)),
    'global_loop': ((1,), (1.0, 0.35, 0.55)),
    'local_loop': ((2, 3), (1.0, 0.75, 0.2)),
    'user': ((4,), (0.75, 0.5, 1.0)),
    'virtual': ((5,), (0.45, 0.55, 0.65)),
    'landmark': ((8,), (0.35, 0.9, 0.7)),
}


def bounds(points):
    return None if not len(points) else dict(
        center=((points.min(axis=0) + points.max(axis=0)) / 2).tolist(),
        span=(points.max(axis=0) - points.min(axis=0)).tolist())


class GraphFeatures:
    """Cache local updates; complete service snapshots reconcile removed graph nodes."""
    def __init__(self, voxel_size=0.08, max_points=15000):
        self.voxel_size, self.max_points = voxel_size, max_points
        self.epoch = 0
        self.revision = 0
        self.clear()

    def clear(self):
        self.epoch += 1
        self.frame = None
        self.poses, self.links, self.features, self.identities = {}, {}, {}, {}
        self.pose_revisions, self.link_revisions, self.feature_revisions = {}, {}, {}

    def update(self, graph, nodes=(), frame='map', snapshot_revision=None):
        frame = (frame or graph.header.frame_id or 'map').strip('/')
        # Reused keyframe ids with a different acquisition stamp indicate a new map.
        if (self.frame is not None and self.frame != frame) or any(
                n.id in self.identities and self.identities[n.id] != (n.map_id, n.stamp)
                and (snapshot_revision is None or self.feature_revisions[n.id] <= snapshot_revision)
                for n in nodes if n.id > 0):
            self.clear()
            snapshot_revision = None
        self.frame = frame
        self.revision += 1
        revision = self.revision
        valid = {}
        for node_id, pose in zip(graph.poses_id, graph.poses):
            try:
                valid[node_id] = pose_matrix(pose)
            except ValueError:
                continue
        if snapshot_revision is not None:
            for node_id in list(self.poses):
                if node_id not in valid and self.pose_revisions[node_id] <= snapshot_revision:
                    self.poses.pop(node_id)
                    self.pose_revisions.pop(node_id)
                    self.features.pop(node_id, None)
                    self.identities.pop(node_id, None)
                    self.feature_revisions.pop(node_id, None)
        for node_id, matrix in valid.items():
            if snapshot_revision is None or self.pose_revisions.get(node_id, -1) <= snapshot_revision:
                self.poses[node_id] = matrix
                self.pose_revisions[node_id] = revision
        keys = set()
        for link in graph.links:
            if link.from_id == link.to_id or link.type not in {t for types, _ in LINK_STYLES.values() for t in types}:
                continue
            key = (min(link.from_id, link.to_id), max(link.from_id, link.to_id), link.type)
            keys.add(key)
            if snapshot_revision is None or self.link_revisions.get(key, -1) <= snapshot_revision:
                self.links[key] = key
                self.link_revisions[key] = revision
        if snapshot_revision is not None:
            for key in list(self.links):
                if key not in keys and self.link_revisions[key] <= snapshot_revision:
                    self.links.pop(key)
                    self.link_revisions.pop(key)
        for node in nodes:
            if node.id <= 0 or (snapshot_revision is not None and
                    self.feature_revisions.get(node.id, -1) > snapshot_revision):
                continue
            # An update without word data must not erase words fetched from the DB.
            if not node.word_pts and node.id in self.features:
                continue
            points = np.asarray([[p.x, p.y, p.z] for p in node.word_pts], dtype=float).reshape(-1, 3)
            points = points[np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > 1e-6)]
            self.features[node.id] = points
            self.identities[node.id] = (node.map_id, node.stamp)
            self.feature_revisions[node.id] = revision

    def graph_markers(self, header):
        def marker(name, kind, color, size):
            result = Marker(header=deepcopy(header), ns='rtabmap/topology/' + name,
                            id=0, type=kind, action=Marker.ADD)
            result.pose.orientation.w = 1.0
            result.scale.x = result.scale.y = result.scale.z = size
            result.color.r, result.color.g, result.color.b, result.color.a = *color, 1.0
            return result

        result = []
        nodes = marker('nodes', Marker.SPHERE_LIST, (0.25, 0.9, 0.8), 0.12)
        nodes.points = [Point(x=float(m[0, 3]), y=float(m[1, 3]), z=float(m[2, 3]))
                        for key, m in sorted(self.poses.items()) if key > 0]
        result.append(nodes)
        counts = {}
        for name, (types, color) in LINK_STYLES.items():
            lines = marker(name, Marker.LINE_LIST, color, 0.035 if name == 'neighbor' else 0.055)
            for a, b, kind in sorted(self.links):
                if kind in types and a in self.poses and b in self.poses:
                    lines.points.extend(Point(x=float(self.poses[key][0, 3]), y=float(self.poses[key][1, 3]),
                                              z=float(self.poses[key][2, 3])) for key in (a, b))
            counts[name] = len(lines.points) // 2
            result.append(lines)
        # Fixed marker identities plus explicit DELETE also clear stale/empty layers.
        for item in result:
            if not item.points:
                item.action = Marker.DELETE
        coordinates = np.asarray([m[:3, 3] for m in self.poses.values()]).reshape(-1, 3)
        return MarkerArray(markers=result), dict(nodes=len(nodes.points), links=sum(counts.values()),
                                                 link_counts=counts, bounds=bounds(coordinates))

    def feature_cloud(self, header):
        transformed = [points @ self.poses[key][:3, :3].T + self.poses[key][:3, 3]
                       for key, points in sorted(self.features.items()) if key in self.poses and len(points)]
        points = np.concatenate(transformed) if transformed else np.empty((0, 3))
        observations = len(points)
        if len(points) and self.voxel_size > 0:
            _, indices = np.unique(np.floor(points / self.voxel_size), axis=0, return_index=True)
            points = points[np.sort(indices)]
        unique = len(points)
        cloud_bounds = bounds(points)
        if len(points) > self.max_points:
            points = points[np.linspace(0, len(points)-1, self.max_points, dtype=int)]
        raw = points.astype('<f4').tobytes()
        cloud = PointCloud2(header=deepcopy(header), height=1, width=len(points),
                            fields=[PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                    for i, name in enumerate(('x', 'y', 'z'))],
                            is_bigendian=False, point_step=12, row_step=12*len(points),
                            data=raw, is_dense=True)
        return cloud, dict(points=len(points), observations=observations, unique_points=unique,
                           feature_nodes=sum(key > 0 and len(p) > 0 and key in self.poses for key, p in self.features.items()),
                           loaded_nodes=sum(key > 0 and key in self.poses for key in self.features),
                           voxel_size=self.voxel_size, bounds=cloud_bounds,
                           reason='' if len(points) else '当前地图没有有效三维视觉特征，等待 RGB-D 关键帧')


class GraphFeatureMapNode(Node):
    def __init__(self):
        super().__init__('graph_feature_map')
        param = lambda name, default: self.declare_parameter(name, default).value
        interval = float(param('snapshot_interval', 15.0))
        publish_interval = float(param('publish_interval', 1.0))
        voxel = float(param('feature_voxel_size', 0.08))
        maximum = int(param('max_feature_points', 15000))
        if not all(math.isfinite(v) and v > 0 for v in (interval, publish_interval)) or not math.isfinite(voxel) or voxel < 0 or maximum < 1:
            raise ValueError('Invalid graph/feature display parameters')
        self.model = GraphFeatures(voxel, maximum)
        self.interval = interval
        self.pending = None
        self.last_request = -float('inf')
        self.last_clock = None
        self.dirty = True
        self.status = dict(schema=1, state='waiting', reason='等待 RTAB-Map 位姿图', topology={}, features={})
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.graph_pub = self.create_publisher(MarkerArray, '/miti/topological_map', latched)
        self.feature_pub = self.create_publisher(PointCloud2, '/miti/sparse_features', latched)
        self.status_pub = self.create_publisher(String, '/miti/graph_feature_status', latched)
        # Volatile subscriptions also match both volatile and latched RTAB-Map publishers.
        self.create_subscription(MapGraph, '/mapGraph', self.on_graph, 1)
        self.create_subscription(MapData, '/mapData', self.on_data, 1)
        self.client = self.create_client(GetMap2, '/rtabmap/get_map_data2')
        self.create_timer(0.5, self.fetch_snapshot)
        self.create_timer(publish_interval, self.publish_maps)
        self.get_logger().info('Topology /miti/topological_map; visual words /miti/sparse_features')

    def on_graph(self, graph):
        self.model.update(graph, frame=graph.header.frame_id)
        self.dirty = True

    def on_data(self, data):
        self.model.update(data.graph, data.nodes, frame=data.header.frame_id)
        self.dirty = True

    def fetch_snapshot(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_clock is not None and now < self.last_clock - 0.1:
            self.model.clear()
            self.last_request = -float('inf')
            self.dirty = True
        self.last_clock = now
        if self.pending is not None:
            future, epoch, revision, started = self.pending
            if future.done():
                self.pending = None
                try:
                    data = future.result().data
                    if epoch == self.model.epoch:
                        self.model.update(data.graph, data.nodes, frame=data.header.frame_id, snapshot_revision=revision)
                        self.dirty = True
                except Exception as error:
                    self.get_logger().warning('Map display snapshot failed: ' + str(error))
            elif time.monotonic() - started > 10:
                self.client.remove_pending_request(future)
                future.cancel()
                self.pending = None
            return
        if time.monotonic() - self.last_request < self.interval or not self.client.service_is_ready():
            return
        # Fetch words only; no images, dense clouds, scans or grids are requested.
        request = GetMap2.Request(global_map=True, optimized=True, with_words=True)
        self.last_request = time.monotonic()
        self.pending = (self.client.call_async(request), self.model.epoch, self.model.revision, self.last_request)

    def publish_maps(self):
        if self.dirty:
            header = Header(frame_id=self.model.frame or 'map', stamp=self.get_clock().now().to_msg())
            markers, topology = self.model.graph_markers(header)
            cloud, features = self.model.feature_cloud(header)
            self.graph_pub.publish(markers)
            self.feature_pub.publish(cloud)
            self.status = dict(schema=1, state='ready' if topology['nodes'] else 'waiting',
                               reason='' if topology['nodes'] else '等待 RTAB-Map 位姿图',
                               frame_id=header.frame_id, epoch=self.model.epoch,
                               topology=topology, features=features,
                               updated_at=time.time())
            self.dirty = False
        self.status_pub.publish(String(data=json.dumps(self.status, ensure_ascii=False, allow_nan=False)))


def main():
    rclpy.init()
    node = GraphFeatureMapNode()
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
