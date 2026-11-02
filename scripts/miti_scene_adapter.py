#!/usr/bin/env python3
"""Present RTAB-Map's optimized graph and object anchors as standard ROS display topics."""
import colorsys
import json
import math
import os
import time
import zlib
from copy import deepcopy
from pathlib import Path as FilePath

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rtabmap_msgs.msg import MapGraph, RGBDImage
from rtabmap_msgs.srv import GetMap
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray
from guard_rgbd_input import validate_rgbd


def pose_matrix(pose):
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z,
                  pose.orientation.w], dtype=float)
    p = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
    if not np.isfinite(q).all() or not np.isfinite(p).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('invalid pose')
    x, y, z, w = q / np.linalg.norm(q)
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = p
    return matrix


def object_position(obj, poses):
    total = np.zeros(3)
    weight = 0.0
    for anchor in obj.observations:
        if anchor.node_id not in poses or not math.isfinite(anchor.weight) or anchor.weight <= 0:
            continue
        point = np.array([anchor.position.x, anchor.position.y, anchor.position.z, 1.0])
        if not np.isfinite(point).all():
            continue
        total += (poses[anchor.node_id] @ point)[:3] * anchor.weight
        weight += anchor.weight
    return total / weight if weight > 0 else None


def markers_for(message, poses, header, positions=None):
    """Build ADD/upsert markers, retaining a known position while its anchors are in transit."""
    positions = {} if positions is None else positions
    current_ids = {obj.object_id for obj in message.objects}
    for object_id in list(positions):
        if object_id not in current_ids:
            del positions[object_id]
    result = MarkerArray()
    for obj in message.objects:
        position = object_position(obj, poses)
        if position is None:
            position = positions.get(obj.object_id)
        if position is None:
            continue
        positions[obj.object_id] = position
        color = ((1.0, 0.85, 0.1) if obj.dynamic else
                 colorsys.hsv_to_rgb((zlib.crc32(obj.label.encode()) % 360) / 360, 0.65, 1.0))
        for kind, marker_type in [('point', Marker.SPHERE), ('label', Marker.TEXT_VIEW_FACING)]:
            # object_id is int64; encode it in ns instead of overflowing Marker's int32 id.
            marker = Marker(header=deepcopy(header), ns=f'miti/{obj.object_id}/{kind}',
                            id=0, type=marker_type, action=Marker.ADD)
            marker.pose.orientation.w = 1.0
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, position)
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            if kind == 'label':
                marker.text = obj.label
                marker.pose.position.z += 0.18
                marker.scale.z = 0.16
            else:
                marker.scale.x = marker.scale.y = marker.scale.z = 0.08
            result.markers.append(marker)
    return result


class SemanticMarkers:
    """Complete live snapshot with stable identities and persistent targeted deletions."""
    def __init__(self):
        self.positions = {}
        self.known_keys = set()
        self.session = None
        self.separate_pairs = set()
        self.label_winners = {}
        self.groups = {}
        self.restored_cache = None

    def restore_cache(self, cache):
        if (not isinstance(cache, dict) or cache.get('schema') != 1 or
                not isinstance(cache.get('session'), list) or len(cache['session']) != 3 or
                not all(isinstance(v, str) for v in cache['session']) or
                not isinstance(cache.get('positions'), dict) or
                not isinstance(cache.get('label_winners', {}), dict)):
            raise ValueError('Invalid semantic display cache')
        positions = {int(key): np.asarray(value, dtype=float) for key, value in cache['positions'].items()}
        if any(key <= 0 or value.shape != (3,) or not np.isfinite(value).all() for key, value in positions.items()):
            raise ValueError('Invalid cached semantic position')
        pairs = {tuple(sorted((int(a), int(b)))) for a, b in cache.get('separate_pairs', [])}
        winners = {int(key): int(value) for key, value in cache.get('label_winners', {}).items()}
        self.restored_cache = (tuple(cache['session']), positions, pairs, winners)

    def cache_snapshot(self):
        if self.session is None:
            return None
        return dict(schema=1, session=list(self.session),
                    positions={str(key): value.tolist() for key, value in self.positions.items()},
                    separate_pairs=sorted(self.separate_pairs), label_winners=self.label_winners)

    def reset(self):
        self.positions.clear()
        self.session = None
        self.separate_pairs.clear()
        self.label_winners.clear()
        self.groups.clear()

    def unique_objects(self, message, markers):
        """Resolve old duplicate tracks using map position and independent-instance evidence.

        Names alone never identify an object. Two detections seen together in one
        keyframe remain separate even if their map centers are very close.
        """
        objects = {obj.object_id: obj for obj in message.objects}
        anchors = {key: {(obs.node_id, obs.node_stamp) for obs in obj.observations}
                   for key, obj in objects.items()}
        all_ids = sorted(objects)
        for i, a in enumerate(all_ids):
            for b in all_ids[i + 1:]:
                if anchors[a] & anchors[b]:
                    self.separate_pairs.add((a, b))
        previous_pairs = {(a, b) for group in self.groups.values()
                          for a in group for b in group if a < b}
        groups = []
        for key in (key for key in all_ids if key in self.positions):
            for group in groups:
                if all(objects[key].dynamic == objects[other].dynamic and
                       (min(key, other), max(key, other)) not in self.separate_pairs and
                       np.linalg.norm(self.positions[key] - self.positions[other]) <=
                       (0.12 if (min(key, other), max(key, other)) in previous_pairs else 0.08)
                       for other in group):
                    group.append(key)
                    break
            else:
                groups.append([key])
        by_identity = {(m.ns, m.id): m for m in markers}
        result, winners = [], {}
        self.groups = {group[0]: group for group in groups}
        for group in groups:
            identity = group[0]
            scores = {key: objects[key].confidence * max(1, len(anchors[key])) for key in group}
            winner = max(group, key=lambda key: (scores[key], -key))
            previous = self.label_winners.get(identity)
            # A small confidence fluctuation must not keep changing the displayed name.
            if previous in group and scores[winner] < scores[previous] * 1.25:
                winner = previous
            winners[identity] = winner
            for kind in ('point', 'label'):
                marker = by_identity[(f'miti/{identity}/{kind}', 0)]
                marker.color = deepcopy(by_identity[(f'miti/{winner}/{kind}', 0)].color)
                if kind == 'label':
                    marker.text = objects[winner].label
                result.append(marker)
        self.label_winners = winners
        return result

    def update(self, message, poses, header):
        session = (message.map_key, message.map_uuid, header.frame_id)
        changed_session = session != self.session
        if changed_session:
            self.reset()
            cache, self.restored_cache = self.restored_cache, None
            if cache is not None and cache[0] == session:
                self.positions, self.separate_pairs, self.label_winners = cache[1:]
        self.session = session
        result = markers_for(message, poses, header, self.positions)
        result.markers = self.unique_objects(message, result.markers)
        current_keys = {(marker.ns, marker.id) for marker in result.markers}
        # Repeat targeted deletions so a throttled/reconnecting viewer cannot miss a removal.
        removed = [Marker(header=deepcopy(header), ns=ns, id=marker_id, action=Marker.DELETE)
                   for ns, marker_id in sorted(self.known_keys - current_keys)]
        self.known_keys.update(current_keys)
        if changed_session:
            removed.insert(0, Marker(header=deepcopy(header), action=Marker.DELETEALL))
        result.markers = removed + result.markers
        return result


class MitiSceneAdapter(Node):
    def __init__(self):
        super().__init__('miti_scene_adapter')
        self.poses = {}
        self.graph = None
        self.objects = None
        self.session = None
        self.markers = SemanticMarkers()
        state_dir = os.environ.get('MITI_STATE_DIR')
        self.cache_path = FilePath(state_dir) / 'semantic_display.json' if state_dir else None
        if self.cache_path is not None and self.cache_path.is_file():
            try:
                self.markers.restore_cache(json.loads(self.cache_path.read_text()))
            except (ValueError, TypeError, KeyError, OSError) as error:
                self.get_logger().warning('Cannot load semantic display cache: ' + str(error))
        self.cache_epoch = 0
        self.graph_request = None
        self.graph_request_epoch = 0
        self.last_graph_request = -float('inf')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/miti/trajectory', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/miti/semantic_markers', latched)
        # Reuse the SLAM input checks. The driver's aligned-depth topic can contain
        # raw-sized frames, so expose only complete, consistent RGB-D pairs to MITI.
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.depth_pub = self.create_publisher(Image, '/miti/aligned_depth/image_raw', image_qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/miti/aligned_depth/camera_info', image_qos)
        self.create_subscription(RGBDImage, '/rgbd_image', self.on_rgbd, qos_profile_sensor_data)
        self.create_subscription(MapGraph, '/mapGraph', self.on_graph, 5)
        self.map_client = self.create_client(GetMap, '/rtabmap/get_map_data')
        self.create_timer(1.0, self.fill_missing_poses)
        self.create_timer(2.0, self.save_display_cache)
        try:
            from rtabmap_semantic_msgs.msg import SemanticMap
            self.create_subscription(SemanticMap, '/semantic/objects', self.on_objects, latched)
        except ImportError:
            self.get_logger().warning('Semantic messages unavailable; trajectory display is enabled.')
        self.get_logger().info('MITI display adapter ready; waiting for /mapGraph, /semantic/objects and /rgbd_image')

    def on_rgbd(self, message):
        error = validate_rgbd(message)
        if error:
            self.get_logger().warning('Skipped invalid display depth: ' + error, throttle_duration_sec=5.0)
            return
        # Alignment uses color intrinsics; retain the original pixels, units and timestamps.
        self.depth_info_pub.publish(message.rgb_camera_info)
        self.depth_pub.publish(message.depth)

    def on_graph(self, graph):
        if self.graph is not None and (self.graph.header.frame_id or 'map') != (graph.header.frame_id or 'map'):
            self.clear_scene()
        poses = {}
        valid = []
        for node_id, pose in zip(graph.poses_id, graph.poses):
            if node_id <= 0:
                continue
            try:
                poses[node_id] = pose_matrix(pose)
            except ValueError:
                continue
            valid.append((node_id, pose))
        # /mapGraph can contain only a local subgraph. Omitted anchors are not deleted nodes.
        self.graph = graph
        self.poses.update(poses)
        header = deepcopy(graph.header)
        header.frame_id = header.frame_id or 'map'
        # The points already use optimized map coordinates; latest time avoids stale TF lookup.
        header.stamp = self.get_clock().now().to_msg()
        path = Path(header=header)
        path.poses = [PoseStamped(header=header, pose=pose) for _, pose in sorted(valid)]
        self.path_pub.publish(path)
        self.publish_markers()

    def on_objects(self, message):
        session = (message.map_key, message.map_uuid)
        frame_changed = self.objects is not None and (self.objects.header.frame_id or 'map') != (message.header.frame_id or 'map')
        if (self.session is not None and self.session != session) or frame_changed:
            self.clear_scene()
        self.session, self.objects = session, message
        self.publish_markers()

    def clear_scene(self):
        self.poses.clear()
        self.graph = None
        self.cache_epoch += 1
        self.markers.reset()
        self.path_pub.publish(Path())

    def restore_graph(self, epoch, data):
        # Ignore a response requested before a map/session change, and never overwrite
        # newer streaming poses with an older service snapshot.
        if epoch != self.cache_epoch or self.objects is None:
            return
        graph = data.graph
        graph.header = deepcopy(data.header)
        if (graph.header.frame_id or 'map') != (self.objects.header.frame_id or 'map'):
            return
        for node_id, pose in zip(graph.poses_id, graph.poses):
            if node_id <= 0 or node_id in self.poses:
                continue
            try:
                self.poses[node_id] = pose_matrix(pose)
            except ValueError:
                continue
        if self.graph is None:
            self.graph = graph
        self.publish_markers()

    def fill_missing_poses(self):
        now = time.monotonic()
        if self.graph_request is not None:
            if self.graph_request.done():
                future, self.graph_request = self.graph_request, None
                try:
                    self.restore_graph(self.graph_request_epoch, future.result().data)
                except Exception as error:
                    self.get_logger().warning('Cannot restore display anchors: ' + str(error), throttle_duration_sec=5.0)
            elif now - self.last_graph_request >= 5.0:
                self.map_client.remove_pending_request(self.graph_request)
                self.graph_request.cancel()
                self.graph_request = None
            else:
                return
        if self.objects is None or now - self.last_graph_request < 5.0 or not self.map_client.service_is_ready():
            return
        if not any(anchor.node_id not in self.poses for obj in self.objects.objects for anchor in obj.observations):
            return
        self.graph_request_epoch = self.cache_epoch
        self.last_graph_request = now
        self.graph_request = self.map_client.call_async(GetMap.Request(global_map=True, optimized=True, graph_only=True))

    def publish_markers(self):
        if self.objects is None:
            return
        header = deepcopy(self.objects.header)
        header.frame_id = header.frame_id or 'map'
        header.stamp = self.get_clock().now().to_msg()
        poses = self.poses
        if self.graph is None:
            poses = {}
        elif (self.graph.header.frame_id or 'map') != header.frame_id:
            poses = {}
            self.markers.positions.clear()
        self.marker_pub.publish(self.markers.update(self.objects, poses, header))

    def save_display_cache(self):
        snapshot = self.markers.cache_snapshot()
        if self.cache_path is None or snapshot is None:
            return
        try:
            self.cache_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.cache_path.with_suffix(f'.{os.getpid()}.tmp')
            temporary.write_text(json.dumps(snapshot, allow_nan=False))
            temporary.replace(self.cache_path)
        except OSError as error:
            self.get_logger().warning('Cannot save semantic display cache: ' + str(error), throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = MitiSceneAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save_display_cache()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
