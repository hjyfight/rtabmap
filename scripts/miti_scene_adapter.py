#!/usr/bin/env python3
"""Present RTAB-Map's optimized graph and object anchors as standard ROS display topics."""
import colorsys
import math
import zlib
from copy import deepcopy

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rtabmap_msgs.msg import MapGraph, RGBDImage
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


def markers_for(message, poses, header):
    # Every semantic message is a complete snapshot. DELETEALL also removes vanished objects
    # and avoids retaining labels when a new map/session is loaded.
    result = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
    for obj in message.objects:
        position = object_position(obj, poses)
        if position is None:
            continue
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


class MitiSceneAdapter(Node):
    def __init__(self):
        super().__init__('miti_scene_adapter')
        self.poses = {}
        self.graph = None
        self.objects = None
        self.session = None
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
        self.graph, self.poses = graph, poses
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
        if self.session is not None and self.session != session:
            self.poses = {}
            self.graph = None
            self.path_pub.publish(Path())
        self.session, self.objects = session, message
        self.publish_markers()

    def publish_markers(self):
        if self.objects is None:
            return
        header = deepcopy(self.objects.header)
        header.frame_id = header.frame_id or 'map'
        header.stamp = self.get_clock().now().to_msg()
        poses = self.poses
        if self.graph is None or (self.graph.header.frame_id or 'map') != header.frame_id:
            poses = {}
        self.marker_pub.publish(markers_for(self.objects, poses, header))


def main():
    rclpy.init()
    node = MitiSceneAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
