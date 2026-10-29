#!/usr/bin/env python3
"""Exercise the compiled bridge and mapper with known RGB-D geometry.

The detector is injected only in this test, so exact geometric assertions do not
depend on a model recognizing a synthetic picture. Real YOLO is checked separately.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import sys

import cv2
import numpy as np
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose, Transform
from rtabmap_msgs.msg import MapData, MapGraph, Node as GraphNode
from rtabmap_semantic_msgs.msg import SemanticMap
from sensor_msgs.msg import CameraInfo

from rtabmap_semantic.node import SemanticMapper


class FixtureDetector:
    def detect(self, rgb):
        mask = np.zeros(rgb.shape[:2], np.uint8)
        mask[35:65, 35:65] = 1
        return [dict(label='chair', confidence=0.9, mask=mask)]


def main():
    real_yolo = '--real-yolo' in sys.argv
    project = Path(__file__).resolve().parents[3]
    rclpy.init()
    with tempfile.TemporaryDirectory(prefix='semantic_ros_') as directory:
        log = open(Path(directory)/'bridge.log', 'w+')
        executable = Path(get_package_prefix('rtabmap_semantic'))/'lib/rtabmap_semantic/keyframe_bridge'
        process = subprocess.Popen([str(executable),
                                    '--ros-args', '-p', 'replay_existing:=false'],
                                   stdout=log, stderr=subprocess.STDOUT)
        executor = SingleThreadedExecutor()
        probe = rclpy.create_node('semantic_pipeline_probe')
        params = [Parameter('database_path', value=str(Path(directory)/'map.db'))]
        if real_yolo:
            params.append(Parameter('model_path', value=str(project/'models/yolov8n-seg.pt')))
        mapper = SemanticMapper(detector=None if real_yolo else FixtureDetector(), parameter_overrides=params)
        executor.add_node(probe)
        executor.add_node(mapper)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps = probe.create_publisher(MapData, '/mapData', 5)
        graphs = probe.create_publisher(MapGraph, '/mapGraph', 5)
        received = []
        subscription = probe.create_subscription(SemanticMap, '/semantic/objects', received.append, latched)
        cv = CvBridge()
        if real_yolo:
            import ultralytics
            rgb = cv2.imread(str(Path(ultralytics.__file__).parent/'assets/bus.jpg'))
            assert rgb is not None
        else:
            rgb = np.full((100, 100, 3), 128, np.uint8)
        height, width = rgb.shape[:2]
        target_label = 'bus' if real_yolo else 'chair'

        def until(predicate, seconds=15):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                if process.poll() is not None:
                    raise AssertionError(f'Bridge exited with code {process.returncode}')
                executor.spin_once(timeout_sec=0.05)
                if predicate():
                    return
            raise AssertionError('Timed out waiting for ROS pipeline condition')

        try:
            until(lambda: maps.get_subscription_count() >= 1)
            for node_id in range(1, 4):
                data = MapData()
                data.header.frame_id = 'map'
                data.header.stamp.sec = node_id
                data.graph.header = data.header
                data.graph.poses_id = list(range(1, node_id+1))
                data.graph.poses = [Pose() for _ in range(node_id)]
                for pose in data.graph.poses:
                    pose.orientation.w = 1.0
                node = GraphNode(id=node_id, stamp=float(node_id))
                node.data.header.stamp.sec = node_id
                node.data.header.frame_id = 'camera_color_optical_frame'
                depth = np.full((height, width), 2000, np.uint16)
                if node_id == 2:
                    # RTAB-Map's PNG RGB-D compression, matching saved keyframe payloads.
                    node.data.left_compressed = cv2.imencode('.png', rgb)[1].tobytes()
                    node.data.right_compressed = cv2.imencode('.png', depth)[1].tobytes()
                else:
                    node.data.left = cv.cv2_to_imgmsg(rgb, 'bgr8')
                    node.data.right = cv.cv2_to_imgmsg(depth, '16UC1')
                camera = CameraInfo(width=width, height=height, distortion_model='plumb_bob')
                fx, fy, cx, cy = float(width), float(width), (width-1)/2, (height-1)/2
                camera.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
                camera.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
                camera.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                node.data.left_camera_info = [camera]
                transform = Transform()
                transform.rotation.w = 1.
                transform.translation.x = 0.2
                node.data.local_transform = [transform]
                data.nodes = [node]
                maps.publish(data)
                until(lambda: mapper.processed == node_id)
            until(lambda: received and any(o.label == target_label for o in received[-1].objects))
            obj = next(o for o in received[-1].objects if o.label == target_label)
            assert len(obj.observations) == 3
            if not real_yolo:
                np.testing.assert_allclose([obj.observations[0].position.x,
                                        obj.observations[0].position.y,
                                        obj.observations[0].position.z], [0.2, 0, 2], atol=1e-5)
            else:
                assert abs(obj.observations[0].position.z-2) < 1e-5
            tracked = next(o for o in mapper.state.objects if o.label == target_label)
            before = mapper.state.position(tracked).copy()
            data.graph.poses[0].position.x = 0.1
            data.graph.poses[1].position.x = 0.2
            data.graph.poses[2].position.x = 0.3
            graphs.publish(data.graph)
            until(lambda: abs(mapper.state.position(tracked)[0]-(before[0]+0.2)) < 1e-5)
            original_uuid = mapper.state.map_uuid
            executor.remove_node(mapper)
            mapper.destroy_node()
            mapper = SemanticMapper(detector=None if real_yolo else FixtureDetector(), parameter_overrides=params)
            executor.add_node(mapper)
            assert mapper.state.map_uuid == original_uuid
            assert any(o.label == target_label for o in mapper.state.objects)
            graphs.publish(data.graph)
            until(lambda: any(o.label == target_label for o in mapper.state.visible_objects()))
            assert next(o for o in mapper.state.visible_objects() if o.label == target_label).object_id == obj.object_id
            data.nodes[0].id = 1
            data.nodes[0].stamp = 101.0
            data.nodes[0].data.header.stamp.sec = 101
            data.graph.poses_id = [1]
            reset_pose = Pose()
            reset_pose.orientation.w = 1.0
            data.graph.poses = [reset_pose]
            maps.publish(data)
            until(lambda: mapper.state.map_uuid != original_uuid)
            assert set(mapper.state.nodes) == {1}
            assert not any(o.label == target_label for o in mapper.state.visible_objects())
            report = dict(passed=True, frames=3, label=target_label, real_yolo=real_yolo,
                          compressed_keyframe=True, projection=True, loop_reprojection=True,
                          reload=True, map_reset=True)
            print(json.dumps(report))
            output = project/'logs/semantic_validation'
            output.mkdir(parents=True, exist_ok=True)
            (output/('ros_real_yolo.json' if real_yolo else 'ros_pipeline.json')).write_text(json.dumps(report, indent=2))
        finally:
            executor.remove_node(mapper)
            mapper.destroy_node()
            executor.remove_node(probe)
            probe.destroy_node()
            executor.shutdown()
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            log.seek(0)
            print(log.read())
            log.close()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
