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


class FixtureRgbdDetector(FixtureDetector):
    def detect(self, rgb, depth):
        # Check that the exact decoded keyframe depth reached the RGB-D backend,
        # including when the bridge decoded RTAB-Map's compressed PNG payload.
        assert depth.dtype == np.uint16
        assert depth.shape == rgb.shape[:2]
        assert np.all(depth == 2000)
        return super().detect(rgb)


def main():
    emsanet_fixture = '--emsanet-fixture' in sys.argv
    real_emsanet = '--real-emsanet' in sys.argv
    real_yoloe = '--real-yoloe' in sys.argv
    real_yolo = '--real-yolo' in sys.argv or real_yoloe
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
        if emsanet_fixture or real_emsanet:
            import yaml
            config = yaml.safe_load((project/'src/rtabmap_semantic/config/emsanet.yaml').read_text())['semantic_mapper']['ros__parameters']
            params.extend([Parameter(name, value=value) for name, value in config.items()])
        if real_emsanet:
            params.append(Parameter('model_path', value=str(project/'models/emsanet/nyuv2_r34_NBt1D_pre.pth')))
        if real_yolo:
            weights = 'yoloe-11s-seg-pf.pt' if real_yoloe else 'yolov8n-seg.pt'
            params.append(Parameter('model_path', value=str(project/'models'/weights)))
        if real_yoloe:
            import yaml
            config = yaml.safe_load((project/'src/rtabmap_semantic/config/yoloe.yaml').read_text())['semantic_mapper']['ros__parameters']
            params.extend([Parameter(name, value=value) for name, value in config.items()])
            params.append(Parameter('detector_backend', value='yoloe_pf'))
        def make_mapper():
            detector = None if real_yolo or real_emsanet else (FixtureRgbdDetector() if emsanet_fixture else FixtureDetector())
            return SemanticMapper(detector=detector, parameter_overrides=params)

        mapper = make_mapper()
        if emsanet_fixture or real_emsanet:
            assert mapper.store.path.name == 'map.emsanet.semantic.sqlite'
        if real_yoloe:
            from rtabmap_semantic.labels import chinese_label, canonical_label
            assert mapper.store.path.name == 'map.yoloe.stable.semantic.sqlite'
            assert all(any('\u3400' <= c <= '\u9fff' for c in chinese_label(canonical_label(name)))
                       for name in mapper.detector.model.names.values())
        executor.add_node(probe)
        executor.add_node(mapper)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps = probe.create_publisher(MapData, '/mapData', 5)
        graphs = probe.create_publisher(MapGraph, '/mapGraph', 5)
        received = []
        subscription = probe.create_subscription(SemanticMap, '/semantic/objects', received.append, latched)
        cv = CvBridge()
        if real_emsanet:
            rgb = cv2.imread(str(project/'.tools/emsanet/samples/sample_rgb.png'))
            sample_depth = cv2.imread(str(project/'.tools/emsanet/samples/sample_depth.png'), cv2.IMREAD_UNCHANGED)
            assert rgb is not None and sample_depth is not None
        elif real_yolo:
            import ultralytics
            rgb = cv2.imread(str(Path(ultralytics.__file__).parent/'assets/bus.jpg'))
            assert rgb is not None
        else:
            rgb = np.full((100, 100, 3), 128, np.uint8)
        height, width = rgb.shape[:2]
        target_label = 'sofa' if real_emsanet else ('bus' if real_yolo else 'chair')
        display_label = '沙发' if real_emsanet else ('椅子' if emsanet_fixture else ('公交车' if real_yoloe else target_label))
        frame_count = mapper.config['min_observations']

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
            for node_id in range(1, frame_count+1):
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
                depth = sample_depth if real_emsanet else np.full((height, width), 2000, np.uint16)
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
                if node_id < frame_count:
                    assert not any(o.label == target_label for o in mapper.state.visible_objects())
            until(lambda: received and any(o.label == display_label for o in received[-1].objects))
            obj = next(o for o in received[-1].objects if o.label == display_label)
            assert len(obj.observations) == frame_count
            if real_emsanet:
                # The calibration here is synthetic: check valid projection and
                # loop correction, not the true metric location of the sample.
                assert all(mapper.config['min_depth'] <= o.position.z <= mapper.config['max_depth']
                           for o in obj.observations)
            elif not real_yolo:
                np.testing.assert_allclose([obj.observations[0].position.x,
                                        obj.observations[0].position.y,
                                        obj.observations[0].position.z], [0.2, 0, 2], atol=1e-5)
            else:
                assert abs(obj.observations[0].position.z-2) < 1e-5
            tracked = next(o for o in mapper.state.objects if o.label == target_label)
            before = mapper.state.position(tracked).copy()
            for i, pose in enumerate(data.graph.poses, 1):
                pose.position.x = i * 0.1
            graphs.publish(data.graph)
            until(lambda: abs(mapper.state.position(tracked)[0]-(before[0]+0.05*(frame_count+1))) < 1e-5)
            original_uuid = mapper.state.map_uuid
            executor.remove_node(mapper)
            mapper.destroy_node()
            mapper = make_mapper()
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
            report = dict(passed=True, frames=frame_count, label=target_label, display_label=display_label,
                          confirmation_window=mapper.config['confirmation_window'], real_yolo=real_yolo,
                          real_yoloe=real_yoloe,
                          emsanet_fixture=emsanet_fixture,
                          real_emsanet=real_emsanet,
                          compressed_keyframe=True, projection=True, loop_reprojection=True,
                          reload=True, map_reset=True)
            print(json.dumps(report))
            output = project/'logs/semantic_validation'
            output.mkdir(parents=True, exist_ok=True)
            filename = 'ros_real_yoloe.json' if real_yoloe else ('ros_real_yolo.json' if real_yolo else 'ros_pipeline.json')
            if emsanet_fixture:
                filename = 'ros_emsanet_pipeline.json'
            if real_emsanet:
                filename = 'ros_real_emsanet.json'
            (output/filename).write_text(json.dumps(report, indent=2))
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
