"""ROS adapter. Inference runs in one worker; graph and persistence stay on the executor."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from rtabmap_msgs.msg import MapGraph
from rtabmap_semantic_msgs.msg import Keyframe, Observation, SemanticMap, SemanticObject

from .fusion import SemanticState, SemanticStore
from .geometry import rigid_matrix, lift_mask


def ros_matrix(pose):
    if hasattr(pose, 'translation'):
        p, q = pose.translation, pose.rotation
    else:
        p, q = pose.position, pose.orientation
    return rigid_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


class SemanticMapper(Node):
    def __init__(self, detector=None, **kwargs):
        super().__init__('semantic_mapper', **kwargs)
        defaults = dict(database_path='', semantic_database_path='', model_path='',
                        device='cpu', confidence=0.5, image_size=640, cpu_threads=2,
                        min_depth=0.3, max_depth=4.0, min_depth_points=30,
                        min_observations=3, association_distance=0.45,
                        dynamic_classes=['person', 'cat', 'dog'], dynamic_ttl=3.0,
                        max_observations=30, classes=['*'])
        self.config = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        c = self.config
        if not c['database_path']:
            raise ValueError('database_path must identify the paired RTAB-Map database')
        if not (0 < c['confidence'] <= 1 and 0 < c['min_depth'] < c['max_depth'] and
                c['min_observations'] >= 1 and c['association_distance'] > 0 and
                c['max_observations'] >= c['min_observations'] and c['dynamic_ttl'] > 0):
            raise ValueError('Invalid semantic mapping thresholds')
        self.map_key = str(Path(c['database_path']).expanduser().resolve())
        semantic_path = c['semantic_database_path'] or str(Path(self.map_key).with_suffix('.semantic.sqlite'))
        self.state = SemanticState(c['min_observations'], c['association_distance'],
                                   c['dynamic_classes'], c['dynamic_ttl'], c['max_observations'])
        model_path = Path(c['model_path']).expanduser()
        # Detect replaced weights even if the filename did not change.
        model_hash = hashlib.sha256(model_path.read_bytes()).hexdigest() if model_path.is_file() else ''
        persistent_config = {k: v for k, v in c.items() if k not in
                             ('database_path', 'semantic_database_path', 'device', 'cpu_threads')}
        persistent_config['model_sha256'] = model_hash
        self.store = SemanticStore(semantic_path, self.map_key, persistent_config)
        self.store.load(self.state)
        if detector is None:
            from .detector import YoloDetector
            detector = YoloDetector(c['model_path'], c['device'], c['confidence'],
                                    c['image_size'], c['cpu_threads'])
        self.detector = detector
        self.bridge = CvBridge()
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='semantic_yolo')
        self.future = None
        self.active = None
        self.pending = None
        self.processed = 0
        self.failed = 0
        self.last_inference_seconds = 0.0
        self.frame_id = 'map'
        self.graph_stamp = None
        self.latest_graph_poses = {}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(SemanticMap, '/semantic/objects', latched)
        self.status = self.create_publisher(String, '/semantic/status', 1)
        self.ack = self.create_publisher(Int32, '/semantic/processed_node', 10)
        self.create_subscription(Keyframe, '/semantic/keyframes', self.on_frame, 5)
        self.create_subscription(MapGraph, '/semantic/graph', self.on_graph, latched)
        self.create_subscription(MapGraph, '/mapGraph', self.on_graph, 5)
        self.create_service(Trigger, '/semantic/save', self.on_save)
        self.create_timer(0.05, self.poll)
        self.create_timer(0.5, self.publish)
        self.get_logger().info(f'Semantic mapping ready: {semantic_path}, restored {len(self.state.objects)} objects')

    def on_graph(self, message):
        try:
            poses = {key: ros_matrix(pose) for key, pose in zip(message.poses_id, message.poses) if key > 0}
            # Local graph messages may omit old, unloaded nodes. Keep their last known poses;
            # the native GUI only renders anchors present in its displayed graph.
            self.state.poses.update(poses)
            self.latest_graph_poses = poses
            if self.state.reconcile():
                self.store.save(self.state)
            self.frame_id = message.header.frame_id or 'map'
            self.graph_stamp = message.header.stamp
        except ValueError as error:
            self.get_logger().warning(f'Invalid graph transform: {error}')

    def on_frame(self, message):
        key = (message.node_id, message.node_stamp)
        if self.active and key == (self.active.node_id, self.active.node_stamp):
            return  # Bridge retry while inference is running.
        if self.state.nodes.get(message.node_id) == message.node_stamp:
            self.ack.publish(Int32(data=message.node_id))
            return
        if self.active is None:
            self.pending = message

    def infer(self, frame):
        started = time.monotonic()
        rgb = self.bridge.imgmsg_to_cv2(frame.rgb, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(frame.depth, desired_encoding='passthrough')
        camera = frame.camera_info
        transform = ros_matrix(frame.camera_to_node)
        detections = []
        for detection in self.detector.detect(rgb):
            if self.config['classes'] != ['*'] and detection['label'] not in self.config['classes']:
                continue
            geometry = lift_mask(detection['mask'], depth, camera.k,
                (camera.width, camera.height), transform, camera.d, camera.distortion_model,
                self.config['min_depth'], self.config['max_depth'], self.config['min_depth_points'])
            if geometry is not None:
                detections.append({**geometry, 'label': detection['label'],
                                   'confidence': detection['confidence']})
        return detections, time.monotonic()-started

    def poll(self):
        if self.future is not None and self.future.done():
            try:
                detections, self.last_inference_seconds = self.future.result()
                result = self.state.add_frame(self.active.node_id, self.active.node_stamp, detections)
                if result == 'reset':
                    self.state.update_graph(self.latest_graph_poses)
                    self.get_logger().warning('Reused graph node ID detected; started a new semantic epoch. Old epoch retained.')
                self.store.save(self.state)
                self.processed += 1
                self.get_logger().info(f'Node {self.active.node_id}: {len(detections)} valid 3D detections, '
                                       f'{self.last_inference_seconds:.2f}s inference')
            except Exception as error:
                self.failed += 1
                self.get_logger().error(f'Node {self.active.node_id} failed: {error}')
            finally:
                self.ack.publish(Int32(data=self.active.node_id))
                self.future = self.active = None
                self.publish()
        if self.future is None and self.pending is not None:
            if self.pending.node_id not in self.state.poses:
                return  # Wait for the exact graph node, never substitute the latest TF.
            self.active, self.pending = self.pending, None
            self.future = self.pool.submit(self.infer, self.active)

    def publish(self):
        message = SemanticMap()
        message.header.frame_id = self.frame_id
        if self.graph_stamp is not None:
            message.header.stamp = self.graph_stamp
        message.map_key, message.map_uuid = self.map_key, self.state.map_uuid
        for obj in self.state.visible_objects():
            item = SemanticObject(object_id=obj.object_id, label=obj.label,
                                  confidence=obj.confidence, dynamic=obj.dynamic)
            for observation in self.state.inlier_observations(obj):
                anchor = Observation(node_id=observation.node_id, node_stamp=observation.node_stamp,
                                     weight=float(observation.weight))
                anchor.position.x, anchor.position.y, anchor.position.z = map(float, observation.position)
                item.observations.append(anchor)
            message.objects.append(item)
        self.publisher.publish(message)
        self.status.publish(String(data=json.dumps(dict(processed=self.processed, failed=self.failed,
            active_node=self.active.node_id if self.active else None,
            confirmed_objects=len(message.objects), total_objects=len(self.state.objects),
            last_inference_seconds=self.last_inference_seconds, map_uuid=self.state.map_uuid))))

    def on_save(self, request, response):
        try:
            self.store.save(self.state)
            response.success, response.message = True, str(self.store.path)
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        self.store.save(self.state)
        self.store.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticMapper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
