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
from rtabmap_msgs.srv import GetMap
from rtabmap_semantic_msgs.msg import Keyframe, Observation, SemanticMap, SemanticObject

from .fusion import SemanticState, SemanticStore
from .geometry import rigid_matrix, lift_mask
from .labels import chinese_label, YOLOE_LABEL_POLICY_VERSION
from .timing import CycleRecorder


POSE_STREAM_WAIT_NS = 2_000_000_000
POSE_REQUEST_TIMEOUT_NS = 3_000_000_000


def ros_matrix(pose):
    if hasattr(pose, 'translation'):
        p, q = pose.translation, pose.rotation
    else:
        p, q = pose.position, pose.orientation
    return rigid_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def store_config(config, model_hash):
    persistent = {k: v for k, v in config.items() if k not in
                  ('database_path', 'semantic_database_path', 'device', 'cpu_threads', 'label_language')}
    # Preserve existing fingerprints for add fusion and both YOLO backends.
    if (persistent.get('detector_backend') != 'emsanet' or
            persistent.get('emsanet_fusion', 'se-add-uni-rgb') == 'se-add-uni-rgb'):
        persistent.pop('emsanet_fusion', None)
    # Retain the exact old fingerprint for existing YOLOv8 maps.
    if persistent.get('detector_backend') == 'yolov8':
        persistent.pop('detector_backend')
        for key, default in (('confirmation_window', 0), ('dynamic_min_observations', 1)):
            if persistent.get(key) == default:
                persistent.pop(key)
    elif persistent.get('detector_backend') == 'emsanet':
        from .emsanet_detector import EMSANET_POLICY_VERSION
        persistent['emsanet_policy_version'] = EMSANET_POLICY_VERSION
    else:
        persistent['label_policy_version'] = YOLOE_LABEL_POLICY_VERSION
    persistent['model_sha256'] = model_hash
    return persistent


class SemanticMapper(Node):
    def __init__(self, detector=None, **kwargs):
        super().__init__('semantic_mapper', **kwargs)
        defaults = dict(database_path='', semantic_database_path='', model_path='',
                        device='cpu', confidence=0.5, image_size=640, cpu_threads=2,
                        min_depth=0.3, max_depth=4.0, min_depth_points=30,
                        min_observations=3, association_distance=0.45,
                        dynamic_classes=['person', 'cat', 'dog'], dynamic_ttl=3.0,
                        max_observations=30, classes=['*'], detector_backend='yolov8',
                        label_language='en', confirmation_window=0, dynamic_min_observations=1,
                        emsanet_fusion='se-add-uni-rgb')
        self.config = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        c = self.config
        if not c['database_path']:
            raise ValueError('database_path must identify the paired RTAB-Map database')
        if c['detector_backend'] not in ('yolov8', 'yoloe_pf', 'emsanet'):
            raise ValueError('detector_backend must be yolov8, yoloe_pf or emsanet')
        from .emsanet_detector import EMSANET_FUSION_MODES
        if c['emsanet_fusion'] not in EMSANET_FUSION_MODES:
            raise ValueError(f'emsanet_fusion must be one of {EMSANET_FUSION_MODES}')
        if c['detector_backend'] != 'emsanet' and c['emsanet_fusion'] != 'se-add-uni-rgb':
            raise ValueError('Fixed concat fusion requires the emsanet backend')
        if c['label_language'] not in ('en', 'zh'):
            raise ValueError('label_language must be en or zh')
        if not (0 < c['confidence'] <= 1 and 0 < c['min_depth'] < c['max_depth'] and
                c['min_observations'] >= 1 and c['association_distance'] > 0 and
                c['max_observations'] >= c['min_observations'] and c['dynamic_ttl'] > 0):
            raise ValueError('Invalid semantic mapping thresholds')
        self.map_key = str(Path(c['database_path']).expanduser().resolve())
        suffix = ('.yoloe.stable.semantic.sqlite' if c['confirmation_window'] else '.yoloe.semantic.sqlite') \
            if c['detector_backend'] == 'yoloe_pf' else '.semantic.sqlite'
        if c['detector_backend'] == 'emsanet':
            suffix = ('.emsanet.concat_fixed.semantic.sqlite'
                      if c['emsanet_fusion'] == 'se-concat-fixed-uni-rgb' else '.emsanet.semantic.sqlite')
        semantic_path = c['semantic_database_path'] or str(Path(self.map_key).with_suffix(suffix))
        self.state = SemanticState(c['min_observations'], c['association_distance'],
                                   c['dynamic_classes'], c['dynamic_ttl'], c['max_observations'],
                                   c['confirmation_window'], c['dynamic_min_observations'],
                                   unique_objects=c['detector_backend'] in ('yoloe_pf', 'emsanet'))
        model_path = Path(c['model_path']).expanduser()
        # Detect replaced weights even if the filename did not change.
        model_hash = hashlib.sha256(model_path.read_bytes()).hexdigest() if model_path.is_file() else ''
        persistent_config = store_config(c, model_hash)
        self.store = SemanticStore(semantic_path, self.map_key, persistent_config)
        self.store.load(self.state)
        if detector is None:
            if c['detector_backend'] == 'emsanet':
                from .emsanet_detector import EmsanetDetector
                detector = EmsanetDetector(c['model_path'], c['device'], c['confidence'],
                                           c['image_size'], c['cpu_threads'], fusion=c['emsanet_fusion'])
            else:
                from .detector import YoloDetector
                detector = YoloDetector(c['model_path'], c['device'], c['confidence'],
                                        c['image_size'], c['cpu_threads'], c['detector_backend'])
        self.detector = detector
        if c['detector_backend'] == 'emsanet':
            self.get_logger().info(f'EMSANet encoder fusion: {c["emsanet_fusion"]}')
        self.bridge = CvBridge()
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='semantic_inference')
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
        self.geometry_publisher = self.create_publisher(String, '/semantic/object_geometry', latched)
        self.status = self.create_publisher(String, '/semantic/status', 1)
        self.timing_publisher = self.create_publisher(String, '/semantic/timing', 100)
        self.timing = CycleRecorder(lambda row: self.timing_publisher.publish(
            String(data=json.dumps(row, ensure_ascii=False, allow_nan=False))))
        self.pending_received_ns = None
        self.active_received_ns = None
        self.pose_request = None
        self.pose_request_key = None
        self.pose_request_started_ns = None
        self.skipped_missing_pose = 0
        self.pose_client = self.create_client(GetMap, '/rtabmap/get_map_data')
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
        received_ns = time.monotonic_ns()
        key = (message.node_id, message.node_stamp)
        if self.active and key == (self.active.node_id, self.active.node_stamp):
            self.timing.record(message, 'duplicate_active')
            return  # Bridge retry while inference is running.
        if self.state.nodes.get(message.node_id) == message.node_stamp:
            self.timing.record(message, 'already_processed')
            self.ack.publish(Int32(data=message.node_id))
            return
        if self.active is None:
            if self.pending is not None:
                if key == (self.pending.node_id, self.pending.node_stamp):
                    self.timing.record(message, 'duplicate_pending')
                    return  # Preserve the original arrival time across retries.
                self.timing.record(self.pending, 'dropped', reason='pending_replaced')
            self.pending = message
            self.pending_received_ns = received_ns
            self.timing.record(message, 'accepted', received_ns=received_ns)
        else:
            self.timing.record(message, 'dropped', reason='busy_unaccepted')

    def infer(self, frame):
        started = time.monotonic_ns()
        rgb = self.bridge.imgmsg_to_cv2(frame.rgb, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(frame.depth, desired_encoding='passthrough')
        camera = frame.camera_info
        transform = ros_matrix(frame.camera_to_node)
        detections = []
        prepared = time.monotonic_ns()
        predictions = self.detector.detect(rgb, depth) if self.config['detector_backend'] == 'emsanet' \
            else self.detector.detect(rgb)
        detected = time.monotonic_ns()
        for detection in predictions:
            if self.config['classes'] != ['*'] and detection['label'] not in self.config['classes']:
                continue
            geometry = lift_mask(detection['mask'], depth, camera.k,
                (camera.width, camera.height), transform, camera.d, camera.distortion_model,
                self.config['min_depth'], self.config['max_depth'], self.config['min_depth_points'])
            if geometry is not None:
                detections.append({**geometry, 'label': detection['label'],
                                   'confidence': detection['confidence']})
        finished = time.monotonic_ns()
        return detections, (finished-started)/1e9, dict(
            worker_start_ns=started, prepared_ns=prepared, detected_ns=detected,
            worker_end_ns=finished, prediction_count=len(predictions), valid_3d_count=len(detections),
            model_timing=getattr(self.detector, 'last_timing', None))

    def clear_pose_request(self):
        if self.pose_request is not None and not self.pose_request.done():
            self.pose_client.remove_pending_request(self.pose_request)
            self.pose_request.cancel()
        self.pose_request = self.pose_request_key = self.pose_request_started_ns = None

    def skip_pending_pose(self, reason):
        frame = self.pending
        self.clear_pose_request()
        self.get_logger().warning(f'Node {frame.node_id}: {reason}; skipping this keyframe to release the semantic queue.')
        self.timing.record(frame, 'dropped', reason=reason, received_ns=self.pending_received_ns)
        self.skipped_missing_pose += 1
        # The bridge sends one frame at a time. A frame removed by rehearsal must
        # be acknowledged even though it cannot be projected or stored.
        self.ack.publish(Int32(data=frame.node_id))
        self.pending = self.pending_received_ns = None
        return False

    def resolve_pending_pose(self):
        frame = self.pending
        key = (self.state.map_uuid, self.frame_id, frame.node_id, frame.node_stamp)
        if self.pose_request is not None and (self.pose_request_key != key or frame.node_id in self.state.poses):
            self.clear_pose_request()
        if frame.node_id in self.state.poses:
            return True
        now = time.monotonic_ns()
        if self.pose_request is None:
            if now - self.pending_received_ns < POSE_STREAM_WAIT_NS:
                return False
            if not self.pose_client.service_is_ready():
                if now - self.pending_received_ns < POSE_STREAM_WAIT_NS + POSE_REQUEST_TIMEOUT_NS:
                    return False
                return self.skip_pending_pose('graph_service_unavailable')
            self.pose_request = self.pose_client.call_async(
                GetMap.Request(global_map=True, optimized=True, graph_only=True))
            self.pose_request_key, self.pose_request_started_ns = key, now
            return False
        if not self.pose_request.done():
            if now - self.pose_request_started_ns < POSE_REQUEST_TIMEOUT_NS:
                return False
            return self.skip_pending_pose('graph_request_timeout')
        try:
            data = self.pose_request.result().data
            graph = data.graph
            frame_id = data.header.frame_id or graph.header.frame_id or 'map'
            if frame_id == self.frame_id:
                for node_id, pose in zip(graph.poses_id, graph.poses):
                    if node_id == frame.node_id:
                        # Recover only this exact optimized anchor; preserve
                        # newer streaming poses and never substitute a live TF.
                        matrix = ros_matrix(pose)
                        self.state.poses[node_id] = matrix
                        self.latest_graph_poses[node_id] = matrix
                        self.clear_pose_request()
                        return True
        except Exception as error:
            self.get_logger().warning(f'Cannot restore node {frame.node_id} pose: {error}')
        return self.skip_pending_pose('missing_graph_pose')

    def poll(self):
        if self.future is not None and self.future.done():
            frame = self.active
            stages = {}
            outcome, reason = 'completed', ''
            try:
                detections, self.last_inference_seconds, stages = self.future.result()
                stages['fusion_start_ns'] = time.monotonic_ns()
                result = self.state.add_frame(self.active.node_id, self.active.node_stamp, detections)
                if result == 'reset':
                    self.state.update_graph(self.latest_graph_poses)
                    self.get_logger().warning('Reused graph node ID detected; started a new semantic epoch. Old epoch retained.')
                stages['fusion_end_ns'] = time.monotonic_ns()
                self.store.save(self.state)
                stages['persist_end_ns'] = time.monotonic_ns()
                self.processed += 1
                self.get_logger().info(f'Node {self.active.node_id}: {len(detections)} valid 3D detections, '
                                       f'{self.last_inference_seconds:.2f}s inference')
            except Exception as error:
                outcome, reason = 'failed', str(error)
                self.failed += 1
                self.get_logger().error(f'Node {self.active.node_id} failed: {error}')
            finally:
                self.ack.publish(Int32(data=self.active.node_id))
                self.future = self.active = None
                stages['publish_start_ns'] = time.monotonic_ns()
                try:
                    self.publish()
                except Exception as error:
                    outcome, reason = 'failed', 'publish: ' + str(error)
                    self.get_logger().error(reason)
                    raise
                finally:
                    stages['publish_end_ns'] = time.monotonic_ns()
                    self.timing.record(frame, outcome, received_ns=self.active_received_ns,
                        submitted_ns=self.active_submitted_ns, stages=stages, reason=reason,
                        backend=self.config['detector_backend'], device=self.config['device'],
                        image_size=self.config['image_size'], map_uuid=self.state.map_uuid,
                        objects_in_state=len(self.state.objects))
                    self.active_received_ns = None
        if self.future is None and self.pending is not None:
            if not self.resolve_pending_pose():
                return
            self.active, self.pending = self.pending, None
            self.active_received_ns, self.pending_received_ns = self.pending_received_ns, None
            self.active_submitted_ns = time.monotonic_ns()
            self.future = self.pool.submit(self.infer, self.active)

    def publish(self):
        message = SemanticMap()
        message.header.frame_id = self.frame_id
        if self.graph_stamp is not None:
            message.header.stamp = self.graph_stamp
        message.map_key, message.map_uuid = self.map_key, self.state.map_uuid
        display_label = chinese_label
        if self.config['detector_backend'] == 'emsanet':
            from .emsanet_detector import emsanet_chinese_label
            display_label = emsanet_chinese_label
        geometry = []
        for obj in self.state.visible_objects():
            label = display_label(obj.label) if self.config['label_language'] == 'zh' else obj.label
            item = SemanticObject(object_id=obj.object_id, label=label,
                                  confidence=obj.confidence, dynamic=obj.dynamic)
            for observation in self.state.inlier_observations(obj):
                anchor = Observation(node_id=observation.node_id, node_stamp=observation.node_stamp,
                                     weight=float(observation.weight))
                anchor.position.x, anchor.position.y, anchor.position.z = map(float, observation.position)
                item.observations.append(anchor)
            message.objects.append(item)
            extent = self.state.extent(obj)
            if extent is not None:
                geometry.append(dict(object_id=obj.object_id, extent=extent.tolist()))
        self.publisher.publish(message)
        # Optional display metadata avoids changing the persisted objects or the
        # SemanticMap wire format. Extents use the mapper's optimized graph.
        self.geometry_publisher.publish(String(data=json.dumps(dict(schema=1,
            map_key=message.map_key, map_uuid=message.map_uuid, frame_id=self.frame_id,
            objects=geometry), allow_nan=False)))
        self.status.publish(String(data=json.dumps(dict(processed=self.processed, failed=self.failed,
            detector_backend=self.config['detector_backend'],
            encoder_fusion=(self.config['emsanet_fusion']
                            if self.config['detector_backend'] == 'emsanet' else None),
            device=self.config['device'],
            active_node=self.active.node_id if self.active else None,
            pending_node=self.pending.node_id if self.pending else None,
            pending_wait_seconds=((time.monotonic_ns() - self.pending_received_ns) / 1e9
                                  if self.pending_received_ns is not None else 0.0),
            skipped_missing_pose=self.skipped_missing_pose,
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
        self.clear_pose_request()
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
