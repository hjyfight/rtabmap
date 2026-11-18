"""Independent RGB segmentation with an asynchronous, label-only fusion interface."""
import base64
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rtabmap_semantic_msgs.msg import (SegmentationInstance, SegmentationFrame,
    SegmentationFusionRequest, SegmentationFusionCandidate)

from .segmentation import (vocabulary, prepare_instances, dense_maps, render_images,
                           color_for, select_candidate)
from .labels import chinese_label


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class SegmentationNode(Node):
    def __init__(self, detector=None, **kwargs):
        super().__init__('yolo_segmentation', **kwargs)
        defaults = dict(model_path='models/yolov8n-seg.pt', detector_backend='yolov8', device='cpu',
            cpu_threads=2, confidence=0.5, image_size=640, rgb_topic='/camera/color/image_raw',
            max_inference_hz=2.0, max_input_age_s=3.0, fusion_mode='shadow',
            rgb_clock_offset_s=0.0, clock_offset_uncertainty_s=0.0, clock_offset_method='none',
            fusion_evaluation_report='',
            fusion_timeout_s=3.0, fusion_sync_tolerance_s=0.05, fusion_min_score=0.8,
            approved_fusion_providers=[''], cache_frames=4)
        self.config = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        c = self.config
        if c['fusion_mode'] not in ('off', 'shadow', 'guarded'):
            raise ValueError('fusion_mode must be off, shadow or guarded')
        for name, minimum, maximum in [('max_inference_hz', .1, 30), ('max_input_age_s', .1, 30),
                ('fusion_timeout_s', .1, 30), ('fusion_sync_tolerance_s', 0, 1),
                ('fusion_min_score', 0, 1), ('cache_frames', 1, 16), ('confidence', 0, 1)]:
            if not minimum <= c[name] <= maximum:
                raise ValueError('Invalid ' + name)
        if not np.isfinite(c['rgb_clock_offset_s']) or not 0 <= c['clock_offset_uncertainty_s'] <= 5:
            raise ValueError('Invalid clock offset or uncertainty')
        self.provenance_json = json.dumps({k: c[k] for k in ('rgb_clock_offset_s', 'clock_offset_uncertainty_s',
            'clock_offset_method', 'device', 'image_size', 'confidence', 'detector_backend')}, sort_keys=True)
        self.bridge = CvBridge()
        self.session_id = uuid.uuid4().hex
        self.sequence = 0
        self.pending = self.future = None
        self.last_received = self.last_start = self.last_completed = None
        self.last_key = None
        self.last_rejected_age = None
        self.cache = OrderedDict()
        self.latest = None
        self.last_error = ''
        self.counters = dict(processed=0, dropped_pending=0, dropped_stale=0, duplicates=0,
                             inference_errors=0, candidates=0, selected_fusion=0)
        self.fusion_reason = 'waiting_candidate' if c['fusion_mode'] != 'off' else 'fusion_disabled'
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='yolo_segmentation')
        prefix = '/perception/segmentation/'
        self.baseline_pub = self.create_publisher(SegmentationFrame, prefix + 'baseline', 2)
        self.selected_pub = self.create_publisher(SegmentationFrame, prefix + 'selected', 2)
        self.request_pub = self.create_publisher(SegmentationFusionRequest, prefix + 'fusion_request', 1)
        self.decision_pub = self.create_publisher(String, prefix + 'fusion_decision', 8)
        self.status_pub = self.create_publisher(String, '/miti/segmentation/status', 2)
        self.view_pub = self.create_publisher(String, '/miti/segmentation/view', 1)
        self.image_pubs = {key: self.create_publisher(Image, '/miti/segmentation/' + name, 1)
                          for key, name in [('raw', 'source'), ('overlay', 'overlay'), ('classes', 'class_image')]}
        self.status_pub.publish(String(data=json.dumps(dict(schema=1, state='loading_model',
            session_id=self.session_id, fusion_mode=c['fusion_mode'], published_at=time.time()))))
        if detector is None:
            from .detector import YoloDetector
            detector = YoloDetector(c['model_path'], c['device'], c['confidence'],
                                    c['image_size'], c['cpu_threads'], c['detector_backend'])
        self.detector = detector
        self.labels, self.vocabulary_version = vocabulary(detector)
        self.model_sha256 = hashlib.sha256(Path(c['model_path']).read_bytes()).hexdigest()
        self.evaluation_report = None
        if c['fusion_evaluation_report']:
            from .fusion_evaluation import load_passed_report
            self.evaluation_report = load_passed_report(c['fusion_evaluation_report'])
            if self.evaluation_report['rgb_model_sha256'] != self.model_sha256:
                raise ValueError('Evaluation RGB model differs from the current baseline')
        self.create_subscription(Image, c['rgb_topic'], self.on_image, qos_profile_sensor_data)
        self.create_subscription(SegmentationFusionCandidate, prefix + 'candidate', self.on_candidate, 4)
        self.create_timer(.025, self.poll)
        self.create_timer(.5, self.publish_status)
        self.get_logger().info('Independent YOLO segmentation ready; fusion=' + c['fusion_mode'])

    def on_image(self, message):
        key = (stamp_ns(message.header), message.header.frame_id)
        if key == self.last_key:
            self.counters['duplicates'] += 1
            return
        age = self.source_age(key[0])
        if key[0] <= 0 or age < -.1-self.config['clock_offset_uncertainty_s'] or age > self.config['max_input_age_s']:
            self.counters['dropped_stale'] += 1
            self.last_rejected_age = age
            return
        self.last_rejected_age = None
        self.last_key = key
        self.last_received = time.monotonic()
        self.sequence += 1
        if self.pending is not None:
            self.counters['dropped_pending'] += 1
        self.pending = (message, self.sequence, self.last_received)

    def infer(self, item):
        message, sequence, received = item
        started = time.monotonic()
        rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding='rgb8').copy()
        rows = self.detector.detect(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        instances = prepare_instances(rows, rgb.shape, self.labels)
        return dict(session_id=self.session_id, sequence=sequence, stamp_ns=stamp_ns(message.header),
            frame_id=message.header.frame_id, header=deepcopy(message.header), width=rgb.shape[1],
            height=rgb.shape[0], model_sha256=self.model_sha256, vocabulary_version=self.vocabulary_version,
            provenance_json=self.provenance_json,
            result_source='rgb_yolo', selection_reason='baseline_passthrough', provider_id='', revision=0,
            inference_seconds=time.monotonic()-started, instances=instances), rgb, received

    def to_message(self, record):
        message = SegmentationFrame()
        for name in ('header', 'session_id', 'sequence', 'revision', 'width', 'height', 'model_sha256',
                     'vocabulary_version', 'provenance_json', 'result_source', 'selection_reason', 'provider_id', 'inference_seconds'):
            setattr(message, name, record[name])
        for row in record['instances']:
            instance = SegmentationInstance()
            for name in ('instance_id', 'class_id', 'label', 'confidence', 'classification_score',
                         'bbox_xyxy', 'mask_offset'):
                setattr(instance, name, row[name])
            instance.mask = self.bridge.cv2_to_imgmsg(row['mask'], encoding='mono8')
            instance.mask.header = message.header
            message.instances.append(instance)
        classes, identities = dense_maps(record['instances'], (record['height'], record['width']))
        message.class_map = self.bridge.cv2_to_imgmsg(classes, encoding='32SC1')
        message.instance_map = self.bridge.cv2_to_imgmsg(identities, encoding='32SC1')
        message.class_map.header = message.instance_map.header = message.header
        return message

    def publish_selected(self, record, rgb, message=None):
        message = message or self.to_message(record)
        self.selected_pub.publish(message)
        classes = self.bridge.imgmsg_to_cv2(message.class_map, desired_encoding='passthrough')
        overlay, color_image = render_images(rgb, classes)
        images = {}
        for key, pixels in [('raw', rgb), ('overlay', overlay), ('classes', color_image)]:
            image = self.bridge.cv2_to_imgmsg(pixels, encoding='rgb8')
            image.header = message.header
            self.image_pubs[key].publish(image)
            success, encoded = cv2.imencode('.jpg', cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR),
                                             [cv2.IMWRITE_JPEG_QUALITY, 82])
            if not success:
                raise RuntimeError('JPEG encoding failed')
            images[key] = base64.b64encode(encoded).decode('ascii')
        rows = [{k: row[k] for k in ('instance_id', 'class_id', 'confidence', 'classification_score', 'bbox_xyxy')}
                | dict(label=chinese_label(row['label']), color=list(color_for(row['class_id'])))
                for row in record['instances']]
        view = dict(schema=1, session_id=record['session_id'], sequence=record['sequence'],
            backend=self.config['detector_backend'],
            revision=record['revision'], stamp=dict(sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec),
            width=record['width'], height=record['height'], result_source=record['result_source'],
            inference_s=record['inference_seconds'], published_at=time.time(),
            source_age_s=max(0., self.source_age(record['stamp_ns'])), images=images, instances=rows)
        # Atomic display snapshot keeps JPEGs, instance labels and selected revision together.
        self.view_pub.publish(String(data=json.dumps(view, ensure_ascii=False, allow_nan=False)))
        self.latest = record

    def poll(self):
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            try:
                record, rgb, received = future.result()
                if time.monotonic() - received > self.config['max_input_age_s']:
                    self.counters['dropped_stale'] += 1
                else:
                    baseline = self.to_message(record)
                    self.baseline_pub.publish(baseline)
                    self.publish_selected(record, rgb, baseline)
                    self.cache[record['sequence']] = (record, rgb, time.monotonic())
                    while len(self.cache) > self.config['cache_frames']:
                        self.cache.popitem(last=False)
                    if self.config['fusion_mode'] != 'off':
                        request = SegmentationFusionRequest(baseline=baseline,
                            rgb=self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8'),
                            class_ids=list(self.labels), class_names=list(self.labels.values()),
                            deadline_seconds=self.config['fusion_timeout_s'])
                        request.rgb.header = baseline.header
                        self.request_pub.publish(request)
                    self.counters['processed'] += 1
                    self.last_completed = time.monotonic()
                    self.last_error = ''
            except Exception as error:
                self.counters['inference_errors'] += 1
                self.last_error = str(error)
                self.get_logger().error('Segmentation failed: ' + str(error))
        now = time.monotonic()
        for key, (_, _, published) in list(self.cache.items()):
            if now-published > self.config['fusion_timeout_s']:
                del self.cache[key]
        if self.future is None and self.pending is not None and (
                self.last_start is None or now-self.last_start >= 1/self.config['max_inference_hz']):
            # A validated asynchronous refiner needs its reference frame retained until
            # the candidate arrives. Sensor callbacks still retain the newest input.
            if (self.config['fusion_mode']=='guarded' and self.evaluation_report and
                    self.evaluation_report['fusion_enabled'] and self.latest and
                    self.latest['revision']==0 and self.latest['sequence'] in self.cache):
                return
            item, self.pending = self.pending, None
            if now-item[2] > self.config['max_input_age_s']:
                self.counters['dropped_stale'] += 1
            else:
                self.last_start = now
                self.future = self.pool.submit(self.infer, item)

    def on_candidate(self, message):
        self.counters['candidates'] += 1
        if self.config['fusion_mode'] == 'off':
            self.publish_decision(message, 'fusion_disabled')
            return
        cached = self.cache.get(message.sequence)
        if not cached:
            self.publish_decision(message, 'unknown_or_expired_frame')
            return
        baseline, rgb, published = cached
        if self.source_age(baseline['stamp_ns']) > self.config['max_input_age_s']:
            self.publish_decision(message, 'stale_source_frame')
            return
        candidate = {key: getattr(message, key) for key in ('session_id', 'sequence', 'model_sha256',
            'vocabulary_version', 'provider_id', 'calibration_version', 'score_calibration_version')}
        candidate.update(stamp_ns=stamp_ns(message.header), frame_id=message.header.frame_id,
            refinements=[dict(instance_id=r.instance_id, class_id=r.class_id, score=r.score,
                valid_mask=list(r.valid_mask), weights=list(r.weights), source_stamps=list(r.source_stamps),
                spatial_alignment_valid=r.spatial_alignment_valid, time_alignment_valid=r.time_alignment_valid)
                for r in message.refinements])
        minimum = self.config['fusion_min_score']
        if self.config['fusion_mode'] == 'guarded':
            report = self.evaluation_report
            if not report or not report['fusion_enabled']:
                self.publish_decision(message, 'heldout_evaluation_required' if not report else 'validated_policy_keeps_baseline')
                return
            if message.score_calibration_version != 'temperature:' + report['report_sha256']:
                self.publish_decision(message, 'evaluation_calibration_mismatch')
                return
            minimum = report['confidence_threshold']
        current = self.latest is not None and baseline['sequence'] == self.latest['sequence'] and self.latest['revision'] == 0
        selected, reason = select_candidate(candidate, baseline, self.labels,
            mode=self.config['fusion_mode'], approved=self.config['approved_fusion_providers'], current=current,
            age=time.monotonic()-published, timeout=self.config['fusion_timeout_s'],
            tolerance=self.config['fusion_sync_tolerance_s'], min_score=minimum)
        if selected is not baseline:
            try:
                self.publish_selected(selected, rgb)
                self.counters['selected_fusion'] += 1
            except Exception as error:
                self.last_error = 'Fusion display failed: ' + str(error)
                reason = 'fusion_publish_error'
        self.publish_decision(message, reason)
        self.publish_status()

    def publish_decision(self, candidate, reason):
        self.fusion_reason = reason
        self.decision_pub.publish(String(data=json.dumps(dict(schema=1, session_id=candidate.session_id,
            sequence=candidate.sequence, provider_id=candidate.provider_id, reason=reason,
            selected=reason == 'fusion_selected', fusion_mode=self.config['fusion_mode'],
            published_at=time.time()))))

    def publish_status(self):
        now = time.monotonic()
        age = None if self.last_received is None else now-self.last_received
        if self.last_rejected_age is not None:
            state = 'timestamp_rejected'
        elif age is None:
            state = 'waiting_image'
        elif age > self.config['max_input_age_s']:
            state = 'input_stale'
        elif self.last_error:
            state = 'inference_error'
        elif self.future is not None:
            state = 'inferencing'
        else:
            state = 'ready'
        self.status_pub.publish(String(data=json.dumps(dict(schema=1, state=state,
            session_id=self.session_id, published_at=time.time(), input_age_s=age,
            frame_age_s=None if self.latest is None else max(0, self.source_age(self.latest['stamp_ns'])),
            last_rejected_age_s=self.last_rejected_age,
            fusion_mode=self.config['fusion_mode'], fusion_reason=self.fusion_reason,
            result_source=self.latest['result_source'] if self.latest else '',
            last_error=self.last_error, counters=self.counters), allow_nan=False)))

    def source_age(self, source_stamp_ns):
        return (self.get_clock().now().nanoseconds-source_stamp_ns)/1e9-self.config['rgb_clock_offset_s']

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SegmentationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
