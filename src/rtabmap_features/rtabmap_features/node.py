"""Read-only sensor subscriptions with a bounded latest-bundle inference queue."""
from collections import deque
import json
import os
from pathlib import Path
import queue
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
from rtabmap_features_msgs.msg import AudioFrame, FeatureSet
from .backbone import FrozenEncoder
from .bundle import extract_bundle, save_features
from .fusion import MODALITIES, Sample, match_samples
from .visualization import feature_snapshot, spectrum_rgb


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class FourModalNode(Node):
    def __init__(self, encoder=None):
        super().__init__('fourmodal_features')
        project = Path(os.environ.get('RTABMAP_PROJECT_ROOT', Path.cwd()))
        defaults = dict(source_path=str(project / '.tools/unibind'),
                        checkpoint_path=str(project / 'models/unibind/pretrained_weights.pt'),
                        device='cpu', cpu_threads=2, point_count=2048, infrared_kind='nir',
                        rgb_topic='/camera/color/image_raw', infrared_topic='/camera/infra2/image_rect_raw',
                        lidar_topic='/livox/lidar', audio_topic='/multimodal/audio',
                        output_topic='/multimodal/features', status_topic='/multimodal/status',
                        visualization_topic='/miti/fourmodal_features',
                        visualization_status_topic='/miti/fourmodal_status',
                        spectrum_topic='/miti/audio_spectrum/image_raw',
                        calibration_version='unverified', sync_tolerance_s=0.05,
                        clock_offsets_s=[0.0] * 4,
                        min_anchor_period_s=1.0, max_input_age_s=3.0,
                        allow_partial=False, buffer_size=16, weights=[1.0] * 4,
                        save_directory='')
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        if not 2 <= self.config['buffer_size'] <= 128:
            raise ValueError('buffer_size must be 2..128')
        if self.config['sync_tolerance_s'] < 0 or self.config['max_input_age_s'] <= 0:
            raise ValueError('Invalid synchronization or freshness settings')
        if self.config['min_anchor_period_s'] < 0 or len(self.config['weights']) != 4:
            raise ValueError('Invalid anchor period or fusion weights')
        if len(self.config['clock_offsets_s']) != 4 or not np.isfinite(self.config['clock_offsets_s']).all():
            raise ValueError('clock_offsets_s requires four finite values')
        self.publisher = self.create_publisher(FeatureSet, self.config['output_topic'], 2)
        self.status_publisher = self.create_publisher(String, self.config['status_topic'], 2)
        self.visualization_publisher = self.create_publisher(String, self.config['visualization_topic'], 2)
        self.visualization_status_publisher = self.create_publisher(String, self.config['visualization_status_topic'], 2)
        self.spectrum_publisher = self.create_publisher(Image, self.config['spectrum_topic'], 2)
        self.get_logger().info('Loading verified frozen UniBind backbone on ' + self.config['device'])
        self.encoder = encoder or FrozenEncoder(
            self.config['source_path'], self.config['checkpoint_path'], self.config['device'],
            self.config['infrared_kind'], self.config['cpu_threads'], self.config['point_count'])
        self.bridge = CvBridge()
        self.buffers = {name: deque(maxlen=self.config['buffer_size']) for name in MODALITIES}
        self.lock = threading.Lock()
        self.pending = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.seen = deque(maxlen=256)
        self.last_queued_stamp = -float('inf')
        self.sequence = 0
        self.state = dict(state='waiting_inputs', processed=0, dropped_pending=0, input_errors=0)
        self.subscriptions_kept = [
            self.create_subscription(Image, self.config['rgb_topic'],
                                     lambda m: self.receive_image('rgb', m), qos_profile_sensor_data),
            self.create_subscription(Image, self.config['infrared_topic'],
                                     lambda m: self.receive_image('infrared', m), qos_profile_sensor_data),
            self.create_subscription(PointCloud2, self.config['lidar_topic'], self.receive_lidar,
                                     qos_profile_sensor_data),
            self.create_subscription(AudioFrame, self.config['audio_topic'], self.receive_audio,
                                     qos_profile_sensor_data)]
        self.worker = threading.Thread(target=self.infer_loop, daemon=True)
        self.worker.start()
        self.create_timer(0.1, self.select_bundle)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Four-sensor feature subscriptions ready; no training is performed')

    def add_sample(self, name, message, payload, duration=0.0, attributes=None):
        stamp = stamp_seconds(message.header.stamp)
        topic = self.config[name + '_topic'] if name != 'infrared' else self.config['infrared_topic']
        sample = Sample(payload, stamp, message.header.frame_id,
                        getattr(message, 'source_id', '') or f'{topic}:{message.header.stamp.sec}:{message.header.stamp.nanosec}',
                        duration, dict(attributes or {}, topic=topic), self.config['clock_offsets_s'][MODALITIES.index(name)])
        with self.lock:
            self.buffers[name].append(sample)

    def input_error(self, name, error):
        self.state['input_errors'] += 1
        self.state['last_input_error'] = dict(modality=name, message=str(error))

    def receive_image(self, name, message):
        try:
            if name == 'infrared' and message.encoding != 'mono8':
                raise ValueError('Infrared topic must provide mono8; do not guess its bit-depth scaling')
            image = self.bridge.imgmsg_to_cv2(message, 'rgb8' if name == 'rgb' else 'mono8').copy()
            self.add_sample(name, message, image, attributes={'encoding': message.encoding})
        except (ValueError, RuntimeError, CvBridgeError) as error:
            self.input_error(name, error)

    def receive_lidar(self, message):
        try:
            points = point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True)
            if isinstance(points, np.ndarray) and points.dtype.names:
                points = np.column_stack([points[name] for name in ('x', 'y', 'z')])
            else:
                points = np.asarray(list(points), np.float32)
            self.add_sample('lidar', message, np.asarray(points, np.float32).reshape(-1, 3),
                            attributes={'units': 'm', 'stamp_policy': 'PointCloud2_header',
                                        'motion_compensation_performed_here': False})
        except (ValueError, RuntimeError, AssertionError) as error:
            self.input_error('lidar', error)

    def receive_audio(self, message):
        try:
            if not 1 <= message.channels <= 64 or not 8000 <= message.sample_rate <= 192000:
                raise ValueError('Invalid audio channel count or sample rate')
            if not message.samples or len(message.samples) % message.channels:
                raise ValueError('Malformed interleaved audio samples')
            samples = np.asarray(message.samples, np.float32).reshape(-1, message.channels)
            self.add_sample('audio', message, samples, len(samples) / message.sample_rate,
                            {'sample_rate': message.sample_rate, 'channels': message.channels,
                             'stamp_quality': message.stamp_quality,
                             'microphone_colocation_validated': False})
        except (ValueError, RuntimeError) as error:
            self.input_error('audio', error)

    def select_bundle(self):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            buffers = {name: list(samples) for name, samples in self.buffers.items()}
        selected = None
        for anchor in reversed(buffers['rgb']):
            if anchor.source_id in self.seen:
                continue
            if anchor.stamp - self.last_queued_stamp < self.config['min_anchor_period_s']:
                continue
            if not -self.config['sync_tolerance_s'] <= now - anchor.alignment_stamp <= self.config['max_input_age_s']:
                continue
            selected = match_samples(buffers, anchor, self.config['sync_tolerance_s'],
                                     self.config['allow_partial'])
            if selected:
                break
        if not selected:
            self.state['input_state'] = 'waiting_inputs_or_acquisition_time_alignment'
            return
        self.seen.append(selected['rgb'].source_id)
        self.last_queued_stamp = selected['rgb'].stamp
        try:
            self.pending.put_nowait(selected)
        except queue.Full:
            try:
                self.pending.get_nowait()
                self.state['dropped_pending'] += 1
            except queue.Empty:
                pass
            self.pending.put_nowait(selected)
        self.state['input_state'] = 'bundle_ready'

    def infer_loop(self):
        while not self.stop.is_set():
            try:
                samples = self.pending.get(timeout=0.1)
            except queue.Empty:
                continue
            self.state['state'] = 'inferencing'
            display_spectrum = None
            try:
                context = dict(alignment='estimated_clock_offsets' if any(self.config['clock_offsets_s']) else 'acquisition_timestamps',
                               clock_offsets_s=dict(zip(MODALITIES, self.config['clock_offsets_s'])),
                               calibration_version=self.config['calibration_version'],
                               sync_tolerance_s=self.config['sync_tolerance_s'])
                result, metadata = extract_bundle(
                    self.encoder, samples, context, dict(zip(MODALITIES, self.config['weights'])),
                    self.config['allow_partial'])
                if self.stop.is_set():
                    break
                self.sequence += 1
                self.publish_features(samples, result, metadata)
                # Display failures must not discard a valid scientific FeatureSet.
                try:
                    display_spectrum = self.publish_visualization(samples, result, metadata)
                except Exception as error:
                    self.get_logger().warning('MITI visualization: ' + str(error))
                if self.config['save_directory']:
                    directory = Path(self.config['save_directory']) / (
                        f'{os.getpid()}_{int(samples["rgb"].stamp * 1e9)}_{self.sequence}')
                    save_features(directory, result, metadata)
                    if display_spectrum is not None:
                        from PIL import Image as PILImage
                        PILImage.fromarray(display_spectrum).save(directory / 'spectrum.png')
                self.state.update(state='ready', processed=self.sequence,
                                  last_inference_s=metadata['total_inference_s'],
                                  last_anchor_stamp=samples['rgb'].stamp,
                                  complete=metadata['complete'])
            except Exception as error:
                self.state.update(state='inference_error', last_error=str(error))
                self.get_logger().error(str(error))

    def publish_features(self, samples, result, metadata):
        message = FeatureSet()
        anchor = samples['rgb']
        stamp_ns = int(round(anchor.stamp * 1e9))
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1000000000)
        message.header.frame_id = anchor.frame_id
        message.sequence = self.sequence
        message.model_sha256 = self.encoder.provenance['checkpoint_sha256']
        message.calibration_version = self.config['calibration_version']
        message.modality_order = list(MODALITIES)
        message.source_stamps = [samples[n].stamp if n in samples else 0.0 for n in MODALITIES]
        message.source_durations = [samples[n].duration if n in samples else 0.0 for n in MODALITIES]
        message.valid_mask = result['valid_mask'].tolist()
        message.weights = result['weights'].tolist()
        message.complete = bool(result['complete'])
        message.fused_valid = bool(result['fused_valid'])
        message.encoder_features = result['encoder_features'].reshape(-1).tolist()
        message.independent_features = result['independent'].reshape(-1).tolist()
        message.joint_features = result['concat'].tolist()
        message.fused_features = result['fused'].tolist()
        message.metadata_json = json.dumps(metadata, ensure_ascii=False, allow_nan=False)
        self.publisher.publish(message)

    def publish_visualization(self, samples, result, metadata):
        message = String()
        snapshot = feature_snapshot(result, metadata, mode='live', sequence=self.sequence)
        message.data = json.dumps(snapshot, ensure_ascii=False, allow_nan=False)
        self.visualization_publisher.publish(message)
        if 'audio' in samples:
            from .preprocessing import audio_tensor
            sample = samples['audio']
            spectrum, info = audio_tensor(sample.payload, sample.attributes['sample_rate'],
                                          samples['rgb'].alignment_stamp - sample.alignment_stamp)
            display = spectrum_rgb(spectrum.cpu().numpy())
            image = self.bridge.cv2_to_imgmsg(display, 'rgb8')
            stamp_ns = int(round((sample.stamp + info['crop_start_offset_s']) * 1e9))
            image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp_ns, 1000000000)
            image.header.frame_id = sample.frame_id
            self.spectrum_publisher.publish(image)
            return display

    def publish_status(self):
        with self.lock:
            counts = {name: len(samples) for name, samples in self.buffers.items()}
        message = String()
        message.data = json.dumps(dict(self.state, buffered_samples=counts,
                                       allow_partial=self.config['allow_partial'],
                                       provenance=self.encoder.provenance), ensure_ascii=False)
        self.status_publisher.publish(message)
        visual = String()
        with self.lock:
            now = self.get_clock().now().nanoseconds / 1e9
            input_ages = {name: max(0, now - samples[-1].alignment_stamp - samples[-1].duration)
                          if samples else None for name, samples in self.buffers.items()}
        visual.data = json.dumps(dict(schema=1, published_at=time.time(), **self.state,
                                      buffered_samples=counts, input_ages_s=input_ages,
                                      max_input_age_s=self.config['max_input_age_s']),
                                  ensure_ascii=False, allow_nan=False)
        self.visualization_status_publisher.publish(visual)

    def close(self):
        self.stop.set()
        self.worker.join(timeout=5)


def main():
    rclpy.init()
    node = None
    try:
        node = FourModalNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
