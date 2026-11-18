"""Bounded live audio classification using the existing AudioFrame stream."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_features_msgs.msg import AudioFrame
from std_msgs.msg import String

from .yamnet import SCHEMA, YamnetClassifier, PatrolEventTracker, make_snapshot


class YamnetAudioNode(Node):
    def __init__(self):
        super().__init__('yamnet_audio_events')
        self.root = Path(os.environ.get('RTABMAP_PROJECT_ROOT', Path.cwd()))
        defaults = dict(model_path='models/yamnet/yamnet.tflite',
                        class_map_path='config/yamnet_class_map.csv',
                        patrol_config_path='config/yamnet_patrol.json',
                        audio_topic='/multimodal/audio', output_topic='/perception/audio/events',
                        status_topic='/perception/audio/status', journal_path='logs/audio/yamnet/events.jsonl',
                        cpu_threads=2, top_k=5, analysis_window_s=2.0, min_period_s=.48,
                        max_input_age_s=3.0, audio_clock_offset_s=0.0)
        self.config = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        for key, low, high in [('analysis_window_s', .975, 10), ('min_period_s', .05, 10),
                               ('max_input_age_s', .1, 60), ('audio_clock_offset_s', -1e9, 1e9)]:
            value = self.config[key]
            if type(value) not in (int, float) or not np.isfinite(value) or not low <= value <= high:
                raise ValueError('Invalid YAMNet runtime parameter: ' + key)
        for key in ('audio_topic', 'output_topic', 'status_topic'):
            if not isinstance(self.config[key], str) or not self.config[key]:
                raise ValueError('Invalid YAMNet topic: ' + key)
        self.publisher = self.create_publisher(String, self.config['output_topic'], 2)
        self.status_publisher = self.create_publisher(String, self.config['status_topic'], 2)
        self.create_subscription(AudioFrame, self.config['audio_topic'], self.receive, qos_profile_sensor_data)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='yamnet')
        self.loading = self.pool.submit(self.load_model)
        self.classifier = self.tracker = self.future = self.pending = self.active = None
        self.seen = deque(maxlen=128)
        self.last_start = -float('inf')
        self.last_end = self.last_result_at = None
        self.sequence = 0
        self.journal = self.root / self.config['journal_path'] if self.config['journal_path'] else None
        self.state = dict(schema=SCHEMA, state='loading_model', classes=521, received=0, processed=0,
                          dropped_pending=0, rejected_inputs=0, rejected_stale=0, duplicates=0,
                          clock_resets=0, last_error='', last_result_at=None)
        self.create_timer(.05, self.poll)
        self.create_timer(1.0, self.publish_status)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def load_model(self):
        classifier = YamnetClassifier(self.root / self.config['model_path'],
                                      self.root / self.config['class_map_path'],
                                      self.config['cpu_threads'], self.config['top_k'])
        tracker = PatrolEventTracker(json.loads((self.root / self.config['patrol_config_path']).read_text()),
                                     classifier.classes)
        # Warm the delegate before receiving the first live window.
        classifier.classify(np.zeros(15600, dtype=np.float32), 16000)
        return classifier, tracker

    def receive(self, message):
        try:
            rate, channels = int(message.sample_rate), int(message.channels)
            if not 8000 <= rate <= 192000 or not 1 <= channels <= 64:
                raise ValueError('Invalid AudioFrame format')
            count = len(message.samples)
            if not count or count % channels or count > rate * channels * 10:
                raise ValueError('Invalid AudioFrame sample count')
            if not message.source_id or not message.header.frame_id:
                raise ValueError('AudioFrame must identify its source and microphone frame')
            raw_stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            duration = count / channels / rate
            end = raw_stamp + duration + self.config['audio_clock_offset_s']
            age = self.now() - end
            if age > self.config['max_input_age_s'] or age < -self.config['max_input_age_s']:
                self.state['rejected_stale'] += 1
                return
            key = (message.source_id, raw_stamp)
            if key in self.seen:
                self.state['duplicates'] += 1
                return
            samples = np.asarray(message.samples, dtype=np.float32).reshape(-1, channels)
            if not np.isfinite(samples).all() or np.max(np.abs(samples)) > 1.00001:
                raise ValueError('AudioFrame samples must be finite normalized PCM')
            tail = min(len(samples), int(round(rate * self.config['analysis_window_s'])))
            source = dict(source_id=message.source_id, frame_id=message.header.frame_id,
                          stamp_quality=message.stamp_quality or 'unspecified',
                          raw_stamp=raw_stamp, recording_duration_s=duration,
                          analysis_start_stamp=raw_stamp + duration - tail / rate,
                          clock_offset_s=self.config['audio_clock_offset_s'])
            self.seen.append(key)
            if self.pending is not None:
                self.state['dropped_pending'] += 1
            self.pending = dict(samples=samples[-tail:].copy(), rate=rate, source=source, end=end)
            self.state['received'] += 1
        except (ValueError, TypeError) as error:
            self.state['rejected_inputs'] += 1
            self.state['last_error'] = str(error)

    def poll(self):
        if self.loading is not None:
            if not self.loading.done():
                return
            try:
                self.classifier, self.tracker = self.loading.result()
                self.state.update(state='waiting_audio', last_error='')
                self.get_logger().info('YAMNet ready: 521 classes; subscribing to ' + self.config['audio_topic'])
            except Exception as error:
                self.state.update(state='model_error', last_error=str(error))
                self.get_logger().error('YAMNet load failed: ' + str(error))
            self.loading = None
        if self.classifier is None:
            return
        if self.future is not None:
            if not self.future.done():
                return
            try:
                result = self.future.result()
                age = self.now() - self.active['end']
                if abs(age) > self.config['max_input_age_s']:
                    self.state['rejected_stale'] += 1
                else:
                    if self.last_end is not None and self.active['end'] <= self.last_end:
                        self.tracker.reset()
                        self.state['clock_resets'] += 1
                    events = self.tracker.update(result['scores'], self.active['end'], result['audio']['rms_dbfs'])
                    self.last_end = self.active['end']
                    self.sequence += 1
                    snapshot = make_snapshot(result, sequence=self.sequence, stamp=self.active['end'],
                                             source=self.active['source'], patrol_events=events,
                                             published_at=self.now(), input_age_s=age)
                    text = json.dumps(snapshot, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
                    self.publisher.publish(String(data=text))
                    self.state.update(state='recognizing', processed=self.sequence, last_error='')
                    self.last_result_at = self.now()
                    self.state['last_result_at'] = self.last_result_at
                    self.write_journal(text)
            except Exception as error:
                self.state.update(state='inference_error', last_error=str(error))
                self.get_logger().error('YAMNet inference failed: ' + str(error))
            self.future = self.active = None
        if self.pending is not None and time.monotonic() - self.last_start >= self.config['min_period_s']:
            pending, self.pending = self.pending, None
            if abs(self.now() - pending['end']) > self.config['max_input_age_s']:
                self.state['rejected_stale'] += 1
                return
            self.active = pending
            self.last_start = time.monotonic()
            self.future = self.pool.submit(self.classifier.classify, pending['samples'], pending['rate'])
            self.state['state'] = 'inferencing'

    def write_journal(self, text):
        if self.journal is None:
            return
        try:
            self.journal.parent.mkdir(parents=True, exist_ok=True)
            with self.journal.open('a', encoding='utf-8') as stream:
                stream.write(text + '\n')
        except OSError as error:
            self.state['last_error'] = 'Audio journal: ' + str(error)
            self.get_logger().warning(self.state['last_error'])

    def publish_status(self):
        status = dict(self.state, published_at=self.now(), audio_topic=self.config['audio_topic'])
        if (self.classifier is not None and self.future is None and
                (self.last_result_at is None or self.now() - self.last_result_at > self.config['max_input_age_s'])):
            status['state'] = 'waiting_audio'
        self.status_publisher.publish(String(data=json.dumps(status, allow_nan=False)))

    def close(self):
        self.pool.shutdown(wait=True, cancel_futures=True)


def main():
    rclpy.init()
    node = None
    try:
        node = YamnetAudioNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
