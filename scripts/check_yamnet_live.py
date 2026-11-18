#!/usr/bin/env python3
"""Read-only live check matching YAMNet results to received microphone frames."""
import argparse
from collections import deque
import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rtabmap_features_msgs.msg import AudioFrame
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rtabmap_features'))
from rtabmap_features.yamnet import SCHEMA, load_class_map


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=6)
    parser.add_argument('--output', type=Path, default=ROOT / 'logs/audio/yamnet/live_check.json')
    args = parser.parse_args()
    if not 2 <= args.seconds <= 30:
        parser.error('Require 2 <= seconds <= 30')
    classes = load_class_map(ROOT / 'config/yamnet_class_map.csv')
    frames, events, statuses, errors = deque(maxlen=128), [], [], []
    rclpy.init()
    node = rclpy.create_node('yamnet_live_read_check')

    def audio(message):
        frames.append(message.source_id)

    def receive(message):
        try:
            value = json.loads(message.data)
            scores = np.asarray(value['scores'], dtype=float)
            assert value['schema'] == SCHEMA and value['mode'] == 'live' and scores.shape == (521,)
            assert np.isfinite(scores).all() and np.all(scores >= 0) and np.all(scores <= 1)
            source = value['source']
            assert abs(value['stamp'] - source['raw_stamp'] - source['recording_duration_s'] - source['clock_offset_s']) < 1e-5
            for row in value['top_k']:
                assert row['label'] == classes[row['index']]['label']
                assert abs(row['score'] - scores[row['index']]) < 1e-6
            events.append(dict(snapshot=value, received_at=node.get_clock().now().nanoseconds / 1e9))
        except (ValueError, KeyError, AssertionError, IndexError) as error:
            errors.append(type(error).__name__ + ': ' + str(error))

    node.create_subscription(AudioFrame, '/multimodal/audio', audio, qos_profile_sensor_data)
    node.create_subscription(String, '/perception/audio/events', receive, 10)
    node.create_subscription(String, '/perception/audio/status', lambda m: statuses.append(json.loads(m.data)), 10)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        matched = [row for row in events if row['snapshot']['source']['source_id'] in frames]
        passed = len(matched) >= 3 and not errors and bool(statuses) and statuses[-1]['processed'] > 0
        output = dict(passed=passed, mode='live_microphone_read_only', audio_frames=len(frames),
                      publications=len(events), matched_publications=len(matched), errors=errors,
                      last_status=statuses[-1] if statuses else None,
                      median_processing_s=float(np.median([row['snapshot']['processing_s'] for row in matched])) if matched else None,
                      max_processing_s=max((row['snapshot']['processing_s'] for row in matched), default=None),
                      median_publish_age_s=float(np.median([row['snapshot']['input_age_s'] for row in matched])) if matched else None,
                      latest_snapshot=matched[-1]['snapshot'] if matched else None)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({k: v for k, v in output.items() if k != 'latest_snapshot'}, ensure_ascii=False))
        if not passed:
            sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
