#!/usr/bin/env python3
"""Isolated ROS check using the official classifier and recorded microphone PCM."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rtabmap_features_msgs.msg import AudioFrame
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rtabmap_features'))
from rtabmap_features.yamnet import YamnetClassifier, read_wav


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wav', type=Path, default=ROOT / 'logs/audio/m260c_comparison_cjkh2_c2/array.wav')
    parser.add_argument('--output', type=Path, default=ROOT / 'logs/audio/yamnet/ros_check.json')
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        parser.error('Use an isolated nonzero ROS_DOMAIN_ID')
    rate, recording = read_wav(args.wav)
    # This interval contains speech; the earlier interval includes rubbing noise.
    recording = recording[6 * rate:8 * rate]
    expected = YamnetClassifier(ROOT / 'models/yamnet/yamnet.tflite', ROOT / 'config/yamnet_class_map.csv').classify(recording, rate)
    rclpy.init()
    node = rclpy.create_node('yamnet_contract_check')
    publisher = node.create_publisher(AudioFrame, '/multimodal/audio', qos_profile_sensor_data)
    results, statuses, checks = [], [], []
    node.create_subscription(String, '/perception/audio/events', lambda m: results.append(json.loads(m.data)), 10)
    node.create_subscription(String, '/perception/audio/status', lambda m: statuses.append(json.loads(m.data)), 10)

    def wait_for(condition, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if condition():
                return
        raise AssertionError('ROS check timed out')

    def publish(source_id, samples, *, age=0, channels=1):
        message = AudioFrame()
        samples = np.asarray(samples, dtype=np.float32)
        raw_ns = node.get_clock().now().nanoseconds - int((len(samples) / channels / rate + age) * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(raw_ns, 1000000000)
        message.header.frame_id = 'test_microphone'
        message.sample_rate, message.channels = rate, channels
        message.source_id, message.stamp_quality = source_id, 'synthetic_test_clock'
        message.samples = samples.reshape(-1).tolist()
        publisher.publish(message)
        return message

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with (args.output.parent / 'ros_node.log').open('w') as log:
        process = subprocess.Popen([str(ROOT / '.venv-yamnet/bin/python3'), str(ROOT / 'scripts/run_yamnet_audio.py'),
                                    '--ros-args', '--params-file', str(ROOT / 'config/yamnet_audio.yaml'),
                                    '-p', 'max_input_age_s:=1.0', '-p', 'min_period_s:=0.05',
                                    '-p', 'journal_path:=' + str(args.output.parent / 'ros_results.jsonl')],
                                   cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        try:
            wait_for(lambda: statuses and statuses[-1]['state'] == 'waiting_audio')
            assert not results
            checks.append('waiting_without_audio')
            publish('malformed', [0], channels=2)
            wait_for(lambda: statuses[-1]['rejected_inputs'] >= 1)
            publish('stale', np.zeros(rate * 2), age=20)
            publish('future', np.zeros(rate * 2), age=-20)
            wait_for(lambda: statuses[-1]['rejected_stale'] >= 2)
            assert not results
            checks.append('malformed_stale_and_future_rejected')
            silent = publish('silence', np.zeros(rate * 2))
            wait_for(lambda: any(v['source']['source_id'] == 'silence' for v in results))
            silence = next(v for v in results if v['source']['source_id'] == 'silence')
            assert silence['model']['classes'] == 521 and len(silence['scores']) == 521
            assert silence['top_k'][0]['label'] == 'Silence' and not silence['patrol_events']
            checks.append('actual_521_class_silence_output')
            publisher.publish(silent)
            wait_for(lambda: statuses[-1]['duplicates'] >= 1)
            assert sum(v['source']['source_id'] == 'silence' for v in results) == 1
            checks.append('duplicate_frame_not_reprocessed')
            for i in range(3):
                publish(f'recording-{i}', recording)
                wait_for(lambda: any(v['source']['source_id'] == f'recording-{i}' for v in results))
            recorded = [v for v in results if v['source']['source_id'].startswith('recording-')]
            assert np.allclose(recorded[-1]['scores'], expected['scores'], atol=1e-6)
            assert recorded[-1]['top_k'][0]['label'] == expected['top_k'][0]['label']
            assert any(e['key'] == 'speech' and e['status'] == 'confirmed' for e in recorded[-1]['patrol_events'])
            assert recorded[-1]['source']['stamp_quality'] == 'synthetic_test_clock'
            assert recorded[-1]['source_localization']['available'] is False
            checks.append('recorded_pcm_scores_and_patrol_confirmation')
            wait_for(lambda: statuses[-1]['state'] == 'waiting_audio', timeout=4)
            checks.append('audio_interruption_reported')
            publish('reconnected', recording)
            wait_for(lambda: any(v['source']['source_id'] == 'reconnected' for v in results))
            checks.append('audio_reconnection_resumes_classification')
            output = dict(passed=True, mode='isolated_recording_replay', checks=checks, publications=len(results),
                          ros_domain_id=int(os.environ['ROS_DOMAIN_ID']), recording=str(args.wav),
                          median_processing_s=float(np.median([v['processing_s'] for v in results])),
                          max_processing_s=max(v['processing_s'] for v in results), latest_snapshot=results[-1])
            args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2) + '\n')
            print(json.dumps({k: v for k, v in output.items() if k != 'latest_snapshot'}, ensure_ascii=False))
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
