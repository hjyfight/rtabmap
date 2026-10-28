#!/usr/bin/env python3
"""Check the diagnostic process over isolated, localhost-only ROS communication."""

import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import Info


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'loop_diagnostics_fixtures', ROOT / 'tests/test_loop_closure_diagnostics.py')
fixtures = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixtures)


def main():
    domain = os.environ.get('RTABMAP_DIAGNOSTICS_TEST_DOMAIN_ID', '179')
    os.environ.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
    os.environ.pop('CYCLONEDDS_URI', None)
    output_dir = Path(tempfile.mkdtemp(prefix='rtabmap_loop_diagnostics_'))
    os.environ['ROS_LOG_DIR'] = str(output_dir / 'ros')
    events_path = output_dir / 'events.jsonl'
    console_path = output_dir / 'console.log'

    def events():
        if not events_path.exists():
            return []
        # Only read complete lines while the other process is appending.
        data = events_path.read_text()
        return [json.loads(line) for line in data.split('\n')[:-1] if line]

    rclpy.init(args=[])
    publisher_node = rclpy.create_node('loop_diagnostics_test_publisher')
    publishers = [
        publisher_node.create_publisher(Info, '/info', 10),
        publisher_node.create_publisher(Info, '/rtabmap/info', qos_profile_sensor_data)]
    process = None
    try:
        with console_path.open('w') as console:
            process = subprocess.Popen(
                [sys.executable, str(ROOT / 'scripts/loop_closure_diagnostics.py'),
                 '--log-file', str(events_path)], stdout=console, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 23

            def wait_for(predicate, label):
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError(f'Diagnostics exited: {console_path.read_text()}')
                    if predicate():
                        return
                    rclpy.spin_once(publisher_node, timeout_sec=0.1)
                raise TimeoutError(f'Timed out waiting for {label}; inspect {console_path}')

            wait_for(lambda: all(publisher.get_subscription_count() for publisher in publishers),
                     'both info subscriptions')
            wait_for(lambda: any(event['type'] == 'DETECTION_HEARTBEAT'
                                 and event['state'] == 'WAITING_FOR_SLAM' for event in events()),
                     'waiting heartbeat')

            sequence = [
                fixtures.make_info(**{'Memory/Working_memory_size/': 0}),
                fixtures.make_info(ref=41),
                fixtures.make_info(ref=42, **{
                    'Loop/Highest_hypothesis_id/': 5,
                    'Loop/Highest_hypothesis_value/': 0.03}),
                fixtures.make_info(ref=43, **{
                    'Loop/Highest_hypothesis_id/': 5,
                    'Loop/Highest_hypothesis_value/': 0.8,
                    'Loop/RejectedHypothesis/': 1}),
                fixtures.make_info(ref=44, global_id=5, **{
                    'Loop/Highest_hypothesis_id/': 5,
                    'Loop/Highest_hypothesis_value/': 0.9,
                    'Loop/Optimization_error/': 0.25,
                    'Loop/Optimization_iterations/': 8}),
            ]
            for index, message in enumerate(sequence, 1):
                for publisher in publishers:
                    publisher.publish(message)
                wait_for(lambda: sum(event['type'] == 'DETECTION_STATUS'
                                     for event in events()) >= index, 'frame processing')
            # Also replay an earlier frame after newer ones to verify delayed duplicate handling.
            for publisher in publishers:
                publisher.publish(sequence[-1])
                publisher.publish(sequence[0])
            wait_for(lambda: any(event['type'] == 'DETECTION_HEARTBEAT'
                                 and event['state'] == 'SLAM_TIMEOUT' for event in events()),
                     'stale-data heartbeat')

            recorded = events()
            states = [event['state'] for event in recorded if event['type'] == 'DETECTION_STATUS']
            assert states == ['COLLECTING_HISTORY', 'SEARCHING', 'CANDIDATE_UNCONFIRMED',
                              'CANDIDATE_REJECTED', 'CONFIRMED'], states
            loops = [event for event in recorded if event['type'] == 'GLOBAL_LOOP']
            assert len(loops) == 1 and loops[0]['target_id'] == 5, loops
            assert loops[0]['optimization_error'] == 0.25
            assert loops[0]['optimization_iterations'] == 8
            assert sum(event['type'] == 'LOOP_CANDIDATE' for event in recorded) == 3
            console_text = console_path.read_text()
            for marker in ('[回环检测状态]', '[回环检测 #', '[回环候选]', '[回环成功 #1]'):
                assert marker in console_text, marker
            summary = {'passed': True, 'domain': domain, 'states': states,
                       'global_loops': len(loops), 'candidates': 3,
                       'output_dir': str(output_dir)}
            (output_dir / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
            print(json.dumps(summary, ensure_ascii=False))
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        publisher_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
