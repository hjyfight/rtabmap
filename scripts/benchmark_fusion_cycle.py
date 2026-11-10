#!/usr/bin/env python3
"""Replay captured real RGB-D frames through the real bridge/mapper in an isolated ROS domain."""
import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rtabmap_semantic'))
from measure_fusion_cycle import save_reports


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--input-period-ms', type=float, default=1000)
    p.add_argument('--task-period-ms', type=float)
    p.add_argument('--warmup', type=int, default=2)
    p.add_argument('--require-model-timing', action='store_true',
                   help='Fail unless every frame has valid RGB/depth/fusion module timings')
    p.add_argument('--bridge', type=Path, default=ROOT/'install/cycle/rtabmap_semantic/lib/rtabmap_semantic/keyframe_bridge')
    args = p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        p.error('Use a dedicated nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1 for replay')
    if not 1 <= args.input_period_ms <= 60000 or args.warmup < 0:
        p.error('invalid input period/warmup')
    if args.task_period_ms is not None and (not math.isfinite(args.task_period_ms) or args.task_period_ms <= 0):
        p.error('task-period-ms must be finite and positive')
    if not args.bridge.is_file():
        p.error('Build install/cycle first; see docs/fusion_cycle_measurement.md')
    if any(args.output.with_suffix(s).exists() for s in ('.json','.md','.jsonl')):
        p.error('output already exists')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.serialization import deserialize_message
    from rtabmap_msgs.msg import MapData
    from std_msgs.msg import String
    import yaml
    from rtabmap_semantic.node import SemanticMapper

    captured = deserialize_message((args.capture/'map_data.cdr').read_bytes(), MapData)
    manifest = json.loads((args.capture/'manifest.json').read_text())
    if len(captured.nodes) <= args.warmup:
        p.error('capture must contain more frames than warmup')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load((ROOT/'src/rtabmap_semantic/config/emsanet.yaml').read_text())['semantic_mapper']['ros__parameters']
    rclpy.init()
    executor = SingleThreadedExecutor()
    probe = rclpy.create_node('fusion_cycle_replay_probe')
    executor.add_node(probe)
    process = mapper = None
    events, send_times = [], []
    metadata = dict(mode='isolated_replay_of_real_keyframes', capture=manifest,
                    configured_input_period_ms=args.input_period_ms,
                    ros_domain_id=os.environ['ROS_DOMAIN_ID'], configuration=config,
                    limitations=['concurrent production workloads remain running',
                                 'upstream sensor/SLAM processing is not replayed',
                                 'separate fresh semantic database; not production map scale'])
    with tempfile.TemporaryDirectory(prefix='fusion_cycle_') as directory, \
            args.output.with_suffix('.jsonl').open('x') as raw, \
            args.output.with_suffix('.bridge.log').open('w') as log:
        def receive(msg):
            row = json.loads(msg.data)
            row['observer_received_ns'] = time.monotonic_ns()
            events.append(row)
            raw.write(json.dumps(row, ensure_ascii=False)+'\n')
        subscriptions = [probe.create_subscription(String, topic, receive, 100)
                         for topic in ('/semantic/timing','/semantic/bridge_timing')]
        publisher = probe.create_publisher(MapData, '/mapData', 5)
        parameters = {**config, 'database_path':str(Path(directory)/'replay.db'),
                      'model_path':str(ROOT/'models/emsanet/nyuv2_r34_NBt1D_pre.pth')}
        try:
            started_load = time.monotonic()
            mapper = SemanticMapper(parameter_overrides=[Parameter(k, value=v) for k,v in parameters.items()])
            metadata['mapper_initialization_ms'] = (time.monotonic()-started_load)*1000
            executor.add_node(mapper)
            process = subprocess.Popen([str(args.bridge), '--ros-args', '-p','replay_existing:=false'],
                                       stdout=log, stderr=subprocess.STDOUT)
            ready_deadline = time.monotonic()+15
            while publisher.get_subscription_count() < 1:
                executor.spin_once(timeout_sec=.05)
                if process.poll() is not None or time.monotonic() > ready_deadline:
                    raise RuntimeError('Isolated bridge did not become ready')
            # Let all small trace subscriptions discover before the first test input.
            deadline = time.monotonic()+2
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            started, sent = time.monotonic(), 0
            next_progress = started+5
            total = len(captured.nodes)
            while time.monotonic()-started < max(90., total*10 + total*args.input_period_ms/1000):
                if process.poll() is not None:
                    raise RuntimeError('Replay bridge exited unexpectedly')
                if sent < total and time.monotonic()-started >= sent*args.input_period_ms/1000:
                    data = MapData(header=captured.header, graph=deepcopy(captured.graph),
                                   nodes=[deepcopy(captured.nodes[sent])])
                    data.header.stamp = data.nodes[0].data.header.stamp
                    send_times.append(time.monotonic_ns())
                    publisher.publish(data)
                    sent += 1
                executor.spin_once(timeout_sec=.01)
                completed = sum(r.get('event') in ('completed','failed') for r in events)
                if completed >= total:
                    break
                if time.monotonic() >= next_progress:
                    print(f'回放已发送 {sent}/{total}，处理完成 {completed}/{total}', flush=True)
                    raw.flush()
                    next_progress = time.monotonic()+5
            metadata.update(sent_frames=sent, elapsed_s=time.monotonic()-started,
                            actual_send_intervals_ms=[(b-a)/1e6 for a,b in zip(send_times, send_times[1:])],
                            objects_in_final_state=len(mapper.state.objects))
        finally:
            if process is not None:
                process.terminate()
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            if mapper is not None:
                executor.remove_node(mapper)
                mapper.destroy_node()
            executor.remove_node(probe)
            probe.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            report = save_reports(args.output, events, args.task_period_ms, args.warmup,
                                  metadata=metadata)
            print(json.dumps(dict(report=str(args.output.with_suffix('.md')),
                                  evaluation=report['evaluation'], quality=report['quality'],
                                  metrics_ms=report['metrics_ms']), ensure_ascii=False), flush=True)
        if report['metrics_ms']['bridge_to_semantic']['count'] != len(captured.nodes)-args.warmup or \
                any(report['quality'][k] for k in ('invalid_records','sequence_gaps','failures_or_drops',
                    'mapper_unfinished_at_end','bridge_unfinished_at_end','completed_without_bridge')):
            raise SystemExit('Replay did not complete cleanly; inspect the saved report')
        if args.require_model_timing and (
                report['model_timing_coverage'].get('valid') != len(captured.nodes) or
                report['metrics_ms'].get('emsanet_feature_fusion', {}).get('count') != len(captured.nodes)-args.warmup):
            raise SystemExit('Missing or invalid model module timings; inspect the saved report')


if __name__ == '__main__':
    main()
