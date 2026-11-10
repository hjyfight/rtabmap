#!/usr/bin/env python3
"""Read-only ROS timing collector; saves raw events, JSON and a Chinese report."""
import argparse
from collections import Counter
import datetime
import json
import math
import os
from pathlib import Path
import platform
import time

from fusion_cycle_report import summarize, markdown


def save_reports(prefix, events, period_ms=None, warmup=0, minimum_samples=30, metadata=None):
    report = summarize(events, period_ms, warmup, minimum_samples)
    report['measurement'] = metadata or {}
    prefix.parent.mkdir(parents=True, exist_ok=True)
    prefix.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    prefix.with_suffix('.md').write_text(markdown(report))
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--task-period-ms', type=float)
    p.add_argument('--warmup', type=int, default=0,
                   help='Exclude first N completed frames per observed mapper run, while retaining raw records')
    p.add_argument('--minimum-samples', type=int, default=30)
    p.add_argument('--odom-info-topic', default='/odom_info_lite')
    p.add_argument('--output', type=Path, default=Path('logs/fusion_cycle') /
                   datetime.datetime.now().strftime('live_%Y%m%d_%H%M%S'))
    p.add_argument('--from-jsonl', type=Path, help='Recompute an existing report without ROS')
    args = p.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 86400:
        p.error('seconds must be 1..86400')
    if args.warmup < 0 or args.minimum_samples < 1:
        p.error('invalid warmup/minimum-samples')
    if args.task_period_ms is not None and (not math.isfinite(args.task_period_ms) or args.task_period_ms <= 0):
        p.error('task-period-ms must be finite and positive')
    if any(args.output.with_suffix(s).exists() for s in ('.json', '.md', '.jsonl')):
        p.error('output already exists; choose a new output prefix')
    if args.from_jsonl:
        events = [json.loads(line) for line in args.from_jsonl.read_text().splitlines() if line.strip()]
        save_reports(args.output, events, args.task_period_ms, args.warmup, args.minimum_samples,
                     dict(mode='reanalyze', source=str(args.from_jsonl)))
        print(args.output.with_suffix('.md'))
        return

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rtabmap_msgs.msg import Info, OdomInfo
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('fusion_cycle_measurement')
    events, counts = [], Counter()
    started = time.monotonic_ns()
    run = str(started)
    previous = {}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stream = args.output.with_suffix('.jsonl').open('x')
    sensor = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = []

    def record(row):
        row = {**row, 'observer_received_ns': time.monotonic_ns()}
        events.append(row)
        stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')

    def trace(msg):
        try:
            row = json.loads(msg.data)
            if not isinstance(row, dict):
                raise ValueError('trace is not an object')
            record(row)
            counts['trace_events'] += 1
        except (ValueError, TypeError):
            counts['malformed_events'] += 1
            record({'invalid_trace': True})

    def native(metrics, event='sample'):
        counts['native_events'] += 1
        record(dict(source='native', run_id=run, sequence=counts['native_events'],
                    event=event, event_ns=time.monotonic_ns(), metrics_ms=metrics))

    def receive_interval(name):
        now = time.monotonic_ns()
        last = previous.get(name)
        previous[name] = now
        return {} if last is None else {name+'_receive_interval': (now-last)/1e6}

    def info(msg):
        stats = dict(zip(msg.stats_keys, msg.stats_values))
        values = receive_interval('slam')
        if 'Timing/Total/ms' in stats:
            values['slam_total'] = float(stats['Timing/Total/ms'])
        for name in ('TimingMem/Keypoints_detection/ms', 'TimingMem/Descriptors_extraction/ms'):
            if name in stats:
                values[name] = float(stats[name])
        native(values)
        counts['slam_updates'] += 1

    def odom(msg):
        values = receive_interval('icp')
        values['icp_estimation'] = 1000*float(msg.time_estimation)
        native(values)
        counts['odom_updates'] += 1
        if msg.lost:
            counts['odom_lost'] += 1

    def status(msg):
        try:
            row = json.loads(msg.data)
            current = int(row['processed']), int(row['failed'])
            old = previous.get('status')
            if old is not None and current[0] > old[0]:
                delta = current[0]-old[0]
                # A repeated status is not a new inference. If multiple completions
                # were missed, only the final duration is known.
                native({'legacy_inference_projection': 1000*float(row['last_inference_seconds'])})
                counts['legacy_completions_without_duration'] += delta-1
            if old is not None and current[1] > old[1]:
                counts['legacy_inference_failures'] += current[1]-old[1]
            if old is not None and (current[0] < old[0] or current[1] < old[1]):
                counts['legacy_counter_resets'] += 1
            previous['status'] = current
            counts['status_updates'] += 1
            previous['latest_status'] = row
        except (ValueError, KeyError, TypeError):
            counts['malformed_status'] += 1

    topics = [('/semantic/timing', String, trace, reliable),
              ('/semantic/bridge_timing', String, trace, reliable),
              ('/semantic/status', String, status, sensor),
              ('/info', Info, info, sensor), (args.odom_info_topic, OdomInfo, odom, sensor)]
    for topic, kind, callback, qos in topics:
        subscriptions.append(node.create_subscription(kind, topic, callback, qos))
    metadata = dict(mode='live_passive', host=platform.node(), platform=platform.platform(),
                    started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    topics=[t[0] for t in topics], environment={k: os.environ.get(k) for k in
                    ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','CYCLONEDDS_URI')})
    next_progress = time.monotonic()+10
    print(f'只读采样 {args.seconds:g} 秒；报告前缀 {args.output}', flush=True)
    try:
        while rclpy.ok() and (time.monotonic_ns()-started)/1e9 < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() >= next_progress:
                print(json.dumps(dict(elapsed_s=(time.monotonic_ns()-started)/1e9,
                                      counts=dict(counts)), ensure_ascii=False), flush=True)
                stream.flush()
                next_progress = time.monotonic()+10
    except KeyboardInterrupt:
        metadata['interrupted'] = True
    finally:
        stream.close()
        metadata.update(elapsed_s=(time.monotonic_ns()-started)/1e9, counters=dict(counts),
                        latest_semantic_status=previous.get('latest_status'),
                        publishers={topic: [dict(node=x.node_name, namespace=x.node_namespace)
                            for x in node.get_publishers_info_by_topic(topic)] for topic, *_ in topics})
        report = save_reports(args.output, events, args.task_period_ms, args.warmup, args.minimum_samples, metadata)
        print(json.dumps(dict(report=str(args.output.with_suffix('.md')),
                              evaluation=report['evaluation'], metrics_ms=report['metrics_ms']), ensure_ascii=False), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
