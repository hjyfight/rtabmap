#!/usr/bin/env python3
"""Measure the derived display topics; optionally run a temporary filter instance."""
import argparse
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from map_geometry_node import cloud_xyz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spawn', action='store_true')
    parser.add_argument('--seconds', type=float, default=25.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 60:
        parser.error('--seconds must be between 5 and 60')
    root = Path(__file__).resolve().parents[1]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log = args.output.with_suffix('.log').open('w')
    child = None
    rclpy.init()
    node = rclpy.create_node('dynamic_cloud_live_check')
    latest, times, status = {}, {}, []
    def receive(topic, msg):
        latest[topic] = msg
        times.setdefault(topic, []).append(time.monotonic())
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs = [node.create_subscription(PointCloud2, name, lambda msg, name=name: receive(name, msg), qos)
            for name in ['/cloud_map', '/miti/clean_map', '/miti/live_scan']]
    subs.append(node.create_subscription(String, '/miti/dynamic_cloud_status',
        lambda msg: status.append(json.loads(msg.data)), qos))
    try:
        if args.spawn:
            deadline = time.monotonic()+2
            while time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if node.count_publishers('/miti/clean_map'):
                raise RuntimeError('A dynamic filter already publishes /miti/clean_map; use without --spawn')
            child = subprocess.Popen(['/usr/bin/python3', str(root/'scripts/dynamic_cloud_node.py'),
                '--ros-args', '--params-file', str(root/'config/dynamic_cloud.yaml')],
                stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic()+args.seconds
        while time.monotonic()<deadline:
            if child is not None and child.poll() is not None:
                raise RuntimeError('Temporary filter exited; see '+str(args.output.with_suffix('.log')))
            rclpy.spin_once(node, timeout_sec=.1)
        report = dict(captured=time.time(), temporary_filter=args.spawn, topics={}, statuses=status,
            publishers={name: node.count_publishers(name) for name in ['/miti/clean_map', '/miti/live_scan']})
        for name, msg in latest.items():
            events = times[name]
            report['topics'][name] = dict(points=msg.width*msg.height, frame=msg.header.frame_id,
                rate_hz=(len(events)-1)/(events[-1]-events[0]) if len(events)>1 else 0., messages=len(events))
        timings = [s['processing_ms'] for s in status if 'processing_ms' in s]
        report['timing_samples_include_repeated_status'] = True
        report['max_reported_processing_ms'] = max(timings, default=None)
        report['last_status'] = status[-1] if status else None
        report['passed'] = all(name in latest for name in ['/cloud_map', '/miti/clean_map', '/miti/live_scan']) and any(
            s.get('state') == 'ready' for s in status) and all(count == 1 for count in report['publishers'].values())
        arrays = {name.strip('/').replace('/', '_'): cloud_xyz(msg) for name, msg in latest.items()}
        np.savez_compressed(args.output.with_suffix('.npz'), **arrays)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps({key: value for key, value in report.items() if key!='statuses'}, ensure_ascii=False, indent=2))
        if not report['passed']:
            raise SystemExit(1)
    finally:
        if child is not None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
