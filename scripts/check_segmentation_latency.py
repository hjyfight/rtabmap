#!/usr/bin/env python3
"""Measure source-frame age and RGB-to-fusion latency using matching frame IDs."""
import argparse
from collections import OrderedDict
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Image
from std_msgs.msg import String


def quantile(values, fraction=.5):
    return float(np.quantile(values, fraction)) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--output', type=Path,
                        default=Path('logs/segmentation_validation/latency.json'))
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error('--seconds must be between 0 and 60')
    rclpy.init()
    node = rclpy.create_node('segmentation_frame_latency_check')
    rgb_arrivals = OrderedDict()
    counts = dict(rgb=0, nir=0, depth=0)
    inputs = {kind: [] for kind in counts}
    views = dict(rgb=[], fusion=[])
    statuses, subscriptions, queries = {}, [], {}

    def on_image(kind, message):
        counts[kind] += 1
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        inputs[kind].append(time.time() - stamp / 1e9)
        if kind == 'rgb':
            rgb_arrivals[stamp] = time.monotonic()
            while len(rgb_arrivals) > 512:
                rgb_arrivals.popitem(last=False)

    def on_view(kind, message):
        data = json.loads(message.data)
        stamp = data['stamp']['sec'] * 1_000_000_000 + data['stamp']['nanosec']
        views[kind].append(dict(
            sequence=data['sequence'], session_id=data['session_id'], stamp=stamp,
            arrival=time.monotonic(), wall=time.time(), published_at=data['published_at'],
            inference_s=data['inference_s'], timings_s=data.get('timings_s', {}),
            input_arrival=rgb_arrivals.get(stamp)))

    for kind, topic in [('rgb', '/camera/color/image_raw'),
                        ('nir', '/camera/infra2/image_rect_raw'),
                        ('depth', '/camera/depth_registered/image_rect')]:
        subscriptions.append(node.create_subscription(
            Image, topic, lambda m, k=kind: on_image(k, m), qos_profile_sensor_data))
    for kind, topic in [('rgb', '/miti/segmentation/view'),
                        ('fusion', '/miti/segmentation/fusion_view')]:
        subscriptions.append(node.create_subscription(
            String, topic, lambda m, k=kind: on_view(k, m), 2))
    for kind, topic in [('rgb', '/miti/segmentation/status'),
                        ('fusion', '/miti/segmentation/fusion_status')]:
        subscriptions.append(node.create_subscription(
            String, topic, lambda m, k=kind: statuses.update({k: json.loads(m.data)}), 2))
    for name, params in [
        ('yolo_segmentation', ['device', 'detector_backend', 'max_inference_hz',
                               'max_input_age_s', 'rgb_clock_offset_s']),
        ('segmentation_fusion', ['device', 'min_period_s', 'sync_tolerance_s']),
    ]:
        queries[name] = dict(client=node.create_client(GetParameters, '/' + name + '/get_parameters'),
                             names=params, future=None)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            for query in queries.values():
                if query['future'] is None and query['client'].service_is_ready():
                    query['future'] = query['client'].call_async(GetParameters.Request(names=query['names']))
            rclpy.spin_once(node, timeout_sec=.05)
        parameters = {}
        for name, query in queries.items():
            future = query['future']
            if future and future.done() and not future.exception():
                parameters[name] = {k: parameter_value_to_python(v) if v.type else None
                                    for k, v in zip(query['names'], future.result().values)}
        offset = parameters.get('yolo_segmentation', {}).get('rgb_clock_offset_s')
        summary = {}
        for kind, rows in views.items():
            intervals = np.diff([r['arrival'] for r in rows]).tolist()
            local_latency = [r['arrival'] - r['input_arrival'] for r in rows
                             if r['input_arrival'] is not None]
            summary[kind] = dict(
                updates=len(rows), matched_rgb_inputs=len(local_latency),
                median_update_interval_s=quantile(intervals),
                p95_update_interval_s=quantile(intervals, .95),
                median_inference_s=quantile([r['inference_s'] for r in rows]),
                median_from_rgb_arrival_s=quantile(local_latency),
                p95_from_rgb_arrival_s=quantile(local_latency, .95),
                median_source_age_s=quantile([r['wall'] - r['stamp']/1e9 - offset for r in rows])
                    if offset is not None else None,
                median_publish_to_receive_s=quantile([r['wall'] - r['published_at'] for r in rows]))
        baseline = {(r['session_id'], r['sequence']): r for r in views['rgb']}
        extra = [r['published_at'] - baseline[(r['session_id'], r['sequence'])]['published_at']
                 for r in views['fusion'] if (r['session_id'], r['sequence']) in baseline]
        duration = time.monotonic() - start
        report = dict(
            seconds=duration, parameters=parameters, sensor_hz={k: v/duration for k, v in counts.items()},
            median_input_source_age_s={k: quantile([v-offset for v in values])
                                       if offset is not None else None for k, values in inputs.items()},
            views=summary, median_rgb_result_to_fusion_s=quantile(extra),
            median_nir_inference_s=quantile([r['timings_s']['nir_inference'] for r in views['fusion']
                                           if 'nir_inference' in r['timings_s']]),
            last_status=statuses, source_age_uses_configured_clock_offset=True,
            local_arrival_latency_uses_monotonic_clock=True)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
