#!/usr/bin/env python3
"""Capture physical sensor messages without replacing acquisition timestamps."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from rtabmap_features_msgs.msg import AudioFrame

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'src/rtabmap_features'))
from rtabmap_features.fusion import MODALITIES, Sample, acquisition_gap, match_samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--tolerance', type=float, default=0.05)
    args = parser.parse_args()
    if not 3 <= args.seconds <= 60 or args.output.exists():
        parser.error('Require 3..60 seconds and a new output directory')
    args.output.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node('fourmodal_hardware_capture')
    bridge = CvBridge()
    buffers = {name: deque(maxlen=48) for name in MODALITIES}
    records = {name: [] for name in MODALITIES}
    errors = []
    topics = dict(rgb='/camera/color/image_raw', infrared='/camera/infra2/image_rect_raw',
                  lidar='/livox/lidar', audio='/multimodal/audio')
    best = None
    def receive(name, message):
        nonlocal best
        received = time.time()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        duration = len(message.samples) / message.sample_rate / message.channels if name == 'audio' else 0
        records[name].append(dict(arrival=received, stamp=stamp, duration=duration))
        try:
            if name in ('rgb', 'infrared'):
                payload = bridge.imgmsg_to_cv2(message, 'rgb8' if name == 'rgb' else 'mono8').copy()
                attributes = dict(encoding=message.encoding)
            elif name == 'lidar':
                points = point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True)
                payload = np.column_stack([points[field] for field in ('x', 'y', 'z')])
                attributes = dict(units='m', source_kind='physical_mid360',
                                  motion_compensation_performed_here=False)
            else:
                payload = np.asarray(message.samples, np.float32).reshape(-1, message.channels)
                attributes = dict(sample_rate=message.sample_rate, channels=message.channels,
                                  stamp_quality=message.stamp_quality,
                                  location='workstation_microphone_not_verified_colocated_with_robot')
            attributes.update(host_arrival_stamp=received, original_header_stamp=stamp,
                              physical_capture=True, topic=topics[name])
            sample = Sample(payload, stamp, message.header.frame_id,
                            f'physical:{topics[name]}:{stamp:.9f}', duration, attributes)
            buffers[name].append(sample)
            for anchor in reversed(buffers['rgb']):
                selected = match_samples(buffers, anchor, args.tolerance)
                if selected:
                    best = selected
                    break
        except Exception as error:
            if len(errors) < 12:
                errors.append(dict(modality=name, error=str(error)))
    subscriptions = [node.create_subscription(kind, topics[name], lambda msg, n=name: receive(n, msg),
                     qos_profile_sensor_data) for name, kind in
                     [('rgb', Image), ('infrared', Image), ('lidar', PointCloud2), ('audio', AudioFrame)]]
    started = time.monotonic()
    print('Capturing physical RGB/NIR/LiDAR/audio; source stamps preserved', flush=True)
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = time.monotonic() - started
        topics_report = {}
        for name in MODALITIES:
            data = records[name]
            unique = len(set(row['stamp'] for row in data))
            topics_report[name] = dict(topic=topics[name], count=len(data), unique_stamps=unique,
                observed_hz=round((len(data) - 1) / max(data[-1]['arrival'] - data[0]['arrival'], 1e-6), 3)
                    if len(data) > 1 else 0,
                publishers=[dict(name=p.node_name, namespace=p.node_namespace)
                            for p in node.get_publishers_info_by_topic(topics[name])],
                first_samples=data[:2], last_samples=data[-2:])
        latest = {name: values[-1] for name, values in buffers.items() if values}
        chosen = best or latest
        manifest = dict(format_version=1, calibration_version='physical_capture_unverified',
                        alignment='acquisition_timestamps' if best else 'unpaired_smoke_test',
                        sync_tolerance_s=args.tolerance, samples={},
                        note='Physical sensors; audio captured on workstation. Microphone colocation, '
                             'hardware clock synchronization and geometric calibration are not validated. '
                             + ('Original source times matched within configured tolerance.' if best else
                                'No full source-time matched bundle. Latest raw inputs retained only; '
                                'any inference must be explicitly labeled an unpaired dataflow test.'))
        from PIL import Image as PILImage
        import soundfile as sf
        for name, sample in chosen.items():
            filename = dict(rgb='rgb.png', infrared='infrared.png', lidar='lidar.npy', audio='audio.wav')[name]
            path = args.output / filename
            if name in ('rgb', 'infrared'):
                PILImage.fromarray(sample.payload).save(path)
            elif name == 'lidar':
                np.save(path, sample.payload, allow_pickle=False)
            else:
                sf.write(path, sample.payload, sample.attributes['sample_rate'], subtype='FLOAT')
            manifest['samples'][name] = dict(path=filename, stamp=sample.stamp,
                duration=sample.duration, frame_id=sample.frame_id, source_id=sample.source_id,
                attributes=sample.attributes)
        gaps = {}
        if 'rgb' in latest:
            for name in MODALITIES:
                if buffers[name]:
                    gaps[name] = min(acquisition_gap(name, sample, latest['rgb'].stamp) for sample in buffers[name])
        report = dict(physical_sensors=True, source_timestamps_preserved=True,
            acquisition_time_matched=bool(best), configured_tolerance_s=args.tolerance,
            all_four_received=all(buffers.values()), min_gaps_to_latest_rgb_s=gaps,
            elapsed_s=elapsed, topics=topics_report, errors=errors,
            environment={key: os.environ.get(key, '') for key in ('ROS_DOMAIN_ID', 'CYCLONEDDS_URI')},
            microphone_colocation_validated=False, geometry_calibrated=False,
            manifest=str((args.output / 'manifest.json').resolve()))
        (args.output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
        (args.output / 'capture_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k not in ('topics', 'environment')}, indent=2))
        for name, row in topics_report.items():
            print(f'{name}: {row["count"]} messages, {row["observed_hz"]} Hz, {row["unique_stamps"]} unique stamps')
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
