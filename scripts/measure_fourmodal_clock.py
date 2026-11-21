#!/usr/bin/env python3
"""Read remote wall time over SSH; report offset and round-trip uncertainty."""
import argparse
import json
from pathlib import Path
import subprocess
import time


def sensor_arrival_estimate():
    """Estimate the current robot's shared sensor clock for a live preview.

    A minimum arrival lag includes unknown capture/transport latency. It is not a
    wall-clock measurement or evidence of hardware synchronization.
    """
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2
    topics = {'rgb': ('/camera/color/image_raw', Image),
              'infrared': ('/camera/infra2/image_rect_raw', Image),
              'lidar': ('/livox/lidar', PointCloud2)}
    samples = {name: [] for name in topics}
    rclpy.init()
    node = rclpy.create_node('fourmodal_sensor_clock_estimate')
    subscriptions = []
    def receive(name, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        arrival = time.time()
        if stamp > 0 and abs(arrival - stamp) < 86400:
            samples[name].append({'source_stamp': stamp, 'arrival_stamp': arrival,
                                  'arrival_minus_source_s': arrival - stamp})
    try:
        for name, (topic, kind) in topics.items():
            subscriptions.append(node.create_subscription(kind, topic,
                lambda message, name=name: receive(name, message), qos_profile_sensor_data))
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if any(len(rows) < 3 for rows in samples.values()):
        raise RuntimeError('Cannot estimate preview offsets: RGB, NIR and LiDAR must each provide at least 3 live frames')
    lags = {name: min(row['arrival_minus_source_s'] for row in rows) for name, rows in samples.items()}
    # RGB/NIR come from one RealSense clock. Separate arrival delays must never
    # be applied as separate clock corrections: that destroys paired exposure times.
    image_gaps = [min(abs(rgb['source_stamp']-nir['source_stamp']) for nir in samples['infrared'])
                  for rgb in samples['rgb']]
    if sum(gap <= .05 for gap in image_gaps) < len(image_gaps) / 2:
        raise RuntimeError('RGB/NIR shared acquisition clock could not be verified from live paired stamps')
    offset = min(lags.values())
    spread = max(max(row['arrival_minus_source_s'] for row in rows) - lags[name]
                 for name, rows in samples.items())
    return dict(method='sensor_minimum_arrival_lag_preview', measured_at=time.time(),
        workstation_minus_robot_s=offset,
        modality_offsets_s=dict(rgb=offset, infrared=offset, lidar=offset, audio=0.0),
        per_stream_minimum_arrival_lag_s=lags, rgb_nir_raw_stamp_gaps_s=image_gaps,
        sensor_clock_scope='shared_current_robot_clock',
        round_trip_uncertainty_s=None, offset_observed_spread_s=spread,
        samples=samples, hardware_synchronization_validated=False, transport_latency_unknown=True,
        note='Receive-time preview estimate; includes unknown capture and network delay. Source timestamps are unchanged. Not valid for precise source/target localization.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='unitree@192.168.123.18')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sensor-fallback', action='store_true', help='If SSH fails, estimate live preview alignment from sensor arrival times')
    args = parser.parse_args()
    samples = []
    try:
        for _ in range(5):
            start = time.time()
            completed = subprocess.run(['ssh', '-n', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', args.host,
                "python3 -c 'import time; print(time.time())'"], capture_output=True, text=True, timeout=12, check=True)
            end = time.time()
            remote = float(completed.stdout.strip())
            samples.append(dict(round_trip_s=end - start, remote_stamp=remote,
                                local_midpoint_stamp=(start + end) / 2,
                                workstation_minus_robot_s=(start + end) / 2 - remote))
        best = min(samples, key=lambda sample: sample['round_trip_s'])
        report = dict(method='ssh_wall_clock_midpoint_min_rtt', measured_at=time.time(), host=args.host,
                      workstation_minus_robot_s=best['workstation_minus_robot_s'],
                      round_trip_uncertainty_s=best['round_trip_s'] / 2, samples=samples,
                      hardware_synchronization_validated=False,
                      note='Clock estimate only. Does not bound sensor exposure/scan/ADC latency or establish hardware synchronization.')
    except (subprocess.SubprocessError, ValueError) as error:
        if not args.sensor_fallback:
            raise
        report = sensor_arrival_estimate()
        report['ssh_error'] = str(error)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
