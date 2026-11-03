#!/usr/bin/env python3
"""Read the real two-map streams through MITI; never publish or alter SLAM settings."""
import argparse
import base64
import json
from pathlib import Path
import subprocess

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from check_miti_live import APP, CLIENT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge-url', default='ws://127.0.0.1:8765')
    parser.add_argument('--seconds', type=float, default=5)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compare-native', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 30:
        parser.error('seconds must be between 1 and 30')
    types = {'/miti/grid_map': OccupancyGrid, '/miti/grid_status': String,
             '/cloud_map': PointCloud2, '/robot_pose': PoseStamped}
    if args.compare_native:
        types['/map'] = OccupancyGrid
    result = subprocess.run([str(APP/'node/bin/node'), '-e', CLIENT,
                            str(APP/'node_modules/next/dist/compiled/ws'), args.bridge_url,
                            str(args.seconds), json.dumps(list(types))],
                            capture_output=True, text=True, check=True, timeout=args.seconds+15)
    rows = json.loads(result.stdout)
    report = {'missing': sorted(set(types)-set(rows)), 'topics': {}, 'browser_rendering_tested': False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for topic, row in rows.items():
        message = deserialize_message(base64.b64decode(row['cdr']), types[topic])
        entry = report['topics'][topic] = {'messages': row['count']}
        if hasattr(message, 'header'):
            entry['frame'] = message.header.frame_id
        if isinstance(message, OccupancyGrid):
            info = message.info
            values = np.asarray(message.data, dtype=np.int8)
            assert info.width*info.height == len(values) and len(values) > 0
            assert np.isin(values, [-1, 0, 100]).all()
            entry.update(width=info.width, height=info.height, resolution=info.resolution,
                size_m=[info.width*info.resolution, info.height*info.resolution],
                origin=[info.origin.position.x, info.origin.position.y, info.origin.position.z],
                unknown=int((values == -1).sum()), free=int((values == 0).sum()),
                occupied=int((values == 100).sum()))
            np.savez_compressed(args.output.with_suffix('.native.npz' if topic == '/map' else '.npz'),
                data=values.reshape(info.height, info.width), resolution=info.resolution, origin=entry['origin'])
        elif isinstance(message, PointCloud2):
            entry['points'] = message.width*message.height
        elif isinstance(message, String):
            entry['data'] = json.loads(message.data)
        else:
            p = message.pose.position
            entry['position'] = [p.x, p.y, p.z]
    grid = report['topics'].get('/miti/grid_map', {})
    status = report['topics'].get('/miti/grid_status', {}).get('data', {})
    report['passed'] = (not report['missing'] and grid.get('free', 0) > 0 and status.get('state') == 'ready'
        and all(row['frame'] == 'map' for row in report['topics'].values() if 'frame' in row)
        and report['topics'].get('/cloud_map', {}).get('points', 0) > 0)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
