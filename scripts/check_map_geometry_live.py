#!/usr/bin/env python3
"""Read-only verification of the wall/pose streams delivered by the live MITI bridge."""
import argparse
import base64
import json
import subprocess
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

from check_miti_live import APP, CLIENT
from map_geometry_node import cloud_xyz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge-url', default='ws://127.0.0.1:8765')
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--capture-only', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 30:
        parser.error('seconds must be between 1 and 30')
    types = {'/cloud_map': PointCloud2, '/icp_odom': Odometry, '/tf': TFMessage, '/tf_static': TFMessage}
    if not args.capture_only:
        types.update({'/robot_pose': PoseStamped, '/miti/walls': MarkerArray, '/miti/map_status': String})
    result = subprocess.run([str(APP/'node/bin/node'), '-e', CLIENT,
                             str(APP/'node_modules/next/dist/compiled/ws'), args.bridge_url,
                             str(args.seconds), json.dumps(list(types))], capture_output=True,
                            text=True, check=True, timeout=args.seconds+15)
    rows = json.loads(result.stdout)
    report = dict(missing=sorted(set(types)-set(rows)), topics={}, browser_rendering_tested=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for topic, row in rows.items():
        message = deserialize_message(base64.b64decode(row['cdr']), types[topic])
        entry = report['topics'][topic] = {'messages': row['count']}
        if isinstance(message, PointCloud2):
            points = cloud_xyz(message)
            np.savez_compressed(args.output.with_suffix('.npz'), points=points)
            entry.update(points=len(points), frame=message.header.frame_id)
        elif isinstance(message, TFMessage):
            entry['transforms'] = [{'parent': t.header.frame_id, 'child': t.child_frame_id,
                                    'stamp': t.header.stamp.sec + t.header.stamp.nanosec/1e9}
                                   for t in message.transforms]
        elif isinstance(message, Odometry):
            entry.update(frame=message.header.frame_id, child=message.child_frame_id)
        elif isinstance(message, PoseStamped):
            p, q = message.pose.position, message.pose.orientation
            entry.update(frame=message.header.frame_id, position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w])
        elif isinstance(message, String):
            entry['data'] = json.loads(message.data)
        elif isinstance(message, MarkerArray):
            entry.update(active=len([m for m in message.markers if m.action == 0]),
                         triangle_vertices=sum(len(m.points) for m in message.markers if m.action == 0 and m.type == 11))
    report['passed'] = not report['missing']
    if not args.capture_only:
        data = report['topics'].get('/miti/map_status', {}).get('data', {})
        report['passed'] = report['passed'] and data.get('pose', {}).get('valid', False) and data.get('walls', {}).get('state') == 'ready'
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
