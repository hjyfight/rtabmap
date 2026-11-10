#!/usr/bin/env python3
"""Verify topology and sparse features delivered through the running MITI bridge."""
import argparse
import base64
import json
import subprocess
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from check_miti_live import APP, CLIENT
from map_geometry_node import cloud_xyz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge-url', default='ws://127.0.0.1:8765')
    parser.add_argument('--seconds', type=float, default=12)
    parser.add_argument('--allow-waiting', action='store_true', help='Accept empty maps while SLAM is stopped')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 30:
        parser.error('seconds must be between 1 and 30')
    types = {'/miti/topological_map': MarkerArray, '/miti/sparse_features': PointCloud2,
             '/miti/graph_feature_status': String}
    result = subprocess.run([str(APP/'node/bin/node'), '-e', CLIENT,
                             str(APP/'node_modules/next/dist/compiled/ws'), args.bridge_url,
                             str(args.seconds), json.dumps(list(types))], capture_output=True,
                            text=True, timeout=args.seconds+15)
    if result.returncode:
        raise SystemExit('MITI bridge check failed: ' + result.stderr.strip())
    rows = json.loads(result.stdout)
    report = dict(missing=sorted(set(types)-set(rows)), topics={}, browser_rendering_tested=False)
    for topic, row in rows.items():
        message = deserialize_message(base64.b64decode(row['cdr']), types[topic])
        entry = report['topics'][topic] = dict(messages=row['count'])
        if isinstance(message, String):
            entry['data'] = json.loads(message.data)
        elif isinstance(message, PointCloud2):
            points = cloud_xyz(message)
            entry.update(points=len(points), frame=message.header.frame_id, finite=bool(np.isfinite(points).all()))
        else:
            active = [m for m in message.markers if m.action == Marker.ADD]
            entry.update(nodes=sum(len(m.points) for m in active if m.ns == 'rtabmap/topology/nodes'),
                         links=sum(len(m.points)//2 for m in active if m.type == Marker.LINE_LIST),
                         frames=sorted({m.header.frame_id for m in message.markers}),
                         namespaces=[m.ns for m in active])
    status = report['topics'].get('/miti/graph_feature_status', {}).get('data', {})
    graph = report['topics'].get('/miti/topological_map', {})
    cloud = report['topics'].get('/miti/sparse_features', {})
    report['transport_passed'] = not report['missing']
    report['map_data_ready'] = (not report['missing'] and status.get('state') == 'ready' and
                        graph.get('nodes', 0) > 0 and cloud.get('points', 0) > 0 and cloud.get('finite', False) and
                        cloud.get('frame') == status.get('frame_id') and graph.get('frames') == [status.get('frame_id')])
    waiting = (report['transport_passed'] and status.get('state') == 'waiting' and
               graph.get('nodes') == 0 and cloud.get('points') == 0)
    report['passed'] = report['map_data_ready'] or (args.allow_waiting and waiting)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
