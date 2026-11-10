#!/usr/bin/env python3
"""Read-only validation of navigation topology delivered by the MITI bridge."""
import argparse
import base64
import json
from pathlib import Path
import subprocess

from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from check_miti_live import APP, CLIENT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=8.)
    parser.add_argument('--bridge-url', default='ws://127.0.0.1:8765')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-waiting', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 30:
        parser.error('seconds must be between 1 and 30')
    types = {'/miti/navigation_map': MarkerArray, '/miti/navigation_graph': String, '/miti/navigation_status': String}
    process = subprocess.run([str(APP/'node/bin/node'), '-e', CLIENT,
        str(APP/'node_modules/next/dist/compiled/ws'), args.bridge_url, str(args.seconds), json.dumps(list(types))],
        capture_output=True, text=True, timeout=args.seconds+15, check=True)
    rows = json.loads(process.stdout)
    report = dict(missing=sorted(set(types)-set(rows)), topics={}, browser_rendering_tested=False)
    decoded = {}
    for topic, row in rows.items():
        message = deserialize_message(base64.b64decode(row['cdr']), types[topic])
        if isinstance(message, String):
            decoded[topic] = json.loads(message.data)
            report['topics'][topic] = dict(messages=row['count'])
        else:
            report['topics'][topic] = dict(messages=row['count'],
                node_markers=sum(m.ns == 'navigation/nodes' and m.action == 0 for m in message.markers))
    graph, status = decoded.get('/miti/navigation_graph', {}), decoded.get('/miti/navigation_status', {})
    report.update(transport_passed=not report['missing'], state=graph.get('state'), reason=graph.get('reason'),
        nodes=len(graph.get('nodes', [])), edges=len(graph.get('edges', [])), components=len(graph.get('components', [])),
        observed_free_cells=graph.get('observed_free_cells'), traversable_cells=graph.get('traversable_cells'),
        required_clearance_m=graph.get('required_clearance_m'), max_observed_clearance_m=graph.get('max_observed_clearance_m'))
    consistent = (not report['missing'] and status.get('revision') == graph.get('revision') and
                  status.get('frame_id') == graph.get('frame_id'))
    report['navigation_ready'] = bool(consistent and graph.get('state') == 'ready' and report['nodes'] > 0 and
        all(e['min_clearance_m'] >= graph['required_clearance_m'] for e in graph['edges']))
    waiting = (consistent and graph.get('state') == 'insufficient_free_space' and not report['nodes'] and not report['edges'])
    report['passed'] = report['navigation_ready'] or bool(args.allow_waiting and waiting)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
