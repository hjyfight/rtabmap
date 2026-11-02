#!/usr/bin/env python3
"""Read-only HTTP/WebSocket check against the running MITI demonstration."""
import argparse
import base64
import datetime
import json
from pathlib import Path
import subprocess
import urllib.request
from zoneinfo import ZoneInfo

from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

PROJECT = Path(__file__).resolve().parents[1]
APP = PROJECT / '.tools/miti/root/usr/lib/miti'
TYPES = {'/cloud_map': PointCloud2, '/camera/color/image_raw': Image,
         '/camera/aligned_depth_to_color/image_raw': Image, '/icp_odom': Odometry,
         '/miti/aligned_depth/image_raw': Image,
         '/miti/trajectory': RosPath, '/miti/semantic_markers': MarkerArray,
         '/semantic/status': String, '/miti/loop_status': String,
         '/tf': TFMessage, '/tf_static': TFMessage}
CLIENT = r"""
const WS = require(process.argv[1]);
const url = process.argv[2], seconds = Number(process.argv[3]);
const wanted = new Set(JSON.parse(process.argv[4]));
const socket = new WS(url, ['foxglove.sdk.v1', 'foxglove.websocket.v1']);
const subscriptions = new Map(), result = {};
socket.on('error', e => { console.error(e.message); process.exit(1); });
socket.on('message', (data, binary) => {
  if (!binary) {
    const message = JSON.parse(data.toString());
    if (message.op === 'advertise') {
      const added = [];
      for (const channel of message.channels) {
        if (wanted.has(channel.topic) && ![...subscriptions.values()].includes(channel.topic)) {
          const id = subscriptions.size + 1;
          subscriptions.set(id, channel.topic);
          added.push({id, channelId: channel.id});
        }
      }
      if (added.length) socket.send(JSON.stringify({op: 'subscribe', subscriptions: added}));
    }
  } else if (data[0] === 1 && data.length > 13) {
    const topic = subscriptions.get(data.readUInt32LE(1));
    if (!topic) return;
    const now = performance.now();
    const row = result[topic] ??= {count: 0, bytes: 0, first: now};
    row.count++; row.bytes += data.length - 13; row.last = now;
    row.cdr = data.subarray(13).toString('base64');
  }
});
setTimeout(() => {
  // Real map payloads exceed the stdout pipe buffer; wait for the write to flush.
  process.stdout.write(JSON.stringify(result), () => { socket.close(); process.exit(0); });
}, seconds * 1000);
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--web-url', default='http://127.0.0.1:3000')
    parser.add_argument('--bridge-url', default='ws://127.0.0.1:8765')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error('--seconds must be between 1 and 60')
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    for route in ('/', '/rtabmap.html', '/rtabmap-layout.json'):
        with http.open(args.web_url + route, timeout=5) as response:
            assert response.status == 200, route
    with http.open(args.web_url + '/api/bridge', timeout=5) as response:
        bridge = json.load(response)
    completed = subprocess.run(
        [str(APP / 'node/bin/node'), '-e', CLIENT,
         str(APP / 'node_modules/next/dist/compiled/ws'), args.bridge_url,
         str(args.seconds), json.dumps(list(TYPES))],
        capture_output=True, text=True, timeout=args.seconds + 8, check=True)
    frames = json.loads(completed.stdout)
    result = {'timestamp': datetime.datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
              'bridge_state': bridge.get('state'), 'domain': bridge.get('domainId'),
              'topics': {}, 'missing': sorted(set(TYPES) - set(frames)),
              'browser_rendering_tested': False}
    for topic, row in frames.items():
        message = deserialize_message(base64.b64decode(row.pop('cdr')), TYPES[topic])
        elapsed = (row.pop('last') - row.pop('first')) / 1000
        row['hz'] = round((row['count'] - 1) / elapsed, 2) if elapsed > 0 else 0
        if hasattr(message, 'header'):
            row['frame'] = message.header.frame_id
            row['stamp'] = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if isinstance(message, PointCloud2):
            row['points'] = message.width * message.height
        elif isinstance(message, Image):
            row.update(width=message.width, height=message.height, encoding=message.encoding)
        elif isinstance(message, RosPath):
            row['poses'] = len(message.poses)
        elif isinstance(message, MarkerArray):
            row['labels'] = [marker.text for marker in message.markers if marker.text and marker.action == 0]
        elif isinstance(message, String):
            row['status'] = json.loads(message.data)
        elif isinstance(message, TFMessage):
            row['frames'] = sorted({f'{t.header.frame_id} -> {t.child_frame_id}' for t in message.transforms})
        result['topics'][topic] = row
    result['passed'] = not result['missing'] and result['bridge_state'] == 'running'
    report = PROJECT / 'logs/miti/live_stream_check.json'
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
