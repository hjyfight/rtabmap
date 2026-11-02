#!/usr/bin/env python3
"""Integration check against start_miti.sh on isolated ROS domain 91, ports 13000/18765.

Publishes synthetic data only in the test domain, receives the actual Foxglove CDR frames,
and decodes them with ROS. This does not validate browser rendering or sensor performance.
"""
import base64
import json
import os
from pathlib import Path
import subprocess
import time
import urllib.request

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Path as RosPath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rtabmap_msgs.msg import MapGraph
from rtabmap_semantic_msgs.msg import Observation, SemanticMap, SemanticObject
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

PROJECT = Path(__file__).resolve().parents[1]
APP = PROJECT / '.tools/miti/root/usr/lib/miti'
TYPES = {'/cloud_map': PointCloud2, '/camera/color/image_raw': Image,
         '/camera/aligned_depth_to_color/image_raw': Image, '/tf_static': TFMessage,
         '/miti/trajectory': RosPath, '/miti/semantic_markers': MarkerArray,
         '/miti/loop_status': String}
CLIENT = r"""
const WS = require(process.argv[1]);
const wanted = new Set(JSON.parse(process.argv[2]));
const socket = new WS('ws://127.0.0.1:18765', ['foxglove.sdk.v1', 'foxglove.websocket.v1']);
const subscriptions = new Map();
const frames = {};
const timer = setTimeout(() => {
  console.error('Missing topics: ' + [...wanted].filter(t => !frames[t]).join(', '));
  socket.terminate(); process.exit(1);
}, 25000);
socket.on('error', error => {console.error(error.message); process.exit(1);});
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
    if (topic) frames[topic] = data.subarray(13).toString('base64');
    // The adapter may publish an initial empty marker snapshot before its graph arrives.
    if ([...wanted].every(t => frames[t])) {
      setTimeout(() => {
        console.log(JSON.stringify(frames));
        clearTimeout(timer); socket.close(); process.exit(0);
      }, 1200).unref();
    }
  }
});
"""


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '91' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires ROS_DOMAIN_ID=91 and ROS_LOCALHOST_ONLY=1; never run on the live domain.')
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    for route, expected in [('/', b'MITI'), ('/rtabmap.html', '实时建图'.encode()),
                            ('/rtabmap-layout.json', b'rtabmap-scene')]:
        with http.open('http://127.0.0.1:13000' + route, timeout=5) as response:
            assert response.status == 200 and expected in response.read(), route
    with http.open('http://127.0.0.1:13000/api/bridge', timeout=5) as response:
        state = json.load(response)
    assert state['state'] == 'running' and state['domainId'] == 91, state
    with http.open('http://127.0.0.1:13000/rtabmap-panel-assets.json', timeout=5) as response:
        assets = json.load(response)['assets']
    bundled = []
    for asset in assets:
        with http.open('http://127.0.0.1:13000' + asset, timeout=5) as response:
            bundled.append(response.read().decode())
    assert any('rtabmapLoopStatusFactory' in source for source in bundled)
    assert any('type:"loop-status",label:' in source for source in bundled)
    client = subprocess.Popen([str(APP / 'node/bin/node'), '-e', CLIENT,
                               str(APP / 'node_modules/next/dist/compiled/ws'),
                               json.dumps(list(TYPES))], stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, text=True)
    rclpy.init()
    node = rclpy.create_node('miti_connection_test')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publications = [(topic, kind, node.create_publisher(kind, topic, latched))
                    for topic, kind in [('/cloud_map', PointCloud2), ('/tf_static', TFMessage),
                                        ('/mapGraph', MapGraph), ('/semantic/objects', SemanticMap),
                                        ('/miti/loop_status', String)]]
    for topic in ['/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw']:
        publications.append((topic, Image, node.create_publisher(Image, topic, qos_profile_sensor_data)))
    try:
        start = time.monotonic()
        while client.poll() is None and time.monotonic() - start < 28:
            header = Header(frame_id='map', stamp=node.get_clock().now().to_msg())
            cloud = create_cloud_xyz32(header, [(0, 0, 0), (1, 2, 3), (2, 3, 4)])
            pose = Pose()
            pose.position.x = 1.0
            graph = MapGraph(header=header, poses_id=[1, 2], poses=[Pose(), pose])
            anchor = Observation(node_id=2, weight=1.0)
            anchor.position.x = anchor.position.y = 1.0
            objects = SemanticMap(header=header, map_key='synthetic-connection-test', map_uuid='test',
                                  objects=[SemanticObject(object_id=1, label='test-chair', observations=[anchor])])
            transform = TransformStamped(header=header, child_frame_id='camera_color_optical_frame')
            camera_header = Header(frame_id=transform.child_frame_id, stamp=header.stamp)
            rgb = Image(header=camera_header, width=2, height=2, encoding='rgb8', step=6, data=[100] * 12)
            depth = Image(header=camera_header, width=2, height=2, encoding='16UC1', step=4, data=[232, 3] * 4)
            loop_status = String(data=json.dumps({
                'schema': 1, 'monitor_id': 'isolated-miti-test', 'published_at': time.time(),
                'display_state': 'SUCCESS', 'title': '回环成功', 'phase': 'CONFIRMED',
                'phase_label': '已接受回环约束', 'nodes': 20,
                'counts': {'total': 1, 'global': 1, 'proximity': 0},
                'last_success': {'type': 'GLOBAL_LOOP', 'ref_id': 40, 'target_id': 5, 'time': time.time()},
                'rates_hz': {'lidar': 10.0, 'rgb': 6.0, 'depth': 6.0, 'slam_info': 1.0},
                'slam_age_seconds': 0.0, 'heartbeat_age_seconds': 0.0, 'success_hold_remaining': 8.0,
            }, ensure_ascii=False))
            messages = {'/cloud_map': cloud, '/mapGraph': graph, '/semantic/objects': objects,
                        '/tf_static': TFMessage(transforms=[transform]),
                        '/camera/color/image_raw': rgb, '/camera/aligned_depth_to_color/image_raw': depth,
                        '/miti/loop_status': loop_status}
            for topic, _, publisher in publications:
                publisher.publish(messages[topic])
            rclpy.spin_once(node, timeout_sec=0.2)
        output, error = client.communicate(timeout=4)
        assert client.returncode == 0, error
        frames = json.loads(output)
        decoded = {topic: deserialize_message(base64.b64decode(frames[topic]), kind)
                   for topic, kind in TYPES.items()}
        point = list(read_points(decoded['/cloud_map'], field_names=('x', 'y', 'z')))[1]
        assert tuple(float(point[field]) for field in ('x', 'y', 'z')) == (1.0, 2.0, 3.0)
        assert decoded['/camera/color/image_raw'].encoding == 'rgb8'
        assert decoded['/camera/aligned_depth_to_color/image_raw'].encoding == '16UC1'
        assert len(decoded['/miti/trajectory'].poses) == 2
        marker = decoded['/miti/semantic_markers'].markers[-1]
        assert marker.text == 'test-chair' and marker.pose.position.x == 2.0
        assert marker.pose.position.y == 1.0
        assert decoded['/tf_static'].transforms[0].child_frame_id == 'camera_color_optical_frame'
        loop_snapshot = json.loads(decoded['/miti/loop_status'].data)
        assert loop_snapshot['counts']['total'] == 1 and loop_snapshot['last_success']['target_id'] == 5
        assert loop_snapshot['display_state'] == 'SUCCESS' and loop_snapshot['title'] == '回环成功'
        result = {'passed': True, 'scope': 'HTTP + Foxglove WebSocket CDR + synthetic ROS messages',
                  'miti': '3.3.6', 'bridge': '3.6.0', 'domain': 91, 'topics': list(decoded),
                  'browser_rendering_tested': False, 'real_sensors_tested': False}
        report = PROJECT / 'logs/miti/connection_test.json'
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        if client.poll() is None:
            client.terminate()
            client.wait(timeout=4)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
